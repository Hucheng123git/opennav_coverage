#!/usr/bin/env python3
# Copyright 2026 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Reload an edited field and occupancy map, then visualize coverage in RViz.

The Trigger response waits for the planning result without blocking executor
threads.  This node only invokes ComputeCoveragePath; it never drives a robot.
"""

from dataclasses import dataclass
import math
from pathlib import Path as FilePath
import threading
import time
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Path
from opennav_coverage_demo.coverage_map import load_field
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class _PlanningRequest:
    field: dict
    map_yaml: FilePath
    goal: Any
    completion: Future
    deadline: float
    phase: str = 'waiting for active map and coverage servers'
    goal_handle: Any = None


class CoverageMapClient(Node):
    """Serve /coverage_map/plan and retain its latest visualization for RViz."""

    def __init__(self):
        super().__init__('coverage_map_client')
        self.declare_parameter('map_directory', str(FilePath.home() / '.ros' / 'coverage_map'))
        self.declare_parameter('route_type', 'SPIRAL')
        self.declare_parameter('spiral_n', 4)
        self.declare_parameter('swath_angle', 0.0)
        self.declare_parameter('auto_plan', True)
        self.declare_parameter('timeout', 60.0)

        self._lock = threading.RLock()
        self._active: Optional[_PlanningRequest] = None
        self._auto_attempted = False
        self._group = ReentrantCallbackGroup()
        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, '/coverage_map/path', retained)
        self._field_pub = self.create_publisher(MarkerArray, '/coverage_map/field', retained)
        self._map_client = self.create_client(
            LoadMap, '/map_server/load_map', callback_group=self._group)
        self._coverage_client = ActionClient(
            self, ComputeCoveragePath, 'compute_coverage_path', callback_group=self._group)
        self._lifecycle = {
            name: {
                'client': self.create_client(
                    GetState, f'/{name}/get_state', callback_group=self._group),
                'active': False, 'label': 'unknown', 'future': None,
                'queried_at': 0.0, 'received_at': 0.0,
            }
            for name in ('map_server', 'coverage_server')
        }
        self._plan_service = self.create_service(
            Trigger, '/coverage_map/plan', self._on_plan, callback_group=self._group)
        self._timer = self.create_timer(0.2, self._tick, callback_group=self._group)
        self.get_logger().info(
            'Edit the map, then call /coverage_map/plan (std_srvs/srv/Trigger).')

    async def _on_plan(self, _request, response):
        # A reentrant callback group lets load-map/action replies and the timeout
        # timer run while this service coroutine awaits its completion future.
        with self._lock:
            self._auto_attempted = True
            completion = self._begin_plan()
        response.success, response.message = await completion
        return response

    def _begin_plan(self):
        """Validate input before changing the map; call with _lock held."""
        completion = Future(executor=self.executor)
        if self._active is not None:
            completion.set_result((False, 'A coverage plan is already in progress.'))
            return completion
        try:
            directory = FilePath(
                self.get_parameter('map_directory').value).expanduser().resolve()
            field = load_field(directory / 'field.json')
            if field['frame_id'] != 'map':
                raise ValueError('field.json frame_id must be "map" for this demo')
            map_yaml = directory / 'map.yaml'
            if not map_yaml.is_file():
                raise ValueError(f'Map YAML does not exist: {map_yaml}')
            timeout = float(self.get_parameter('timeout').value)
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError('timeout must be a positive, finite number of seconds')
            goal = self._make_goal(field)
        except (OSError, ValueError, TypeError, OverflowError) as error:
            message = f'Invalid map or planning settings: {error}'
            self.get_logger().error(message)
            completion.set_result((False, message))
            return completion

        request = _PlanningRequest(
            field=field, map_yaml=map_yaml, goal=goal, completion=completion,
            deadline=time.monotonic() + timeout)
        self._active = request
        self._advance(request)
        return completion

    def _make_goal(self, field):
        route = self.get_parameter('route_type').value.upper()
        if route not in ('SPIRAL', 'BOUSTROPHEDON', 'SNAKE'):
            raise ValueError('route_type must be SPIRAL, BOUSTROPHEDON, or SNAKE')
        spiral_n = self.get_parameter('spiral_n').value
        if not 1 <= spiral_n <= 65535:
            raise ValueError('spiral_n must be between 1 and 65535')
        angle = float(self.get_parameter('swath_angle').value)
        if not math.isfinite(angle) or abs(angle) > 3.402823466e38:
            raise ValueError('swath_angle must be finite and fit in float32 (radians)')

        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = True
        goal.generate_route = True
        goal.generate_path = True
        goal.use_gml_file = False
        goal.frame_id = field['frame_id']
        for ring in [field['boundary'], *field['holes']]:
            polygon = Coordinates()
            for x, y in ring:
                if max(abs(x), abs(y)) > 3.402823466e38:
                    raise ValueError('field coordinates exceed the action message float32 range')
                point = Coordinate()
                point.axis1 = float(x)
                point.axis2 = float(y)
                polygon.coordinates.append(point)
            goal.polygons.append(polygon)
        # UNKNOWN uses the coverage server's configured headland/turn settings.
        goal.headland_mode.mode = 'UNKNOWN'
        goal.path_mode.mode = 'UNKNOWN'
        goal.path_mode.continuity_mode = 'UNKNOWN'
        goal.swath_mode.objective = 'LENGTH'
        goal.swath_mode.mode = 'SET_ANGLE'
        goal.swath_mode.best_angle = angle
        goal.route_mode.mode = route
        goal.route_mode.spiral_n = spiral_n
        return goal

    def _tick(self):
        with self._lock:
            self._poll_lifecycle()
            if self._active is not None:
                if time.monotonic() >= self._active.deadline:
                    request = self._active
                    self._cancel(request)
                    states = ', '.join(
                        f'{name}={state["label"]}' for name, state in self._lifecycle.items())
                    canceled = '; cancellation requested' if request.goal_handle else ''
                    self._finish(
                        request, False, f'Timed out {request.phase} ({states}){canceled}.')
                elif self._active.phase == 'waiting for active map and coverage servers':
                    self._advance(self._active)
            elif (not self._auto_attempted and self.get_parameter('auto_plan').value
                  and self._servers_ready()):
                self._auto_attempted = True
                self._begin_plan()

    def _servers_ready(self):
        return (self._map_client.service_is_ready()
                and self._coverage_client.server_is_ready()
                and all(
                    state['active'] and state['client'].service_is_ready()
                    and time.monotonic() - state['received_at'] < 2.0
                    for state in self._lifecycle.values()))

    def _poll_lifecycle(self):
        """Query lifecycle states asynchronously, including before auto planning."""
        now = time.monotonic()
        for state in self._lifecycle.values():
            if not state['client'].service_is_ready():
                state['active'] = False
                state['label'] = 'unavailable'
                continue
            if state['future'] is not None:
                if now - state['queried_at'] < 2.0:
                    continue
                state['future'].cancel()
                state['future'] = None
                state['active'] = False
                state['label'] = 'state query timed out'
            if now - state['queried_at'] < 0.5:
                continue
            state['queried_at'] = now
            try:
                future = state['client'].call_async(GetState.Request())
                state['future'] = future
                future.add_done_callback(
                    lambda completed, tracked=state: self._lifecycle_done(tracked, completed))
            except Exception:
                state['active'] = False
                state['label'] = 'state query failed'

    def _lifecycle_done(self, state, future):
        with self._lock:
            if state['future'] is not future:
                return
            state['future'] = None
            try:
                response = future.result()
                state['active'] = response.current_state.id == State.PRIMARY_STATE_ACTIVE
                state['label'] = response.current_state.label
                state['received_at'] = time.monotonic()
            except Exception:
                state['active'] = False
                state['label'] = 'state query failed'

    def _advance(self, request):
        if not self._servers_ready():
            return
        request.phase = 'loading map'
        # An empty retained Path removes a previous result while this map loads.
        self._clear_path(request.field['frame_id'])
        map_request = LoadMap.Request()
        map_request.map_url = str(request.map_yaml)
        try:
            future = self._map_client.call_async(map_request)
            future.add_done_callback(lambda result: self._map_loaded(request, result))
        except Exception as error:
            self._finish(request, False, f'Could not request map load: {error}')

    def _map_loaded(self, request, future):
        with self._lock:
            if self._active is not request:
                return
            try:
                result = future.result()
                if result is None or result.result != LoadMap.Response.RESULT_SUCCESS:
                    code = 'no response' if result is None else str(result.result)
                    self._finish(
                        request, False, f'Map server could not load map.yaml (code {code}).')
                    return
                self._publish_field(request.field)
                request.phase = 'waiting for coverage goal acceptance'
                sent = self._coverage_client.send_goal_async(request.goal)
                sent.add_done_callback(lambda accepted: self._goal_sent(request, accepted))
            except Exception as error:
                self._finish(request, False, f'Map load or coverage request failed: {error}')

    def _goal_sent(self, request, future):
        with self._lock:
            try:
                handle = future.result()
                if self._active is not request:
                    # A goal can be accepted after our deadline.  Cancel it too.
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                    return
                if handle is None or not handle.accepted:
                    self._finish(request, False, 'Coverage server rejected the planning goal.')
                    return
                request.goal_handle = handle
                request.phase = 'waiting for coverage planning result'
                result = handle.get_result_async()
                result.add_done_callback(lambda completed: self._plan_done(request, completed))
            except Exception as error:
                self._finish(request, False, f'Coverage goal submission failed: {error}')

    def _plan_done(self, request, future):
        with self._lock:
            if self._active is not request:
                return
            try:
                wrapped = future.result()
                if wrapped is None:
                    self._finish(request, False, 'Coverage server returned no result.')
                    return
                result = wrapped.result
                if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                        or result.error_code != ComputeCoveragePath.Result.NONE):
                    self._finish(
                        request, False,
                        f'Coverage planning failed: status={wrapped.status}, '
                        f'error_code={result.error_code}.')
                    return
                path = result.nav_path
                if not path.poses:
                    self._finish(request, False, 'Coverage planning returned an empty path.')
                    return
                if path.header.frame_id != request.field['frame_id']:
                    self._finish(
                        request, False, 'Coverage path frame does not match the map frame.')
                    return
                length = 0.0
                previous = None
                for pose in path.poses:
                    point = pose.pose.position
                    if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                        raise ValueError('coverage path contains a non-finite position')
                    if previous is not None:
                        length += math.hypot(point.x - previous.x, point.y - previous.y)
                    previous = point
                self._path_pub.publish(path)
                self._finish(
                    request, True, f'Planned {len(path.poses)} poses, {length:.2f} m; '
                    'published /coverage_map/path.')
            except Exception as error:
                self._finish(request, False, f'Could not obtain the coverage path: {error}')

    def _finish(self, request, success, message):
        if self._active is not request:
            return
        self._active = None
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        request.completion.set_result((success, message))

    def _cancel(self, request):
        if request.goal_handle is not None:
            try:
                request.goal_handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().warning(f'Could not cancel coverage goal: {error}')

    def _clear_path(self, frame_id):
        empty = Path()
        empty.header.frame_id = frame_id
        empty.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(empty)

    def _publish_field(self, field):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for index, ring in enumerate([field['boundary'], *field['holes']]):
            marker = Marker()
            marker.header.frame_id = field['frame_id']
            marker.header.stamp = stamp
            marker.ns = 'coverage_field'
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08
            marker.color.a = 1.0
            marker.color.r = 1.0 if index else 0.1
            marker.color.g = 0.55 if index else 0.8
            marker.color.b = 0.1 if index else 1.0
            for x, y in ring:
                point = Point()
                point.x, point.y, point.z = float(x), float(y), 0.05
                marker.points.append(point)
            markers.markers.append(marker)
        self._field_pub.publish(markers)

    def destroy_node(self):
        with self._lock:
            if self._active is not None:
                self._cancel(self._active)
                self._finish(self._active, False, 'Coverage map client is shutting down.')
        self._coverage_client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMapClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
