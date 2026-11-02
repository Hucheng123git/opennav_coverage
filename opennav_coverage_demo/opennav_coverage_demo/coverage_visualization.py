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

"""Build RViz markers from the planner's returned path and ordered components."""

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


_WORK = (0.12, 0.45, 1.0)
_TURN = (1.0, 0.45, 0.05)
_DIRECTION = (1.0, 0.9, 0.2)
_MAX_ARROWS = 1000


def _point(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def _marker(frame_id, stamp, namespace, identifier, kind, color):
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame_id, stamp
    marker.ns, marker.id = namespace, identifier
    marker.type, marker.action = kind, Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    return marker


def _text(frame_id, stamp, namespace, identifier, text, x, y, height, color):
    marker = _marker(frame_id, stamp, namespace, identifier, Marker.TEXT_VIEW_FACING, color)
    marker.pose.position = _point(x, y, 0.65)
    marker.scale.z, marker.text = height, text
    return marker


def _coordinates(path):
    return [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses
            if math.isfinite(pose.pose.position.x) and math.isfinite(pose.pose.position.y)]


def empty_plan_markers(frame_id, stamp):
    """Clear a plan's MarkerArray display, including markers from an older plan."""
    clear = Marker()
    clear.header.frame_id, clear.header.stamp = frame_id, stamp
    clear.action = Marker.DELETEALL
    result = MarkerArray()
    result.markers.append(clear)
    return result


def _direction_markers(points, frame_id, stamp, spacing, clip_corners=False):
    segments, total = [], 0.0
    for start, end in zip(points, points[1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length > 1e-9:
            if clip_corners and segments:
                begin, _, previous_start, previous_end = segments[-1]
                px, py = previous_end[0] - previous_start[0], previous_end[1] - previous_start[1]
                dx, dy = end[0] - start[0], end[1] - start[1]
                # Merge dense samples along one straight run, retaining actual
                # corners and direction reversals as segment boundaries.
                aligned = abs(px * dy - py * dx) <= 1e-7 * math.hypot(px, py) * length
                connected = math.hypot(start[0] - previous_end[0],
                                       start[1] - previous_end[1]) <= 1e-8
                if connected and aligned and px * dx + py * dy > 0.0:
                    total += length
                    segments[-1] = (begin, total, previous_start, end)
                    continue
            segments.append((total, total + length, start, end))
            total += length
    if not segments:
        return []
    # Increase spacing for unusually long/dense paths instead of truncating the
    # arrows at the beginning of a plan. Positions follow physical path length.
    spacing = max(spacing, total / _MAX_ARROWS)
    arrow_length = min(0.75, 0.45 * spacing, total)
    markers, segment_index = [], 0
    for identifier in range(_MAX_ARROWS):
        distance = min(spacing, total) / 2 + identifier * spacing
        if distance >= total:
            break
        while segments[segment_index][1] < distance:
            segment_index += 1
        begin, finish, start, end = segments[segment_index]
        current_length = arrow_length
        if clip_corners:
            # Both arrow endpoints must stay inside this complete straight run;
            # a tangent extension at a vertex can otherwise cut across a hole.
            current_length = min(arrow_length, 2.0 * min(distance - begin, finish - distance))
            if current_length < min(0.05, 0.25 * arrow_length):
                continue
        ux, uy = (end[0] - start[0]) / (finish - begin), (end[1] - start[1]) / (finish - begin)
        x, y = start[0] + ux * (distance - begin), start[1] + uy * (distance - begin)
        marker = _marker(frame_id, stamp, 'coverage_direction', identifier,
                         Marker.ARROW, _DIRECTION)
        # Explicit tail/head points express travel order, including reverse gear;
        # robot orientation quaternions can point opposite to the actual motion.
        marker.points = [_point(x - ux * current_length / 2, y - uy * current_length / 2, 0.24),
                         _point(x + ux * current_length / 2, y + uy * current_length / 2, 0.24)]
        marker.scale.x = min(0.07, current_length / 8)
        marker.scale.y = min(0.22, current_length / 3)
        marker.scale.z = min(0.28, current_length * 0.35)
        markers.append(marker)
    return markers


def make_plan_markers(path, components, route_type, spiral_n, stamp,
                      arrow_spacing=2.0, show_swath_numbers=True, contour_spacing=0.0):
    """Visualize work, connections, travel direction, endpoints, and visit order.

    ``path`` is the full nav_msgs/Path result; ``components`` is its PathComponents
    result. Swath numbers follow returned array order only when swaths_ordered is
    true. Arrows follow XY arclength and point order, independently of robot yaw.
    Arrow spacing must be finite and positive; at most 1000 arrows are generated.
    CONCENTRIC labels C1, C2, ... identify closed contours in visit order. Their
    edges remain separate swaths in PathComponents for message compatibility.
    No ROS node or publisher is created, and the returned array starts with DELETEALL.
    """
    if isinstance(arrow_spacing, bool):
        raise ValueError('arrow_spacing must be a finite positive number')
    try:
        arrow_spacing = float(arrow_spacing)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('arrow_spacing must be a finite positive number') from error
    if not math.isfinite(arrow_spacing) or arrow_spacing <= 0.0:
        raise ValueError('arrow_spacing must be a finite positive number')
    mode = str(route_type).upper()
    if mode == 'CONCENTRIC':
        try:
            spacing = float(contour_spacing)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError('contour_spacing must be finite and nonnegative') from error
        if isinstance(contour_spacing, bool) or not math.isfinite(spacing) or spacing < 0.0:
            raise ValueError('contour_spacing must be finite and nonnegative')
        spacing_label = f'{spacing:g} m' if spacing else 'server/default'
    frame_id = path.header.frame_id or components.header.frame_id or 'map'
    result = empty_plan_markers(frame_id, stamp)
    points = _coordinates(path)
    bounds = list(points)
    ordered_numbers = show_swath_numbers and components.swaths_ordered
    contour_start, contour_number = None, 0
    work = _marker(frame_id, stamp, 'coverage_work', 0, Marker.LINE_LIST, _WORK)
    work.scale.x = 0.075
    for index, swath in enumerate(components.swaths):
        start, end = swath.start, swath.end
        if not all(math.isfinite(value) for value in (start.x, start.y, end.x, end.y)):
            continue
        work.points.extend([_point(start.x, start.y, 0.12), _point(end.x, end.y, 0.12)])
        bounds.extend([(start.x, start.y), (end.x, end.y)])
        length = math.hypot(end.x - start.x, end.y - start.y)
        label = None
        if ordered_numbers and length > 1e-9:
            if mode == 'CONCENTRIC':
                if contour_start is None:
                    contour_start = (start.x, start.y)
                    contour_number += 1
                    label = f'C{contour_number}'
                if math.hypot(end.x - contour_start[0], end.y - contour_start[1]) <= 1e-5:
                    contour_start = None
            else:
                label = str(index + 1)
        if label is not None:
            x = (start.x + end.x) / 2 - 0.08 * (end.y - start.y) / length
            y = (start.y + end.y) / 2 + 0.08 * (end.x - start.x) / length
            result.markers.append(_text(frame_id, stamp, 'coverage_swath_numbers', index,
                                        label, x, y, 0.4, (0.03, 0.06, 0.12)))
    if work.points:
        result.markers.append(work)
    for index, turn in enumerate(components.turns):
        turn_points = _coordinates(turn)
        if len(turn_points) < 2:
            continue
        marker = _marker(frame_id, stamp, 'coverage_turns', index, Marker.LINE_STRIP, _TURN)
        marker.scale.x = 0.08
        marker.points = [_point(x, y, 0.14) for x, y in turn_points]
        result.markers.append(marker)
        bounds.extend(turn_points)
    result.markers.extend(_direction_markers(
        points, frame_id, stamp, arrow_spacing, clip_corners=(mode == 'CONCENTRIC')))
    if points:
        for index, (label, position, color) in enumerate([
                ('START', points[0], (0.1, 1.0, 0.3)),
                ('END', points[-1], (1.0, 0.15, 0.75))]):
            x, y = position
            marker = _marker(frame_id, stamp, 'coverage_endpoints', index, Marker.SPHERE, color)
            marker.pose.position = _point(x, y, 0.28)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            result.markers.append(marker)
            label_y = y + (0.45 if index == 0 else -0.45)
            result.markers.append(_text(frame_id, stamp, 'coverage_endpoint_labels', index,
                                        label, x, label_y, 0.35, color))
    if bounds:
        description = {
            'SPIRAL': f'SPIRAL swath ordering | group={spiral_n}',
            'BOUSTROPHEDON': 'BOUSTROPHEDON | alternating rows',
            'SNAKE': 'SNAKE | row ordering',
            'CUSTOM': 'CUSTOM | supplied swath permutation',
            'CONCENTRIC': 'CONCENTRIC | inward contours',
        }.get(mode, mode)
        if mode == 'CONCENTRIC':
            description += f' | spacing={spacing_label} | segments={len(components.swaths)}'
        else:
            description += f' | swaths={len(components.swaths)}'
        detail = 'Arrows: travel direction'
        if ordered_numbers:
            order_label = 'contour visit order' if mode == 'CONCENTRIC' else 'visit order'
            detail += f' | numbers: {order_label}'
        elif not components.swaths_ordered:
            detail += ' | segments unordered' if mode == 'CONCENTRIC' else ' | swaths unordered'
        if mode == 'CONCENTRIC':
            legend = (f'{description}\nBlue: contour edges | Orange: links\n{detail}\n'
                      'Polyline: turn in place (no Dubins smoothing)')
        else:
            legend = f'{description}\nBlue: work | Orange: turns\n{detail}'
        # Text markers are centered. Estimate half the text width to align the
        # legend's left edge with the path's leftmost point in the top-down view.
        x = min(point[0] for point in bounds) + max(map(len, legend.splitlines())) * 0.1
        y = max(point[1] for point in bounds) + 1.4
        result.markers.append(_text(frame_id, stamp, 'coverage_legend', 0,
                                    legend, x, y, 0.4, (1.0, 1.0, 1.0)))
    return result
