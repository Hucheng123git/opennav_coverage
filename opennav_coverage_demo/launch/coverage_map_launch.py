# Copyright (c) 2026 Open Navigation LLC
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

"""Load an editable map and visualize coverage planning without a simulator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the map server, coverage server, client service, and RViz2."""
    demo_dir = get_package_share_directory('opennav_coverage_demo')
    map_directory = LaunchConfiguration('map_directory')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_directory', default_value=os.path.expanduser('~/.ros/coverage_map'),
            description='Directory containing field.json, map.yaml, and map.pgm'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('software_rendering', default_value='true'),
        DeclareLaunchArgument('auto_plan', default_value='true'),
        DeclareLaunchArgument('route_type', default_value='SPIRAL'),
        DeclareLaunchArgument('spiral_n', default_value='4'),
        DeclareLaunchArgument('swath_angle', default_value='0.0'),
        DeclareLaunchArgument(
            'contour_spacing', default_value='0.0',
            description='CONCENTRIC contour spacing in meters; 0 uses the server default'),
        DeclareLaunchArgument('arrow_spacing', default_value='2.0'),
        DeclareLaunchArgument('show_swath_numbers', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(demo_dir, 'coverage_map_params.yaml')),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='coverage_map_frame', output='screen',
            arguments=[
                '--frame-id', 'map', '--child-frame-id', 'coverage_map_visualization',
            ]),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{
                'use_sim_time': False,
                'yaml_filename': PathJoinSubstitution([map_directory, 'map.yaml']),
                'frame_id': 'map',
                'topic_name': 'map',
            }]),
        Node(
            package='opennav_coverage', executable='opennav_coverage',
            name='coverage_server', output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': False}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_coverage_map', output='screen', parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'coverage_server'],
            }]),
        Node(
            package='opennav_coverage_demo', executable='coverage_map_client',
            name='coverage_map_client', output='screen', parameters=[{
                'use_sim_time': False,
                'map_directory': map_directory,
                'route_type': LaunchConfiguration('route_type'),
                'spiral_n': ParameterValue(LaunchConfiguration('spiral_n'), value_type=int),
                'swath_angle': ParameterValue(
                    LaunchConfiguration('swath_angle'), value_type=float),
                'contour_spacing': ParameterValue(
                    LaunchConfiguration('contour_spacing'), value_type=float),
                'arrow_spacing': ParameterValue(
                    LaunchConfiguration('arrow_spacing'), value_type=float),
                'show_swath_numbers': ParameterValue(
                    LaunchConfiguration('show_swath_numbers'), value_type=bool),
                'auto_plan': ParameterValue(LaunchConfiguration('auto_plan'), value_type=bool),
            }]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(demo_dir, 'coverage_map.rviz')],
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': LaunchConfiguration('software_rendering'),
            },
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
