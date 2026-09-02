# Copyright 2026 zhouyi
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
#
# Coverage-mode simulation (C0-C3 MVP): static map + AMCL localisation
# (slam:=False) on a benchmark world, plus the coverage executor node.
# The exploration launch (stage0_simulation.launch.py) is unchanged; this
# launch only starts the coverage track (R15).
#
# Usage:
#   ros2 launch tunnel_explorer_bringup coverage_simulation.launch.py \
#     world:=cleaning_room_rect.sdf map:=.../cleaning_room_rect.yaml \
#     headless:=True
#
# The default robot pose comes from tunnel_worlds/maps/initial_poses.yaml
# (profile cleaning_room_rect); pass x_pose/y_pose/yaw to override.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_pose_profile(profile):
    """Read x/y/yaw from tunnel_worlds/maps/initial_poses.yaml."""
    pkg_worlds = get_package_share_directory('tunnel_worlds')
    path = os.path.join(pkg_worlds, 'maps', 'initial_poses.yaml')
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        profiles = (data or {}).get('profiles', {})
        p = profiles.get(profile, {})
        return str(p.get('x', 0.0)), str(p.get('y', 0.0)), str(p.get('yaw', 0.0))
    except Exception as exc:  # noqa: BLE001 - degrade to origin on any error
        print(f'[coverage_simulation] cannot read {path}: {exc} '
              f'(falling back to 0,0,0)')
        return '0.0', '0.0', '0.0'


def generate_launch_description():
    pkg_bringup = get_package_share_directory('tunnel_explorer_bringup')
    pkg_worlds = get_package_share_directory('tunnel_worlds')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(pkg_nav2_bringup, 'launch')

    default_world = os.path.join(pkg_worlds, 'worlds', 'cleaning_room_rect.sdf')
    default_map = os.path.join(pkg_worlds, 'maps', 'cleaning_room_rect.yaml')
    default_params = os.path.join(
        pkg_bringup, 'config', 'nav2_params_coverage_dwb.yaml')
    executor_params = os.path.join(
        get_package_share_directory('tunnel_coverage_executor'),
        'config', 'coverage_executor_params.yaml')

    # Default pose bound to the world asset (deterministic initial pose).
    dx, dy, dyaw = _load_pose_profile('cleaning_room_rect')

    # Arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='False',
        description='Launch RViz2 (headless recommended in WSL2)'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Run Gazebo headless (no GUI)'
    )
    use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composable nodes (default False for WSL2 DDS reliability)'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Path to the benchmark Gazebo world SDF'
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Path to the static map YAML for map_server + AMCL'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 params YAML for the coverage run'
    )
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value=dx,
        description='Robot spawn x (map frame)'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value=dy,
        description='Robot spawn y (map frame)'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value=dyaw,
        description='Robot spawn yaw (map frame)'
    )

    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'slam': 'False',             # static-map + AMCL mode (KTD1)
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'True',
            'autostart': 'True',
            'headless': LaunchConfiguration('headless'),
            'use_rviz': 'False',
            'use_composition': LaunchConfiguration('use_composition'),
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'stage0_view.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    coverage_executor = Node(
        package='tunnel_coverage_executor',
        executable='tunnel_coverage_executor',
        name='tunnel_coverage_executor',
        output='screen',
        parameters=[executor_params, {'use_sim_time': True}],
    )

    fastdds_env = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(pkg_bringup, 'config', 'fastdds_udp_only.xml'),
    )

    # AMCL needs an initial pose estimate (headless automation has no RViz
    # "2D Pose Estimate"); publish the deterministic spawn pose once the
    # stack has had a moment to start.
    initial_pose_pub = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: map}, '
                    'pose: {pose: {position: {x: ' + dx + ', y: ' + dy + '}, '
                    'orientation: {w: 1.0}}, '
                    'covariance: [0.25, 0, 0, 0, 0, 0, '
                    '0, 0.25, 0, 0, 0, 0, '
                    '0, 0, 0, 0, 0, 0, '
                    '0, 0, 0, 0.25, 0, 0, '
                    '0, 0, 0, 0, 0.25, 0, '
                    '0, 0, 0, 0, 0, 0.25]}}',
                ],
                output='log',
            ),
        ],
    )

    return LaunchDescription([
        fastdds_env,
        initial_pose_pub,
        rviz_arg,
        headless_arg,
        use_composition_arg,
        world_arg,
        map_arg,
        params_file_arg,
        x_pose_arg,
        y_pose_arg,
        yaw_arg,
        tb3_sim,
        coverage_executor,
        rviz_node,
    ])
