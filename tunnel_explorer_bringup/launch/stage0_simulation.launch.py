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
# Stage 0: WSL2 environment feasibility verification.
# Thin wrapper around nav2_bringup tb3_simulation_launch.py with
# custom params and RViz config. SLAM Toolbox is enabled via slam:=True.
#
# Usage:
#   ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py
#   ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py headless:=True rviz:=False

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('tunnel_explorer_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_tb3_sim = get_package_share_directory('nav2_minimal_tb3_sim')
    launch_dir = os.path.join(pkg_nav2_bringup, 'launch')

    default_world = os.path.join(
        pkg_tb3_sim, 'worlds', 'tb3_sandbox.sdf.xacro',
    )

    # Arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Launch RViz2 with stage0 custom config'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Run Gazebo headless (no GUI)'
    )
    use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composable nodes (default False for WSL2 DDS reliability)'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 params YAML file'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to Gazebo world SDF/SDF.xacro file',
    )

    # Nav2 all-in-one TB3 simulation
    # We always disable the built-in RViz in tb3_simulation; launch our own below.
    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'True',
            'autostart': 'True',
            'headless': LaunchConfiguration('headless'),
            'use_rviz': 'False',
            'use_composition': LaunchConfiguration('use_composition'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # Custom RViz with our stage0 view
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'stage0_view.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    fastdds_env = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(
            pkg_bringup, 'config', 'fastdds_udp_only.xml'
        ),
    )

    return LaunchDescription([
        fastdds_env,
        rviz_arg,
        headless_arg,
        use_composition_arg,
        params_file_arg,
        world_arg,
        tb3_sim,
        rviz_node,
    ])
