# Copyright 2026 zhouyi
# License: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tunnel_centerline_extractor')

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'centerline_extractor_params.yaml'),
        description='Path to centerline extractor YAML parameter file',
    )

    node = Node(
        package='tunnel_centerline_extractor',
        executable='tunnel_centerline_extractor',
        name='tunnel_centerline_extractor',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([params_file, node])
