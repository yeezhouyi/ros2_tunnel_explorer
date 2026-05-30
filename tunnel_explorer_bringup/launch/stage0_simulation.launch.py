# Copyright 2026 zhouyi
# License: Apache-2.0
#
# Stage 0: WSL2 environment feasibility verification.
# Launches TurtleBot3 + Gazebo Harmonic + SLAM Toolbox + Nav2 + RViz2.
#
# Usage:
#   ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py
#   ros2 launch tunnel_explorer_bringup stage0_simulation.launch.py world:=tunnel_straight

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Paths ──────────────────────────────────────────────
    pkg_bringup = FindPackageShare("tunnel_explorer_bringup")
    pkg_nav2_bringup = FindPackageShare("nav2_bringup")
    pkg_tb3_sim = FindPackageShare("nav2_minimal_tb3_sim")
    pkg_slam_toolbox = FindPackageShare("slam_toolbox")

    # ── Arguments ──────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world", default_value="default",
        description="TurtleBot3 simulation world name"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2"
    )

    # ── Includes ───────────────────────────────────────────
    # TB3 simulation (Gazebo + robot_state_publisher + controller)
    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tb3_sim, "launch", "simulation.launch.py"])
        ]),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # SLAM Toolbox (online async mapping)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_slam_toolbox, "launch", "online_async_launch.py"])
        ]),
        launch_arguments={
            "slam_params_file": PathJoinSubstitution([
                pkg_bringup, "config", "slam_toolbox_params.yaml"
            ]),
            "use_sim_time": "true",
        }.items(),
    )

    # Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav2_bringup, "launch", "navigation_launch.py"])
        ]),
        launch_arguments={
            "params_file": PathJoinSubstitution([
                pkg_bringup, "config", "nav2_params.yaml"
            ]),
            "use_sim_time": "true",
        }.items(),
    )

    # ── Nodes ──────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([
            pkg_bringup, "rviz", "stage0_view.rviz"
        ])],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        tb3_sim,
        slam,
        nav2,
        rviz_node,
    ])
