#!/usr/bin/env python3

"""
Master launch file for the Chess-Playing Robot System
Supports both simulation (Gazebo) and real hardware modes

Usage:
  - Simulation: ros2 launch final_work chess_system.launch.py mode:=sim
  - Real: ros2 launch final_work chess_system.launch.py mode:=real ur_type:=ur3e
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    """Generate launch description for the chess robot system."""

    # Arguments
    mode = LaunchConfiguration("mode", default="sim")  # sim or real
    ur_type = LaunchConfiguration("ur_type", default="ur3")
    launch_rviz = LaunchConfiguration("rviz", default="true")
    gui = LaunchConfiguration("gui", default="true")

    declare_mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        description="Mode: 'sim' for Gazebo simulation or 'real' for real hardware",
    )

    declare_ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3",
        description="UR robot type: 'ur3' or 'ur3e'",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz visualization"
    )

    declare_gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI (only in sim mode)",
    )

    # Include chesslab_setup2 launch file (provides robot, scene, controllers)
    chesslab_share = FindPackageShare("chesslab_setup2")
    chesslab_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([chesslab_share, "launch", "chesslab_setup2_demo.launch.py"])
        ),
        launch_arguments=[
            ("ur_type", ur_type),
            ("gui", gui),
            ("launch_gazebo", IfCondition(mode == "sim")),
        ],
        condition=IfCondition(mode == "sim"),
    )

    # Aruco broadcaster node (detects ArUco markers)
    aruco_share = FindPackageShare("aruco_broadcaster")
    aruco_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_share, "launch", "aruco_broadcaster.launch.py"])
        ),
    )

    # Sensing module node (provides piece location services)
    sensing_share = FindPackageShare("sensing_module")
    sensing_node = Node(
        package="sensing_module",
        executable="sensing_node",
        name="sensing_node",
        output="screen",
        parameters=[
            {
                "aruco_ids": [
                    # Black pieces
                    201, 202, 203, 204, 205, 206, 207, 208,  # pawns
                    209, 210,  # rooks
                    211, 212,  # knights
                    213, 214,  # bishops
                    215,  # queen
                    216,  # king
                    # White pieces
                    301, 302, 303, 304, 305, 306, 307, 308,  # pawns
                    309, 310,  # rooks
                    311, 312,  # knights
                    313, 314,  # bishops
                    315,  # queen
                    316,  # king
                ],
                "yaml_output_path": "/tmp/chess_configuration_sensed.yaml",
            }
        ],
    )

    # Planning module node (plans trajectories)
    planning_node = Node(
        package="planning_module",
        executable="planning_node",
        name="planning_node",
        output="screen",
    )

    # Action manager node (orchestrates chess moves)
    action_manager_node = Node(
        package="action_manager",
        executable="action_manager_node",
        name="action_manager",
        output="screen",
    )

    # RViz visualization
    rviz_config = PathJoinSubstitution([chesslab_share, "rviz", "chesslab_setup2_demo.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            declare_mode_arg,
            declare_ur_type_arg,
            declare_rviz_arg,
            declare_gui_arg,
            # Launch in order
            chesslab_demo_launch,
            aruco_broadcaster_launch,
            sensing_node,
            planning_node,
            action_manager_node,
            rviz_node,
        ]
    )
