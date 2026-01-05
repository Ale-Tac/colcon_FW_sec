#!/usr/bin/env python3

"""
Launch file for perception and sensing only (debugging/visualization)
Usage: ros2 launch final_work chess_perception.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch perception and sensing for debugging."""

    launch_rviz = LaunchConfiguration("rviz", default="true")

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz visualization"
    )

    # Aruco broadcaster node
    aruco_share = FindPackageShare("aruco_broadcaster")
    aruco_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_share, "launch", "aruco_broadcaster.launch.py"])
        ),
    )

    # Sensing module node
    sensing_node = Node(
        package="sensing_module",
        executable="sensing_node",
        name="sensing_node",
        output="screen",
        parameters=[
            {
                "aruco_ids": [
                    # All chess piece IDs
                    201, 202, 203, 204, 205, 206, 207, 208,
                    209, 210, 211, 212, 213, 214, 215, 216,
                    301, 302, 303, 304, 305, 306, 307, 308,
                    309, 310, 311, 312, 313, 314, 315, 316,
                ],
                "yaml_output_path": "/tmp/chess_configuration_sensed.yaml",
            }
        ],
    )

    # RViz
    chesslab_share = FindPackageShare("chesslab_setup2")
    rviz_config = PathJoinSubstitution([chesslab_share, "rviz", "chesslab_setup2_demo.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=LaunchConfiguration("rviz"),
    )

    return LaunchDescription(
        [
            declare_rviz_arg,
            aruco_broadcaster_launch,
            sensing_node,
            rviz_node,
        ]
    )
