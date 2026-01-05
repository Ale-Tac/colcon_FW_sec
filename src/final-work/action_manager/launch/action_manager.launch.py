#!/usr/bin/env python3

"""
Launch file for action_manager node
Usage: ros2 launch action_manager action_manager.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    action_manager_node = Node(
        package="action_manager",
        executable="action_manager_node",
        name="action_manager",
        output="screen",
        parameters=[],
        remappings=[
            ("piece_location", "piece_location"),
            ("plan_pick_place", "plan_pick_place"),
            ("set_robot_configuration", "set_robot_configuration"),
            ("set_object_pose", "set_object_pose"),
            ("gripper_order", "gripper_order"),
        ],
    )

    return LaunchDescription([action_manager_node])
