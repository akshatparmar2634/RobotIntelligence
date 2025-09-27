#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ri_pkg',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen',
            parameters=[],
        )
    ])