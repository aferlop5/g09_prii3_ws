#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g09_prii3',
            executable='jetbot_obstacle_avoidance',
            name='jetbot_obstacle_avoidance',
            output='screen',
            parameters=[{'avoidance_mode': 'simple'}],
        )
    ])
