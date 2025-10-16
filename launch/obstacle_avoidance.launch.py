#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    avoidance_mode = LaunchConfiguration('avoidance_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'avoidance_mode',
            default_value='simple',
            description="Obstacle avoidance mode: 'simple' or 'advanced'",
        ),
        Node(
            package='g09_prii3',
            executable='jetbot_obstacle_avoidance',
            name='jetbot_obstacle_avoidance',
            output='screen',
            parameters=[{'avoidance_mode': avoidance_mode}],
        )
    ])
