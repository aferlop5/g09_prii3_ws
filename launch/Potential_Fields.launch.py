#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	goal_x_arg = DeclareLaunchArgument(
		'goal_x', default_value='1.0', description='Objetivo X en marco global (map/odom)'
	)
	goal_y_arg = DeclareLaunchArgument(
		'goal_y', default_value='0.0', description='Objetivo Y en marco global (map/odom)'
	)
	global_frame_arg = DeclareLaunchArgument(
		'global_frame', default_value='odom', description='Marco global (map u odom)'
	)
	base_frame_arg = DeclareLaunchArgument(
		'base_frame', default_value='base_link', description='Marco base del robot'
	)

	node = Node(
		package='g09_prii3',
		executable='jetbot_potential_fields',
		name='jetbot_potential_fields',
		output='screen',
		parameters=[
			{
				'goal_x': LaunchConfiguration('goal_x'),
				'goal_y': LaunchConfiguration('goal_y'),
				'global_frame': LaunchConfiguration('global_frame'),
				'base_frame': LaunchConfiguration('base_frame'),
			}
		],
	)

	return LaunchDescription([
		goal_x_arg,
		goal_y_arg,
		global_frame_arg,
		base_frame_arg,
		node,
	])

