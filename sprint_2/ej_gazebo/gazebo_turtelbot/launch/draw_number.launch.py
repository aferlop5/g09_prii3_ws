from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    group_number = LaunchConfiguration('group_number', default='09')

    return LaunchDescription([
        DeclareLaunchArgument(
            'group_number',
            default_value='09',
            description='Número del grupo a dibujar'),
        Node(
            package='gazebo_turtelbot',
            executable='draw_number',
            name='turtlebot_number_node',
            output='screen',
            parameters=[{'group_number': group_number}]
        )
    ])
