from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g09_prii3',
            executable='autonomous_navigation',
            name='autonomous_navigation',
            output='screen'
        )
    ])
