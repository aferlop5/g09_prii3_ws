import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Configuración del nodo de navegación con ArUcos
    aruco_nav_node = Node(
        package='g09_prii3',
        executable='aruco_nav_node',
        name='aruco_navigation',
        output='screen',
        parameters=[],
        remappings=[
            # Remappings si es necesario
            # ('/camera/image_raw', '/camera/color/image_raw'),
        ]
    )

    return LaunchDescription([
        aruco_nav_node,
    ])