from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detector_pkg',  # Cambia por el nombre de tu paquete
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[
                # Puedes agregar parámetros aquí si los necesitas
            ]
        ),
    ])