from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gazebo_to_map_offset_x = LaunchConfiguration('gazebo_to_map_offset_x')
    gazebo_to_map_offset_y = LaunchConfiguration('gazebo_to_map_offset_y')
    nav_start_delay = LaunchConfiguration('nav_start_delay')

    nav_node = Node(
        package='g09_prii3',
        executable='nav_f1l3_cordenadas',
        name='nav_f1l3_cordenadas',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'frame_id': 'map',
            'gazebo_to_map_offset_x': gazebo_to_map_offset_x,
            'gazebo_to_map_offset_y': gazebo_to_map_offset_y,
            'nav_start_delay': nav_start_delay,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_to_map_offset_x',
            default_value='4.527328',
            description='Offset en X para convertir coordenadas de Gazebo a mapa.',
        ),
        DeclareLaunchArgument(
            'gazebo_to_map_offset_y',
            default_value='2.852645',
            description='Offset en Y para convertir coordenadas de Gazebo a mapa.',
        ),
        DeclareLaunchArgument(
            'nav_start_delay',
            default_value='0.0',
            description='Tiempo de espera antes de enviar el objetivo de navegación.',
        ),
        nav_node,
    ])
