from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Parámetros para la posición final (en COORDENADAS GAZEBO)
    goal_pose_x_gazebo = LaunchConfiguration('goal_pose_x_gazebo')
    goal_pose_y_gazebo = LaunchConfiguration('goal_pose_y_gazebo')
    goal_pose_z_gazebo = LaunchConfiguration('goal_pose_z_gazebo')
    goal_pose_roll_gazebo = LaunchConfiguration('goal_pose_roll_gazebo')
    goal_pose_pitch_gazebo = LaunchConfiguration('goal_pose_pitch_gazebo')
    goal_pose_yaw_gazebo = LaunchConfiguration('goal_pose_yaw_gazebo')

    # Nodo principal
    rviz_predefinido_node = Node(
        package='g09_prii3',
        executable='rviz_predefinido_node',
        name='rviz_predefinido_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            # COORDENADAS EN EL SISTEMA GAZEBO
            'goal_pose_x_gazebo': goal_pose_x_gazebo,
            'goal_pose_y_gazebo': goal_pose_y_gazebo,
            'goal_pose_z_gazebo': goal_pose_z_gazebo,
            'goal_pose_roll_gazebo': goal_pose_roll_gazebo,
            'goal_pose_pitch_gazebo': goal_pose_pitch_gazebo,
            'goal_pose_yaw_gazebo': goal_pose_yaw_gazebo,
            # Transformación Gazebo -> Mapa
            'gazebo_to_map_offset_x': 2.0,    # x_mapa = x_gazebo + 2.0
            'gazebo_to_map_offset_y': 0.5,    # y_mapa = y_gazebo + 0.5
        }],
    )

    return LaunchDescription([
        # Argumentos para el objetivo EN COORDENADAS GAZEBO
        DeclareLaunchArgument('goal_pose_x_gazebo', default_value='1.87'),
        DeclareLaunchArgument('goal_pose_y_gazebo', default_value='0.81'),
        DeclareLaunchArgument('goal_pose_z_gazebo', default_value='0.00782'),
        DeclareLaunchArgument('goal_pose_roll_gazebo', default_value='0.0'),
        DeclareLaunchArgument('goal_pose_pitch_gazebo', default_value='0.006'),
        DeclareLaunchArgument('goal_pose_yaw_gazebo', default_value='0.000122'),

        # Nodo
        rviz_predefinido_node,
    ])