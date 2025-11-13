import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('g09_prii3')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Ruta al mundo instalado bajo share/<pkg>/worlds
    world = os.path.join(pkg_share, 'worlds', 'f1l3.world')

    # Asegurar que Gazebo encuentra los modelos (f1l3 y turtlebot3)
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    models_dir = os.path.join(pkg_share, 'models')
    tb3_models_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    composed_model_path = ':'.join([p for p in [current_model_path, models_dir, tb3_models_dir] if p])
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', composed_model_path)
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_model_path)
    ld.add_action(set_tb3_model)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # No spawn_turtlebot3 here: the robot is already included in the world SDF.

    return ld