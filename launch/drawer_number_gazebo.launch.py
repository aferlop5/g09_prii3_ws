from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Ensure TurtleBot3 model is set for Gazebo bringup
    set_tb3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # Bring up Gazebo empty world with TurtleBot3
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        )
    )

    # Our node that publishes the drawing motion on /cmd_vel
    drawer_node = Node(
        package='g09_prii3',
        executable='drawer_number_gazebo',
        name='turtlebot_number_node',
        output='screen'
    )

    # Delay the drawer a bit to allow Gazebo/robot to be ready
    delayed_drawer = TimerAction(period=5.0, actions=[drawer_node])

    return LaunchDescription([
        set_tb3_model,
        gazebo_world,
        delayed_drawer,
    ])
