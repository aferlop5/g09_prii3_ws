import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	# Base packages
	pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
	pkg_share = get_package_share_directory('g09_prii3')
	tb3_gazebo_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

	# Common configs
	use_sim_time = LaunchConfiguration('use_sim_time', default='true')
	x_pose = LaunchConfiguration('x_pose', default='0.0')
	y_pose = LaunchConfiguration('y_pose', default='0.0')
	z_pose = LaunchConfiguration('z_pose', default='0.0')

	# World path installed under share/<pkg>/worlds
	world = os.path.join(pkg_share, 'worlds', 'eurobot.world')

	# Ensure Gazebo finds our models and TurtleBot3 models
	current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
	models_dir = os.path.join(pkg_share, 'models')
	tb3_models_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
	composed_model_path = ':'.join([p for p in [current_model_path, models_dir, tb3_models_dir] if p])
	set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', composed_model_path)

	# Use Waffle for EUROBOT
	set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

	# Gazebo server/client with the EUROBOT world
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

	# Robot state publisher para TB3 (usa turtlebot3_description URDF)
	# Algunas distros no tienen el launch predefinido, por lo que usamos el binario directamente
	from launch_ros.actions import Node
	robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		arguments=[os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')],
		parameters=[{'use_sim_time': use_sim_time}],
		output='screen'
	)

	# Spawner explícito usando gazebo_ros (compatible en Foxy/Humble)
	spawn_tb3_cmd = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-entity', 'turtlebot3_waffle',
				   '-database', 'turtlebot3_waffle',
				   '-x', x_pose,
				   '-y', y_pose,
				   '-z', z_pose],
		output='screen'
	)

	ld = LaunchDescription()

	ld.add_action(set_model_path)
	ld.add_action(set_tb3_model)
	ld.add_action(gzserver_cmd)
	ld.add_action(gzclient_cmd)
	ld.add_action(robot_state_publisher_cmd)
	# Añadimos spawn explícito como respaldo
	ld.add_action(spawn_tb3_cmd)

	return ld
