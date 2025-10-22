from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
	# Launch parameters with sane defaults matching node's internal defaults
	k_att = DeclareLaunchArgument('k_att', default_value='1.0')
	k_rep = DeclareLaunchArgument('k_rep', default_value='0.32')
	d0_rep = DeclareLaunchArgument('d0_rep', default_value='0.55')
	max_lin_vel = DeclareLaunchArgument('max_lin_vel', default_value='0.3')
	max_ang_vel = DeclareLaunchArgument('max_ang_vel', default_value='1.0')
	escape_gain = DeclareLaunchArgument('escape_gain', default_value='0.2')
	goal_x = DeclareLaunchArgument('goal_x', default_value='0.0')
	goal_y = DeclareLaunchArgument('goal_y', default_value='0.0')
	goal_tolerance = DeclareLaunchArgument('goal_tolerance', default_value='0.1')
	odom_topic = DeclareLaunchArgument('odom_topic', default_value='/odom')
	ang_gain = DeclareLaunchArgument('ang_gain', default_value='1.5')
	lin_gain = DeclareLaunchArgument('lin_gain', default_value='1.0')
	slowdown_min_scale = DeclareLaunchArgument('slowdown_min_scale', default_value='0.18')
	# Aumentar prioridad frontal y dar más peso a laterales para mayor holgura al girar
	front_weight_deg = DeclareLaunchArgument('front_weight_deg', default_value='80.0')
	rep_scale_side = DeclareLaunchArgument('rep_scale_side', default_value='0.42')
	smooth_alpha = DeclareLaunchArgument('smooth_alpha', default_value='0.3')
	stuck_timeout = DeclareLaunchArgument('stuck_timeout', default_value='3.0')

	pf_node = Node(
		package='g09_prii3',
		executable='jetbot_potential_fields',
		name='potential_fields_nav',
		output='screen',
		parameters=[{
			'k_att': ParameterValue(LaunchConfiguration('k_att'), value_type=float),
			'k_rep': ParameterValue(LaunchConfiguration('k_rep'), value_type=float),
			'd0_rep': ParameterValue(LaunchConfiguration('d0_rep'), value_type=float),
			'max_lin_vel': ParameterValue(LaunchConfiguration('max_lin_vel'), value_type=float),
			'max_ang_vel': ParameterValue(LaunchConfiguration('max_ang_vel'), value_type=float),
			'escape_gain': ParameterValue(LaunchConfiguration('escape_gain'), value_type=float),
			'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
			'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
			'goal_tolerance': ParameterValue(LaunchConfiguration('goal_tolerance'), value_type=float),
			'odom_topic': LaunchConfiguration('odom_topic'),
			'ang_gain': ParameterValue(LaunchConfiguration('ang_gain'), value_type=float),
			'lin_gain': ParameterValue(LaunchConfiguration('lin_gain'), value_type=float),
			'slowdown_min_scale': ParameterValue(LaunchConfiguration('slowdown_min_scale'), value_type=float),
			'front_weight_deg': ParameterValue(LaunchConfiguration('front_weight_deg'), value_type=float),
			'rep_scale_side': ParameterValue(LaunchConfiguration('rep_scale_side'), value_type=float),
			'smooth_alpha': ParameterValue(LaunchConfiguration('smooth_alpha'), value_type=float),
			'stuck_timeout': ParameterValue(LaunchConfiguration('stuck_timeout'), value_type=float),
		}],
		# Remappings (uncomment if your topics differ)
		# remappings=[
		#     ('/scan', '/your_scan_topic'),
		#     ('/cmd_vel', '/your_cmd_vel_topic'),
		# ]
	)

	return LaunchDescription([
		k_att,
		k_rep,
		d0_rep,
		max_lin_vel,
		max_ang_vel,
		escape_gain,
		goal_x,
		goal_y,
		goal_tolerance,
		odom_topic,
		ang_gain,
		lin_gain,
		slowdown_min_scale,
		front_weight_deg,
		rep_scale_side,
		smooth_alpha,
		stuck_timeout,
		pf_node,
	])

