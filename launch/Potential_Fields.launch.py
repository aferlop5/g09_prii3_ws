from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
	# Ganancia atractiva hacia la meta (más alto = más "tirón" hacia el objetivo)
	# Sube si el robot tarda en orientar/dirigirse a la meta; baja si ignora obstáculos al insistir demasiado en la meta
	k_att = DeclareLaunchArgument('k_att', default_value='1.0')

	# Ganancia repulsiva de obstáculos (más alto = se aleja más fuerte de obstáculos)
	# Sube si roza objetos; baja si se vuelve demasiado conservador y avanza poco
	k_rep = DeclareLaunchArgument('k_rep', default_value='0.32')

	# Distancia de influencia del campo repulsivo (m). Por debajo de esta distancia comienzan las fuerzas de repulsión
	# Sube si quieres empezar a evitar antes (más margen); baja si quieres arrimarte más
	d0_rep = DeclareLaunchArgument('d0_rep', default_value='0.55')

	# Saturación de velocidad lineal (m/s). Límite superior del comando de avance
	max_lin_vel = DeclareLaunchArgument('max_lin_vel', default_value='0.3')

	# Saturación de velocidad angular (rad/s). Límite superior del giro
	max_ang_vel = DeclareLaunchArgument('max_ang_vel', default_value='1.0')

	# Magnitud de una pequeña perturbación de escape cuando el campo se queda casi nulo (mínimos locales)
	# Sube si a veces se "atasca" oscilando; baja si introduce giros innecesarios
	escape_gain = DeclareLaunchArgument('escape_gain', default_value='0.2')

	# Objetivo (x, y) en el marco del mundo/odometría
	goal_x = DeclareLaunchArgument('goal_x', default_value='0.0')
	goal_y = DeclareLaunchArgument('goal_y', default_value='0.0')

	# Tolerancia de llegada a la meta (m). Por debajo de esta distancia se considera alcanzada y se para
	goal_tolerance = DeclareLaunchArgument('goal_tolerance', default_value='0.1')

	# Tópico de odometría usado para calcular posición y orientación (para proyectar el vector a marco del robot)
	odom_topic = DeclareLaunchArgument('odom_topic', default_value='/odom')

	# Ganancia que convierte el ángulo del vector resultante en velocidad angular (más alto = gira más agresivo)
	ang_gain = DeclareLaunchArgument('ang_gain', default_value='1.5')

	# Ganancia que convierte la magnitud del vector en velocidad lineal (más alto = acelera más ante el mismo campo)
	lin_gain = DeclareLaunchArgument('lin_gain', default_value='1.0')

	# Factor mínimo de reducción de la velocidad cerca de obstáculos [0..1]
	# La velocidad lineal se escala con la distancia al obstáculo; este valor evita que caiga por debajo de un mínimo
	slowdown_min_scale = DeclareLaunchArgument('slowdown_min_scale', default_value='0.2')

	# Anchura (grados) del sector frontal con mayor prioridad repulsiva
	# Sube para que "mire" más al frente y evite con más decisión lo que hay delante; baja para equilibrar laterales
	front_weight_deg = DeclareLaunchArgument('front_weight_deg', default_value='80.0')

	# Peso relativo de la repulsión en los laterales fuera del sector frontal [0..1]
	# Sube para dar más importancia a obstáculos laterales (más holgura al girar); baja si se vuelve tímido al avanzar
	rep_scale_side = DeclareLaunchArgument('rep_scale_side', default_value='0.42')

	# Suavizado de comandos (filtro paso bajo). 0 = muy suave/lento, 1 = sin suavizado (muy reactivo)
	# Sube para que responda más rápido; baja para movimientos más suaves y estables
	smooth_alpha = DeclareLaunchArgument('smooth_alpha', default_value='0.4')

	# Tiempo (s) sin progreso hacia la meta para activar recuperación (giro en el sitio)
	stuck_timeout = DeclareLaunchArgument('stuck_timeout', default_value='3.0')

	# --- Fallback de atascos: seguir aperturas (Follow-The-Gap) ---
	# Activa el modo de elegir la mayor apertura cuando no hay progreso
	use_gap_follow = DeclareLaunchArgument('use_gap_follow', default_value='true')
	# Distancia mínima considerada "libre" para formar una apertura (si dudas, usa d0_rep)
	gap_clear_threshold = DeclareLaunchArgument('gap_clear_threshold', default_value=LaunchConfiguration('d0_rep'))
	# Anchura mínima (en grados) de una apertura válida (puerta/paso)
	gap_min_width_deg = DeclareLaunchArgument('gap_min_width_deg', default_value='12.0')
	# Peso de preferencia hacia la dirección de la meta frente a la apertura (0..1)
	gap_prefer_goal_weight = DeclareLaunchArgument('gap_prefer_goal_weight', default_value='0.6')
	# Modo de recuperación cuando está atascado: 'gap' | 'spin' | 'spin+gap'
	recovery_mode = DeclareLaunchArgument('recovery_mode', default_value='spin+gap')
	# Duración (s) de seguimiento de apertura antes de re-evaluar
	recovery_gap_duration = DeclareLaunchArgument('recovery_gap_duration', default_value='3.0')

	# --- Fallback adicional: seguimiento de paredes (Wall-Follow) ---
	use_wall_follow = DeclareLaunchArgument('use_wall_follow', default_value='true')
	wall_side = DeclareLaunchArgument('wall_side', default_value='auto')  # auto|left|right
	wall_distance = DeclareLaunchArgument('wall_distance', default_value='0.38')
	wall_kp = DeclareLaunchArgument('wall_kp', default_value='1.2')
	wall_lin_vel = DeclareLaunchArgument('wall_lin_vel', default_value='0.28')
	wall_timeout = DeclareLaunchArgument('wall_timeout', default_value='6.0')
	wall_front_switch_thresh = DeclareLaunchArgument('wall_front_switch_thresh', default_value='0.45')
	wall_switch_margin = DeclareLaunchArgument('wall_switch_margin', default_value='0.07')
	wall_switch_cooldown = DeclareLaunchArgument('wall_switch_cooldown', default_value='1.5')

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
				'use_gap_follow': ParameterValue(LaunchConfiguration('use_gap_follow'), value_type=bool),
				'gap_clear_threshold': ParameterValue(LaunchConfiguration('gap_clear_threshold'), value_type=float),
				'gap_min_width_deg': ParameterValue(LaunchConfiguration('gap_min_width_deg'), value_type=float),
				'gap_prefer_goal_weight': ParameterValue(LaunchConfiguration('gap_prefer_goal_weight'), value_type=float),
				'recovery_mode': LaunchConfiguration('recovery_mode'),
				'recovery_gap_duration': ParameterValue(LaunchConfiguration('recovery_gap_duration'), value_type=float),
				'use_wall_follow': ParameterValue(LaunchConfiguration('use_wall_follow'), value_type=bool),
				'wall_side': LaunchConfiguration('wall_side'),
				'wall_distance': ParameterValue(LaunchConfiguration('wall_distance'), value_type=float),
				'wall_kp': ParameterValue(LaunchConfiguration('wall_kp'), value_type=float),
				'wall_lin_vel': ParameterValue(LaunchConfiguration('wall_lin_vel'), value_type=float),
				'wall_timeout': ParameterValue(LaunchConfiguration('wall_timeout'), value_type=float),
				'wall_front_switch_thresh': ParameterValue(LaunchConfiguration('wall_front_switch_thresh'), value_type=float),
				'wall_switch_margin': ParameterValue(LaunchConfiguration('wall_switch_margin'), value_type=float),
				'wall_switch_cooldown': ParameterValue(LaunchConfiguration('wall_switch_cooldown'), value_type=float),
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
		use_gap_follow,
		gap_clear_threshold,
		gap_min_width_deg,
		gap_prefer_goal_weight,
		recovery_mode,
		recovery_gap_duration,
		use_wall_follow,
		wall_side,
		wall_distance,
		wall_kp,
		wall_lin_vel,
		wall_timeout,
		wall_front_switch_thresh,
		wall_switch_margin,
		wall_switch_cooldown,
		pf_node,
	])

