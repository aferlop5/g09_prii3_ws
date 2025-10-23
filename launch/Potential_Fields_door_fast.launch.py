from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # Perfil puerta-friendly rápido: valores recomendados para avanzar ágil
    # manteniendo seguridad al atravesar puertas y pasillos estrechos.

    # Atractivo y repulsivo
    k_att = DeclareLaunchArgument('k_att', default_value='1.2')
    k_rep = DeclareLaunchArgument('k_rep', default_value='0.34')
    d0_rep = DeclareLaunchArgument('d0_rep', default_value='0.60')

    # Límites y ganancias
    max_lin_vel = DeclareLaunchArgument('max_lin_vel', default_value='0.40')
    max_ang_vel = DeclareLaunchArgument('max_ang_vel', default_value='1.40')
    ang_gain = DeclareLaunchArgument('ang_gain', default_value='1.6')
    lin_gain = DeclareLaunchArgument('lin_gain', default_value='1.15')
    escape_gain = DeclareLaunchArgument('escape_gain', default_value='0.25')

    # Meta y odometría
    goal_x = DeclareLaunchArgument('goal_x', default_value='0.0')
    goal_y = DeclareLaunchArgument('goal_y', default_value='0.0')
    goal_tolerance = DeclareLaunchArgument('goal_tolerance', default_value='0.10')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='/odom')

    # Suavizado y repulsión angular
    slowdown_min_scale = DeclareLaunchArgument('slowdown_min_scale', default_value='0.22')
    front_weight_deg = DeclareLaunchArgument('front_weight_deg', default_value='85.0')
    rep_scale_side = DeclareLaunchArgument('rep_scale_side', default_value='0.48')
    smooth_alpha = DeclareLaunchArgument('smooth_alpha', default_value='0.35')
    stuck_timeout = DeclareLaunchArgument('stuck_timeout', default_value='3.0')

    # --- Fallback de atascos: seguir aperturas (Follow-The-Gap) ---
    use_gap_follow = DeclareLaunchArgument('use_gap_follow', default_value='true')
    gap_clear_threshold = DeclareLaunchArgument(
        'gap_clear_threshold', default_value=LaunchConfiguration('d0_rep')
    )
    gap_min_width_deg = DeclareLaunchArgument('gap_min_width_deg', default_value='10.0')
    gap_prefer_goal_weight = DeclareLaunchArgument('gap_prefer_goal_weight', default_value='0.55')
    recovery_mode = DeclareLaunchArgument('recovery_mode', default_value='spin+gap')
    recovery_gap_duration = DeclareLaunchArgument('recovery_gap_duration', default_value='3.0')

    # Wall-follow rápido
    use_wall_follow = DeclareLaunchArgument('use_wall_follow', default_value='true')
    wall_side = DeclareLaunchArgument('wall_side', default_value='auto')
    wall_distance = DeclareLaunchArgument('wall_distance', default_value='0.38')
    wall_kp = DeclareLaunchArgument('wall_kp', default_value='1.2')
    wall_lin_vel = DeclareLaunchArgument('wall_lin_vel', default_value='0.30')
    wall_timeout = DeclareLaunchArgument('wall_timeout', default_value='6.0')
    wall_front_switch_thresh = DeclareLaunchArgument('wall_front_switch_thresh', default_value='0.45')
    wall_switch_margin = DeclareLaunchArgument('wall_switch_margin', default_value='0.07')
    wall_switch_cooldown = DeclareLaunchArgument('wall_switch_cooldown', default_value='1.2')

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
        # Remappings (descomenta si tus tópicos difieren)
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
        ang_gain,
        lin_gain,
        escape_gain,
        goal_x,
        goal_y,
        goal_tolerance,
        odom_topic,
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
