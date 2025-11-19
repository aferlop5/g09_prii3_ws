import math
import sys
from typing import IO, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose

try:
    from tf_transformations import quaternion_from_euler
except ImportError:

    def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

_TTY_IN: Optional[IO[str]] = None
_TTY_OUT: Optional[IO[str]] = None
_CONSOLE_SOURCE: str = 'desconocido'


def _get_console_streams() -> Tuple[IO[str], IO[str]]:
    """Devuelve streams de entrada/salida preferiblemente conectados al TTY."""

    global _TTY_IN, _TTY_OUT, _CONSOLE_SOURCE

    if _TTY_IN is not None and _TTY_OUT is not None:
        return (_TTY_IN, _TTY_OUT)

    try:
        _TTY_IN = open('/dev/tty', 'r', encoding='utf-8', errors='ignore')
        _TTY_OUT = open('/dev/tty', 'w', encoding='utf-8', errors='ignore')
        _CONSOLE_SOURCE = '/dev/tty'
    except OSError:
        _TTY_IN = sys.stdin
        _TTY_OUT = sys.stdout
        _CONSOLE_SOURCE = 'stdin/stdout (sin TTY específico)'

    return (_TTY_IN, _TTY_OUT)


def _close_console_streams() -> None:
    global _TTY_IN, _TTY_OUT

    for stream in (_TTY_IN, _TTY_OUT):
        if stream is not None and stream not in (sys.stdin, sys.stdout):
            try:
                stream.close()
            except Exception:  # noqa: BLE001
                pass

    _TTY_IN = None
    _TTY_OUT = None


def _console_write(message: str, newline: bool = True) -> None:
    _, tty_out = _get_console_streams()

    try:
        tty_out.write(message)
        if newline:
            tty_out.write('\n')
        tty_out.flush()
    except Exception:  # noqa: BLE001
        # Fallback a print estándar si algo falla
        print(message, flush=True)


def _read_float(prompt: str, allow_empty: bool = False, default: float = 0.0) -> float:
    while True:
        try:
            tty_in, tty_out = _get_console_streams()
            tty_out.write(prompt)
            tty_out.flush()
            raw_value = tty_in.readline()
        except KeyboardInterrupt:  # permitir Ctrl+C limpio
            raise
        except Exception as err:  # noqa: BLE001
            raise RuntimeError('No se pudieron leer valores desde la terminal.') from err

        if raw_value == '':
            raise RuntimeError('Entrada finalizada antes de completar las coordenadas.')

        raw_value = raw_value.strip()
        if not raw_value:
            if allow_empty:
                return default
            print('Introduce un número válido.', flush=True)
            continue

        normalized_value = raw_value.replace(',', '.')
        try:
            return float(normalized_value)
        except ValueError:
            print('Formato inválido. Usa números decimales, por ejemplo: -3.5', flush=True)


def prompt_for_gazebo_goal() -> Tuple[float, float, float]:
    """Solicita al usuario las coordenadas objetivo en el marco de Gazebo."""

    _console_write('Introduce las coordenadas objetivo en Gazebo (en metros).')
    _console_write(f'[Fuente de entrada detectada: {_CONSOLE_SOURCE}]')
    print(f'ℹ️  Leyendo coordenadas desde {_CONSOLE_SOURCE}', flush=True)

    x_value = _read_float('  Coordenada X (m): ', allow_empty=False)
    _console_write(f'✅ Coordenada X recibida: {x_value:.3f}')
    print(f'✅ Coordenada X recibida: {x_value:.3f}', flush=True)

    y_value = _read_float('  Coordenada Y (m): ', allow_empty=False)
    _console_write(f'✅ Coordenada Y recibida: {y_value:.3f}')
    print(f'✅ Coordenada Y recibida: {y_value:.3f}', flush=True)

    yaw_degrees = _read_float('  Yaw (grados, Enter=0): ', allow_empty=True, default=0.0)
    _console_write(f'✅ Yaw recibido: {yaw_degrees:.1f}°')
    print(f'✅ Yaw recibido: {yaw_degrees:.1f}°', flush=True)

    yaw_rad = math.radians(yaw_degrees)
    _console_write(f'➡️  Objetivo Gazebo: x={x_value:.3f}, y={y_value:.3f}, yaw={yaw_degrees:.1f}°')
    print(f'➡️  Objetivo Gazebo: x={x_value:.3f}, y={y_value:.3f}, yaw={yaw_degrees:.1f}°', flush=True)
    return (x_value, y_value, yaw_rad)


class NavF1l3CordenadasNode(Node):
    """Nodo que transforma coordenadas de Gazebo a mapa y lanza un objetivo Nav2."""

    def __init__(self, gazebo_goal: Tuple[float, float, float]) -> None:
        super().__init__('nav_f1l3_cordenadas')

        goal_x_gazebo, goal_y_gazebo, goal_yaw_gazebo = gazebo_goal

        self.declare_parameters(
            '',
            [
                ('frame_id', 'map'),
                ('goal_pose_x_gazebo', float(goal_x_gazebo)),
                ('goal_pose_y_gazebo', float(goal_y_gazebo)),
                ('goal_pose_z_gazebo', 0.0),
                ('goal_pose_roll_gazebo', 0.0),
                ('goal_pose_pitch_gazebo', 0.0),
                ('goal_pose_yaw_gazebo', float(goal_yaw_gazebo)),
                # 0,0 del mapa (RViz) coincide con (-4.527328, -2.852645) en Gazebo
                ('gazebo_to_map_offset_x', 4.527328),
                ('gazebo_to_map_offset_y', 2.852645),
                ('nav_start_delay', 0.0),
            ],
        )

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_sent = False
        self._nav_timer = None

        self._log_goal_overview()
        self.publish_initial_pose()

    def _log_goal_overview(self) -> None:
        goal_x_gazebo = self.get_parameter('goal_pose_x_gazebo').value
        goal_y_gazebo = self.get_parameter('goal_pose_y_gazebo').value
        goal_yaw_gazebo = self.get_parameter('goal_pose_yaw_gazebo').value
        offset_x = self.get_parameter('gazebo_to_map_offset_x').value
        offset_y = self.get_parameter('gazebo_to_map_offset_y').value

        goal_x_map = goal_x_gazebo + offset_x
        goal_y_map = goal_y_gazebo + offset_y

        self.get_logger().info('✅ Nodo nav_f1l3_cordenadas iniciado')
        self.get_logger().info('🎯 Objetivo definido en sistema Gazebo:')
        self.get_logger().info(f'   📍 Gazebo: x={goal_x_gazebo:.3f}, y={goal_y_gazebo:.3f}, yaw={math.degrees(goal_yaw_gazebo):.1f}°')
        self.get_logger().info(f'   🗺️  Mapa:   x={goal_x_map:.3f}, y={goal_y_map:.3f}, yaw={math.degrees(goal_yaw_gazebo):.1f}°')

    def publish_initial_pose(self) -> None:
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'

        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
        initial_pose_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        initial_pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
        ]

        self.initial_pose_pub.publish(initial_pose_msg)

        nav_delay = self.get_parameter('nav_start_delay').value

        self.get_logger().info('✅ Pose inicial publicada en (0.0, 0.0) del mapa')
        self.get_logger().info('📍 Equivalente a (-4.527328, -2.852645) en Gazebo')

        if nav_delay > 0.0:
            self.get_logger().info(f'⏳ Enviando objetivo en {nav_delay:.1f} segundos...')
            self._nav_timer = self.create_timer(nav_delay, self._nav_timer_callback)
        else:
            self.get_logger().info('🚀 Enviando objetivo inmediatamente...')
            self.send_navigation_goal()

    def _nav_timer_callback(self) -> None:
        if self._nav_timer is not None:
            self._nav_timer.cancel()
            self._nav_timer = None
        self.send_navigation_goal()

    def send_navigation_goal(self) -> None:
        if self.goal_sent:
            return

        try:
            self.get_logger().info('🔌 Verificando disponibilidad de navigate_to_pose...')
            if not self.nav_to_pose_client.server_is_ready():
                self.get_logger().error('❌ navigate_to_pose no disponible. Lanza navigation2 antes de este nodo.')
                return
            self.get_logger().info('✅ Action server disponible.')

            goal_x_gazebo = self.get_parameter('goal_pose_x_gazebo').value
            goal_y_gazebo = self.get_parameter('goal_pose_y_gazebo').value
            goal_yaw_gazebo = self.get_parameter('goal_pose_yaw_gazebo').value

            offset_x = self.get_parameter('gazebo_to_map_offset_x').value
            offset_y = self.get_parameter('gazebo_to_map_offset_y').value

            goal_x_map = goal_x_gazebo + offset_x
            goal_y_map = goal_y_gazebo + offset_y

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._build_pose_stamped(
                goal_x_map,
                goal_y_map,
                self.get_parameter('goal_pose_z_gazebo').value,
                self.get_parameter('goal_pose_roll_gazebo').value,
                self.get_parameter('goal_pose_pitch_gazebo').value,
                goal_yaw_gazebo,
            )

            self.get_logger().info('🚀 Enviando objetivo de navegación:')
            self.get_logger().info(f'   📍 Gazebo: x={goal_x_gazebo:.3f}, y={goal_y_gazebo:.3f}, yaw={math.degrees(goal_yaw_gazebo):.1f}°')
            self.get_logger().info(f'   🗺️  Mapa:   x={goal_x_map:.3f}, y={goal_y_map:.3f}, yaw={math.degrees(goal_yaw_gazebo):.1f}°')

            send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback,
            )
            send_goal_future.add_done_callback(self._goal_response_callback)

            self.goal_sent = True
            self.get_logger().info('✅ Goal enviado correctamente a Nav2')

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'❌ Error al enviar el objetivo: {exc}')

    def _feedback_callback(self, feedback_msg) -> None:
        try:
            feedback = feedback_msg.feedback
            if hasattr(feedback, 'current_pose'):
                current_pose = feedback.current_pose

                pos_x_map = current_pose.pose.position.x
                pos_y_map = current_pose.pose.position.y

                offset_x = self.get_parameter('gazebo_to_map_offset_x').value
                offset_y = self.get_parameter('gazebo_to_map_offset_y').value

                pos_x_gazebo = pos_x_map - offset_x
                pos_y_gazebo = pos_y_map - offset_y

                goal_x_gazebo = self.get_parameter('goal_pose_x_gazebo').value
                goal_y_gazebo = self.get_parameter('goal_pose_y_gazebo').value

                distance = math.hypot(goal_x_gazebo - pos_x_gazebo, goal_y_gazebo - pos_y_gazebo)

                self.get_logger().info(
                    f'📊 Progreso Gazebo: x={pos_x_gazebo:.3f}, y={pos_y_gazebo:.3f} | Distancia={distance:.2f}m',
                    throttle_duration_sec=3.0,
                )
        except Exception:  # noqa: BLE001
            pass

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rechazado por Nav2')
            self.goal_sent = False
            return

        self.get_logger().info('✅ Goal aceptado. Robot en movimiento...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('🎉 Navegación completada con éxito')
            else:
                self.get_logger().warn(f'⚠️ Navegación terminada con estado: {status}')

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'❌ Error al recuperar el resultado: {exc}')

    def _build_pose_stamped(
        self,
        x_value: float,
        y_value: float,
        z_value: float,
        roll_value: float,
        pitch_value: float,
        yaw_value: float,
    ) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id

        pose_stamped.pose.position.x = float(x_value)
        pose_stamped.pose.position.y = float(y_value)
        pose_stamped.pose.position.z = float(z_value)

        qx, qy, qz, qw = quaternion_from_euler(float(roll_value), float(pitch_value), float(yaw_value))
        pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        return pose_stamped


def main(args=None) -> None:
    rclpy.init(args=args)

    # Inicializar consola antes de pedir datos y mostrar información útil
    _get_console_streams()
    print(f'ℹ️  Fuente de entrada detectada: {_CONSOLE_SOURCE}', flush=True)

    gazebo_goal = prompt_for_gazebo_goal()
    node = NavF1l3CordenadasNode(gazebo_goal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        _close_console_streams()

