import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

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


class RvizPredefinidoNode(Node):
    """Nodo que trabaja con coordenadas GAZEBO y asume posición inicial conocida."""

    def __init__(self) -> None:
        super().__init__('rviz_predefinido_node')

        # Parámetros de la posición final (destino) en COORDENADAS GAZEBO
        self.declare_parameters('', [
            ('frame_id', 'map'),
            # Pose meta 6DOF - EN COORDENADAS GAZEBO
            ('goal_pose_x_gazebo', 0.1763),   # Coordenada X en Gazebo
            ('goal_pose_y_gazebo', 4.1733),   # Coordenada Y en Gazebo
            ('goal_pose_z_gazebo', 0.0088),
            ('goal_pose_roll_gazebo', 0.0014),
            ('goal_pose_pitch_gazebo', -0.0102),
            ('goal_pose_yaw_gazebo', 1.5692),
            # Transformación Gazebo -> Mapa
            ('gazebo_to_map_offset_x', 4.527328),    # x_mapa = x_gazebo + 4.527328
            ('gazebo_to_map_offset_y', 2.852645),    # y_mapa = y_gazebo + 2.852645
            # Tiempos de espera
            ('initial_pose_delay', 1.0),        # Espera después de publicar pose inicial
            ('nav_start_delay', 3.0),           # Espera antes de empezar navegación
        ])

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # 1. PUBLICADOR para establecer la pose inicial
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 2. CLIENTE de acción para navegación
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Variables de control
        self.goal_sent = False

        # Mostrar información de transformación
        goal_x_gazebo = self.get_parameter('goal_pose_x_gazebo').value
        goal_y_gazebo = self.get_parameter('goal_pose_y_gazebo').value
        self.gazebo_to_map_offset_x = self.get_parameter('gazebo_to_map_offset_x').value
        self.gazebo_to_map_offset_y = self.get_parameter('gazebo_to_map_offset_y').value
        goal_x_mapa = goal_x_gazebo + self.gazebo_to_map_offset_x
        goal_y_mapa = goal_y_gazebo + self.gazebo_to_map_offset_y
        
        self.get_logger().info('✅ Nodo rviz_predefinido_node iniciado')
        self.get_logger().info('🎯 OBJETIVO EN COORDENADAS GAZEBO:')
        self.get_logger().info(f'   📍 Gazebo: x={goal_x_gazebo:.2f}, y={goal_y_gazebo:.2f}')
        self.get_logger().info(f'   🗺️  Mapa:   x={goal_x_mapa:.2f}, y={goal_y_mapa:.2f}')
        
        # Iniciar secuencia inmediatamente - SIN ESPERAR DETECCIÓN
        self.get_logger().info('📍 Estableciendo posición inicial en (0,0) del mapa...')
        self.publish_initial_pose()

    def publish_initial_pose(self) -> None:
        """Publica la pose inicial conocida del mapa inmediatamente."""
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        
        initial_pose_msg.pose.pose.position.x = 0.0  # Mapa
        initial_pose_msg.pose.pose.position.y = 0.0  # Mapa
        initial_pose_msg.pose.pose.position.z = 0.0
        
        # Orientación derivada de roll/pitch/yaw proporcionados
        qx, qy, qz, qw = quaternion_from_euler(0.001529, -0.008578, 0.008052)
        initial_pose_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        # Covarianza (igual que antes)
        initial_pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        self.initial_pose_pub.publish(initial_pose_msg)
        
        initial_delay = self.get_parameter('initial_pose_delay').value
        nav_delay = self.get_parameter('nav_start_delay').value
        
        gazebo_x_for_map_origin = -self.gazebo_to_map_offset_x
        gazebo_y_for_map_origin = -self.gazebo_to_map_offset_y
        self.get_logger().info('✅ Pose inicial publicada en (0,0) del mapa')
        self.get_logger().info(
            f'📍 Equivale a ({gazebo_x_for_map_origin:.6f}, {gazebo_y_for_map_origin:.6f}) en Gazebo'
        )
        self.get_logger().info(f'⏳ Iniciando navegación en {nav_delay} segundos...')
        
        # Programar el envío del goal después del delay
        self.create_timer(nav_delay, self.send_navigation_goal)

    def send_navigation_goal(self) -> None:
        """Envía el goal de navegación transformando coordenadas Gazebo → Mapa."""
        if self.goal_sent:
            return
            
        try:
            # Obtener coordenadas de Gazebo desde parámetros
            goal_x_gazebo = self.get_parameter('goal_pose_x_gazebo').value
            goal_y_gazebo = self.get_parameter('goal_pose_y_gazebo').value
            
            # TRANSFORMAR: Gazebo → Mapa
            offset_x = self.gazebo_to_map_offset_x
            offset_y = self.gazebo_to_map_offset_y
            goal_x_mapa = goal_x_gazebo + offset_x
            goal_y_mapa = goal_y_gazebo + offset_y

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._build_pose_stamped(
                goal_x_mapa,  # Usar coordenadas transformadas a MAPA
                goal_y_mapa,
                self.get_parameter('goal_pose_z_gazebo').value,
                self.get_parameter('goal_pose_roll_gazebo').value,
                self.get_parameter('goal_pose_pitch_gazebo').value,
                self.get_parameter('goal_pose_yaw_gazebo').value,
            )

            self.get_logger().info(
                f"🚀 ENVIANDO GOAL DE NAVEGACIÓN:"
            )
            self.get_logger().info(
                f"   📍 Coordenadas GAZEBO: x={goal_x_gazebo:.2f}, y={goal_y_gazebo:.2f}"
            )
            self.get_logger().info(
                f"   🗺️  Coordenadas MAPA:   x={goal_x_mapa:.2f}, y={goal_y_mapa:.2f}"
            )

            # Enviar goal a Nav2
            send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback,
            )
            send_goal_future.add_done_callback(self._goal_response_callback)
            
            self.goal_sent = True

        except Exception as e:
            self.get_logger().error(f'❌ Error enviando goal: {e}')

    def _feedback_callback(self, feedback_msg) -> None:
        """Muestra el progreso de la navegación."""
        try:
            feedback = feedback_msg.feedback
            if hasattr(feedback, 'current_pose'):
                current_pose = feedback.current_pose
                
                # Mostrar posición en ambos sistemas
                pos_x_mapa = current_pose.pose.position.x
                pos_y_mapa = current_pose.pose.position.y
                pos_x_gazebo = pos_x_mapa - self.gazebo_to_map_offset_x
                pos_y_gazebo = pos_y_mapa - self.gazebo_to_map_offset_y
                
                # Calcular distancia al objetivo
                goal_x_gazebo = self.get_parameter('goal_pose_x_gazebo').value
                goal_y_gazebo = self.get_parameter('goal_pose_y_gazebo').value
                distance = math.sqrt(
                    (goal_x_gazebo - pos_x_gazebo) ** 2 + 
                    (goal_y_gazebo - pos_y_gazebo) ** 2
                )
                
                self.get_logger().info(
                    f'📊 Progreso GAZEBO: x={pos_x_gazebo:.2f}, y={pos_y_gazebo:.2f} | '
                    f'Distancia: {distance:.2f}m',
                    throttle_duration_sec=3.0
                )
        except Exception:
            pass

    def _goal_response_callback(self, future) -> None:
        """Maneja la respuesta del servidor de navegación."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rechazado por Nav2')
            return
            
        self.get_logger().info('✅ Goal aceptado. Robot en movimiento...')
        
        # Esperar resultado final
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        """Maneja el resultado final de la navegación."""
        try:
            result = future.result()
            status = result.status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('🎉 ¡Navegación completada con éxito!')
            else:
                self.get_logger().warn(f'⚠️ Navegación terminada con estado: {status}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en navegación: {e}')

    def _build_pose_stamped(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> PoseStamped:
        """Construye un mensaje PoseStamped para el goal de navegación."""
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id

        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        qx, qy, qz, qw = quaternion_from_euler(float(roll), float(pitch), float(yaw))
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        return ps


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizPredefinidoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()