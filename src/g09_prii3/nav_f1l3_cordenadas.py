#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
import time
import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

def gazebo_to_map_transform(gazebo_x, gazebo_y):
    """
    Transform coordinates from Gazebo frame to map frame
    """
    offset_x = 4.527328
    offset_y = 2.852645
    
    map_x = gazebo_x + offset_x
    map_y = gazebo_y + offset_y
    
    return map_x, map_y

class AutoNav(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publicador para establecer la pose inicial
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Lista de puntos en coordenadas GAZEBO (x, y, yaw en radianes)
        gazebo_waypoints = [
            # antes puerta
            (4.2549, -2.7820, 1.5692),
            # delante código puerta
            (3.4197, -0.5843, 1.5692),
            # puerta
            (4.2887, 0.0093, 1.5692),
            # pasillo1
            (3.0256, 3.9167, 1.5692),
            # pasillo2
            (3.0446, 8.8183, 1.5692),
            # pasillo3
            (3.0506, 12.7281, 1.5692),
            # pasillo3izq
            (-2.3742, 12.7366, -3.131)
        ]

        # Transformar waypoints de Gazebo a coordenadas del mapa
        self.waypoints = []
        for wp in gazebo_waypoints:
            map_x, map_y = gazebo_to_map_transform(wp[0], wp[1])
            self.waypoints.append((map_x, map_y, wp[2]))
            self.get_logger().info(f'Waypoint transformado: Gazebo({wp[0]:.2f}, {wp[1]:.2f}) -> Map({map_x:.2f}, {map_y:.2f})')

        # Iniciar secuencia después de una espera inicial para que todo esté listo
        self.get_logger().info('Esperando 5 segundos para que Navigation2 se inicialice...')
        
        # Crear timer para iniciar después de la inicialización
        self.start_timer = self.create_timer(5.0, self.start_navigation)

    def start_navigation(self):
        """Inicia la navegación después de que todo esté inicializado"""
        self.destroy_timer(self.start_timer)
        
        self.get_logger().info('Intentando establecer pose inicial automáticamente...')
        self.publish_initial_pose()
        
        # Esperar a que AMCL se inicialice
        time.sleep(3.0)
        
        # Esperar al servidor de Navigation2
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Esperando al servidor de Navigation2...')

        self.get_logger().info('Servidor de Navigation2 activo')
        
        # Intentar enviar el primer goal
        self.get_logger().info('Iniciando secuencia de navegación...')
        self.send_next_goal(0)

    def publish_initial_pose(self):
        """Publica la pose inicial en una posición razonable del mapa"""
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        
        # Publicar en una posición que probablemente exista en el mapa
        # Usamos una posición cerca del inicio de la ruta
        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0
        
        # Orientación hacia donde empieza la ruta
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)  # Mirando hacia Y positiva
        initial_pose_msg.pose.pose.orientation.x = qx
        initial_pose_msg.pose.pose.orientation.y = qy
        initial_pose_msg.pose.pose.orientation.z = qz
        initial_pose_msg.pose.pose.orientation.w = qw
        
        # Covarianza
        initial_pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        # Publicar múltiples veces para asegurar
        for i in range(3):
            self.initial_pose_pub.publish(initial_pose_msg)
            self.get_logger().info(f'Pose inicial publicada {i+1}/3 en (7.0, 0.0)')
            time.sleep(0.5)

    def send_next_goal(self, idx):
        if idx >= len(self.waypoints):
            self.get_logger().info('Todos los puntos alcanzados. Terminando.')
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[idx]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'Enviando goal {idx+1}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, idx))

    def goal_response_callback(self, future, idx):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'Goal {idx+1} rechazado - AMCL probablemente no inicializado')
            self.get_logger().info('Esperando 5 segundos y reintentando...')
            # Esperar más tiempo y reintentar
            time.sleep(5.0)
            self.send_next_goal(idx)
            return

        self.get_logger().info(f'Goal {idx+1} aceptado')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.get_result_callback(future, idx))

    def get_result_callback(self, future, idx):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Goal {idx+1} completado con estado: {status}')
        time.sleep(1.0)
        self.send_next_goal(idx + 1)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()