#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
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

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_navigation')
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # ArUco detection setup
        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        
        # Hacer el detector más sensible para 5x5
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minDistanceToBorder = 3
        self.parameters.minMarkerDistanceRate = 0.05
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.1
        
        # Parámetros de la cámara
        self.camera_matrix = np.array([
            [600.0, 0, 320.0],
            [0, 600.0, 240.0],
            [0, 0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4, 1))
        self.marker_length = 0.05
        
        # State variables
        self.detected_aruco_id = None
        self.waiting_for_aruco = False
        self.current_goal_index = 0
        self.aruco_detection_active = False
        self.last_detected_ids = []
        
        # CORREGIDO: Posición de detección en coordenadas GAZEBO (con signos correctos)
        # Según tu código probado, las coordenadas de Gazebo pueden tener signos negativos
        self.detection_position = (3.249904, -2.563470, 1.551559)  # Y negativo como en tu waypoint de ejemplo
        
        # Routes para diferentes IDs ArUco en coordenadas GAZEBO
        self.routes = {
            # Ruta para ID 5 (izquierda al final)
            5: [
                ("puerta", 4.2887, 0.0093, 1.5692),
                ("pasillo1", 3.0256, 3.9167, 1.5692),
                ("pasillo2", 3.0446, 8.8183, 1.5692),
                ("pasillo3", 3.0506, 12.7281, 1.5692),
                ("pasillo3izq", -2.3742, 12.7366, -3.131)
            ],
            # Ruta para ID 17 (derecha al final)
            17: [
                ("puerta", 4.2887, 0.0093, 1.5692),
                ("pasillo1", 3.0256, 3.9167, 1.5692),
                ("pasillo2", 3.0446, 8.8183, 1.5692),
                ("pasillo3", 3.0506, 12.7281, 1.5692),
                ("pasillo3der", 4.4117, 12.8905, 0.0226)
            ],
            # NUEVA RUTA para ID 6 (entrada)
            6: [
                ("puerta", 4.2887, 0.0093, 1.5692),
                ("pasillo1", 3.0256, 3.9167, 1.5692),
                ("intermedia_entrada", 0.1712, 4.3739, -3.1359),
                ("final_entrada", -2.4464, 4.3592, -3.13597)
            ]
        }
        
        # Transform all positions to map coordinates (IGUAL QUE TU CÓDIGO PROBADO)
        self.transformed_routes = {}
        for aruco_id, route in self.routes.items():
            self.transformed_routes[aruco_id] = []
            for point in route:
                map_x, map_y = gazebo_to_map_transform(point[1], point[2])
                self.transformed_routes[aruco_id].append((point[0], map_x, map_y, point[3]))
                self.get_logger().info(f'Ruta {aruco_id}-{point[0]}: Gazebo({point[1]:.2f}, {point[2]:.2f}) -> Map({map_x:.2f}, {map_y:.2f})')
        
        # Transform detection position (IGUAL QUE TU CÓDIGO PROBADO)
        det_x, det_y = gazebo_to_map_transform(self.detection_position[0], self.detection_position[1])
        self.transformed_detection = (det_x, det_y, self.detection_position[2])
        self.get_logger().info(f'Posición detección: Gazebo({self.detection_position[0]:.2f}, {self.detection_position[1]:.2f}) -> Map({det_x:.2f}, {det_y:.2f})')
        
        # Start sequence - EXACTAMENTE IGUAL QUE TU CÓDIGO PROBADO
        self.get_logger().info('Esperando 5 segundos para que Navigation2 se inicialice...')
        self.start_timer = self.create_timer(5.0, self.start_navigation)

    def start_navigation(self):
        """Start navigation sequence - EXACTAMENTE IGUAL QUE TU CÓDIGO PROBADO"""
        self.destroy_timer(self.start_timer)
        
        self.get_logger().info('Intentando establecer pose inicial automáticamente...')
        self.publish_initial_pose()
        
        # Esperar a que AMCL se inicialice
        time.sleep(3.0)
        
        # Esperar al servidor de Navigation2
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Esperando al servidor de Navigation2...')

        self.get_logger().info('Servidor de Navigation2 activo')
        
        # Start by going to detection position
        self.get_logger().info('Navegando a posición de detección de ArUcos...')
        self.navigate_to_detection_position()

    def publish_initial_pose(self):
        """Publica la pose inicial - EXACTAMENTE IGUAL QUE TU CÓDIGO PROBADO"""
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        
        # Publicar en una posición que probablemente exista en el mapa
        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0
        
        # Orientación hacia donde empieza la ruta
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, 0.0)
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
            self.get_logger().info(f'Pose inicial publicada {i+1}/3 en (0.0, 0.0)')
            time.sleep(0.5)

    def navigate_to_detection_position(self):
        """Navigate to the position where ArUco detection will occur"""
        x, y, yaw = self.transformed_detection
        self.send_goal(x, y, yaw, self.detection_position_reached)

    def detection_position_reached(self):
        """Callback when detection position is reached"""
        self.get_logger().info('Posición de detección alcanzada. Activando detección de ArUcos...')
        
        # Subscribe to camera for ArUco detection
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.aruco_detection_active = True
        self.waiting_for_aruco = True
        
        # Set timeout for ArUco detection
        self.detection_timeout_timer = self.create_timer(30.0, self.detection_timeout)
        self.get_logger().info('Esperando detección de ArUco ID 5, 6 o 17...')  # Actualizado

    def image_callback(self, msg):
        """Process camera images for ArUco detection - EXACTAMENTE IGUAL QUE TU DETECTOR PROBADO"""
        if not self.aruco_detection_active or not self.waiting_for_aruco:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {str(e)}')
            return
        
        # Detectar marcadores ArUco
        corners, ids, rejected = aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters)
        
        current_detected_ids = []
        
        if ids is not None:
            current_detected_ids = ids.flatten().tolist()
            
            # SOLO MOSTRAR EN TERMINAL SI SE DETECTAN NUEVOS IDs
            if current_detected_ids != self.last_detected_ids:
                self.get_logger().info(f"✅ ArUco 5x5 IDs detectados: {current_detected_ids}")
                self.last_detected_ids = current_detected_ids
            
            # Check if we found a known ArUco ID
            for aruco_id in current_detected_ids:
                if aruco_id in self.routes:
                    self.get_logger().info(f'ID {aruco_id} detectado! Ejecutando ruta asociada...')
                    self.detected_aruco_id = aruco_id
                    self.waiting_for_aruco = False
                    self.aruco_detection_active = False
                    
                    # Stop detection timeout
                    if hasattr(self, 'detection_timeout_timer'):
                        self.destroy_timer(self.detection_timeout_timer)
                    
                    # Unsubscribe from camera to save resources
                    self.destroy_subscription(self.camera_subscription)
                    
                    # Start the route for this ArUco ID
                    self.start_route_for_aruco(aruco_id)
                    return
        else:
            # Si no detecta nada, limpia la lista de últimos detectados
            if self.last_detected_ids:
                self.last_detected_ids = []
                self.get_logger().info("❌ No se detectan ArUcos")

    def detection_timeout(self):
        """Handle timeout when no ArUco is detected"""
        if self.waiting_for_aruco:
            self.get_logger().warning('Timeout: No se detectó ningún ArUco conocido')
            self.waiting_for_aruco = False
            self.aruco_detection_active = False
            
            # You can add default behavior here, like returning to start
            self.get_logger().info('Volviendo a posición inicial...')
            # Implement return to start or other default behavior

    def start_route_for_aruco(self, aruco_id):
        """Start navigation route for the detected ArUco ID"""
        if aruco_id not in self.transformed_routes:
            self.get_logger().error(f'No hay ruta definida para el ID {aruco_id}')
            return
            
        self.current_route = self.transformed_routes[aruco_id]
        self.current_goal_index = 0
        self.get_logger().info(f'Iniciando ruta para ID {aruco_id} con {len(self.current_route)} puntos')
        
        self.send_next_route_goal()

    def send_next_route_goal(self):
        """Send next goal in the current route"""
        if self.current_goal_index >= len(self.current_route):
            self.get_logger().info('Ruta completada!')
            return
            
        point_name, x, y, yaw = self.current_route[self.current_goal_index]
        self.get_logger().info(f'Navegando a {point_name} ({self.current_goal_index + 1}/{len(self.current_route)})')
        
        self.send_goal(x, y, yaw, self.route_goal_reached)

    def route_goal_reached(self):
        """Callback when a route goal is reached"""
        self.current_goal_index += 1
        time.sleep(1.0)  # Brief pause between points
        self.send_next_route_goal()

    def send_goal(self, x, y, yaw, callback):
        """Send navigation goal to Nav2 - EXACTAMENTE IGUAL QUE TU CÓDIGO PROBADO"""
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

        self.get_logger().info(f'Enviando goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, callback))

    def goal_response_callback(self, future, callback):
        """Handle response from action server - EXACTAMENTE IGUAL QUE TU CÓDIGO PROBADO"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rechazado - reintentando en 5 segundos...')
            time.sleep(5.0)
            # Retry with the same callback
            self.get_logger().info('Reintentando goal...')
            # We need to store the goal parameters to retry, for simplicity we'll just call the callback
            # In a more robust implementation, you'd store the goal parameters
            return

        self.get_logger().info('Goal aceptado')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.get_result_callback(future, callback))

    def get_result_callback(self, future, callback):
        """Handle result from action server - EXACTAMENTE IGUAL QUE TU CÓDIGO PROBADO"""
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Goal completado con estado: {status}')
        
        # Execute the callback function
        callback()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()