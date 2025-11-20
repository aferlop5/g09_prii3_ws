import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Suscriptor para las imágenes de la cámara - VERIFICADO topic correcto
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic verificado
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publicador para visualización (opcional)
        self.image_pub = self.create_publisher(Image, '/aruco_detection_image', 10)
        
        # Bridge para convertir entre ROS Image y OpenCV
        self.bridge = CvBridge()
        
        # Parámetros de detección ArUco
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        
        # Parámetros de la cámara del JetBot (valores aproximados)
        self.camera_matrix = np.array([
            [600.0, 0, 320.0],
            [0, 600.0, 240.0],
            [0, 0, 1.0]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4, 1))
        
        # Tamaño del marcador en metros
        self.marker_length = 0.05
        
        # Variable para controlar la impresión en terminal
        self.last_detected_ids = []
        
        self.get_logger().info('Nodo de detección de ArUcos inicializado - Suscrito a /camera/image_raw')

    def image_callback(self, msg):
        try:
            # Convertir mensaje de ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {str(e)}')
            return
        
        # Detectar marcadores ArUco
        corners, ids, rejected = aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters)
        
        # Crear imagen de salida con las detecciones
        output_image = cv_image.copy()
        
        current_detected_ids = []
        
        if ids is not None:
            current_detected_ids = ids.flatten().tolist()
            
            # SOLO MOSTRAR EN TERMINAL SI SE DETECTAN NUEVOS IDs
            if current_detected_ids != self.last_detected_ids:
                print(f"ArUco IDs detectados: {current_detected_ids}")
                self.last_detected_ids = current_detected_ids
            
            # Dibujar marcadores detectados
            aruco.drawDetectedMarkers(output_image, corners, ids)
            
            # Estimar pose para cada marcador
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]
                
                # Dibujar ejes de coordenadas
                cv2.drawFrameAxes(
                    output_image, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_length, 2
                )
                
                # Dibujar ID del marcador
                marker_id = ids[i][0]
                center = np.mean(corners[i][0], axis=0).astype(int)
                cv2.putText(
                    output_image, f"ID: {marker_id}", 
                    (center[0] - 30, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
        else:
            # Si no detecta nada, limpia la lista de últimos detectados
            if self.last_detected_ids:
                self.last_detected_ids = []
        
        # Publicar imagen con detecciones (opcional)
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
            detection_msg.header = msg.header
            self.image_pub.publish(detection_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando imagen: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        print("\nDeteniendo nodo de detección de ArUcos...")
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()