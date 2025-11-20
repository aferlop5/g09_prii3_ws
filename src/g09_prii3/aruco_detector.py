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
        
        # Suscriptor para las imágenes de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publicador para visualización (opcional)
        self.image_pub = self.create_publisher(Image, '/aruco_detection_image', 10)
        
        # Bridge para convertir entre ROS Image y OpenCV
        self.bridge = CvBridge()
        
        # Parámetros de detección ArUco - DICCIONARIO 5X5
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
        
        # Contador de frames para diagnóstico
        self.frame_count = 0
        
        print("🚀 Nodo de detección de ArUcos 5x5 inicializado")
        print("📡 Suscrito a /camera/image_raw")
        print("🎯 Usando diccionario: DICT_5X5_50")

    def image_callback(self, msg):
        self.frame_count += 1
        
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
                print(f"✅ ArUco 5x5 IDs detectados: {current_detected_ids}")
                self.last_detected_ids = current_detected_ids
            
            # Dibujar marcadores detectados
            aruco.drawDetectedMarkers(output_image, corners, ids)
            
            # Estimar pose para cada marcador
            try:
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
                    
                    # Opcional: mostrar distancia
                    distance = np.sqrt(tvec[0][0]**2 + tvec[0][1]**2 + tvec[0][2]**2)
                    cv2.putText(
                        output_image, f"Dist: {distance:.2f}m", 
                        (center[0] - 30, center[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1
                    )
                    
            except Exception as e:
                print(f"⚠️ Error en estimación de pose: {e}")
                
        else:
            # Si no detecta nada, limpia la lista de últimos detectados
            if self.last_detected_ids:
                self.last_detected_ids = []
                print("❌ No se detectan ArUcos")
            
            # Mostrar mensaje de diagnóstico cada 50 frames
            if self.frame_count % 50 == 0:
                print(f"🔍 Frame {self.frame_count}: Esperando detección de ArUco 5x5...")
        
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
        print("\n🛑 Deteniendo nodo de detección de ArUcos 5x5...")
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()