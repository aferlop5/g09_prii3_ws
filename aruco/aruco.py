import cv2
import cv2.aruco as aruco
import numpy as np
import os
import glob

# Configuración inicial
IMAGES_PATH = "./"  # Ruta donde buscar imágenes
IMAGES_EXT = ["jpg", "jpeg", "png", "bmp"]  # Extensiones de imagen a buscar
OUTPUT_PATH = "./output/"  # Carpeta para guardar resultados
MARKER_LENGTH = 0.05  # Tamaño del marcador en metros (ajustar según necesidad)
# Tamaño del texto para mostrar la ID (ajústalo si lo necesitas)
ID_FONT_SCALE = 4
ID_TEXT_THICKNESS = 5

# Crear carpeta de salida si no existe
os.makedirs(OUTPUT_PATH, exist_ok=True)

# Inicializar detector ArUco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# Matriz de cámara aproximada (deberías calibrar tu cámara para mejores resultados)
camera_matrix = np.array([
    [1000, 0, 640],
    [0, 1000, 360],
    [0, 0, 1]
], dtype=np.float32)

# Coeficientes de distorsión (normalmente cercanos a cero)
dist_coeffs = np.zeros((4, 1))

# Buscar imágenes en el directorio
image_files = []
for ext in IMAGES_EXT:
    image_files.extend(glob.glob(f"{IMAGES_PATH}*.{ext}"))

print(f"Encontradas {len(image_files)} imágenes para procesar")

for image_file in image_files:
    print(f"Procesando: {image_file}")
    
    # Cargar imagen
    image = cv2.imread(image_file)
    if image is None:
        print(f"  Error: No se pudo cargar {image_file}")
        continue
    
    # Detectar marcadores
    corners, ids, rejected = detector.detectMarkers(image)
    
    if ids is not None:
        print(f"  Detectados {len(ids)} marcadores: {ids.flatten()}")
        
        # Dibujar marcadores detectados
        image_out = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        
        # Procesar cada marcador detectado
        for i in range(len(ids)):
            # Estimar pose del marcador
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_LENGTH, camera_matrix, dist_coeffs
            )
            
            # Dibujar sistema de coordenadas
            cv2.drawFrameAxes(
                image_out, camera_matrix, dist_coeffs, 
                rvec, tvec, MARKER_LENGTH, 3
            )
            
            # Dibujar ID del marcador
            marker_id = ids[i][0]
            center = np.mean(corners[i][0], axis=0).astype(int)
            cv2.putText(
                image_out, f"ID: {marker_id}", 
                (center[0] - 50, center[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, ID_FONT_SCALE, (0, 255, 0), ID_TEXT_THICKNESS
            )
    else:
        print("  No se detectaron marcadores")
        image_out = image.copy()
    
    # Guardar imagen resultante
    filename = os.path.basename(image_file)
    name, ext = os.path.splitext(filename)
    output_file = f"{OUTPUT_PATH}{name}_result{ext}"
    cv2.imwrite(output_file, image_out)
    print(f"  Resultado guardado: {output_file}")

print("Procesamiento completado!")