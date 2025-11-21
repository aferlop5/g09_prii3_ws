import cv2
import cv2.aruco as aruco
import numpy as np
import os
import glob
import shutil

# Configuración inicial
IMAGES_PATH = "./"  # Ruta donde buscar imágenes
IMAGES_EXT = ["jpg", "jpeg", "png", "bmp"]  # Extensiones de imagen a buscar
OUTPUT_BASE_PATH = "./output/"  # Carpeta base para guardar resultados

# TODOS los diccionarios ArUco disponibles en OpenCV
ARUCO_DICTIONARIES = {
    # 4x4
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    
    # 5x5
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    
    # 6x6
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    
    # 7x7
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    
    # Diccionarios originales y especiales
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": aruco.DICT_APRILTAG_36h11
}

# Parámetros para la detección - MÁS SENSIBLES
parameters = aruco.DetectorParameters_create()

# Parámetros extremadamente sensibles
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10
parameters.minMarkerPerimeterRate = 0.02  # Más bajo para detectar marcadores más pequeños
parameters.maxMarkerPerimeterRate = 8.0   # Más alto para marcadores más grandes
parameters.polygonalApproxAccuracyRate = 0.08  # Más tolerante
parameters.minCornerDistanceRate = 0.03   # Más bajo
parameters.minDistanceToBorder = 1        # Más bajo
parameters.minMarkerDistanceRate = 0.02   # Más bajo
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 6
parameters.cornerRefinementMaxIterations = 50
parameters.cornerRefinementMinAccuracy = 0.05
parameters.markerBorderBits = 1           # Para marcadores con bordes pequeños
parameters.minOtsuStdDev = 2.0            # Más bajo para imágenes de bajo contraste
parameters.perspectiveRemovePixelPerCell = 8  # Para marcadores más pequeños
parameters.perspectiveRemoveIgnoredMarginPerCell = 0.15
parameters.maxErroneousBitsInBorderRate = 0.5  # Más tolerante con errores en bordes
parameters.errorCorrectionRate = 0.6      # Más agresivo en corrección de errores

# Parámetros de cámara (aproximados)
camera_matrix = np.array([
    [1000, 0, 640],
    [0, 1000, 360],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((4, 1))
MARKER_LENGTH = 0.05

def detect_aruco_dictionary(image):
    """
    Detecta automáticamente el diccionario ArUco de la imagen
    Retorna: (nombre_diccionario, corners, ids)
    """
    best_dict = None
    best_corners = None
    best_ids = None
    max_detections = 0
    all_detections = {}
    
    print(f"  🔍 Probando {len(ARUCO_DICTIONARIES)} diccionarios...")
    
    for dict_name, dict_type in ARUCO_DICTIONARIES.items():
        try:
            dictionary = aruco.getPredefinedDictionary(dict_type)
            corners, ids, rejected = aruco.detectMarkers(image, dictionary, parameters=parameters)
            
            if ids is not None:
                num_detections = len(ids)
                detected_ids = ids.flatten().tolist()
                all_detections[dict_name] = detected_ids
                
                if num_detections > max_detections:
                    max_detections = num_detections
                    best_dict = dict_name
                    best_corners = corners
                    best_ids = ids
                    print(f"  ✅ {dict_name}: {num_detections} marcadores (IDs: {detected_ids})")
                else:
                    print(f"  ➖ {dict_name}: {num_detections} marcadores (IDs: {detected_ids})")
                    
        except Exception as e:
            print(f"  ⚠️ Error con {dict_name}: {e}")
            continue
    
    # Si no se detectó nada, mostrar los diccionarios que casi funcionaron
    if best_dict is None and rejected is not None and len(rejected) > 0:
        print(f"  💡 Se encontraron {len(rejected)} formas que podrían ser marcadores pero no coincidieron")
        print(f"  💡 Esto sugiere que podría ser un diccionario personalizado o no estándar")
    
    return best_dict, best_corners, best_ids, all_detections

def preprocess_image(image):
    """
    Preprocesa la imagen para mejorar la detección
    """
    # Convertir a escala de grises
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Aplicar diferentes preprocesamientos y probar todos
    processed_images = []
    
    # 1. Imagen original en grises
    processed_images.append(("original_gray", gray))
    
    # 2. Ecualización de histograma
    eq = cv2.equalizeHist(gray)
    processed_images.append(("histogram_eq", eq))
    
    # 3. Filtro Gaussiano
    gaussian = cv2.GaussianBlur(gray, (3, 3), 0)
    processed_images.append(("gaussian_blur", gaussian))
    
    # 4. CLAHE (Equalización adaptativa)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    clahe_img = clahe.apply(gray)
    processed_images.append(("clahe", clahe_img))
    
    # 5. Threshold adaptativo
    adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                           cv2.THRESH_BINARY, 11, 2)
    processed_images.append(("adaptive_thresh", adaptive_thresh))
    
    return processed_images

def process_image(image_path, output_base_path):
    """
    Procesa una imagen individual y la guarda en la carpeta correspondiente
    """
    filename = os.path.basename(image_path)
    name, ext = os.path.splitext(filename)
    
    print(f"\n🎯 Procesando: {filename}")
    print("-" * 40)
    
    # Cargar imagen
    original_image = cv2.imread(image_path)
    if original_image is None:
        print(f"  ❌ Error: No se pudo cargar {image_path}")
        return None
    
    # Preprocesar la imagen de múltiples formas
    processed_versions = preprocess_image(original_image)
    
    best_dict_overall = None
    best_corners_overall = None
    best_ids_overall = None
    best_preprocessing = "original"
    
    for preprocess_name, processed_image in processed_versions:
        print(f"  🔧 Probando preprocesamiento: {preprocess_name}")
        
        # Convertir de vuelta a BGR si es necesario para la detección
        if len(processed_image.shape) == 2:
            image_for_detection = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
        else:
            image_for_detection = processed_image
            
        dict_name, corners, ids, all_detections = detect_aruco_dictionary(image_for_detection)
        
        if dict_name is not None:
            if best_dict_overall is None or len(ids) > len(best_ids_overall):
                best_dict_overall = dict_name
                best_corners_overall = corners
                best_ids_overall = ids
                best_preprocessing = preprocess_name
                print(f"  🏆 Mejor resultado hasta ahora: {dict_name} con preprocesamiento {preprocess_name}")
    
    # Usar el mejor resultado encontrado
    if best_dict_overall is not None:
        dict_name = best_dict_overall
        corners = best_corners_overall
        ids = best_ids_overall
        print(f"  🎯 Diccionario identificado: {dict_name} (con preprocesamiento: {best_preprocessing})")
        
        # Crear carpeta para este diccionario
        dict_output_path = os.path.join(output_base_path, dict_name)
        os.makedirs(dict_output_path, exist_ok=True)
        
        # Dibujar detecciones en la imagen original
        image_out = original_image.copy()
        aruco.drawDetectedMarkers(image_out, corners, ids)
        
        # Estimar pose y dibujar ejes para cada marcador
        try:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH, camera_matrix, dist_coeffs
            )
            
            for i in range(len(ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]
                
                # Dibujar ejes de coordenadas
                cv2.drawFrameAxes(
                    image_out, camera_matrix, dist_coeffs, 
                    rvec, tvec, MARKER_LENGTH
                )
                
                # Dibujar ID del marcador
                marker_id = ids[i][0]
                center = np.mean(corners[i][0], axis=0).astype(int)
                cv2.putText(
                    image_out, f"ID: {marker_id}", 
                    (center[0] - 50, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
                )
                
        except Exception as e:
            print(f"  ⚠️ Error en estimación de pose: {e}")
        
        # Guardar imagen procesada
        output_file = os.path.join(dict_output_path, f"{name}_detected{ext}")
        cv2.imwrite(output_file, image_out)
        
        # También guardar copia original en la carpeta del diccionario
        original_copy = os.path.join(dict_output_path, filename)
        shutil.copy2(image_path, original_copy)
        
        print(f"  💾 Guardado en: {dict_output_path}/")
        return dict_name
    
    else:
        print(f"  ❌ No se detectó ningún marcador ArUco en {filename}")
        print(f"  💡 Posibles causas:")
        print(f"     - Diccionario personalizado no incluido en OpenCV")
        print(f"     - Marcador dañado o de muy baja calidad")
        print(f"     - Formato no estándar")
        
        # Guardar en carpeta "unknown" con información de diagnóstico
        unknown_path = os.path.join(output_base_path, "unknown")
        os.makedirs(unknown_path, exist_ok=True)
        
        output_file = os.path.join(unknown_path, f"{name}_unknown{ext}")
        cv2.imwrite(output_file, original_image)
        
        # Guardar también una versión preprocesada para análisis
        diagnostic_path = os.path.join(output_base_path, "diagnostic")
        os.makedirs(diagnostic_path, exist_ok=True)
        
        for preprocess_name, processed_img in processed_versions:
            diagnostic_file = os.path.join(diagnostic_path, f"{name}_{preprocess_name}{ext}")
            if len(processed_img.shape) == 2:
                cv2.imwrite(diagnostic_file, processed_img)
            else:
                cv2.imwrite(diagnostic_file, processed_img)
        
        return "unknown"

def main():
    # Crear carpeta base de salida
    os.makedirs(OUTPUT_BASE_PATH, exist_ok=True)
    
    # Buscar imágenes en el directorio
    image_files = []
    for ext in IMAGES_EXT:
        image_files.extend(glob.glob(f"{IMAGES_PATH}*.{ext}"))
    
    print(f"🔍 Encontradas {len(image_files)} imágenes para procesar")
    print("📂 Los resultados se organizarán en:")
    print(f"   {OUTPUT_BASE_PATH}")
    print("   ├── DICT_4X4_50/")
    print("   ├── DICT_5X5_50/") 
    print("   ├── ... etc.")
    print("   ├── unknown/ (imágenes no detectadas)")
    print("   └── diagnostic/ (versiones preprocesadas para análisis)")
    print("=" * 60)
    
    # Estadísticas
    stats = {}
    
    # Procesar cada imagen
    for image_file in image_files:
        dict_name = process_image(image_file, OUTPUT_BASE_PATH)
        
        if dict_name:
            if dict_name in stats:
                stats[dict_name] += 1
            else:
                stats[dict_name] = 1
    
    # Mostrar resumen
    print("\n" + "=" * 60)
    print("📊 RESUMEN FINAL DE PROCESAMIENTO")
    print("=" * 60)
    total_detected = sum(count for dict_name, count in stats.items() if dict_name != "unknown")
    total_unknown = stats.get("unknown", 0)
    
    for dict_name, count in sorted(stats.items()):
        if dict_name == "unknown":
            print(f"   ❌ {dict_name}: {count} imágenes")
        else:
            print(f"   ✅ {dict_name}: {count} imágenes")
    
    print(f"   📈 Total detectadas: {total_detected}")
    print(f"   ❓ Total no detectadas: {total_unknown}")
    print(f"   🎯 Efectividad: {(total_detected/len(image_files))*100:.1f}%")
    
    # Mostrar estructura de carpetas creada
    print("\n📁 Estructura de carpetas creada:")
    for root, dirs, files in os.walk(OUTPUT_BASE_PATH):
        level = root.replace(OUTPUT_BASE_PATH, '').count(os.sep)
        indent = ' ' * 2 * level
        print(f'{indent}📁 {os.path.basename(root)}/')
        subindent = ' ' * 2 * (level + 1)
        for file in files[:3]:  # Mostrar solo primeros 3 archivos por carpeta
            print(f'{subindent}📄 {file}')
        if len(files) > 3:
            print(f'{subindent}... y {len(files) - 3} más')

if __name__ == '__main__':
    main()