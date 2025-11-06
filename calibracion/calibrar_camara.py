#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Script de calibración de cámara con OpenCV
# - Busca automáticamente imágenes .jpg en la carpeta "dataset_calibracion" dentro
#   del directorio de este script (calibracion/dataset_calibracion)
# - Detecta tablero de ajedrez (por defecto 9x6 esquinas internas, ajustable)
# - Reporta progreso y estadísticas en la terminal
# - Calibra usando solo detecciones exitosas
# - Guarda parámetros en YAML y muestra una imagen corregida de ejemplo

import os
import sys
import argparse
import cv2
import numpy as np
import glob

def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibración de cámara usando un tablero de ajedrez con OpenCV."
    )
    parser.add_argument("--cols", type=int, default=9, help="Número de esquinas internas en columnas (default: 9).")
    parser.add_argument("--rows", type=int, default=6, help="Número de esquinas internas en filas (default: 6).")
    parser.add_argument("--square-size", type=float, default=1.0, help="Tamaño de celda del tablero en unidades arbitrarias (default: 1.0).")
    parser.add_argument("--no-gui", action="store_true", help="No abrir ventana gráfica; solo guardar ejemplo corregido.")
    return parser.parse_args()

def find_calibration_images(folder):
    """Busca imágenes de calibración en la misma carpeta que el script.
    Formatos soportados: .jpg, .png, .bmp (insensible a mayúsculas/minúsculas).
    """
    exts = {".jpg", ".png", ".bmp"}
    paths = []
    try:
        for name in os.listdir(folder):
            # Ignorar archivos ocultos y el propio script
            if name.startswith('.'):
                continue
            if name == os.path.basename(__file__):
                continue
            _, ext = os.path.splitext(name)
            if ext.lower() in exts:
                paths.append(os.path.join(folder, name))
    except Exception:
        # Fallback usando glob si falla os.listdir por permisos u otros
        for pattern in ["*.jpg", "*.JPG", "*.png", "*.PNG", "*.bmp", "*.BMP"]:
            paths.extend(glob.glob(os.path.join(folder, pattern)))

    return sorted(paths)

def build_object_points(cols, rows, square_size):
    # Genera la grilla de puntos 3D del patrón (z=0)
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)  # (x, y)
    objp[:, :2] = grid * float(square_size)
    return objp

def save_yaml(yaml_path, camera_matrix, dist_coeffs, image_size, rms, rows, cols, square_size):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise IOError(f"No se pudo abrir el archivo YAML para escribir: {yaml_path}")

    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("rms_reprojection_error", float(rms))
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("pattern_rows", int(rows))
    fs.write("pattern_cols", int(cols))
    fs.write("square_size", float(square_size))
    fs.release()

def main():
    args = parse_args()

    # Directorio del script y búsqueda de imágenes en el mismo directorio
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dataset_dir = os.path.join(script_dir, "dataset_calibracion")

    if not os.path.isdir(dataset_dir):
        print(f"ERROR: No se encontró la carpeta de dataset: {dataset_dir}", file=sys.stderr, flush=True)
        print("Crea la carpeta 'calibracion/dataset_calibracion' y coloca dentro las imágenes de calibración.", file=sys.stderr, flush=True)
        sys.exit(1)

    image_paths = find_calibration_images(dataset_dir)

    # Configuración del patrón de tablero
    cols = int(args.cols)
    rows = int(args.rows)
    square_size = float(args.square_size)
    pattern_size = (cols, rows)

    # Preparación de estructuras para calibración
    objpoints = []  # Puntos 3D del patrón
    imgpoints = []  # Puntos 2D detectados en imágenes
    example_success_path = None
    image_size = None

    # Plantilla de puntos 3D del patrón
    objp_template = build_object_points(cols, rows, square_size)

    print(f"Procesando imágenes de calibración en: {dataset_dir}", flush=True)

    # Criterio de refinamiento subpíxel para esquinas detectadas
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Flags de búsqueda de tablero
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    total = len(image_paths)
    detected = 0

    for idx, path in enumerate(image_paths, start=1):
        filename = os.path.basename(path)
        # Intentar leer imagen
        img = cv2.imread(path)
        if img is None:
            # Imagen corrupta o ilegible
            print(f"Imagen {idx}: {filename} - ✗ NO DETECTADO (imagen no legible)", flush=True)
            continue

        # Convertir a escala de grises
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detectar esquinas del tablero
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, flags=find_flags)

        if ret:
            # Refinamiento subpíxel
            corners_refined = cv2.cornerSubPix(
                gray, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=criteria
            )
            # Almacenar puntos
            objpoints.append(objp_template.copy())
            imgpoints.append(corners_refined)

            # Guardar tamaño de imagen (una sola vez)
            if image_size is None:
                image_size = (gray.shape[1], gray.shape[0])

            # Guardar una ruta de ejemplo para mostrar corrección
            if example_success_path is None:
                example_success_path = path

            detected += 1
            print(f"Imagen {idx}: {filename} - ✓ DETECTADO", flush=True)
        else:
            print(f"Imagen {idx}: {filename} - ✗ NO DETECTADO", flush=True)

    # Cálculo de métricas
    success_rate = int(round((detected / total) * 100)) if total > 0 else 0

    if total == 0:
        print("No se encontraron imágenes en la carpeta de dataset. Asegúrate de colocar imágenes .jpg/.png/.bmp.", flush=True)
        sys.exit(1)

    print("=== RESUMEN DE CALIBRACIÓN ===", flush=True)
    print(f"Imágenes totales procesadas: {total}", flush=True)
    print(f"Imágenes con patrón detectado: {detected}", flush=True)
    print(f"Tasa de éxito: {success_rate}%", flush=True)

    if detected >= 3 and image_size is not None:
        # Calibración usando solo imágenes exitosas
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None
        )

        # Guardar parámetros en YAML
        yaml_path = os.path.join(script_dir, "camera_calibration.yml")
        try:
            save_yaml(yaml_path, camera_matrix, dist_coeffs, image_size, rms, rows, cols, square_size)
            print(f"Parámetros guardados en: {os.path.basename(yaml_path)}", flush=True)
        except Exception as e:
            print(f"Error al guardar YAML: {e}", file=sys.stderr, flush=True)

        # Mostrar y guardar imagen de ejemplo corregida
        if example_success_path is not None:
            img_example = cv2.imread(example_success_path)
            if img_example is not None:
                undistorted = cv2.undistort(img_example, camera_matrix, dist_coeffs)
                out_example = os.path.join(script_dir, "ejemplo_corregido.jpg")
                try:
                    cv2.imwrite(out_example, undistorted)
                except Exception as e:
                    print(f"No se pudo guardar la imagen corregida de ejemplo: {e}", file=sys.stderr, flush=True)

                # Crear y guardar comparativa lado a lado (original vs corregida)
                def _annotate(img, text):
                    overlay = img.copy()
                    bar_h = max(30, int(0.06 * img.shape[0]))
                    cv2.rectangle(overlay, (0, 0), (img.shape[1], bar_h), (0, 0, 0), -1)
                    out = cv2.addWeighted(overlay, 0.5, img, 0.5, 0)
                    cv2.putText(out, text, (10, int(bar_h * 0.75)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    return out

                comp_left = img_example
                comp_right = undistorted
                if comp_left.shape[:2] != comp_right.shape[:2]:
                    comp_right = cv2.resize(comp_right, (comp_left.shape[1], comp_left.shape[0]))

                comp_left = _annotate(comp_left, "ORIGINAL")
                comp_right = _annotate(comp_right, "CORREGIDA")
                try:
                    comparativa = cv2.hconcat([comp_left, comp_right])
                except Exception:
                    # Fallback por si hconcat falla (p.ej. tamaños incompatibles)
                    min_h = min(comp_left.shape[0], comp_right.shape[0])
                    comp_left_r = cv2.resize(comp_left, (int(comp_left.shape[1] * min_h / comp_left.shape[0]), min_h))
                    comp_right_r = cv2.resize(comp_right, (int(comp_right.shape[1] * min_h / comp_right.shape[0]), min_h))
                    comparativa = np.hstack((comp_left_r, comp_right_r))

                out_comp = os.path.join(script_dir, "comparativa_ejemplo.jpg")
                try:
                    cv2.imwrite(out_comp, comparativa)
                    print(f"Comparativa guardada en: {os.path.basename(out_comp)}", flush=True)
                except Exception as e:
                    print(f"No se pudo guardar la comparativa: {e}", file=sys.stderr, flush=True)

                if not args.no_gui:
                    try:
                        cv2.imshow("Comparativa: Original (izq) vs Corregida (der)", comparativa)
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()
                    except Exception:
                        # Entornos sin GUI
                        pass
            else:
                print("No se pudo volver a cargar la imagen de ejemplo para mostrar la corrección.", flush=True)
    else:
        # No hay suficientes imágenes buenas para calibrar
        print("No hay suficientes imágenes válidas para calibrar la cámara (se requieren al menos 3).", flush=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nEjecución interrumpida por el usuario.", flush=True)
