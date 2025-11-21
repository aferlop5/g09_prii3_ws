#! /usr/bin/env python
from __future__ import print_function

import argparse
from xml.dom.minidom import parse
import os
import sys
import cv2
import numpy as np
import shutil

def resize_image_to_170x170(input_path, output_path):
    """
    Redimensiona cualquier imagen a 170x170 píxeles SIN mantener relación de aspecto
    """
    try:
        # Cargar imagen
        image = cv2.imread(input_path)
        if image is None:
            print(f"❌ Error: No se pudo cargar la imagen {input_path}")
            return False
        
        # Obtener dimensiones originales
        original_height, original_width = image.shape[:2]
        print(f"  📐 Original: {original_width}x{original_height} -> 170x170")
        
        # Redimensionar a 170x170 exactos (sin mantener relación de aspecto)
        resized_image = cv2.resize(image, (170, 170), interpolation=cv2.INTER_AREA)
        
        # Guardar imagen redimensionada
        cv2.imwrite(output_path, resized_image)
        print(f"  ✅ Redimensionada: {output_path}")
        return True
        
    except Exception as e:
        print(f"❌ Error redimensionando {input_path}: {e}")
        return False

def process_images_directory(images_dir, temp_dir):
    """
    Procesa todas las imágenes en el directorio, redimensionándolas a 170x170
    """
    print("🔄 Procesando y redimensionando imágenes...")
    
    # Crear directorio temporal si no existe
    os.makedirs(temp_dir, exist_ok=True)
    
    processed_images = []
    
    # Buscar todos los archivos PNG en el directorio
    for filename in os.listdir(images_dir):
        if filename.lower().endswith('.png'):
            input_path = os.path.join(images_dir, filename)
            output_path = os.path.join(temp_dir, filename)
            
            # Redimensionar imagen
            if resize_image_to_170x170(input_path, output_path):
                processed_images.append(filename)
    
    return processed_images

def main():
    parser = argparse.ArgumentParser(
        usage='generate gazebo models for AR tags - AUTO-RESIZE VERSION',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-i', '--images-dir',
        default="./images",
        help='directory where the marker images are located')
    parser.add_argument(
        '-g', '--gazebodir',
        default="$HOME/.gazebo/models",
        help='Gazebo models directory')
    parser.add_argument(
        '-l', '--local-model-dir',
        default="./model",
        help='Local model directory (where to create local copies)')
    parser.add_argument(
        '-s', '--size',
        default=500, type=int,
        help='marker size in mm')
    parser.add_argument(
        '-v', '--verbose',
        dest='verbose', action='store_true',
        help='verbose mode')
    parser.add_argument(
        '-w', '--white-contour-size-mm',
        default=0, type=int,
        help='Add white contour around images, default to no contour')
    parser.add_argument(
        '--temp-dir',
        default="/tmp/aruco_resized",
        help='Temporary directory for resized images')

    parser.set_defaults(verbose=False)

    args = parser.parse_args()

    args.gazebodir = os.path.expandvars(args.gazebodir)
    args.images_dir = os.path.expandvars(args.images_dir)
    args.local_model_dir = os.path.expandvars(args.local_model_dir)
    args.temp_dir = os.path.expandvars(args.temp_dir)
    
    # CORRECCIÓN: Usar la ruta actual del script para encontrar el directorio model
    script_path = os.path.dirname(os.path.realpath(__file__))
    model_dir = os.path.join(script_path, 'model')  # Cambiado para buscar en la misma carpeta que el script
    
    ORIGINAL_MARKER_SIZE_MM = 500
    ORIGINAL_IMAGE_SIZE_PX = 170
    white_contour_px = \
        args.white_contour_size_mm * ORIGINAL_IMAGE_SIZE_PX / args.size

    # Verificar que el directorio de imágenes existe
    if not os.path.isdir(args.images_dir):
        print(f"❌ Error: El directorio de imágenes '{args.images_dir}' no existe")
        sys.exit(1)

    # Verificar que el directorio model existe
    if not os.path.isdir(model_dir):
        print(f"❌ Error: No se encuentra el directorio de plantillas 'model' en {model_dir}")
        print(f"💡 Asegúrate de que existe la carpeta 'model' con la plantilla 'marker0'")
        sys.exit(1)

    # Verificar que existe marker0 en el directorio model
    marker0_template = os.path.join(model_dir, 'marker0')
    if not os.path.isdir(marker0_template):
        print(f"❌ Error: No se encuentra la plantilla 'marker0' en {marker0_template}")
        sys.exit(1)

    # Paso 1: Procesar y redimensionar todas las imágenes a 170x170
    print("🎯 INICIANDO PROCESAMIENTO DE IMÁGENES")
    print("=" * 50)
    
    processed_images = process_images_directory(args.images_dir, args.temp_dir)
    
    if not processed_images:
        print("❌ No se encontraron imágenes PNG para procesar")
        sys.exit(1)
    
    print(f"✅ Imágenes procesadas: {len(processed_images)}")
    print("-" * 50)

    # Paso 2: Generar modelos de Gazebo con las imágenes redimensionadas
    print("🏗️  GENERANDO MODELOS DE GAZEBO")
    print("=" * 50)

    # Asegurar que los directorios de destino existen
    os.makedirs(args.gazebodir, exist_ok=True)
    os.makedirs(args.local_model_dir, exist_ok=True)

    # Copy marker0 directory into gazebo model directory
    cp_marker0_cmd = "cp -r " + marker0_template + " " + os.path.join(args.gazebodir, "marker0")
    if args.verbose:
        print(cp_marker0_cmd)
    
    # Usar shutil en lugar de os.system para mejor control de errores
    try:
        if os.path.exists(os.path.join(args.gazebodir, "marker0")):
            shutil.rmtree(os.path.join(args.gazebodir, "marker0"))
        shutil.copytree(marker0_template, os.path.join(args.gazebodir, "marker0"))
        print("✅ Plantilla marker0 copiada correctamente a Gazebo models")
    except Exception as e:
        print(f"❌ Error copiando plantilla marker0 a Gazebo: {e}")
        sys.exit(1)

    # También copiar marker0 al directorio local si no existe
    local_marker0 = os.path.join(args.local_model_dir, "marker0")
    if not os.path.exists(local_marker0):
        try:
            shutil.copytree(marker0_template, local_marker0)
            print("✅ Plantilla marker0 copiada correctamente al directorio local")
        except Exception as e:
            print(f"⚠️  Error copiando plantilla marker0 al directorio local: {e}")

    processed_count = 0
    
    for image_file in processed_images:
        image_file_path = os.path.join(args.temp_dir, image_file)
        filename_without_ext = image_file[0:image_file.rfind('.')]
        
        # Limpiar el nombre del archivo (remover espacios y caracteres especiales)
        clean_name = filename_without_ext.replace(' ', '_').replace('.', '_')
        
        print(f"🔄 Procesando: {image_file} -> {clean_name.lower()}")

        # Generar en Gazebo models
        gazebo_target_dir = os.path.join(args.gazebodir, clean_name.lower())
        local_target_dir = os.path.join(args.local_model_dir, clean_name.lower())

        # ignore marker0 as it has already been copied above
        if not clean_name.lower() == 'marker0':
            # Copiar a Gazebo models
            try:
                if os.path.exists(gazebo_target_dir):
                    shutil.rmtree(gazebo_target_dir)
                shutil.copytree(os.path.join(args.gazebodir, "marker0"), gazebo_target_dir)
                if args.verbose:
                    print(f"✅ Copiada plantilla a {gazebo_target_dir}")
            except Exception as e:
                print(f"❌ Error copiando plantilla para {clean_name} en Gazebo: {e}")
                continue

            # Copiar al directorio local
            try:
                if os.path.exists(local_target_dir):
                    shutil.rmtree(local_target_dir)
                shutil.copytree(os.path.join(args.local_model_dir, "marker0"), local_target_dir)
                if args.verbose:
                    print(f"✅ Copiada plantilla a {local_target_dir}")
            except Exception as e:
                print(f"⚠️  Error copiando plantilla para {clean_name} en local: {e}")

        # Función para procesar un modelo (Gazebo o local)
        def process_model(target_dir, is_local=False):
            # Remove the original Marker0.png from the new model
            old_texture_path = os.path.join(target_dir, "materials", "textures", "Marker0.png")
            if os.path.exists(old_texture_path):
                os.remove(old_texture_path)
            
            # Copy the resized image
            image_dest_path = os.path.join(target_dir, "materials", "textures", image_file)
            try:
                shutil.copy2(image_file_path, image_dest_path)
                if args.verbose:
                    location = "local" if is_local else "Gazebo"
                    print(f"✅ Imagen copiada a {location}: {image_dest_path}")
            except Exception as e:
                location = "local" if is_local else "Gazebo"
                print(f"❌ Error copiando imagen {image_file} a {location}: {e}")
                return False

            # add white contour if applicable:
            if white_contour_px > 0:
                try:
                    # Usar ImageMagick para añadir contorno blanco
                    convert_cmd = f"convert {image_dest_path} -bordercolor white -border {int(white_contour_px)}x{int(white_contour_px)} {image_dest_path}"
                    os.system(convert_cmd)
                    if args.verbose:
                        print(f"✅ Contorno blanco añadido: {white_contour_px}px")
                except Exception as e:
                    print(f"⚠️ Error añadiendo contorno blanco: {e}")

            # Modify model.config
            model_config_path = os.path.join(target_dir, "model.config")
            try:
                dom = parse(model_config_path)
                for node in dom.getElementsByTagName('name'):
                    node.firstChild.nodeValue = clean_name
                    break
                with open(model_config_path, 'w') as f:
                    f.write(dom.toxml())
                if args.verbose:
                    print(f"✅ model.config actualizado: {clean_name}")
            except Exception as e:
                print(f"❌ Error modificando model.config: {e}")
                return False

            # Modify model.sdf
            model_noversion_sdf_path = os.path.join(target_dir, "model.sdf")
            try:
                dom = parse(model_noversion_sdf_path)
                
                # Update model name
                for node in dom.getElementsByTagName('model'):
                    node.attributes["name"].value = clean_name
                    break

                # Update scale and URI
                scale = (args.size + 2 * args.white_contour_size_mm) / float(ORIGINAL_MARKER_SIZE_MM)
                scaleModified = False
                
                for node in dom.getElementsByTagName('mesh'):
                    for child in node.childNodes:
                        if child.nodeName == "scale":
                            child.firstChild.nodeValue = "{} {} {}".format(scale, scale, scale)
                            scaleModified = True
                        if child.nodeName == "uri":
                            child.firstChild.nodeValue = "model://" + os.path.join(
                                clean_name.lower(), "meshes", clean_name + ".dae")
                    if not scaleModified:
                        x = dom.createElement("scale")
                        y = dom.createTextNode("{} {} {}".format(scale, scale, scale))
                        x.appendChild(y)
                        node.appendChild(x)

                with open(model_noversion_sdf_path, 'w') as f:
                    f.write(dom.toxml())
                if args.verbose:
                    print(f"✅ model.sdf actualizado")
            except Exception as e:
                print(f"❌ Error modificando model.sdf: {e}")
                return False

            # Copy and modify model-1_4.sdf
            model_sdf_14_path = os.path.join(target_dir, "model-1_4.sdf")
            try:
                shutil.copy2(model_noversion_sdf_path, model_sdf_14_path)
                dom = parse(model_sdf_14_path)
                for node in dom.getElementsByTagName('sdf'):
                    node.attributes["version"].value = "1.4"
                    break
                with open(model_sdf_14_path, 'w') as f:
                    f.write(dom.toxml())
                if args.verbose:
                    print(f"✅ model-1_4.sdf creado")
            except Exception as e:
                print(f"❌ Error creando model-1_4.sdf: {e}")

            # Copy and modify model-1_5.sdf
            model_sdf_15_path = os.path.join(target_dir, "model-1_5.sdf")
            try:
                shutil.copy2(model_noversion_sdf_path, model_sdf_15_path)
                dom = parse(model_sdf_15_path)
                for node in dom.getElementsByTagName('sdf'):
                    node.attributes["version"].value = "1.5"
                    break
                with open(model_sdf_15_path, 'w') as f:
                    f.write(dom.toxml())
                if args.verbose:
                    print(f"✅ model-1_5.sdf creado")
            except Exception as e:
                print(f"❌ Error creando model-1_5.sdf: {e}")

            # Rename mesh file
            meshes_dir = os.path.join(target_dir, "meshes")
            old_mesh_path = os.path.join(meshes_dir, "Marker0.dae")
            new_mesh_path = os.path.join(meshes_dir, clean_name + ".dae")
            try:
                if os.path.exists(old_mesh_path):
                    os.rename(old_mesh_path, new_mesh_path)
                if args.verbose:
                    print(f"✅ Mesh renombrado: {clean_name}.dae")
            except Exception as e:
                print(f"❌ Error renombrando mesh: {e}")

            # Modify the .dae file
            try:
                if os.path.exists(new_mesh_path):
                    dom = parse(new_mesh_path)
                    for node in dom.getElementsByTagName('init_from'):
                        node.firstChild.nodeValue = image_file
                        break
                    with open(new_mesh_path, 'w') as f:
                        f.write(dom.toxml())
                    if args.verbose:
                        print(f"✅ Archivo .dae actualizado")
            except Exception as e:
                print(f"❌ Error modificando .dae: {e}")
            
            return True

        # Procesar modelo en Gazebo
        if not process_model(gazebo_target_dir, is_local=False):
            continue

        # Procesar modelo en directorio local
        process_model(local_target_dir, is_local=True)
        
        processed_count += 1
        print(f"✅ Completado: {clean_name}")

    # Limpiar directorio temporal
    if os.path.exists(args.temp_dir):
        shutil.rmtree(args.temp_dir)
        print(f"🧹 Directorio temporal limpiado: {args.temp_dir}")

    print(f"\n🎉 PROCESAMIENTO COMPLETADO!")
    print(f"   📦 Modelos generados: {processed_count}")
    print(f"   📍 Ubicación Gazebo: {args.gazebodir}")
    print(f"   📍 Ubicación Local: {args.local_model_dir}")
    print(f"   🎯 Tamaño de marcadores: {args.size} mm")
    if args.white_contour_size_mm > 0:
        print(f"   ⚪ Contorno blanco: {args.white_contour_size_mm} mm")
    
    print(f"\n💡 Ahora puedes usar estos modelos en tu archivo .world de Gazebo:")
    for image_file in processed_images[:5]:  # Mostrar solo los primeros 5
        name = image_file[0:image_file.rfind('.')].replace(' ', '_').replace('.', '_')
        print(f"   <include><uri>model://{name.lower()}</uri></include>")

if __name__ == '__main__':
    main()