# PRII3 · Grupo 09 – Workspace ROS2
<div align="center"> <img alt="ROS2" src="https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble-blue?logo=ros2" /> &nbsp; <img alt="Python" src="https://img.shields.io/badge/Python-3.8%2B-blue.svg?logo=python" /> &nbsp; <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04%2F22.04-orange?logo=ubuntu" /> </div>

## 📋 Descripción del Proyecto
Este repositorio contiene el workspace ROS2 para el proyecto Robots Inteligentes (PRII3) del Grupo 09. El objetivo principal es diseñar, construir y programar un sistema robótico autónomo capaz de superar los retos de la competición EUROBOT 2026.

El proyecto se desarrolla utilizando metodología SCRUM a través de sprints iterativos, implementando funcionalidades progresivas que van desde el control básico hasta la navegación autónoma avanzada con percepción visual.

## 🏗️ Estructura del Workspace
```
g09_prii3_ws/
├── launch/                         # Archivos de lanzamiento ROS2 organizados por funcionalidad
│   ├── sprint1.launch.py          # Lanzamiento para Sprint 1 (Turtlesim)
│   ├── jetbot_drawer.launch.py    # Control de dibujo para JetBot real
│   ├── drawer_number_gazebo.launch.py  # Simulación de dibujo en Gazebo
│   ├── obstacle_avoidance_*.launch.py  # Algoritmos de evitación de obstáculos
│   ├── Potential_Fields.launch.py # Navegación por campos potenciales (Sprint 2)
│   ├── f1l3_world.launch.py       # Mundo personalizado del laboratorio
│   ├── rviz_predefinido_node.launch.py # Navegación predefinida (Sprint 3)
│   └── aruco_nav_launch.py        # Navegación autónoma con ArUcos (Sprint 3)
├── src/g09_prii3/                 # Código fuente principal del paquete
│   ├── prii3_turtlesim_node.py    # Nodo para control de turtlesim (Sprint 1)
│   ├── jetbot_drawer_node.py      # Control de movimiento para JetBot real (Sprint 2)
│   ├── drawer_number_gazebo.py    # Control de TurtleBot3 en simulación (Sprint 2)
│   ├── obstacle_avoidance_node.py # Algoritmos de evitación de obstáculos (Sprint 2)
│   ├── Potential_Fields.py        # Navegación por campos potenciales (Sprint 2)
│   └── rviz_predefinido_node.py   # Navegación predefinida (Sprint 3)
├── mundos_gazebo/                 # Entornos de simulación personalizados
│   └── f1l3.world                 # Réplica del laboratorio F1L3 para Gazebo
├── maps/                          # Mapas generados con técnicas SLAM
│   ├── mapa_f1l3_gazebo.yaml      # Configuración del mapa del laboratorio
│   └── mapa_f1l3_gazebo.pgm       # Mapa en formato imagen
├── calibracion/                   # Utilidades para calibración de cámara
│   ├── calibrar_camara.py         # Script de calibración
│   └── dataset_calibracion/       # Imágenes para calibración
├── aruco/                         # Herramientas para detección de marcadores ArUco
│   └── aruco.py                   # Utilidades de procesamiento ArUco
└── install/                       # Archivos generados durante la instalación
```

## ⚙️ Instalación y Configuración del Entorno
### Requisitos del Sistema
- **Sistema Operativo**: Ubuntu 20.04 (Foxy) o Ubuntu 22.04 (Humble)
- **ROS 2**: Distribución Foxy Fitzroy o Humble Hawksbill
- **Python**: Versión 3.8 o superior
- **Dependencias adicionales**: Gazebo, OpenCV, TurtleBot3 (para simulación)

### Configuración Paso a Paso
1. **Instalación de Dependencias del Sistema**
   ```bash
   # Actualizar el sistema e instalar dependencias básicas
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y libjpeg-dev libpng-dev libtiff-dev \
                    libavcodec-dev libavformat-dev libswscale-dev \
                    libv4l-dev libxvidcore-dev libx264-dev \
                    libgtk-3-dev libatlas-base-dev gfortran
   ```

2. **Instalación de Paquetes Python**
   ```bash
   # Actualizar pip e instalar paquetes necesarios
   sudo python3 -m pip install --upgrade pip
   sudo python3 -m pip install opencv-contrib-python numpy
   ```

3. **Verificación de la Instalación**
   ```bash
   # Verificar versión de OpenCV
   python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"

   # Verificar instalación de ROS2
   ros2 --version
   ```

4. **Construcción del Workspace**
   ```bash
   # Navegar al directorio del workspace
   cd g09_prii3_ws

   # Construir el paquete con enlaces simbólicos para desarrollo
   colcon build --symlink-install

   # Cargar el entorno del workspace
   source install/setup.bash
   ```

## 🚀 Ejecución por Sprints
### 📋 Sprint 1: Configuración del Entorno y Control Básico
**Objetivo**: Establecer un entorno de desarrollo ROS2 completamente operativo y implementar control básico del simulador turtlesim.

**Funcionalidades Implementadas**
- ✅ Instalación y configuración de ROS2 Foxy
- ✅ Creación del workspace g09_prii3_ws y paquete g09_prii3
- ✅ Nodo Python para control autónomo de turtlesim
- ✅ Servicios ROS para control del dibujo (pausar, reanudar, reiniciar)
- ✅ Sistema de lanzamiento unificado con archivos launch

**Ejecución**
```bash
# Lanzar el entorno completo del Sprint 1
ros2 launch g09_prii3 sprint1.launch.py
```

**Servicios Disponibles**
Una vez ejecutado, puedes controlar el dibujo mediante servicios ROS:
```bash
# Pausar el dibujo
ros2 service call /drawer/pause std_srvs/srv/Trigger "{}"

# Reanudar el dibujo
ros2 service call /drawer/resume std_srvs/srv/Trigger "{}"

# Reiniciar el dibujo desde el inicio
ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"
```

### 🤖 Sprint 2: Movimiento Autónomo y Evitación de Obstáculos
**Objetivo**: Implementar control avanzado del movimiento del robot y algoritmos de evitación de obstáculos utilizando sensor LIDAR, tanto en simulación como con el robot real.

**Arquitectura de Implementación**
- **Robot Real (NVIDIA JetBot)**
  - Nodo: `jetbot_drawer_node`
  - Funcionalidad: Control de movimiento para dibujar el número "09" en el espacio físico
  - Requisito: Stack del JetBot ejecutándose en segundo plano

- **Simulación (TurtleBot3 en Gazebo)**
  - Nodo: `drawer_number_gazebo`
  - Funcionalidad: Réplica del comportamiento en entorno simulado
  - Característica: Configuración automática del modelo y mundo de simulación

**Ejecución para Robot Real**
```bash
# Terminal 1: Ejecutar stack del JetBot (requerido para comunicación)
ros2 launch jetbot_pro_ros2 jetbot.py

# Terminal 2: Lanzar nodo de dibujo autónomo
ros2 launch g09_prii3 jetbot_drawer.launch.py
```

**Ejecución en Simulación**
```bash
# Lanzar simulación completa en Gazebo
ros2 launch g09_prii3 drawer_number_gazebo.launch.py
```

...