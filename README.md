<sub>README optimizado para GitHub — estilo claro y directo</sub>

# PRII3 · Grupo 09 – Workspace ROS2

<div align="center">
   <img alt="ROS2" src="https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble-blue?logo=ros2" />
   &nbsp;
   <img alt="Python" src="https://img.shields.io/badge/Python-3.8%2B-blue.svg?logo=python" />
   &nbsp;
   <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04%2F22.04-orange?logo=ubuntu" />
</div>

---

## 🌍 Mundo EUROBOT (Gazebo)

### Descripción
Mundo personalizado `eurobot.world` que carga el modelo `model://eurobot_world` (instalado en `setup.py`) y spawnea un TurtleBot3 Waffle compatible con ROS2 Foxy mediante un spawner explícito (`gazebo_ros spawn_entity.py`). Se replica el patrón usado en `f1l3`, pero adaptado porque en Foxy no existe `spawn_turtlebot3.launch.py`.

### Preparación del modelo
1. Carpeta del modelo: `mundos_gazebo/eurobot_world/` con `model.config` y `model.sdf`.
2. Instalación añadida en `setup.py`:
   - `(share/g09_prii3/models/eurobot_world)` para que `model://eurobot_world` se resuelva.
3. Archivo del mundo: `mundos_gazebo/eurobot.world` instalado en `share/g09_prii3/worlds/`.

### Launch (`eurobot.launch.py`)
Acciones:
- Ajusta `GAZEBO_MODEL_PATH` (modelos del paquete + TurtleBot3).
- Exporta `TURTLEBOT3_MODEL=waffle`.
- Lanza `gzserver` + `gzclient` con `eurobot.world`.
- Publica el URDF con `robot_state_publisher` directo (URDF de `turtlebot3_description`).
- Spawnea el Waffle con `spawn_entity.py` (robusto en Foxy/Humble).

### Comandos para lanzar
```bash
cd /home/agusti/universitat_agusti/tercero/proyecto/g09_prii3_ws
colcon build --packages-select g09_prii3 --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=waffle
ros2 launch g09_prii3 eurobot.launch.py
```

### Parámetros opcionales de posición
```bash
ros2 launch g09_prii3 eurobot.launch.py x_pose:=1.0 y_pose:=0.5 z_pose:=0.0
```

### Diagnóstico rápido
```bash
echo "$GAZEBO_MODEL_PATH"
ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models | grep waffle || echo "Falta modelo waffle"
```

Instalar si falta:
```bash
sudo apt install ros-foxy-turtlebot3-gazebo ros-foxy-turtlebot3-description
```

### Cómo lo hicimos (resumen técnico)
1. Creamos el directorio del modelo EUROBOT con `model.sdf` y `model.config`.
2. Añadimos su instalación en `setup.py` para que Gazebo pueda resolver `model://eurobot_world`.
3. Generamos `eurobot.world` siguiendo la estructura de `f1l3.world` (luz, plano, include del mundo y robot).
4. Ajustamos el launch a Foxy reemplazando includes que no existen por nodos (`robot_state_publisher`, `spawn_entity.py`).
5. Verificamos rutas con `GAZEBO_MODEL_PATH` y confirmamos presencia de `turtlebot3_waffle`.
6. Añadimos fallback de spawn explícito para asegurar robot aunque el `<include>` falle.

### Próximos pasos sugeridos
- Integrar rutas de navegación y ArUco sobre el mundo EUROBOT reutilizando los nodos de Sprint 3.
- Añadir mapa generado (SLAM) específico de EUROBOT a `maps/` y documentar conversión de coordenadas si difiere.
- Incluir modelos adicionales (zonas de puntuación, obstáculos) bajo `models/eurobot_world`.

---

---

## 📋 Descripción del proyecto

Este repositorio contiene el workspace ROS2 del proyecto *Robots Inteligentes (PRII3)* desarrollado por el **Grupo 09**. El objetivo es diseñar y programar un sistema robótico autónomo capaz de resolver los retos de la competición EUROBOT 2026.

El desarrollo sigue metodología SCRUM por sprints, con incrementos que van desde control básico (turtlesim) hasta navegación autónoma con percepción visual (ArUco, SLAM, Nav2).

---

## ⚙️ Requisitos y preparación del entorno

### Requisitos principales
- Sistema operativo: Ubuntu 20.04 (Foxy) o 22.04 (Humble)
- ROS 2: Foxy Fitzroy o Humble Hawksbill
- Python 3.8+
- Dependencias adicionales: Gazebo, OpenCV (contrib), TurtleBot3 packages

### Instalación rápida (ejemplo básico)
```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Dependencias comunes para OpenCV
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev \
                         libavcodec-dev libavformat-dev libswscale-dev \
                         libv4l-dev libxvidcore-dev libx264-dev \
                         libgtk-3-dev libatlas-base-dev gfortran

# Actualizar pip e instalar paquetes Python
sudo python3 -m pip install --upgrade pip
sudo python3 -m pip install opencv-contrib-python numpy

# Verificar OpenCV
python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"

# Verificar ROS2
ros2 --version
```

### Construcción del workspace
```bash
# Sitúate en el directorio raíz del workspace
cd g09_prii3_ws

# Compilar con enlaces simbólicos para desarrollo
colcon build --symlink-install

# Cargar entorno
source install/setup.bash
```

---

## 🚀 Ejecución por sprints (resumen)

### Sprint 1 — Configuración y control básico (turtlesim)
Objetivo: Entorno operativo ROS2 y nodo para controlar `turtlesim`.

Funcionalidades claves:
- Nodo Python para dibujar el número "9" en turtlesim
- Servicios ROS para pausar/reanudar/reiniciar el dibujo
- Launch file unificado: `sprint1.launch.py`

Lanzamiento:
```bash
ros2 launch g09_prii3 sprint1.launch.py
```

Servicios disponibles:
```bash
# Pausar
ros2 service call /drawer/pause std_srvs/srv/Trigger "{}"

# Reanudar
ros2 service call /drawer/resume std_srvs/srv/Trigger "{}"

# Reiniciar
ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"
```

### Sprint 2 — Movimiento autónomo y evitación de obstáculos
Objetivo: Control de movimiento y algoritmos de evitación usando LIDAR. Soporta robot real (JetBot) y simulación (TurtleBot3/Gazebo).

Lanzamientos de ejemplo:
```bash
# JetBot (real)
ros2 launch g09_prii3 jetbot_drawer.launch.py

# Simulación (Gazebo)
ros2 launch g09_prii3 drawer_number_gazebo.launch.py

# Evitación simple
ros2 launch g09_prii3 obstacle_avoidance_simple.launch.py

# Evitación avanzada
ros2 launch g09_prii3 obstacle_avoidance_advanced.launch.py
```

#### Campos Potenciales (navegación reactiva)
Concepto: combinación de campo atractivo (hacia goal) y repulsivo (obstáculos LIDAR). Parámetros ajustables: `goal_x`, `goal_y`, `k_att`, `k_rep`, `d0_rep`, `max_lin_vel`, `max_ang_vel`, etc.

Ejemplo de lanzamiento:
```bash
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.80 goal_y:=-0.03
```

Perfil puerta‑rápida:
```bash
ros2 launch g09_prii3 Potential_Fields_door_fast.launch.py goal_x:=1.80 goal_y:=-0.03
```

### Sprint 3 — SLAM y navegación con ArUcos
Objetivo: MAPEO (Cartographer), navegación con Nav2 y detección de ArUco para comportamientos dependientes de marcadores.

Ejemplo de ejecución (simulación completa):
```bash
# Terminal 1: mundo Gazebo
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/universitat_agusti/tercero/proyecto/g09_prii3_ws/mundos_gazebo/ar_tags/model
ros2 launch g09_prii3 f1l3_world.launch.py

# Terminal 2: Navigation2 (use_sim_time true)
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/mapa_f1l3_gazebo.yaml

# Terminal 3: Nodo que combina navegación y detección ArUco
ros2 launch g09_prii3 aruco_nav_launch.py
```

Rutas predefinidas por ID de ArUco (ejemplo):
- ID 5 → `pasillo3izq`
- ID 17 → `pasillo3der`

Para añadir/editar rutas, modifica el diccionario `self.routes` en el nodo `aruco_nav_node`.

---

## 🧭 SLAM y mapas (generación y uso)

Proceso resumido para mapear con Cartographer y guardar mapa:
```bash
# Limpiar procesos de Gazebo
pkill -f gazebo || true

# Lanzar mundo F1L3
export TURTLEBOT3_MODEL=waffle
gazebo --verbose install/g09_prii3/share/g09_prii3/worlds/f1l3.world

# Ejecutar Cartographer
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Teleop para mapear
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard

# Guardar mapa
ros2 run nav2_map_server map_saver_cli -f maps/mapa_f1l3_gazebo
```

Conversión simple Gazebo → RViz usada internamente por nuestro nodo de soporte:
```text
x_mapa = x_gazebo + 2.0
y_mapa = y_gazebo + 0.5
```

Coordenadas de spawn oficiales en F1L3 (ejemplo):
- Posición: (-4.527328, -2.852645, 0.008854)
- Orientación: (0.001529, -0.008578, 0.008052) (radianes)

---

## 🛠️ Solución de problemas comunes

- Problema: "Package 'g09_prii3' not found"
   ```bash
   cd g09_prii3_ws
   rm -rf build/ install/ log/
   colcon build --symlink-install
   source install/setup.bash
   ros2 pkg list | grep g09_prii3
   ```

- Problema: robot no responde a `/cmd_vel` (JetBot)
   ```bash
   # Verificar tópicos
   ros2 topic list | grep cmd_vel

   # Si no está activo, en otra terminal:
   ros2 launch jetbot_pro_ros2 jetbot.py
   ```

- Problema: conflictos de overlay entre workspaces
   ```bash
   unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH
   source /opt/ros/foxy/setup.bash
   source ~/g09_prii3_ws/install/setup.bash
   ```

- Problema: Gazebo no encuentra modelos
   ```bash
   export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$(ros2 pkg prefix g09_prii3)/share/g09_prii3/models"
   gz model --list
   ```

---

## 📊 Especificaciones técnicas (resumen)

- Robots físicos: NVIDIA JetBot (LIDAR + cámara)
- Simulación: TurtleBot3 (Burger / Waffle) en Gazebo
- Sensores: LIDAR 2D, cámara RGB, odometría por encoders
- Algoritmos: SLAM (Cartographer), Nav2, campos potenciales, detección ArUco

Parámetros por defecto relevantes:
- Tolerancia llegada: `0.10` m
- Velocidad máxima lineal: `0.30` m/s
- Velocidad máxima angular: `1.0` rad/s
- Marcadores ArUco: tamaño 5x5 bits
- Distancia óptima detección ArUco: `0.5 - 3.0` m

---

## 🔗 Enlaces rápidos

- Launch files: `launch/` (varios según funcionalidad)
- Código fuente: `src/g09_prii3/`
- Modelos y recursos: `resource/`, `mundos_gazebo/`

---

## 👥 Metodología de desarrollo

Trabajamos con SCRUM:
- Product Owner: profesorado
- Development Team: estudiantes (Grupo 09)
- Scrum Master: profesorado

Cada sprint incluye demo funcional, revisión técnica y consolidación en `main`.

---

<div align="center">
Desarrollado por el Grupo 09 - PRII3
Autor principal: Agustí Ferrandiz

<img alt="status" src="https://img.shields.io/badge/status-active-brightgreen" />
 <img alt="license" src="https://img.shields.io/badge/license-MIT-blue" />
 <img alt="ROS2" src="https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble-blue?logo=ros" />
</div>
