<sub>README optimizado para verse en GitHub</sub>

# PRII3 · Grupo 09 – Workspace ROS2

<div align="center">
  <!-- Badges centrados -->
  <img alt="ROS2" src="https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble-blue?logo=ros2" />
  &nbsp;
  <img alt="Python" src="https://img.shields.io/badge/Python-3.8%2B-blue.svg?logo=python" />
  &nbsp;
  <img alt="Ubuntu" src="https://img.shields.io/badge/Ubuntu-20.04%2F22.04-orange?logo=ubuntu" />
</div>

---

Descripción
---
Repositorio con un único paquete raíz `g09_prii3`. Contiene nodos Python organizados por sprint y launch files para facilitar pruebas locales con turtlesim, JetBot o simuladores como Gazebo.

Estructura del workspace
---
```
g09_prii3_ws/
├─ aruco/
├─ build/
├─ calibracion/
├─ install/
├─ launch/
│  ├─ sprint1.launch.py
│  ├─ jetbot_drawer.launch.py
│  ├─ drawer_number_gazebo.launch.py
│  ├─ obstacle_avoidance_simple.launch.py
│  ├─ obstacle_avoidance_advanced.launch.py
│  ├─ Potential_Fields.launch.py
│  ├─ Potential_Fields_door_fast.launch.py
│  └─ rviz_predefinido_node.launch.py
├─ log/
├─ maps/
├─ resource/
│  └─ g09_prii3
├─ src/
│  └─ g09_prii3/
│     ├─ __init__.py
│     ├─ prii3_turtlesim_node.py
│     ├─ drawer_number_gazebo.py
│     ├─ jetbot_drawer_node.py
│     ├─ obstacle_avoidance_node.py
│     ├─ Potential_Fields.py
│     └─ rviz_predefinido_node.py
├─ setup.cfg
├─ setup.py
├─ .gitignore
└─ README.md
```

Archivos clave (abre con un clic)
---
- [package.xml](package.xml)  
- [setup.py](setup.py)  
- [setup.cfg](setup.cfg)  
- [.gitignore](.gitignore)  
- [resource/g09_prii3](resource/g09_prii3)  
- [launch/sprint1.launch.py](launch/sprint1.launch.py)  
- [launch/jetbot_drawer.launch.py](launch/jetbot_drawer.launch.py)  
- [launch/drawer_number_gazebo.launch.py](launch/drawer_number_gazebo.launch.py)  
- [launch/obstacle_avoidance_simple.launch.py](launch/obstacle_avoidance_simple.launch.py)  
- [launch/obstacle_avoidance_advanced.launch.py](launch/obstacle_avoidance_advanced.launch.py)  
- [launch/Potential_Fields.launch.py](launch/Potential_Fields.launch.py)  
- [launch/Potential_Fields_door_fast.launch.py](launch/Potential_Fields_door_fast.launch.py)  
- [src/g09_prii3/__init__.py](src/g09_prii3/__init__.py)  
- [src/g09_prii3/prii3_turtlesim_node.py](src/g09_prii3/prii3_turtlesim_node.py) — clase principal: [`TurtleNine`](src/g09_prii3/prii3_turtlesim_node.py)  
- [src/g09_prii3/jetbot_drawer_node.py](src/g09_prii3/jetbot_drawer_node.py) — clase principal: [`JetbotDrawer`](src/g09_prii3/jetbot_drawer_node.py)  
- [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py) — clase principal: [`TurtlebotNine`](src/g09_prii3/drawer_number_gazebo.py)  
- [src/g09_prii3/obstacle_avoidance_node.py](src/g09_prii3/obstacle_avoidance_node.py) — clase principal: [`JetbotAvoider`](src/g09_prii3/obstacle_avoidance_node.py)  
- [src/g09_prii3/Potential_Fields.py](src/g09_prii3/Potential_Fields.py) — clase principal: [`PotentialFieldsNavigator`](src/g09_prii3/Potential_Fields.py)  
- [.vscode/settings.json](.vscode/settings.json)

---

Requisitos
---
- ROS 2 (Foxy / Humble) instalado y configurado.
- Python 3.8+.
- Opcional: turtlesim, JetBot stack o simulador (Gazebo / TurtleBot3).

Instalación rápida (ejemplo para turtlesim en Foxy)
```bash
sudo apt update
sudo apt install -y ros-foxy-turtlesim
```

Instalación completa de OpenCV (contrib) en Ubuntu 20.04
```bash
# 1) Actualiza sistema
sudo apt update && sudo apt upgrade -y

# 2) Instala dependencias del sistema
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev \
                 libavcodec-dev libavformat-dev libswscale-dev \
                 libv4l-dev libxvidcore-dev libx264-dev \
                 libgtk-3-dev libatlas-base-dev gfortran

# 3) Actualiza pip
sudo python3 -m pip install --upgrade pip

# 4) Instala OpenCV con módulos extra (incluye ArUco) y NumPy
sudo python3 -m pip install opencv-contrib-python numpy
```

Verifica instalación
```bash
python3 -c "import cv2; print(cv2.__version__)"
```
Debería mostrar una versión similar a 4.12.0.

---

Cómo ejecutar (rápido)
---
1. Construye e instala el paquete (en un workspace ROS 2 estándar):
```bash
colcon build
source install/setup.bash
```

2. Opciones de lanzamiento / ejecución según el tema (ver secciones abajo).

---

Requisito previo para mover el robot (JetBot)
---
Antes de lanzar cualquier nodo que haga que el robot se mueva (por ejemplo, `jetbot_drawer`, evitación de obstáculos o navegación), asegúrate de tener ejecutado en OTRA terminal el stack del JetBot:

```bash
ros2 launch jetbot_pro_ros2 jetbot.py
```

Déjalo corriendo mientras lanzas tus nodos desde otra terminal. Sin este proceso activo, los comandos de velocidad (`/cmd_vel`) no llegarán al controlador del robot.

---

Solución a "Package 'g09_prii3' not found"
---
Si al lanzar ves un error como:

```
Package 'g09_prii3' not found
```

es porque tu shell no ha cargado el overlay de este workspace. Arréglalo así:

1) Construye y fuentea este workspace en la terminal donde vas a lanzar
```bash
cd ~/agus/g09_prii3_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash   # IMPORTANTE: hacer esto en cada terminal nueva
```

2) Verifica que ROS lo ve
```bash
ros2 pkg list | grep g09_prii3 || true
ros2 pkg prefix g09_prii3
```
Deberías ver la ruta de `install/` de este workspace.

3) Lanza de nuevo
```bash
ros2 launch g09_prii3 obstacle_avoidance_simple.launch.py
```

Notas importantes (overlay y .bashrc)
- El orden de sourcing importa. Primero el sistema base y después tus overlays (el último gana):
```bash
source /opt/ros/foxy/setup.bash
source ~/agus/g09_prii3_ws/install/setup.bash
```
- Para hacerlo persistente, añade esas líneas al final de tu `~/.bashrc` y abre una nueva terminal.
- Si ya estás sourceando otro workspace (p.ej. `~/jetbot_ws/install/setup.bash`), asegúrate de sourcear ESTE después para que se vea `g09_prii3`.

Alternativa rápida (sin paquete en índice)
```bash
ros2 launch ~/agus/g09_prii3_ws/launch/obstacle_avoidance_simple.launch.py
```
Puedes lanzar por ruta absoluta del archivo de launch mientras verificas el entorno, aunque lo recomendable es el método por paquete.

---

Sprint 1 — Turtlesim
---
Nodo: `prii3_turtlesim_node`  
Archivo: [src/g09_prii3/prii3_turtlesim_node.py](src/g09_prii3/prii3_turtlesim_node.py) — clase [`TurtleNine`](src/g09_prii3/prii3_turtlesim_node.py)

Descripción
- Dibuja el número "9" en el simulador `turtlesim` y expone servicios para controlar la ejecución.

Lanzar (turtlesim + nodo)
```bash
ros2 launch g09_prii3 sprint1.launch.py
```

Servicios útiles
```bash
ros2 service call /drawer/pause   std_srvs/srv/Trigger "{}"
ros2 service call /drawer/resume  std_srvs/srv/Trigger "{}"
ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"
```

---

Sprint 2 — JetBot
---
Qué lanza el dibujo en cada entorno y por qué tenemos dos nodos/launch separados.

- Robot real (JetBot)
  - Launch: `launch/jetbot_drawer.launch.py`
  - Conexión con robot: `ssh -X jetbot@172.16.190.147`
  - Ejecutable: `jetbot_drawer`
  - Ejecuta el nodo que publica `/cmd_vel` para dibujar “09” en el hardware real.
  - Asume que el stack del JetBot ya está corriendo y escuchando `/cmd_vel`.
  - Cómo lanzarlo:
    ```bash
    ros2 launch g09_prii3 jetbot_drawer.launch.py
    ```

- Simulación (Gazebo con TurtleBot3 burger)
  - Launch: `launch/drawer_number_gazebo.launch.py`
  - Ejecutable: `drawer_number_gazebo`
  - Este launch configura `TURTLEBOT3_MODEL=burger`, abre `turtlebot3_gazebo/empty_world.launch.py` y, tras unos segundos, arranca el nodo que publica `/cmd_vel` para dibujar “9”.
  - Requiere tener instalado el paquete `turtlebot3_gazebo`.
  - Cómo lanzarlo:
    ```bash
    ros2 launch g09_prii3 drawer_number_gazebo.launch.py
    ```

Por qué dos nodos/launch (decisión de diseño)
- Especialización específica: Cada nodo está optimizado para su entorno sin código condicional complejo. El launch de Gazebo levanta el simulador y el entorno, mientras que el de JetBot asume el robot real ya está listo.
- Mantenimiento simplificado: Código más limpio y fácil de actualizar por separado. Los launch files tienen responsabilidades claras: uno para simulación (con Gazebo) y otro para el robot real (sin Gazebo).
- Configuración directa: Launch files específicos que evitan parámetros condicionales y errores. El launch de Gazebo incluye la puesta en marcha del simulador, mientras que el de JetBot se enfoca solo en el control del robot real.
- Depuración más eficiente: Problemas identificados más rápido al tener responsabilidades separadas. Al lanzar Gazebo en un launch y el robot real en otro, se aísla mejor los problemas de simulación de los del hardware.

Sprint 2 — Evitación de obstáculos
---
Qué lanzan los modos y en qué se diferencian. Mismo ejecutable, dos launch para dos comportamientos.

- Robot real (JetBot)
  - Launch (modo simple): `launch/obstacle_avoidance_simple.launch.py`
  - Launch (modo advanced): `launch/obstacle_avoidance_advanced.launch.py`
  - Ejecutable: `jetbot_obstacle_avoidance`
  - Comportamiento:
    - Simple → "collision avoidance": se para delante del obstáculo y reanuda cuando desaparece.
    - Advanced → "obstacle avoidance": esquiva el obstáculo bordeándolo sin detener la marcha.
  - Cómo lanzarlo:
    ```bash
    # Simple (para y reanuda)
    ros2 launch g09_prii3 obstacle_avoidance_simple.launch.py

    # Advanced (evita en marcha)
    ros2 launch g09_prii3 obstacle_avoidance_advanced.launch.py
    ```

- Simulación (Gazebo con TurtleBot3 burger)
  - Si Gazebo ya está abierto con un robot que publique `/scan` y escuche `/cmd_vel` (p. ej., TurtleBot3), cualquiera de los dos launch anteriores (simple o advanced) funcionará en la simulación sin cambios.
  - Requisito: tener `turtlebot3_gazebo` instalado si deseas abrir la simulación estándar de TurtleBot3.

Argumentos para usar un único nodo con dos launch files
- Parámetros configurables: el mismo nodo acepta parámetros ROS (por ejemplo, `avoidance_mode`) que cambian completamente el comportamiento entre modo simple y avanzado.
- Mantenimiento centralizado: todas las mejoras y correcciones se aplican una sola vez en un único archivo de nodo, evitando duplicación de código.
- Configuración específica: cada launch file establece parámetros diferentes (p. ej., `obstacle_threshold`, `advanced_detect_factor`, etc.) optimizados para cada modo.
- Flexibilidad operativa: permite cambiar entre comportamientos sin recompilar, solo modificando parámetros de lanzamiento.
- Consistencia garantizada: ambos modos comparten la misma lógica base de procesado LIDAR y publicación a `/cmd_vel`.
---

Navegación — Campos Potenciales (JetBot / Gazebo)
---
Nodo: `jetbot_potential_fields`  
Archivo: [src/g09_prii3/Potential_Fields.py](src/g09_prii3/Potential_Fields.py) — clase [`PotentialFieldsNavigator`](src/g09_prii3/Potential_Fields.py)

Descripción
- Navegación reactiva hacia una meta global `(goal_x, goal_y)` usando Campos Potenciales: atracción hacia la meta + repulsión por LIDAR.
- Integra odometría (`/odom`) para calcular el vector atractivo real (posición del robot en mundo → vector en el frame del robot).
- Estabilidad: suavizado low‑pass de comandos, reducción de velocidad cerca de obstáculos y ante grandes errores angulares.
- Robustez: detección de estancamiento y recuperación con estrategia combinada "spin+gap" puerta‑friendly por defecto:
  - Follow‑The‑Gap: cuando no hay progreso, detecta la mayor apertura (p. ej., una puerta) y avanza hacia su centro.
  - Si no hay hueco claro, alterna con un giro en el sitio para salir del mínimo local.
  - Al pulsar Ctrl+C, publica un `Twist(0,0)` para detener motores antes de apagar.

Lanzar (meta en metros en `odom`)
```bash
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.80 goal_y:=-0.03
```

Perfil rápido puerta‑friendly (recomendado para puertas/pasillos)
```bash
ros2 launch g09_prii3 Potential_Fields_door_fast.launch.py goal_x:=1.80 goal_y:=-0.03
```

Ejemplo real (valores usados en pruebas)
```bash
ros2 launch g09_prii3 Potential_Fields_door_fast.launch.py goal_x:=-6.357 goal_y:=-2.92
```

Parámetros principales
- `goal_x`, `goal_y` (float): objetivo en `odom`.
- `goal_tolerance` (float, default 0.10): radio de llegada en metros.
- `odom_topic` (string, default `/odom`): tópico de odometría.
- `k_att` (float, default 1.0): ganancia atractiva.
- `k_rep` (float, default 0.32): ganancia repulsiva.
- `d0_rep` (float, default 0.55): radio de influencia repulsiva.
- `max_lin_vel` (float, default 0.30), `max_ang_vel` (float, default 1.0): límites de velocidad.
- `ang_gain` (float, default 1.5), `lin_gain` (float, default 1.0): ganancias del controlador.
- `slowdown_min_scale` (float, default 0.20): factor mínimo de velocidad cerca de obstáculos.
- `front_weight_deg` (float, default 80.0): ancho del sector frontal priorizado en repulsión.
- `rep_scale_side` (float, default 0.42): peso relativo de repulsión en laterales.
- `smooth_alpha` (float, default 0.40): coeficiente de suavizado de `v`/`w` (0..1).
- `stuck_timeout` (float, default 3.0): tiempo sin progreso para activar recuperación.
- `escape_gain` (float, default 0.20): pequeña perturbación aleatoria para escapar de mínimos locales.

Parámetros de recuperación puerta‑friendly
- `use_gap_follow` (bool, default `true`): activa el seguimiento de aperturas cuando no hay progreso.
- `recovery_mode` (string, default `spin+gap`): `gap` | `spin` | `spin+gap`.
- `gap_clear_threshold` (float, default = `d0_rep`): distancia considerada libre para formar una apertura.
- `gap_min_width_deg` (float, default 12.0): anchura mínima de la apertura válida (grados).
- `gap_prefer_goal_weight` (float, default 0.6): peso [0..1] para sesgar la apertura hacia la dirección de la meta.
- `recovery_gap_duration` (float, default 3.0): tiempo de seguimiento de la apertura antes de re‑evaluar.

Tópicos
- Sub: `/scan` (sensor_msgs/LaserScan), `/odom` (nav_msgs/Odometry)
- Pub: `/cmd_vel` (geometry_msgs/Twist)

Consejos de afinado
- Más distancia frontal: sube `d0_rep` (+0.02..0.05) y/o `k_rep` (+0.02..0.05).
- Más holgura lateral al girar: sube `rep_scale_side` (0.42→0.50) y/o `front_weight_deg` (80→90).
- Menos oscilación: baja `ang_gain` o sube `smooth_alpha` (p.ej. 0.45).
- Más decisión hacia meta (en despeje): sube `lin_gain` o baja `slowdown_min_scale` con cuidado.
- Si duda en puertas: baja `gap_min_width_deg` (p.ej. 10) o `gap_prefer_goal_weight` (0.5).

Archivo de launch: [launch/Potential_Fields.launch.py](launch/Potential_Fields.launch.py)

---

Simulación SLAM y navegación con Gazebo y TurtleBot
---
Nodo: `rviz_predefinido_node`  
Archivo: [src/g09_prii3/rviz_predefinido_node.py](src/g09_prii3/rviz_predefinido_node.py) — clase `RvizPredefinidoNode`

Descripción Técnica
Este nodo resuelve la discrepancia fundamental entre los sistemas de coordenadas de Gazebo y RViz mediante la implementación de una transformada de coordenadas lineal. Cuando se genera un mapa con Cartographer, el origen (0,0) en RViz corresponde al punto de spawn inicial del robot en Gazebo, que en nuestro entorno de simulación se encuentra en las coordenadas (-2, -0.5).

Funcionamiento del Nodo
- Transformación de coordenadas: Aplica la transformada lineal x_mapa = x_gazebo + 2.0 y y_mapa = y_gazebo + 0.5 para convertir automáticamente las coordenadas especificadas en Gazebo al sistema de referencia del mapa utilizado por Nav2.
- Gestión de suscripciones: Se suscribe a los tópicos de estimación de pose (/amcl_pose) para determinar la posición inicial del robot y monitorizar su localización durante la navegación.
- Publicación de objetivos: Gestiona el envío de objetivos de navegación al stack de Nav2, transformando previamente las coordenadas de destino del sistema Gazebo al sistema mapa.
- Control de flujo: Implementa una secuencia automatizada que publica la pose inicial en (0,0) del mapa (equivalente a (-2,-0.5) en Gazebo) y posteriormente envía el goal de navegación tras los delays configurados.

Ejecución del Sistema Completo
Para ejecutar la simulación completa, abre tres terminales independientes:

Terminal 1 - Simulador Gazebo:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 - Navegación:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/agusti/universitat_agusti/tercero/proyecto/g09_prii3_ws/maps/map1.yaml
```

Terminal 3 - Nodo de Navegación Predefinida:
```bash
ros2 launch g09_prii3 rviz_predefinido_node.launch.py
```

Sistema de Coordenadas: Gazebo vs RViz

Durante el desarrollo identificamos una discrepancia fundamental entre los sistemas de coordenadas de Gazebo y RViz. Los mapas generados con Cartographer en RViz utilizan un sistema de coordenadas diferente al mundo de simulación de Gazebo, donde el origen (0,0) en RViz corresponde exactamente a la posición de spawn inicial del robot en Gazebo.

En nuestro caso específico, esta relación se define como:
- Posición en Gazebo: (-2, -0.5)
- Posición equivalente en RViz: (0, 0)

Solución Implementada

Para resolver esta discrepancia, implementamos un nodo que gestiona automáticamente la transformación de coordenadas mediante la relación:
```text
x_mapa = x_gazebo + 2.0
y_mapa = y_gazebo + 0.5
```
El nodo incluye:
- Suscripción al topic de pose estimada para determinar la posición inicial
- Capacidad para publicar en el topic de initialpose
- Transformación automática de coordenadas de Gazebo al sistema de mapa de RViz
- Integración con el action client de NavigateToPose de Nav2

Esta solución permite trabajar intuitivamente con las coordenadas visibles en Gazebo, mientras el sistema de navegación opera correctamente con las coordenadas transformadas del mapa de RViz.

Características Técnicas
- Abstracción completa: El usuario trabaja exclusivamente con coordenadas de Gazebo
- Transformación automática: Conversión transparente entre sistemas de coordenadas
- Inicialización inmediata: No requiere espera para detección de pose inicial
- Integración con Nav2: Compatibilidad total con el stack de navegación ROS2

---
Notas finales y buenas prácticas
---
- Para compilar en limpio:
```bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```
- Si ves problemas con rutas antiguas:
```bash
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH
source install/setup.bash
```
- Para añadir un nuevo sprint:
  1. Crea un script en `src/g09_prii3/` (nodo Python con función `main`).
  2. Añade entrada `console_scripts` en [setup.py](setup.py).
  3. Crea un `launch/sprintX.launch.py`.

---

Recursos y enlaces rápidos
---
- Paquete: [package.xml](package.xml)  
- Entradas ejecutables: [setup.py](setup.py)  
- Launch: [launch/sprint1.launch.py](launch/sprint1.launch.py), [launch/jetbot_drawer.launch.py](launch/jetbot_drawer.launch.py)  
- Código fuente: [src/g09_prii3/prii3_turtlesim_node.py](src/g09_prii3/prii3_turtlesim_node.py), [src/g09_prii3/jetbot_drawer_node.py](src/g09_prii3/jetbot_drawer_node.py), [src/g09_prii3/obstacle_avoidance_node.py](src/g09_prii3/obstacle_avoidance_node.py), [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py)

---
Notas
- El launch `drawer_number_gazebo.launch.py` exporta automáticamente `TURTLEBOT3_MODEL=burger` e incluye `turtlebot3_gazebo/empty_world.launch.py`.


<center>
**Autor:** Agustí Ferrandiz

<img alt="status" src="https://img.shields.io/badge/status-active-brightgreen" /> 
<img alt="license" src="https://img.shields.io/badge/license-MIT-blue" />
</center>

