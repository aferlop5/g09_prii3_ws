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
├─ launch/
│  ├─ sprint1.launch.py
│  ├─ jetbot_drawer.launch.py
│  ├─ obstacle_avoidance_simple.launch.py
│  └─ obstacle_avoidance_advanced.launch.py
├─ resource/
│  └─ g09_prii3
├─ src/
│  └─ g09_prii3/
│     ├─ __init__.py
│     ├─ prii3_turtlesim_node.py
│     ├─ drawer_number_gazebo.py
│     ├─ jetbot_drawer_node.py
│     └─ obstacle_avoidance_node.py
├─ package.xml
├─ setup.py
├─ setup.cfg
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
- [launch/obstacle_avoidance_simple.launch.py](launch/obstacle_avoidance_simple.launch.py)  
- [launch/obstacle_avoidance_advanced.launch.py](launch/obstacle_avoidance_advanced.launch.py)  
- [launch/Potential_Fields.launch.py](launch/Potential_Fields.launch.py)  
- [src/g09_prii3/__init__.py](src/g09_prii3/__init__.py)  
- [src/g09_prii3/prii3_turtlesim_node.py](src/g09_prii3/prii3_turtlesim_node.py) — clase principal: [`TurtleNine`](src/g09_prii3/prii3_turtlesim_node.py)  
- [src/g09_prii3/jetbot_drawer_node.py](src/g09_prii3/jetbot_drawer_node.py) — clase principal: [`JetbotDrawer`](src/g09_prii3/jetbot_drawer_node.py)  
- [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py) — clase principal: [`TurtlebotNine`](src/g09_prii3/drawer_number_gazebo.py)  
- [src/g09_prii3/obstacle_avoidance_node.py](src/g09_prii3/obstacle_avoidance_node.py) — clase principal: [`JetbotAvoider`](src/g09_prii3/obstacle_avoidance_node.py)  
- [src/g09_prii3/Potential_Fields.py](src/g09_prii3/Potential_Fields.py) — clase principal: [`JetbotAvoider`](src/g09_prii3/Potential_Fields.py)  
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
Nodo: `jetbot_drawer`  
Archivo: [src/g09_prii3/jetbot_drawer_node.py](src/g09_prii3/jetbot_drawer_node.py) — clase [`JetbotDrawer`](src/g09_prii3/jetbot_drawer_node.py)

Descripción
- Publica comandos en `/cmd_vel` para que el stack de JetBot (o un controlador compatible) reproduzca la trayectoria "09". Parámetros de velocidad declarables (`linear_speed`, `angular_speed`).

Ejecución (si está instalado con setup.py)
```bash
ros2 run g09_prii3 jetbot_drawer
```

Servicios asociados
```bash
ros2 service call /jetbot_drawer/pause   std_srvs/srv/Trigger {}
ros2 service call /jetbot_drawer/resume  std_srvs/srv/Trigger {}
ros2 service call /jetbot_drawer/restart std_srvs/srv/Trigger {}
```

También puedes lanzar con:
```bash
ros2 launch g09_prii3 jetbot_drawer.launch.py
```
Archivo de launch: [launch/jetbot_drawer.launch.py](launch/jetbot_drawer.launch.py)

Evitación de obstáculos con Lidar (Sprint 2)
---
Nodo: `jetbot_obstacle_avoidance`  
Archivo: [src/g09_prii3/obstacle_avoidance_node.py](src/g09_prii3/obstacle_avoidance_node.py) — clase [`JetbotAvoider`](src/g09_prii3/obstacle_avoidance_node.py)

Descripción
- Publica velocidad en `/cmd_vel` y se suscribe a `/scan` (Lidar) para detectar obstáculos.
- Modo `simple`: se detiene si un obstáculo está más cerca que un umbral (por defecto 0.3 m) y reanuda cuando despeja.
- Modo `advanced`: esquiva el obstáculo girando hacia el lado con mayor despeje mientras avanza lentamente.
- Logs informativos: "Avanzando", "obstáculo detectado — deteniendo", "evitando obstáculo", etc.

Parámetros
- `linear_speed` (float, default 0.15): velocidad lineal (m/s)
- `angular_speed` (float, default 0.6): velocidad angular (rad/s)
- `obstacle_threshold` (float, default 0.3): umbral de detección frontal (m)
- `avoidance_mode` (string, default `simple`): `simple` | `advanced`

Ejecución directa
```bash
ros2 run g09_prii3 jetbot_obstacle_avoidance
```

Launch dedicados por modo (sin parámetros)
```bash
# Modo simple (se detiene ante el obstáculo y reanuda cuando despeja)
ros2 launch g09_prii3 obstacle_avoidance_simple.launch.py

# Modo avanzado (evita el obstáculo girando y avanzando lentamente)
ros2 launch g09_prii3 obstacle_avoidance_advanced.launch.py
```
Archivos de launch:
- [launch/obstacle_avoidance_simple.launch.py](launch/obstacle_avoidance_simple.launch.py)
- [launch/obstacle_avoidance_advanced.launch.py](launch/obstacle_avoidance_advanced.launch.py)

Nota: para el dibujo del "09" usa `jetbot_drawer.launch.py`. Para evitación de obstáculos usa uno de los launch dedicados indicados arriba.


Ejercicios — Gazebo / TurtleBot3
---
Nodo: `drawer_number` (simulación / TB3)  
Archivo: [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py) — clase [`TurtlebotNine`](src/g09_prii3/drawer_number_gazebo.py)

Descripción
- Publica en `/cmd_vel` y termina al completar la secuencia. Diseñado para pruebas en simulador (Gazebo / TurtleBot3).

Ejecución directa:
```bash
ros2 run g09_prii3 drawer_number_gazebo
```

---

Navegación — Campos Potenciales (ejercicio)
---
Nodo: `jetbot_potential_fields`  
Archivo: [src/g09_prii3/Potential_Fields.py](src/g09_prii3/Potential_Fields.py) — clase [`JetbotAvoider`](src/g09_prii3/Potential_Fields.py)

Descripción
- Navegación hacia una meta en el marco global utilizando campos potenciales con evitación por LIDAR.
- Algoritmo mejorado para espacios pequeños con muchos obstáculos: suavizado de heading, empuje mínimo hacia delante, y un modo de seguimiento de pared (wall‑follow) automático para escapar de mínimos locales.

Lanzar (con objetivo en metros en `odom`)
```bash
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03
```

Modos de lanzamiento rápidos (según entorno)
```bash
# 1) Espacios pequeños / muy obstaculizados (wall-follow más activo)
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args \
  -p min_linear_speed:=0.08 -p heading_cruise_angle:=0.6 -p front_cruise_clearance:=0.6 \
  -p wall_follow_enter_front:=0.30 -p wall_follow_exit_front:=0.70 \
  -p wall_distance:=0.40 -p v_wall:=0.10 -p w_wall_gain:=1.0 \
  -p k_att:=1.0 -p k_rep:=0.5 -p repulsive_radius:=0.9 -p goal_tolerance:=0.08

# 2) Pasillo/corredor (seguir pared derecha)
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args \
  -p wall_follow_side:=right -p wall_distance:=0.35 \
  -p wall_follow_enter_front:=0.35 -p wall_follow_exit_front:=0.80 \
  -p min_linear_speed:=0.07 -p heading_cruise_angle:=0.5 -p v_wall:=0.12 -p w_wall_gain:=1.2 \
  -p k_rep:=0.4 -p repulsive_radius:=0.8

#    (seguir pared izquierda)
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args -p wall_follow_side:=left

# 3) Área abierta (pocos obstáculos, menos repulsivo)
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args \
  -p k_rep:=0.3 -p repulsive_radius:=0.7 \
  -p heading_cruise_angle:=0.9 -p front_cruise_clearance:=0.4 -p min_linear_speed:=0.06 \
  -p progress_timeout:=7.0

# 4) Aproximación precisa a la meta (más fino cerca del objetivo)
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args \
  -p max_linear_speed:=0.18 -p max_angular_speed:=0.9 \
  -p goal_tolerance:=0.05 -p repulsive_radius:=0.8 -p k_rep:=0.45 \
  -p v_wall:=0.08

# 5) Anti-atascos agresivo (salir rápido de mínimos locales)
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args \
  -p progress_timeout:=3.0 -p progress_epsilon:=0.03 \
  -p wall_follow_enter_front:=0.32 -p wall_follow_exit_front:=0.70 \
  -p w_wall_gain:=1.2
```

Parámetros principales
- `goal_x`, `goal_y` (float): objetivo en marco global (por defecto 1.0, 0.0)
- `goal_tolerance` (float, default 0.10): tolerancia de llegada (m)
- `global_frame` (string, default `odom`) y `base_frame` (string, default `base_link`)
- `max_linear_speed` (float, default 0.25) y `max_angular_speed` (float, default 1.2)
- `min_linear_speed` (float, default 0.06): empuje mínimo hacia delante si hay despeje
- `heading_cruise_angle` (float rad, default 0.7): umbral angular para aplicar `min_linear_speed`
- `front_cruise_clearance` (float m, default 0.5): despeje frontal requerido para `min_linear_speed`
- `heading_smoothing_alpha` (float, default 0.3): suavizado exponencial del heading para reducir zig‑zag
- `wall_follow_enter_front` (float m, default 0.35): entra a seguimiento de pared si el despeje frontal cae por debajo
- `wall_follow_exit_front` (float m, default 0.7): sale del seguimiento de pared cuando hay despeje
- `wall_follow_side` (string, default `auto`): `auto` | `left` | `right`
- `wall_distance` (float m, default 0.35): distancia objetivo a la pared en seguimiento
- `v_wall` (float m/s, default 0.10): velocidad lineal máxima en modo pared
- `w_wall_gain` (float, default 1.0): ganancia de corrección angular manteniendo distancia a pared
- `progress_timeout` (float s, default 5.0): tiempo sin mejorar distancia a meta para activar pared
- `progress_epsilon` (float m, default 0.05): umbral de mejora mínima
- `k_att` (float, default 1.0): ganancia atractiva a la meta
- `k_rep` (float, default 0.4): ganancia repulsiva de obstáculos
- `repulsive_radius` (float m, default 0.8): radio de influencia de obstáculos
- `min_range_clip` (float m, default 0.05): recorte inferior de lecturas LIDAR
- `front_stop_threshold` (float m, default 0.25): parada dura por seguridad
- `search_turn_speed` (float rad/s, default 0.6): giro suave al esperar TF

Ejemplo de ajuste (Gazebo)
```bash
ros2 launch g09_prii3 Potential_Fields.launch.py goal_x:=1.8 goal_y:=-0.03 \
  --ros-args \
  -p max_linear_speed:=0.25 -p max_angular_speed:=1.2 \
  -p min_linear_speed:=0.06 -p heading_cruise_angle:=0.7 -p front_cruise_clearance:=0.6 \
  -p k_att:=1.0 -p k_rep:=0.4 -p repulsive_radius:=0.9 -p goal_tolerance:=0.08
```

Requisitos de topics/TF
- LIDAR publicando en `/scan`.
- TF disponible de `odom -> base_link`.

Archivo de launch: [launch/Potential_Fields.launch.py](launch/Potential_Fields.launch.py)

Consejos
- En espacios muy pequeños, activa el seguimiento de pared más agresivo reduciendo `wall_follow_enter_front` (p.ej. 0.30) y aumentando `wall_distance` (0.40–0.45). 
- Si el robot no avanza y sólo gira, incrementa `min_linear_speed` a 0.08–0.10 y/o `front_cruise_clearance` a 0.6–0.8 m.
- Si se acerca demasiado a obstáculos, sube `k_rep` o `repulsive_radius`.
- Si hay muchas oscilaciones, reduce `heading_smoothing_alpha` (p.ej. 0.2).

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

<center>
**Autor:** Agustí Ferrandiz

<img alt="status" src="https://img.shields.io/badge/status-active-brightgreen" /> 
<img alt="license" src="https://img.shields.io/badge/license-MIT-blue" />
</center>

