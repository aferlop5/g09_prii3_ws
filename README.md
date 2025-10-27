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
│  ├─ obstacle_avoidance_advanced.launch.py
│  ├─ Potential_Fields.launch.py
│  └─ Potential_Fields_door_fast.launch.py
├─ resource/
│  └─ g09_prii3
├─ src/
│  └─ g09_prii3/
│     ├─ __init__.py
│     ├─ prii3_turtlesim_node.py
│     ├─ drawer_number_gazebo.py
│     ├─ jetbot_drawer_node.py
│     ├─ obstacle_avoidance_node.py
│     └─ Potential_Fields.py
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

Ejercicio de ML ligero — TurtleBot3 + Gazebo
---
Nodo: `ml_test`  
Archivo: [src/g09_prii3/ml_test.py](src/g09_prii3/ml_test.py)

¿Qué hace?
- Entrena/ejecuta un agente de Aprendizaje por Refuerzo ligero (PPO/SAC, stable-baselines3) para navegar hacia una meta fija con TurtleBot3 en Gazebo.
- Observación = LIDAR downsampleado + vector objetivo relativo (dx, dy) en el marco del robot.
- Acción = [velocidad lineal, velocidad angular] publicadas en `/cmd_vel`.
- Recompensas:
  - +10 si llega al objetivo (el robot se detiene al llegar)
  - -1 si colisiona (LIDAR por debajo del umbral)
  - -0.05 si se queda quieto
  - +1 cuando se acerca a la meta respecto al paso anterior
- Reinicio automático: si no llega en 20 s, termina el episodio y reinicia.
- Persistencia: guarda el modelo por meta (coordenadas) en `~/.g09_prii3/models/` para recordar rutas en ejecuciones futuras.

Requisitos Python (instalar una vez)
- Para ROS 2 Foxy (Python 3.8): usa versiones compatibles y ligeras
```bash
# PyTorch CPU y dependencias compatibles con Python 3.8
pip install --user "torch==1.13.1+cpu" --extra-index-url https://download.pytorch.org/whl/cpu

# Estable y compatible con Gym clásico
pip install --user "stable-baselines3==1.8.0" "gym==0.21.0" cloudpickle==2.2.1

# Opcional (métricas y utilidades)
pip install --user tensorboard==2.14.0 tqdm rich
```
Nota: Si previamente instalaste versiones más nuevas y hay conflictos, puedes limpiar con `pip uninstall` de los paquetes problemáticos (por ejemplo `gymnasium`, `stable-baselines3` sin versión, etc.).

Lanzar todo (Gazebo + agente ML)
```bash
# 1) Compila y sourcea este workspace si no lo has hecho
colcon build --symlink-install
source install/setup.bash

# 2) Lanza el mundo de TurtleBot3 y el nodo de RL
ros2 launch g09_prii3 ml_test.launch.py goal_x:=1.0 goal_y:=0.0 total_timesteps:=10000 algorithm:=PPO
```

Notas
- El launch exporta automáticamente `TURTLEBOT3_MODEL=burger` y abre `turtlebot3_world.launch.py`.
- Puedes cambiar la meta con `goal_x`, `goal_y` (en metros, frame `odom`).
- El primer entrenamiento puede tardar; el modelo se guarda en `~/.g09_prii3/models/ppo_tb3_goal_<x>_<y>.zip` y se recarga si lanzas de nuevo con la misma meta.
- Si no tienes `turtlebot3_gazebo` instalado, instálalo según tu distro ROS 2 (p. ej. `sudo apt install ros-foxy-turtlebot3-gazebo`).

Ejecutar solo el nodo (si ya tienes Gazebo y TB3 corriendo)
```bash
ros2 run g09_prii3 ml_test --ros-args -p goal_x:=1.0 -p goal_y:=0.0 -p total_timesteps:=5000 -p algorithm:=PPO
```

Solución de problemas
- Error de importación SB3/torch/gymnasium: instala los requisitos Python indicados arriba.
- No arranca Gazebo: comprueba que el paquete `turtlebot3_gazebo` está instalado y que tu shell está sourceada (`source /opt/ros/<distro>/setup.bash`).
- El robot no se mueve: verifica que Gazebo está corriendo, que `/scan` y `/odom` publican, y que no hay colisiones inmediatas.


<center>
**Autor:** Agustí Ferrandiz

<img alt="status" src="https://img.shields.io/badge/status-active-brightgreen" /> 
<img alt="license" src="https://img.shields.io/badge/license-MIT-blue" />
</center>

