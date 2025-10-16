<sub>README optimizado para verse en GitHub</sub>

# PRII3 · Grupo 09 – Workspace ROS 2

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
│  └─ sprint2.launch.py
├─ resource/
│  └─ g09_prii3
├─ src/
│  └─ g09_prii3/
│     ├─ __init__.py
│     ├─ prii3_turtlesim_node.py
│     ├─ drawer_number_gazebo.py
│     └─ jetbot_drawer_node.py
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
- [launch/sprint2.launch.py](launch/sprint2.launch.py)  
- [src/g09_prii3/__init__.py](src/g09_prii3/__init__.py)  
- [src/g09_prii3/prii3_turtlesim_node.py](src/g09_prii3/prii3_turtlesim_node.py) — clase principal: [`TurtleNine`](src/g09_prii3/prii3_turtlesim_node.py)  
- [src/g09_prii3/jetbot_drawer_node.py](src/g09_prii3/jetbot_drawer_node.py) — clase principal: [`JetbotDrawer`](src/g09_prii3/jetbot_drawer_node.py)  
- [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py) — clase principal: [`TurtlebotNine`](src/g09_prii3/drawer_number_gazebo.py)  
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
ros2 launch g09_prii3 sprint2.launch.py
```
Archivo de launch: [launch/sprint2.launch.py](launch/sprint2.launch.py)

---

Ejercicios — Gazebo / TurtleBot3
---
Nodo: `drawer_number` (simulación / TB3)  
Archivo: [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py) — clase [`TurtlebotNine`](src/g09_prii3/drawer_number_gazebo.py)

Descripción
- Publica en `/cmd_vel` y termina al completar la secuencia. Diseñado para pruebas en simulador (Gazebo / TurtleBot3).

Lanzar:
```bash
ros2 launch g09_prii3 sprint2.launch.py
```
Archivo de launch: [launch/sprint2.launch.py](launch/sprint2.launch.py)

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
- Launch: [launch/sprint1.launch.py](launch/sprint1.launch.py), [launch/sprint2.launch.py](launch/sprint2.launch.py)  
- Código fuente: [src/g09_prii3/prii3_turtlesim_node.py](src/g09_prii3/prii3_turtlesim_node.py), [src/g09_prii3/jetbot_drawer_node.py](src/g09_prii3/jetbot_drawer_node.py), [src/g09_prii3/drawer_number_gazebo.py](src/g09_prii3/drawer_number_gazebo.py)

---

<center>
**Autor:** Agustí Ferrandiz

<img alt="status" src="https://img.shields.io/badge/status-active-brightgreen" /> 
<img alt="license" src="https://img.shields.io/badge/license-MIT-blue" />
</center>

