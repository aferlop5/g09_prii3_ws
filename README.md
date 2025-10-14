PRII3 – Grupo 09 – Workspace ROS 2 (nodos en src/)

Este repositorio contiene un workspace ROS 2 organizado para crecer por sprints, con paquetes modulares bajo `src/` y lanzadores en `launch/`.

Probado en Ubuntu 20.04/22.04 con ROS 2 (Foxy/Humble). Ajusta los nombres de distro en los ejemplos si es necesario.

Requisitos

- ROS 2 instalado y configurado (Foxy/Humble). Ejemplo para turtlesim en Foxy:

```bash
sudo apt update && sudo apt install -y ros-foxy-turtlesim
```

Estructura actual (paquete único ament_python):

```
g09_prii3_ws/
├─ launch/
│  ├─ sprint1.launch.py               # turtlesim + prii3_turtlesim_node
│  └─ drawer_number_gazebo.launch.py  # lanza el nodo drawer_number_gazebo
├─ src/
│  └─ g09_prii3/
│     ├─ prii3_turtlesim_node.py      # Nodo sprint 1 (turtlesim)
│     └─ drawer_number_gazebo.py      # Nodo Gazebo: publica en /cmd_vel
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ resource/g09_prii3
├─ .gitignore
└─ README.md
```

Construcción

1) Cargar el entorno de ROS 2 en la terminal actual

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

2) Compilar desde la raíz del workspace

```bash
colcon build
```

3) Cargar el overlay del workspace

```bash
source install/setup.bash
```

Ejecución

- Sprint 1 (turtlesim + nodo que dibuja el 9):

```bash
ros2 launch g09_prii3 sprint1.launch.py
```

Servicios útiles (Sprint 1):

```bash
ros2 service call /drawer/pause   std_srvs/srv/Trigger "{}"
ros2 service call /drawer/resume  std_srvs/srv/Trigger "{}"
ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"
```

- Nodo Gazebo (publica en /cmd_vel):

Ejecutar directamente el nodo:

```bash
ros2 run g09_prii3 drawer_number_gazebo
```

O lanzarlo con su launch dedicado:

```bash
ros2 launch g09_prii3 drawer_number_gazebo.launch.py
```

Notas

- build/, install/ y log/ están ignorados en git. Compila en limpio si cambias de entorno.
- Si ves avisos sobre COLCON_PREFIX_PATH/AMENT_PREFIX_PATH apuntando a rutas antiguas, en una sesión puedes hacer:

```bash
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH
source install/setup.bash
```

- Para añadir un nuevo sprint: agrega un nuevo script a `src/g09_prii3/` y crea su correspondiente `launch/sprintX.launch.py`.

