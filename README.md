PRII3 – Grupo 09 – Workspace ROS2 (nodos en src/)

Este repositorio contiene un workspace ROS 2 organizado para crecer por sprints, con paquetes modulares bajo `src/` y lanzadores comunes en un paquete de bringup.

Probado en Ubuntu 20.04/22.04 con ROS 2 (Foxy/Humble). Ajusta los nombres de distro en los ejemplos si es necesario.

Requisitos

- ROS 2 instalado y configurado (Foxy/Humble). Ejemplo para turtlesim en Foxy:

    sudo apt update && sudo apt install -y ros-foxy-turtlesim

Estructura actual (paquete único ament_python):

g09_prii3_ws/
├─ launch/
│  ├─ sprint1.launch.py       # turtlesim + prii3_turtlesim_node
│  └─ sprint2.launch.py       # drawer_number (TurtleBot pattern)
├─ src/
│  └─ g09_prii3/
│     ├─ prii3_turtlesim_node.py  # Nodo sprint 1
│     └─ drawer_number.py         # Nodo sprint 2
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ resource/g09_prii3
├─ .gitignore
└─ README.md

Construcción

1) Cargar el entorno de ROS 2 en la terminal actual

source /opt/ros/$ROS_DISTRO/setup.bash

2) Compilar desde la raíz del workspace

colcon build

3) Cargar el overlay del workspace

source install/setup.bash

Ejecución por sprint

- Sprint 1 (turtlesim + nodo que dibuja el 9):

    ros2 launch g09_prii3 sprint1.launch.py

    Servicios útiles:
    ros2 service call /drawer/pause   std_srvs/srv/Trigger "{}"
    ros2 service call /drawer/resume  std_srvs/srv/Trigger "{}"
    ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"

- Sprint 2 (nodo para TurtleBot):

    ros2 launch g09_prii3 sprint2.launch.py

Notas

- build/, install/ y log/ están ignorados en git. Compila en limpio si cambias de entorno.
- Para añadir un nuevo sprint: agrega un nuevo script a `src/g09_prii3/` y crea su correspondiente `launch/sprintX.launch.py`.

