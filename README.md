PRII3 – Sprint 1 · Grupo 09 · prii3_turtlesim · Gorka German y Agusti Ferrandiz

Este README explica cómo construir y ejecutar el workspace del Sprint 1 (ROS 2 Foxy) para dibujar el número 9 en turtlesim y controlarlo por servicios.

Probado en WSL2 · Ubuntu 20.04 con ROS 2 Foxy.

1) Requisitos

Ubuntu 20.04 (WSL2 o nativo) con ROS 2 Foxy instalado.
Paquetes estándar de Foxy (incluye turtlesim, std_srvs, geometry_msgs). Si faltara turtlesim:

  sudo apt update && sudo apt install -y ros-foxy-turtlesim

2) Estructura del workspace

Este ZIP contiene el workspace del sprint dentro de sprint_1/prii3_ws:

sprint_1/prii3_ws/
├─ src/
│  └─ prii3_turtlesim/
│     ├─ prii3_turtlesim/           # Código fuente (nodo Python)
│     │  └─ prii3_turtlesim_node.py
│     ├─ launch/
│     │  └─ turtlesim_launch.py     # Lanza turtlesim + nodo drawer
│     ├─ package.xml
│     ├─ setup.py
│     └─ setup.cfg
├─ (build/ install/ log/)            # Se generan al compilar
└─ README.md

3) Instalación rápida (primera vez en TU máquina)

Consejo: compila limpio en tu equipo, aunque el ZIP traiga build/, install/ o log/ de otra persona.

    0) Cargar ROS Foxy en la shell actual
    source /opt/ros/foxy/setup.bash

    1) Ir al workspace del sprint
    cd <RUTA>/g09_prii3_ws/sprint_1/prii3_ws

    2) Limpiar artefactos previos y compilar
    rm -rf build/ install/ log/
    colcon build

    3) Cargar el overlay del workspace
    source install/setup.bash

IMPORTANTE: Cada vez que abras una terminal nueva: ejecuta los dos source anteriores (Foxy + install/setup.bash).

4) Ejecución

Lanza todo (turtlesim + nodo que dibuja el 9) con un único launch:

    En el directorio del workspace, con los dos source hechos

    ros2 launch prii3_turtlesim turtlesim_launch.py

Se abrirá la ventana de turtlesim.
El nodo prii3_turtlesim_node.py empezará a mover la tortuga para dibujar el 9.

5) Servicios disponibles

En otra terminal (repitiendo los dos source), puedes controlar el dibujo:

    Pausa donde esté:

    ros2 service call /drawer/pause   std_srvs/srv/Trigger "{}"

    Reanuda el dibujo:

    ros2 service call /drawer/resume  std_srvs/srv/Trigger "{}"

    Reinicia(limpia y vuelve a dibujar desde el principio):

    ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"

Comprobaciones útiles:

ros2 node list
ros2 service list | grep drawer
ros2 topic list | grep cmd_vel

6) Mostrar la estructura

    sudo apt-get update && sudo apt-get install -y tree
    cd <RUTA>/g09_prii3_ws/sprint_1/prii3_ws

Muestra solo los tres primeros niveles
tree -L 3 .

