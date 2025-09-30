PRIi3 Turtlesim - Proyecto ROS2

Este repositorio contiene el workspace y paquete inicial prii3_turtlesim para la asignatura PRIi3. El objetivo es implementar un nodo de ROS2 en Python que controle el simulador turtlesim y permita dibujar figuras (actualmente un número 9 simple y cerrado).

Este README está diseñado para futuros sprints, de manera que cualquier integrante del equipo pueda configurar su entorno y ejecutar el código correctamente después de clonar el repositorio.

Contenido del repositorio

El repositorio incluye el workspace prii3_ws con el paquete prii3_turtlesim, que contiene el código fuente Python del nodo, los archivos de configuración package.xml y setup.py, así como pruebas básicas en la carpeta test/.

Requisitos

Sistema operativo: Ubuntu 20.04

ROS2 Foxy

Python 3

Paquetes ROS2 turtlesim y rclpy instalados

Editor de código recomendado: VS Code

Instrucciones para configurar después de clonar

Clonar el repositorio y entrar al workspace:

git clone <URL_DEL_REPOSITORIO>
cd <ruta_al_workspace>/prii3_ws


Instalar dependencias (si no están instaladas):

sudo apt update
sudo apt install ros-foxy-turtlesim python3-colcon-common-extensions


Compilar el workspace:

colcon build


Cargar el entorno (necesario en cada terminal nueva):

source install/setup.bash


Ejecutar turtlesim en una terminal:

ros2 run turtlesim turtlesim_node


Ejecutar el nodo prii3_turtlesim en otra terminal:

ros2 run prii3_turtlesim prii3_turtlesim_node

Uso de VS Code

Se recomienda abrir todo el workspace prii3_ws en VS Code para tener autocompletado y navegación de paquetes ROS2. Si se hacen cambios en el código fuente, guardar los cambios, volver a compilar el workspace con colcon build y cargar el entorno con source install/setup.bash antes de ejecutar de nuevo el nodo.

Futuras consideraciones para próximos sprints

Añadir nuevos nodos para controlar otras figuras o funcionalidades.

Mantener un repositorio limpio con commits claros por sprint.

Usar branches por funcionalidades para evitar conflictos.

Asegurarse de que todos los miembros hayan hecho source install/setup.bash antes de ejecutar cualquier nodo.

Actualizar este README con instrucciones específicas de nuevos nodos o scripts.

Comprobación del entorno

Para verificar que ROS2 y el paquete están configurados correctamente:

printenv | grep -i ROS
ros2 pkg executables prii3_turtlesim
ros2 node list
