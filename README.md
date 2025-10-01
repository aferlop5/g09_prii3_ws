PRIi3 Turtlesim - Proyecto ROS2
🚀 Pasos para ponerlo en marcha

1. Clonar el repositorio

cd ~/universidad_agusti/tercero/proyecto/sprint_1
git clone <URL_DEL_REPO> prii3_ws
cd prii3_ws


2. Crear el workspace de ROS 2
mkdir -p src
cd src


3. Verificar el paquete prii3_turtlesim

Dentro de prii3_turtlesim encontrarás:
package.xml → define dependencias (rclpy, turtlesim, launch, launch_ros).
setup.py → instala el nodo y los ficheros de lanzamiento.
prii3_turtlesim_node.py → nodo Python que dibuja el número.
launch/turtlesim_launch.py → archivo de lanzamiento que abre turtlesim y tu nodo drawer.


4. Compilar el workspace

Desde la raíz del workspace (~/universidad_agusti/tercero/proyecto/sprint_1/prii3_ws):
colcon build
Después, cada vez que abras una nueva terminal, carga el entorno:
source install/setup.bash


5. Ejecutar el sistema completo

Lanza turtlesim + el nodo drawer desde el único archivo launch:
ros2 launch prii3_turtlesim turtlesim_launch.py
Esto abrirá la ventana de turtlesim y la tortuga dibujará el número 9.

6. Control del dibujo con servicios

Tu nodo expone servicios ROS2 para pausar, reanudar o reiniciar el dibujo.

📌 Pausar el dibujo

Detiene inmediatamente la tortuga en la posición actual:
ros2 service call /drawer/pause std_srvs/srv/Trigger "{}"

📌 Reanudar el dibujo

Continúa la secuencia donde se había detenido:
ros2 service call /drawer/resume std_srvs/srv/Trigger "{}"

📌 Reiniciar el dibujo

Limpia la pantalla, teletransporta la tortuga a la posición inicial y vuelve a empezar el número 9 desde cero:
ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"

