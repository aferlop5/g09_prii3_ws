PRII3 – Grupo 09 – Workspace ROS 2 (nodos en src/)

Este repositorio contiene un workspace ROS 2 con un solo paquete raíz (`g09_prii3`). Dentro de `src/` solo hay los nodos Python; los launch están en `launch/`.

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
│  └─ sprint2.launch.py               # lanza drawer_number (ejercicios Gazebo)
├─ src/
│  └─ g09_prii3/
│     ├─ prii3_turtlesim_node.py      # Sprint 1 · Turtlesim (dibuja el 9)
│     ├─ drawer_number.py             # Ejercicios · Gazebo/TB3 (dibuja el 9)
│     └─ jetbot_drawer.py             # Sprint 2 · JetBot (dibuja el 9)
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
PRII3 – Grupo 09 – Workspace ROS 2 (nodos en src/)

=============================================================
Resumen rápido
=============================================================

Este repositorio contiene un workspace ROS 2 con un único paquete raíz
(`g09_prii3`). Los nodos de aplicación están en `src/g09_prii3/` y los
lanzadores en `launch/`. A continuación se documentan los nodos
separados por su pertenencia temática: Sprint 1, Sprint 2 y Ejercicios.

-------------------------------------------------------------

Probado en Ubuntu 20.04/22.04 con ROS 2 (Foxy/Humble). Ajusta los nombres de
distro en los ejemplos si es necesario.

## Requisitos

- ROS 2 instalado y configurado (Foxy/Humble). Ejemplo para turtlesim en Foxy:

```bash
sudo apt update && sudo apt install -y ros-foxy-turtlesim
```

## Estructura actual (paquete único ament_python)

```text
g09_prii3_ws/
├─ launch/
│  ├─ sprint1.launch.py               # turtlesim + prii3_turtlesim_node
│  └─ sprint2.launch.py               # lanza drawer_number (ejercicios Gazebo)
├─ src/
│  └─ g09_prii3/
│     ├─ prii3_turtlesim_node.py      # Sprint 1 · Turtlesim (dibuja el 9)
│     ├─ drawer_number.py             # Ejercicios · Gazebo/TB3 (dibuja el 9)
│     └─ jetbot_drawer.py             # Sprint 2 · JetBot (dibuja el 9) [opcional]
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ resource/g09_prii3
├─ .gitignore
└─ README.md
```

=============================================================
Ejecución por temas
=============================================================

### Sprint 1 · prii3_turtlesim_node (turtlesim)

Descripción
- Dibuja el número 9 en `turtlesim` y expone servicios para control
(pause / resume / restart).

Lanzamiento

```bash
ros2 launch g09_prii3 sprint1.launch.py
```

Servicios útiles (Sprint 1)

```bash
ros2 service call /drawer/pause   std_srvs/srv/Trigger "{}"
ros2 service call /drawer/resume  std_srvs/srv/Trigger "{}"
ros2 service call /drawer/restart std_srvs/srv/Trigger "{}"
```

-------------------------------------------------------------

### Sprint 2 · JetBot — `jetbot_drawer`

Descripción
- Nodo pensado para JetBot que publica comandos en `/cmd_vel`. Se integra
	con el stack JetBot que traduce estos comandos a control de motores.

Ejecución (si está instalado como ejecutable en `setup.py`)

```bash
ros2 run g09_prii3 jetbot_drawer
```

Servicios asociados (ejemplo)

```bash
ros2 service call /jetbot_drawer/pause   std_srvs/srv/Trigger {}
ros2 service call /jetbot_drawer/resume  std_srvs/srv/Trigger {}
ros2 service call /jetbot_drawer/restart std_srvs/srv/Trigger {}
```

Nota: si quieres un launch dedicado para JetBot, puedo añadir
`launch/sprint2_jetbot.launch.py` que envíe parámetros específicos.

-------------------------------------------------------------

### Ejercicios · Gazebo — `drawer_number` (TurtleBot3)

Descripción
- Nodo de ejercicios para simular el trazado del número 9 en Gazebo /
	TurtleBot3. Publica en `/cmd_vel` y finaliza al completar la secuencia.

Lanzamiento

```bash
ros2 launch g09_prii3 sprint2.launch.py
```

=============================================================
Notas
=============================================================

- `build/`, `install/` y `log/` están ignorados en git. Compila en limpio si
	cambias de entorno.
- Si ves avisos acerca de `COLCON_PREFIX_PATH` o `AMENT_PREFIX_PATH` apuntando
	a rutas antiguas, en la sesión puedes ejecutar:

```bash
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH
source install/setup.bash
```

- Para añadir un nuevo sprint: añade un nuevo script a `src/g09_prii3/` y crea
	su correspondiente `launch/sprintX.launch.py`.
- Si no ves el ejecutable `jetbot_drawer`, asegúrate de que `src/g09_prii3/jetbot_drawer.py`
	existe y que en `setup.py` está la entrada
	`jetbot_drawer = g09_prii3.jetbot_drawer:main`.

