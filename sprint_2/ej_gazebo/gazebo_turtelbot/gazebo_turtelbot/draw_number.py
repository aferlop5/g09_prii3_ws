#!/usr/bin/env python3
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Nodo ROS2 que dibuja el número 9 en Gazebo con un TurtleBot3.

Este nodo publica comandos de velocidad lineal y angular en el tópico
"/cmd_vel" siguiendo una secuencia de segmentos cronometrados que forman el
número 9, similar al ejemplo de turtlesim, pero adaptado a Gazebo. No expone
servicios de pausa, reanudación o reinicio: únicamente dibuja el número y se
detiene al terminar.
"""

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtlebotNine(Node):
    """Publica una trayectoria cronometrada para trazar el número 9.

    Contrato breve:
    - Entrada: ninguna.
    - Salida: mensajes Twist en "/cmd_vel" a ~20 Hz.
    - Éxito: completa todos los segmentos y publica velocidad cero al final.
    - Error: no aplica (se detiene si la secuencia finaliza).
    """

    def __init__(self) -> None:
        """Inicializa el nodo, el publicador y la secuencia del número 9."""
        super().__init__('turtlebot_number_node')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Ajusta velocidades para Gazebo (más seguras que 1.0 m/s en TB3).
        self._angular_speed = 0.6  # rad/s
        self._linear_speed = 0.2   # m/s

        # Construimos la secuencia estilo turtlesim para el "9".
        self._segments = self._build_segments()
        self._idx = 0
        self._seg_elapsed = 0.0
        self._last_tick = time.time()
        self._finished = False

        # Bucle de control a ~20 Hz.
        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Iniciando dibujo del número 9 en Gazebo...')

    def _build_segments(self):
        """Crea la lista de segmentos para formar el número 9.

        Cada segmento define velocidades constantes y su duración en segundos.
        Los valores están escalados por las velocidades configuradas.
        """
        w = self._angular_speed
        v = self._linear_speed

        return [
            # 1) Giro 90º a la izquierda
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_up"},
            # 2) Línea vertical izquierda hacia arriba (4.0 m)
            {"vx": v, "wz": 0.0, "duration": 4.0 / v, "label": "left_up"},
            # 3) Giro 90º a la izquierda (equivale a derecha manteniendo signo)
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_top"},
            # 4) Línea horizontal superior (1.5 m)
            {"vx": v, "wz": 0.0, "duration": 1.5 / v, "label": "top_right"},
            # 5) Giro 90º a la izquierda -> línea vertical derecha
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_right"},
            # 6) Línea vertical descendente de la cabeza (1.5 m)
            {"vx": v, "wz": 0.0, "duration": 1.5 / v, "label": "right_down"},
            # 7) Giro 90º a la izquierda -> cerrar horizontal superior
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_close"},
            # 8) Línea horizontal de cierre hacia la izquierda (2.0 m)
            {"vx": v, "wz": 0.0, "duration": 2.0 / v, "label": "close_left"},
        ]

    def _control_loop(self) -> None:
        """Aplica las velocidades del segmento actual y avanza la secuencia."""
        if self._finished:
            return

        now = time.time()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        if self._idx >= len(self._segments):
            self._finish()
            return

        seg = self._segments[self._idx]

        # Publica velocidades constantes del segmento
        self._publish(seg["vx"], seg["wz"])

        # Avanza el tiempo del segmento actual
        self._seg_elapsed += dt
        if self._seg_elapsed >= seg["duration"]:
            self._idx += 1
            self._seg_elapsed = 0.0
            if self._idx >= len(self._segments):
                self._finish()

    def _publish(self, vx: float, wz: float) -> None:
        """Publica un mensaje Twist con velocidades vx y wz."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._pub.publish(msg)

    def _finish(self) -> None:
        """Detiene el robot y cierra el nodo."""
        if self._finished:
            return
        self._finished = True
        self._publish(0.0, 0.0)
        self.get_logger().info('Número 9 completado. Deteniendo nodo...')
        rclpy.shutdown()


def main(args=None) -> None:
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = TurtlebotNine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
