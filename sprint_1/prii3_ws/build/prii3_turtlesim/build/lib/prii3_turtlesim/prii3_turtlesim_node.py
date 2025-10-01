#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Importamos librerías de ROS2 y Python
import rclpy                         # Librería principal de ROS2 en Python
from rclpy.node import Node          # Clase base para crear nodos en ROS2
from geometry_msgs.msg import Twist  # Mensaje para enviar velocidades lineales y angulares
from std_srvs.srv import Trigger, Empty  # Servicios estándar para control y reset
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS para compatibilidad con turtlesim
import time                          # Para medir tiempos y duraciones
import math                          # Para usar valores de pi y funciones trigonométricas


# Definimos la clase del nodo
class TurtleNine(Node):
    def __init__(self):
        # Inicializamos el nodo con el nombre "turtle_nine"
        super().__init__('turtle_nine')

        # Publicador de velocidades para turtlesim (QoS fiable para emparejar con turtlesim)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', qos)

        # Servicios para controlar el dibujo
        # Nombres bajo prefijo "drawer/" (recomendado)
        self.pause_srv = self.create_service(Trigger, 'drawer/pause', self.handle_pause)
        self.resume_srv = self.create_service(Trigger, 'drawer/resume', self.handle_resume)
        self.restart_srv = self.create_service(Trigger, 'drawer/restart', self.handle_restart)
        # Alias simples para compatibilidad (p. ej. /pause_drawing)
        self.pause_srv_alias = self.create_service(Trigger, 'pause_drawing', self.handle_pause)
        self.resume_srv_alias = self.create_service(Trigger, 'resume_drawing', self.handle_resume)
        self.restart_srv_alias = self.create_service(Trigger, 'restart_drawing', self.handle_restart)

        # Cliente para reiniciar turtlesim (resetea posición y limpia dibujo)
        self.reset_client = self.create_client(Empty, '/reset')

        # Estado interno del "maquinista" del dibujo
        self.started = False      # Comienza a dibujar tras un pequeño retraso
        self.paused = False       # Pausa/Reanuda
        self.finished = False     # Dibujo completado

        # Construimos la secuencia de segmentos que forman el número 9
        self.segments = self._build_segments()
        self.current_index = 0
        self.segment_elapsed = 0.0
        self.last_tick = None

        # Temporizadores: uno para iniciar y otro para el bucle de control a 20 Hz
        self.create_timer(1.0, self._start_after_delay)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            'Nodo listo. Servicios disponibles: '
            'drawer/pause | pause_drawing, '
            'drawer/resume | resume_drawing, '
            'drawer/restart | restart_drawing'
        )

    def _build_segments(self):
        # Cada segmento define velocidades constantes y su duración
        # vx [m/s], wz [rad/s], duration [s]
        angular_speed = 1.0
        linear_speed = 1.0

        return [
            # 1) Giro 90º a la izquierda
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_up"},
            # 2) Línea vertical izquierda hacia arriba (4.0)
            {"vx": linear_speed, "wz": 0.0, "duration": 4.0/linear_speed, "label": "left_up"},
            # 3) Giro 90º a la derecha (otro giro izquierdo si mantenemos wz positivo) -> línea superior
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_top"},
            # 4) Línea horizontal superior (1.5)
            {"vx": linear_speed, "wz": 0.0, "duration": 1.5/linear_speed, "label": "top_right"},
            # 5) Giro 90º a la derecha -> línea vertical derecha
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_right"},
            # 6) Línea vertical descendente de la cabeza (1.5)
            {"vx": linear_speed, "wz": 0.0, "duration": 1.5/linear_speed, "label": "right_down"},
            # 7) Giro 90º a la izquierda -> cerrar horizontal superior
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_close"},
            # 8) Línea horizontal de cierre hacia la izquierda (2.0)
            {"vx": linear_speed, "wz": 0.0, "duration": 2.0/linear_speed, "label": "close_left"},
        ]

    def _start_after_delay(self):
        if not self.started:
            self.started = True
            self.last_tick = time.time()
            self.get_logger().info('Dibujando el número 9 (controlable por servicios).')

    def _control_loop(self):
        # Bucle periódico a ~20Hz que aplica velocidades según el segmento actual
        if not self.started or self.finished:
            return

        now = time.time()
        if self.last_tick is None:
            self.last_tick = now
        dt = now - self.last_tick
        self.last_tick = now

        # Si está en pausa, no avanzamos el tiempo ni el segmento; paramos la tortuga
        if self.paused:
            self._publish_twist(0.0, 0.0)
            return

        # Seguridad por si ya no hay segmentos
        if self.current_index >= len(self.segments):
            self._finish()
            return

        seg = self.segments[self.current_index]

        # Publicamos velocidades constantes del segmento actual
        self._publish_twist(seg["vx"], seg["wz"])

        # Avanzamos el tiempo transcurrido en el segmento
        self.segment_elapsed += dt
        if self.segment_elapsed >= seg["duration"]:
            # Pasamos al siguiente segmento
            self.current_index += 1
            self.segment_elapsed = 0.0

            if self.current_index >= len(self.segments):
                self._finish()

    def _publish_twist(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def _finish(self):
        # Marca el dibujo como completado y detiene la tortuga
        if not self.finished:
            self.finished = True
            self._publish_twist(0.0, 0.0)
            self.get_logger().info('Número 9 dibujado y cerrado correctamente.')

    # =====================
    #  Manejadores Servicio
    # =====================
    def handle_pause(self, request, response):  # Trigger -> no usa request
        self.paused = True
        self._publish_twist(0.0, 0.0)
        self.get_logger().info('Servicio pause recibido: pausando dibujo y deteniendo tortuga.')
        response.success = True
        response.message = 'Dibujo pausado.'
        return response

    def handle_resume(self, request, response):  # Trigger -> no usa request
        # Reanudar sin saltos de tiempo
        self.paused = False
        self.last_tick = time.time()
        self.get_logger().info('Servicio resume recibido: reanudando dibujo.')
        response.success = True
        response.message = 'Dibujo reanudado.'
        return response

    def handle_restart(self, request, response):  # Trigger -> no usa request
        # Reinicia estado interno del dibujo
        self.started = True
        self.paused = False
        self.finished = False
        self.current_index = 0
        self.segment_elapsed = 0.0
        self.last_tick = time.time()

        # Paramos instantáneamente
        self._publish_twist(0.0, 0.0)

        # Lanza reset de turtlesim de forma asíncrona (si está disponible)
        try:
            # Espera muy breve para no bloquear si turtlesim aún no expone el servicio
            if self.reset_client.wait_for_service(timeout_sec=0.1):
                self.reset_client.call_async(Empty.Request())
            else:
                self.get_logger().warning('Servicio /reset no disponible por ahora.')
        except Exception as exc:  # noqa: BLE001 - logueamos cualquier error de llamada
            self.get_logger().warning(f'No se pudo llamar a /reset: {exc}')

        self.get_logger().info('Servicio restart recibido: reiniciando dibujo al inicio.')
        response.success = True
        response.message = 'Dibujo reiniciado.'
        return response


# Función principal de ROS2
def main():
    rclpy.init()                # Inicializa la comunicación de ROS2
    node = TurtleNine()         # Crea el nodo
    rclpy.spin(node)            # Mantiene el nodo ejecutándose
    node.destroy_node()         # Destruye el nodo al cerrar
    rclpy.shutdown()            # Apaga el cliente de ROS2


# Si el archivo se ejecuta directamente, llama a main()
if __name__ == '__main__':
    main()
