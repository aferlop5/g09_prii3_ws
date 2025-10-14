#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math


class TurtleNine(Node):
    def __init__(self):
        super().__init__('turtle_nine')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', qos)

        self.pause_srv = self.create_service(Trigger, 'drawer/pause', self.handle_pause)
        self.resume_srv = self.create_service(Trigger, 'drawer/resume', self.handle_resume)
        self.restart_srv = self.create_service(Trigger, 'drawer/restart', self.handle_restart)
        self.pause_srv_alias = self.create_service(Trigger, 'pause_drawing', self.handle_pause)
        self.resume_srv_alias = self.create_service(Trigger, 'resume_drawing', self.handle_resume)
        self.restart_srv_alias = self.create_service(Trigger, 'restart_drawing', self.handle_restart)

        self.reset_client = self.create_client(Empty, '/reset')

        self.started = False
        self.paused = False
        self.finished = False

        self.segments = self._build_segments()
        self.current_index = 0
        self.segment_elapsed = 0.0
        self.last_tick = None

        self.create_timer(1.0, self._start_after_delay)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            'Nodo listo. Servicios: drawer/pause,resume,restart (y alias).'
        )

    def _build_segments(self):
        angular_speed = 1.0
        linear_speed = 1.0
        return [
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_up"},
            {"vx": linear_speed, "wz": 0.0, "duration": 4.0/linear_speed, "label": "left_up"},
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_top"},
            {"vx": linear_speed, "wz": 0.0, "duration": 1.5/linear_speed, "label": "top_right"},
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_right"},
            {"vx": linear_speed, "wz": 0.0, "duration": 1.5/linear_speed, "label": "right_down"},
            {"vx": 0.0, "wz": angular_speed, "duration": math.pi/2, "label": "turn_close"},
            {"vx": linear_speed, "wz": 0.0, "duration": 2.0/linear_speed, "label": "close_left"},
        ]

    def _start_after_delay(self):
        if not self.started:
            self.started = True
            self.last_tick = time.time()
            self.get_logger().info('Dibujando el número 9 (controlable por servicios).')

    def _control_loop(self):
        if not self.started or self.finished:
            return
        now = time.time()
        if self.last_tick is None:
            self.last_tick = now
        dt = now - self.last_tick
        self.last_tick = now
        if self.paused:
            self._publish_twist(0.0, 0.0)
            return
        if self.current_index >= len(self.segments):
            self._finish()
            return
        seg = self.segments[self.current_index]
        self._publish_twist(seg["vx"], seg["wz"])
        self.segment_elapsed += dt
        if self.segment_elapsed >= seg["duration"]:
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
        if not self.finished:
            self.finished = True
            self._publish_twist(0.0, 0.0)
            self.get_logger().info('Número 9 dibujado y cerrado correctamente.')

    def handle_pause(self, request, response):
        self.paused = True
        self._publish_twist(0.0, 0.0)
        self.get_logger().info('pause: pausando dibujo y deteniendo tortuga.')
        response.success = True
        response.message = 'Dibujo pausado.'
        return response

    def handle_resume(self, request, response):
        self.paused = False
        self.last_tick = time.time()
        self.get_logger().info('resume: reanudando dibujo.')
        response.success = True
        response.message = 'Dibujo reanudado.'
        return response

    def handle_restart(self, request, response):
        self.started = True
        self.paused = False
        self.finished = False
        self.current_index = 0
        self.segment_elapsed = 0.0
        self.last_tick = time.time()
        self._publish_twist(0.0, 0.0)
        try:
            if self.reset_client.wait_for_service(timeout_sec=0.1):
                self.reset_client.call_async(Empty.Request())
            else:
                self.get_logger().warning('Servicio /reset no disponible por ahora.')
        except Exception as exc:
            self.get_logger().warning(f'No se pudo llamar a /reset: {exc}')
        self.get_logger().info('restart: reiniciando dibujo al inicio.')
        response.success = True
        response.message = 'Dibujo reiniciado.'
        return response


def main():
    rclpy.init()
    node = TurtleNine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
