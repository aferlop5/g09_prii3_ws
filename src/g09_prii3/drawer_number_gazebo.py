#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtlebotNine(Node):
    def __init__(self) -> None:
        super().__init__('turtlebot_number_node')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._angular_speed = 0.6
        self._linear_speed = 0.2
        self._segments = self._build_segments()
        self._idx = 0
        self._seg_elapsed = 0.0
        self._last_tick = time.time()
        self._finished = False
        self._timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Iniciando dibujo del número 9 en Gazebo...')

    def _build_segments(self):
        w = self._angular_speed
        v = self._linear_speed
        return [
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_up"},
            {"vx": v, "wz": 0.0, "duration": 4.0 / v, "label": "left_up"},
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_top"},
            {"vx": v, "wz": 0.0, "duration": 1.5 / v, "label": "top_right"},
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_right"},
            {"vx": v, "wz": 0.0, "duration": 1.5 / v, "label": "right_down"},
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_close"},
            {"vx": v, "wz": 0.0, "duration": 2.0 / v, "label": "close_left"},
        ]

    def _control_loop(self) -> None:
        if self._finished:
            return
        now = time.time()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now
        if self._idx >= len(self._segments):
            self._finish()
            return
        seg = self._segments[self._idx]
        self._publish(seg["vx"], seg["wz"])
        self._seg_elapsed += dt
        if self._seg_elapsed >= seg["duration"]:
            self._idx += 1
            self._seg_elapsed = 0.0
            if self._idx >= len(self._segments):
                self._finish()

    def _publish(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._pub.publish(msg)

    def _finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._publish(0.0, 0.0)
        self.get_logger().info('Número 9 completado. Deteniendo nodo...')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtlebotNine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
