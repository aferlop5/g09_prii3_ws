#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class JetbotDrawer(Node):
    """
    ROS2 node that publishes to /cmd_vel to draw the group number "09".

    Provides services to control execution:
      - /jetbot_drawer/pause   (std_srvs/Trigger)
      - /jetbot_drawer/resume  (std_srvs/Trigger)
      - /jetbot_drawer/restart (std_srvs/Trigger)

    The trajectory is implemented as a timed sequence of (vx, wz) segments.
    This is open-loop and intended for a simple demo with the low-level driver
    from jetbot_pro_ros2 consuming /cmd_vel.
    """

    def __init__(self) -> None:
        super().__init__('jetbot_drawer')

        # QoS suitable for command velocity
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        # Services
        self.pause_srv = self.create_service(Trigger, 'jetbot_drawer/pause', self._on_pause)
        self.resume_srv = self.create_service(Trigger, 'jetbot_drawer/resume', self._on_resume)
        self.restart_srv = self.create_service(Trigger, 'jetbot_drawer/restart', self._on_restart)

        # State
        self._paused = False
        self._finished = False
        # Parameters (tunable speeds)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self._segments = self._build_segments()
        self._idx = 0
        self._seg_elapsed = 0.0
        self._last_tick = time.time()

        # 20 Hz control loop
        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Jetbot Drawer listo. Servicios: jetbot_drawer/(pause|resume|restart)')

    # ---- Trajectory definition -------------------------------------------------
    def _build_segments(self) -> List[Dict[str, float]]:
        """
        Build the timed sequence for drawing "09".

        We draw a zero (circle-ish using constant turn) followed by a nine
        shaped by segments. Durations are tuned for demo scale; adjust speeds
        as needed for the real robot.
        """
        v = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        w = float(self.get_parameter('angular_speed').get_parameter_value().double_value)
        # Guards to avoid degenerate motion
        if abs(w) < 1e-3:
            self.get_logger().warning('angular_speed demasiado bajo; usando 0.6 rad/s por seguridad')
            w = 0.6
        if v <= 0.0:
            self.get_logger().warning('linear_speed <= 0; usando 0.1 m/s por seguridad')
            v = 0.1

        segments: List[Dict[str, float]] = []

        

    # Small pause to stabilize
        segments.append({"vx": 0.0, "wz": 0.0, "duration": 0.5, "label": "pause_between"})

        # Draw '9' similar to turtlesim example
        segments.extend([
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_up"},
            {"vx": v, "wz": 0.0, "duration": 0.8 / v, "label": "short_left_up"},
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_top"},
            {"vx": v, "wz": 0.0, "duration": 0.5 / v, "label": "top_right"},
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_right"},
            {"vx": v, "wz": 0.0, "duration": 0.5 / v, "label": "right_down"},
            {"vx": 0.0, "wz": w, "duration": math.pi / 2.0 / w, "label": "turn_close"},
            {"vx": v, "wz": 0.0, "duration": 0.7 / v, "label": "close_left"},
        ])

        return segments

    # ---- Control loop ----------------------------------------------------------
    def _control_loop(self) -> None:
        if self._finished:
            return

        now = time.time()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        if self._paused:
            self._publish(0.0, 0.0)
            return

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
        self.cmd_pub.publish(msg)

    def _finish(self) -> None:
        self._finished = True
        self._publish(0.0, 0.0)
        self.get_logger().info('Trayectoria "09" finalizada.')

    # ---- Services --------------------------------------------------------------
    def _on_pause(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._paused = True
        self._publish(0.0, 0.0)
        res.success = True
        res.message = 'Pausado'
        self.get_logger().info('Servicio pause: pausando movimiento.')
        return res

    def _on_resume(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._paused = False
        self._last_tick = time.time()  # avoid jump in elapsed time
        res.success = True
        res.message = 'Reanudado'
        self.get_logger().info('Servicio resume: reanudando movimiento.')
        return res

    def _on_restart(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._paused = False
        self._finished = False
        self._idx = 0
        self._seg_elapsed = 0.0
        self._last_tick = time.time()
        self._publish(0.0, 0.0)
        res.success = True
        res.message = 'Reiniciado'
        self.get_logger().info('Servicio restart: reiniciando trayectoria desde el inicio.')
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetbotDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
