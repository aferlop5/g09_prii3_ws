#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Jetbot obstacle avoidance node.

This ROS 2 node commands the Jetbot to move forward while monitoring the Lidar
(/scan). If an obstacle is detected closer than a threshold in front of the
robot, it either stops (simple mode) or tries to avoid it by turning while
creeping forward (advanced mode).

Topics:
  - Publishes geometry_msgs/Twist to /cmd_vel
  - Subscribes sensor_msgs/LaserScan from /scan

Parameters:
  - linear_speed (float, default: 0.15)   -> forward speed (m/s)
  - angular_speed (float, default: 0.6)   -> turn speed (rad/s)
  - obstacle_threshold (float, default: 0.3) -> distance threshold (m)
  - avoidance_mode (str, default: 'simple') -> 'simple' or 'advanced'

Notes:
  - QoS: /cmd_vel uses RELIABLE; /scan uses sensor data QoS (BEST_EFFORT).
  - This node aims to be compatible with jetbot_pro_ros2 (drivers for motors
    and sensors). It only publishes /cmd_vel and consumes /scan.
"""

from typing import List, Optional, Tuple
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class JetbotAvoider(Node):
    """ROS2 node to perform basic obstacle avoidance using a Lidar scan."""

    def __init__(self) -> None:
        super().__init__('jetbot_obstacle_avoidance')

        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('avoidance_mode', 'simple')  # 'simple' | 'advanced'

        # Resolve params
        self._v = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        self._w = float(self.get_parameter('angular_speed').get_parameter_value().double_value)
        self._threshold = float(self.get_parameter('obstacle_threshold').get_parameter_value().double_value)
        self._mode = str(self.get_parameter('avoidance_mode').get_parameter_value().string_value or 'simple').lower()
        if self._mode not in ('simple', 'advanced'):
            self.get_logger().warning("avoidance_mode inválido; usando 'simple'")
            self._mode = 'simple'

        if self._v <= 0.0:
            self.get_logger().warning('linear_speed <= 0; usando 0.1 m/s por seguridad')
            self._v = 0.1
        if abs(self._w) < 1e-3:
            self.get_logger().warning('angular_speed demasiado bajo; usando 0.6 rad/s por seguridad')
            self._w = 0.6

        # Publisher (RELIABLE for /cmd_vel)
        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_cmd)

        # Subscriber (sensor data QoS for /scan)
        self._scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        # State
        self._last_scan = None  # type: Optional[LaserScan]
        self._state = 'forward'  # 'forward' | 'stopped' | 'avoid_left' | 'avoid_right'

        # Control loop at ~10 Hz
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'JetbotAvoider listo. Modo: {self._mode} | v={self._v:.2f} m/s, w={self._w:.2f} rad/s, threshold={self._threshold:.2f} m'
        )

    # ------------------------- Callbacks -------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    # ------------------------- Helpers ---------------------------------------
    @staticmethod
    def _finite_values(values: List[float]) -> List[float]:
        return [x for x in values if x is not None and not math.isinf(x) and not math.isnan(x)]

    def _front_distance(self) -> Optional[float]:
        """
        Compute the minimal distance in the frontal sector using the first and
        last 20 samples of the scan ranges (covers forward direction in many
        Lidar configurations).
        """
        if self._last_scan is None:
            return None
        ranges = list(self._last_scan.ranges)
        if not ranges:
            return None
        sector = ranges[:20] + ranges[-20:]
        vals = self._finite_values(sector)
        return min(vals) if vals else None

    def _side_means(self) -> Tuple[Optional[float], Optional[float]]:
        """Compute average distances for left and right sectors for avoidance."""
        if self._last_scan is None:
            return None, None
        r = list(self._last_scan.ranges)
        if not r:
            return None, None
        n = len(r)
        # Define sectors: right = indices around -90 deg, left = +90 deg
        # As a simple heuristic across many scans, use quarter slices.
        right_sector = r[n//8 : n//4]  # ~45-90 deg on right
        left_sector = r[-n//4 : -n//8]  # ~-90 to -45 deg on left (wrap)
        right_vals = self._finite_values(right_sector)
        left_vals = self._finite_values(left_sector)
        right_mean = sum(right_vals) / len(right_vals) if right_vals else None
        left_mean = sum(left_vals) / len(left_vals) if left_vals else None
        return left_mean, right_mean

    def _publish(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _set_state(self, new_state: str, reason: Optional[str] = None) -> None:
        if new_state == self._state:
            return
        self._state = new_state
        if new_state == 'forward':
            self.get_logger().info('Avanzando — sin obstáculos frontales.')
        elif new_state == 'stopped':
            self.get_logger().info('Obstáculo detectado — deteniendo.')
        elif new_state == 'avoid_left':
            self.get_logger().info('Evitando obstáculo — girando a la izquierda.' + (f' ({reason})' if reason else ''))
        elif new_state == 'avoid_right':
            self.get_logger().info('Evitando obstáculo — girando a la derecha.' + (f' ({reason})' if reason else ''))

    # ----------------------- Control loop ------------------------------------
    def _control_loop(self) -> None:
        # Determine front distance
        d_front = self._front_distance()

        if d_front is None:
            # No scan yet — just be conservative: slow forward
            self._set_state('forward')
            self._publish(self._v * 0.5, 0.0)
            return

        # Advanced vs simple behavior
        if d_front < self._threshold:
            if self._mode == 'advanced':
                # Pick the side with larger clearance
                left_mean, right_mean = self._side_means()
                if left_mean is None and right_mean is None:
                    # No info — rotate in place slowly to search
                    self._set_state('avoid_left', 'escaneo insuficiente')
                    self._publish(0.0, +self._w * 0.6)
                    return
                # Prefer the side with greater distance
                if right_mean is None or (left_mean is not None and left_mean > right_mean):
                    self._set_state('avoid_left', f'clearance L={left_mean:.2f} R={right_mean if right_mean is not None else float("nan"):.2f}')
                    # Turn left while creeping forward
                    self._publish(self._v * 0.3, +self._w)
                else:
                    self._set_state('avoid_right', f'clearance L={left_mean if left_mean is not None else float("nan"):.2f} R={right_mean:.2f}')
                    # Turn right while creeping forward
                    self._publish(self._v * 0.3, -self._w)
                return
            else:
                # Simple mode: stop
                self._set_state('stopped')
                self._publish(0.0, 0.0)
                return

        # No obstacle — go forward
        if self._state != 'forward':
            # Transition log happens in _set_state
            self._set_state('forward')
        # Steady logging: avoid spamming, only log occasionally when cruising
        self._publish(self._v, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetbotAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
