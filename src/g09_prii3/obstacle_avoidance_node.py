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
import time

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
        # Advanced-only sensitivity: detect earlier than threshold (multiplier > 1)
        self.declare_parameter('advanced_detect_factor', 1.3)

        # Resolve params
        self._v = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        self._w = float(self.get_parameter('angular_speed').get_parameter_value().double_value)
        self._threshold = float(self.get_parameter('obstacle_threshold').get_parameter_value().double_value)
        self._mode = str(self.get_parameter('avoidance_mode').get_parameter_value().string_value or 'simple').lower()
        self._adv_detect_factor = float(self.get_parameter('advanced_detect_factor').get_parameter_value().double_value or 1.3)
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
        self._last_turn_dir = +1  # +1 left, -1 right (advanced)
        self._avoid_start_sec = None  # type: Optional[float]

        # Advanced-mode tuning (does not affect simple mode)
        self._resume_factor = 1.25  # need a bit more clearance before resuming forward
        self._front_cone_deg = 30.0  # degrees around the front to assess obstacle
        self._side_inner_deg = 40.0  # start of side sector from forward
        self._side_outer_deg = 100.0  # end of side sector
        self._creep_factor = 0.3  # fraction of linear speed while avoiding
        self._search_w_factor = 0.8  # fraction of angular speed when searching in place
        self._stuck_timeout = 3.0  # seconds blocked before trying rotate-in-place

        # Control loop at ~20 Hz (más correcciones por segundo)
        self._timer = self.create_timer(0.05, self._control_loop)

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

    # ---- Advanced-only helpers (angle-aware sectors; keep simple path unchanged) ----
    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _sector_values_by_deg(self, min_deg: float, max_deg: float) -> List[float]:
        """Return finite scan values within [min_deg, max_deg] sector (degrees around 0=front)."""
        if self._last_scan is None:
            return []
        scan = self._last_scan
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return []
        # Convert to radians
        a_min = scan.angle_min
        a_inc = scan.angle_increment
        # Target angles in radians around 0 (front)
        lo = math.radians(min_deg)
        hi = math.radians(max_deg)
        # Compute indices corresponding to [lo, hi]
        # Index i corresponds to angle a_min + i*a_inc
        def ang_to_idx(a: float) -> int:
            i = int(round((a - a_min) / a_inc))
            return max(0, min(n - 1, i))
        i0 = ang_to_idx(lo)
        i1 = ang_to_idx(hi)
        if i0 > i1:
            i0, i1 = i1, i0
        sector = list(scan.ranges[i0:i1 + 1])
        return self._finite_values(sector)

    @staticmethod
    def _percentile(vals: List[float], p: float) -> Optional[float]:
        if not vals:
            return None
        vals_sorted = sorted(vals)
        k = max(0, min(len(vals_sorted) - 1, int(round((p / 100.0) * (len(vals_sorted) - 1)))))
        return vals_sorted[k]

    def _front_distance_adv(self) -> Optional[float]:
        """More robust front distance: 10th percentile in +/- front_cone/2."""
        half = 0.5 * self._front_cone_deg
        vals = self._sector_values_by_deg(-half, +half)
        # If angle info is missing or empty, fall back
        if not vals:
            return self._front_distance()
        return self._percentile(vals, 10.0)

    def _side_clearance_adv(self) -> Tuple[Optional[float], Optional[float]]:
        """Return representative clearances for left/right using side sectors.
        Uses 30th percentile to reduce outlier influence.
        """
        # Left: +inner .. +outer deg | Right: -outer .. -inner deg
        left_vals = self._sector_values_by_deg(self._side_inner_deg, self._side_outer_deg)
        right_vals = self._sector_values_by_deg(-self._side_outer_deg, -self._side_inner_deg)
        left = self._percentile(left_vals, 30.0) if left_vals else None
        right = self._percentile(right_vals, 30.0) if right_vals else None
        return left, right

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

    # Public stop helper to ensure robot halts on shutdown/interruption
    def stop_robot(self) -> None:
        try:
            self._publish(0.0, 0.0)
        except Exception:
            # Best-effort; ignore errors during teardown
            pass

    # Ensure a stop is sent when the node is destroyed (extra safety)
    def destroy_node(self) -> bool:  # type: ignore[override]
        self.stop_robot()
        return super().destroy_node()

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
        # Determine front distance (simple baseline)
        d_front_simple = self._front_distance()

        if d_front_simple is None:
            # No scan yet — just be conservative: slow forward
            self._set_state('forward')
            self._publish(self._v * 0.5, 0.0)
            return

        # Advanced vs simple behavior
        if self._mode == 'advanced':
            # Detect earlier in advanced mode using robust front and a multiplier
            d_front_adv = self._front_distance_adv()
            d_check = d_front_adv if d_front_adv is not None else d_front_simple
            if d_check is not None and d_check < (self._threshold * self._adv_detect_factor):
                # Robust front and side assessments
                left_clear, right_clear = self._side_clearance_adv()

                # If no side info, rotate in place to search
                if left_clear is None and right_clear is None:
                    self._set_state('avoid_left', 'escaneo insuficiente')
                    self._last_turn_dir = +1
                    self._avoid_start_sec = self._avoid_start_sec or self._now_sec()
                    self._publish(0.0, +self._w * self._search_w_factor)
                    return

                # Choose the side with greater clearance
                if right_clear is None or (left_clear is not None and left_clear > right_clear):
                    turn_dir = +1  # left
                    reason = f'clearance L={left_clear:.2f} R={right_clear if right_clear is not None else float("nan"):.2f}'
                    self._set_state('avoid_left', reason)
                else:
                    turn_dir = -1  # right
                    reason = f'clearance L={left_clear if left_clear is not None else float("nan"):.2f} R={right_clear:.2f}'
                    self._set_state('avoid_right', reason)

                # Stuck detection: if we've been avoiding for too long with very close front, rotate in place
                now = self._now_sec()
                if self._avoid_start_sec is None:
                    self._avoid_start_sec = now
                blocked_front = d_front_adv if d_front_adv is not None else d_front_simple
                blocked = (blocked_front is not None) and (blocked_front < (0.8 * self._threshold))
                if blocked and (now - self._avoid_start_sec) > self._stuck_timeout:
                    # Try rotate-in-place to break free; alternate direction next time
                    self._publish(0.0, self._w * turn_dir)
                    self._last_turn_dir = turn_dir
                    # Reset timer after a rotate-in-place attempt
                    self._avoid_start_sec = now
                    return

                # Normal avoid: creep forward while turning toward chosen side
                self._publish(self._v * self._creep_factor, self._w * turn_dir)
                self._last_turn_dir = turn_dir
                return
        else:
            # Simple mode detection as before
            if d_front_simple < self._threshold:
                self._set_state('stopped')
                self._publish(0.0, 0.0)
                return

        # No obstacle — go forward (advanced: add hysteresis so we don't flip-flop)
        if self._mode == 'advanced':
            # If we were avoiding, require a bit of extra clearance to resume
            d_front_adv = self._front_distance_adv()
            if self._state in ('avoid_left', 'avoid_right') and d_front_adv is not None:
                if d_front_adv < self._threshold * self._resume_factor:
                    # Keep avoiding in same direction gently
                    turn_dir = +1 if self._state == 'avoid_left' else -1
                    self._publish(self._v * self._creep_factor, self._w * turn_dir)
                    return
                # Clear enough — drop back to forward
            # Reset avoid timer when resuming forward
            self._avoid_start_sec = None

        if self._state != 'forward':
            # Transition log happens in _set_state
            self._set_state('forward')
        # Steady logging: avoid spamming, only log occasionally when cruising
        self._publish(self._v, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetbotAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown/interrupt, stop the robot explicitly
        node.get_logger().info('Apagando nodo: enviando stop a /cmd_vel.')
        node.stop_robot()
        # tiny delay to allow message to flush
        time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
