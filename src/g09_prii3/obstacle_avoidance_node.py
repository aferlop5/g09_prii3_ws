#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Optional, Tuple, Deque
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class JetbotAvoider(Node):
    """ROS 2 node for JetBot obstacle avoidance using a LIDAR."""

    def __init__(self) -> None:
        super().__init__('jetbot_obstacle_avoidance')

        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('obstacle_threshold', 0.35)
        self.declare_parameter('avoidance_mode', 'simple')  # 'simple' | 'advanced'
        # Advanced-only sensitivity: detect earlier than threshold (multiplier > 1)
        self.declare_parameter('advanced_detect_factor', 1.8)
        # Advanced-only: prefer rotate-in-place when close (>=1.0 rotates even before threshold)
        self.declare_parameter('advanced_rotate_factor', 1.2)
        # Advanced-only: front cone aperture in degrees (total span)
        self.declare_parameter('advanced_front_cone_deg', 90.0)
    # Advanced-only: factor to keep a wider lateral clearance during avoidance
    self.declare_parameter('safe_clearance_factor', 1.8)

        # Resolve params
        self._v = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        self._w = float(self.get_parameter('angular_speed').get_parameter_value().double_value)
        self._threshold = float(self.get_parameter('obstacle_threshold').get_parameter_value().double_value)
        self._mode = str(
            self.get_parameter('avoidance_mode').get_parameter_value().string_value or 'simple'
        ).lower()
        self._adv_detect_factor = float(
            self.get_parameter('advanced_detect_factor').get_parameter_value().double_value or 1.3
        )
        self._rotate_factor = float(
            self.get_parameter('advanced_rotate_factor').get_parameter_value().double_value or 1.05
        )
        self._safe_clearance_factor = float(
            self.get_parameter('safe_clearance_factor').get_parameter_value().double_value or 1.8
        )
        try:
            front_cone_param = float(
                self.get_parameter('advanced_front_cone_deg').get_parameter_value().double_value
            )
        except Exception:
            front_cone_param = 60.0

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
        qos_cmd = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
        )
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_cmd)

        # Subscriber (sensor data QoS for /scan)
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
        )

        # State
        self._last_scan: Optional[LaserScan] = None
        self._state: str = 'forward'  # 'forward' | 'stopped' | 'avoid_left' | 'avoid_right'
        self._last_turn_dir: int = +1  # +1 left, -1 right (advanced)
        self._avoid_start_sec: Optional[float] = None

        # Advanced-mode smoothing: keep short history of front distance
        self._front_hist: Deque[float] = deque(maxlen=3)

    # Advanced-mode return-to-straight helpers
    self._recenter_active: bool = False
    self._recenter_start_sec: Optional[float] = None
    self._recenter_duration: float = 1.5  # seconds to ramp angular speed down to 0
    self._return_dir: int = 0  # opposite sign of last avoidance to recenter
    self._last_dir_change_sec: Optional[float] = None  # rate limit direction flips
    self._clear_since_sec: Optional[float] = None  # time since front is clear over resume

        # LIDAR indexing cache (computed on first scan or when scan meta changes)
        self._idx_ready: bool = False
        self._idx_n: int = 0
        self._idx_zero: int = 0
        self._deg_per_index: float = 1.0  # degrees per index
        self._angle_span_deg: float = 0.0
        # Cached sector index ranges (inclusive tuples)
        self._idx_front: Tuple[int, int] = (0, 0)
        self._idx_left: Tuple[int, int] = (0, 0)
        self._idx_right: Tuple[int, int] = (0, 0)

        # Advanced-mode tuning (does not affect simple mode)
        self._resume_factor = 1.25  # need a bit more clearance before resuming forward
        self._front_cone_deg = front_cone_param  # degrees around the front to assess obstacle
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
        # Prepare/correct indexer if needed
        self._ensure_indexer(msg)

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

    def _ensure_indexer(self, scan: LaserScan) -> None:
        """Compute and cache LIDAR index mapping and common sector index ranges.
        We define 0 deg as the real front of the robot = middle index of ranges.
        """
        n = len(scan.ranges)
        if n <= 0:
            self._idx_ready = False
            return
        # Compute only if first time or scan meta changed
        angle_span = float(scan.angle_max - scan.angle_min)
        angle_span_deg = angle_span * 180.0 / math.pi
        if (
            not self._idx_ready
            or self._idx_n != n
            or abs(angle_span_deg - self._angle_span_deg) > 1e-6
        ):
            self._idx_n = n
            # FIX: Force front index to 0 to match simple mode (front at array edges)
            # Many JetBot/TurtleBot3 setups publish scans from ~0 to ~2π, where index 0 (and n-1) look forward.
            # Advanced mode will share this reference to ensure front sectors coincide with simple mode.
            min_rad = float(scan.angle_min)
            max_rad = float(scan.angle_max)
            self._idx_zero = 0  # FIX:
            # Degrees per index from total FoV; independent of angle_min origin
            self._angle_span_deg = angle_span_deg
            self._deg_per_index = (angle_span_deg / max(1, n))
            # DEBUG: Log angular metadata and chosen zero index
            self.get_logger().info(
                f"[DEBUG] angle_min={min_rad:.3f}rad angle_max={max_rad:.3f}rad idx_zero={self._idx_zero} fov_deg={self._angle_span_deg:.1f}"
            )

            # Helper to convert degrees around front to index (clamped)
            def deg_to_idx(deg: float) -> int:
                # FIX: Use modular wrap to support [0, 2pi] scans where negative degrees wrap to end
                i_float = self._idx_zero + (deg / self._deg_per_index)
                i = int(round(i_float)) % self._idx_n
                return i

            # Precompute sector index ranges based on configured degrees
            half_front = 0.5 * self._front_cone_deg
            f0, f1 = deg_to_idx(-half_front), deg_to_idx(+half_front)
            if f0 > f1:
                f0, f1 = f1, f0
            self._idx_front = (f0, f1)

            # Side sectors: Left +inner..+outer, Right -outer..-inner
            li, lo = deg_to_idx(self._side_inner_deg), deg_to_idx(self._side_outer_deg)
            if li > lo:
                li, lo = lo, li
            self._idx_left = (li, lo)

            ro, ri = deg_to_idx(-self._side_outer_deg), deg_to_idx(-self._side_inner_deg)
            if ro > ri:
                ro, ri = ri, ro
            self._idx_right = (ro, ri)

            self._idx_ready = True

    def _sector_values_by_deg(self, min_deg: float, max_deg: float) -> List[float]:
        """Return finite scan values within [min_deg, max_deg] using zero at middle index.
        Uses cached deg-to-index mapping for efficiency.
        """
        if self._last_scan is None:
            return []
        scan = self._last_scan
        n = len(scan.ranges)
        if not self._idx_ready or n == 0:
            return []

        def deg_to_idx(deg: float) -> int:
            # FIX: Modular wrap to handle sectors that cross index boundaries
            i_float = self._idx_zero + (deg / self._deg_per_index)
            return int(round(i_float)) % self._idx_n

        i0, i1 = deg_to_idx(min_deg), deg_to_idx(max_deg)
        if i0 <= i1:
            sector = list(scan.ranges[i0 : i1 + 1])
        else:
            # FIX: Sector crosses the array end; concatenate wrapped slices
            sector = list(scan.ranges[i0:]) + list(scan.ranges[: i1 + 1])
        return self._finite_values(sector)

    @staticmethod
    def _percentile(vals: List[float], p: float) -> Optional[float]:
        if not vals:
            return None
        vals_sorted = sorted(vals)
        k = max(0, min(len(vals_sorted) - 1, int(round((p / 100.0) * (len(vals_sorted) - 1)))))
        return vals_sorted[k]

    def _front_distance_adv(self) -> Optional[float]:
        """More robust front distance: 20th percentile in +/- front_cone/2."""
        half = 0.5 * self._front_cone_deg
        vals = self._sector_values_by_deg(-half, +half)
        # If angle info is missing or empty, fall back
        if not vals:
            return self._front_distance()
        return self._percentile(vals, 20.0)

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
        right_sector = r[n // 8 : n // 4]  # ~45-90 deg on right
        left_sector = r[-n // 4 : -n // 8]  # ~-90 to -45 deg on left (wrap)
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
            self.get_logger().info(
                'Evitando obstáculo — girando a la izquierda.' + (f' ({reason})' if reason else '')
            )
        elif new_state == 'avoid_right':
            self.get_logger().info(
                'Evitando obstáculo — girando a la derecha.' + (f' ({reason})' if reason else '')
            )

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
            # FIX: Use the same front distance method as simple mode
            d_front_base = self._front_distance()  # FIX: same as simple
            # Keep smoothing only in advanced
            if d_front_base is not None:
                self._front_hist.append(d_front_base)  # FIX: smoothing based on _front_distance()
            # Compute smoothed front distance
            if self._mode == 'advanced':
                d_front_smooth = sum(self._front_hist) / len(self._front_hist)
            else:
                d_front_smooth = d_front_base if d_front_base is not None else d_front_simple  # FIX

            # Side clearances for decision and logging
            left_clear, right_clear = self._side_clearance_adv()

            # Compact debug log
            def fmt(x: Optional[float]) -> str:
                return f"{x:.2f}" if x is not None else "nan"

            self.get_logger().info(
                f"[ADV] front={fmt(d_front_smooth)} L={fmt(left_clear)} R={fmt(right_clear)} state={self._state}"
            )

            d_check = d_front_smooth if d_front_smooth is not None else d_front_simple  # FIX
            if d_check is not None and d_check < (self._threshold * self._adv_detect_factor):
                # Determine desired turn direction based on clearances
                if left_clear is None and right_clear is None:
                    # No side info: rotate to search using last direction
                    turn_dir = self._last_turn_dir
                    self._set_state('avoid_left' if turn_dir > 0 else 'avoid_right', 'escaneo insuficiente')
                now = self._now_sec()

                # Safety: never advance if too close to front
                too_close = (d_front_smooth is not None) and (d_front_smooth < 0.9 * self._threshold)

                # Re-centering phase: gradually reduce rotation to 0 and keep reduced forward speed
                if self._recenter_active and self._recenter_start_sec is not None:
                    t = now - self._recenter_start_sec
                    if t >= self._recenter_duration:
                        # End recentering
                        self._recenter_active = False
                        self._recenter_start_sec = None
                        self._return_dir = 0
                        # fall through to normal forward/advanced handling
                    else:
                        w_scale = max(0.0, 1.0 - (t / self._recenter_duration))
                        wz = self._w * w_scale * (-1 if self._return_dir >= 0 else +1)
                        vx = self._v * 0.6 if not too_close else 0.0
                        self._publish(vx, wz)
                        return

                    self._avoid_start_sec = self._avoid_start_sec or self._now_sec()
                    self._publish(0.0, self._w * self._search_w_factor * turn_dir)
                    return
                    if left_clear is None and right_clear is None:
                        # Both sides unknown: rotate in place slowly using last direction
                        turn_dir = self._last_turn_dir or +1
                        self._set_state('avoid_left' if turn_dir > 0 else 'avoid_right', 'escaneo insuficiente')
                        self._avoid_start_sec = self._avoid_start_sec or now
                        self._publish(0.0, self._w * self._search_w_factor * 0.7 * turn_dir)
                        return

                    # Prefer side with greater margin above safe clearance threshold
                    clear_req = self._threshold * self._safe_clearance_factor
                    left_margin = (left_clear - clear_req) if (left_clear is not None) else -1e9
                    right_margin = (right_clear - clear_req) if (right_clear is not None) else -1e9

                    if left_margin <= 0.0 and right_margin <= 0.0:
                        # Both sides constrained: stop briefly and rotate in place slowly
                        turn_dir = self._last_turn_dir or +1
                        self._set_state('avoid_left' if turn_dir > 0 else 'avoid_right', 'ambos lados cerca')
                        self._avoid_start_sec = self._avoid_start_sec or now
                        self._publish(0.0, self._w * self._search_w_factor * 0.7 * turn_dir)
                        return

                    desired_dir = +1 if left_margin > right_margin else -1

                    # Rate-limit direction flips to at most once per second
                    turn_dir = desired_dir
                    if self._last_dir_change_sec is not None and desired_dir != self._last_turn_dir:
                        if (now - self._last_dir_change_sec) < 1.0:
                            turn_dir = self._last_turn_dir
                    if turn_dir != self._last_turn_dir:
                        self._last_dir_change_sec = now
                    self._last_turn_dir = turn_dir
                    turn_dir = desired_dir

                # Update state text based on direction
                if turn_dir > 0:
                            turn_dir = self._last_turn_dir
                else:
                            turn_dir = self._last_turn_dir

                        turn_dir = self._last_turn_dir
                if d_check < (self._threshold * self._rotate_factor):
                    if self._avoid_start_sec is None:
                        self._avoid_start_sec = now
                    self._publish(0.0, self._w * turn_dir)
                    self._last_turn_dir = turn_dir
                    return

                # Stuck detection using smoothed front distance
                if self._avoid_start_sec is None:
                    self._avoid_start_sec = now
                if d_check < (0.8 * self._threshold) and (now - self._avoid_start_sec) > self._stuck_timeout:
                    # Strong rotate-in-place to break free
                    self._publish(0.0, self._w * 1.5 * turn_dir)
                    self._last_turn_dir = turn_dir
                    # Reset timer after a rotate-in-place attempt
                    self._avoid_start_sec = now
                    return

                # Normal avoid: creep forward while turning toward chosen side
                vx = (self._v * self._creep_factor) if not too_close else 0.0
                self._publish(vx, self._w * turn_dir)
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
            # If we were avoiding, require a bit of extra clearance to start recentering
            d_front_base = self._front_distance()  # FIX: same method as simple
            if d_front_base is not None:
                # update smoothing too so resume uses recent history
                self._front_hist.append(d_front_base)  # FIX
            d_front_smooth = sum(self._front_hist) / len(self._front_hist) if self._front_hist else d_front_base  # FIX

            if self._state in ('avoid_left', 'avoid_right') and d_front_smooth is not None and not self._recenter_active:
                if d_front_smooth >= self._threshold * self._resume_factor:
                    # Start or continue counting clear time
                    now2 = self._now_sec()
                    if self._clear_since_sec is None:
                        self._clear_since_sec = now2
                    elif (now2 - self._clear_since_sec) >= 1.0:
                        # Trigger recentering phase
                        self._recenter_active = True
                        self._recenter_start_sec = now2
                        self._return_dir = -self._last_turn_dir
                        # During recentering, we take over control at the top on next tick
                        # Keep moving gently this tick
                        self._publish(self._v * 0.6, self._w * 0.5 * (-1 if self._return_dir >= 0 else +1))
                        return
                    # Keep avoiding gently until we complete the 1s window
                    turn_dir = +1 if self._state == 'avoid_left' else -1
                    self._publish(self._v * self._creep_factor, self._w * turn_dir)
                    return
                else:
                    # Not clear enough; reset clear timer
                    self._clear_since_sec = None

            # When fully clear and not avoiding, reset timers/state trackers
            if self._state not in ('avoid_left', 'avoid_right'):
                self._clear_since_sec = None
                self._recenter_active = False
                self._recenter_start_sec = None
            # Reset avoid timer when resuming forward; keep last turn dir
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
