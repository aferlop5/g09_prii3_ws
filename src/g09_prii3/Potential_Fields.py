#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import List, Optional, Tuple
import math
import time

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

# tf2 for pose in global frame
from tf2_ros import Buffer, TransformListener

# Optional dependency: tf_transformations. Provide a safe fallback if missing.
try:
	from tf_transformations import euler_from_quaternion as _tf_euler_from_quaternion

	def euler_from_quaternion(q):
		# Keep the exact signature used in the code: [x, y, z, w] -> (roll, pitch, yaw)
		return _tf_euler_from_quaternion(q)
except Exception:
	# Minimal, dependency-free quaternion -> Euler implementation.
	# Accepts [x, y, z, w] and returns (roll, pitch, yaw) in radians.
	def euler_from_quaternion(q):
		x, y, z, w = q
		# Roll (x-axis rotation)
		sinr_cosp = 2.0 * (w * x + y * z)
		cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
		roll = math.atan2(sinr_cosp, cosr_cosp)

		# Pitch (y-axis rotation)
		sinp = 2.0 * (w * y - z * x)
		if abs(sinp) >= 1.0:
			pitch = math.copysign(math.pi / 2.0, sinp)
		else:
			pitch = math.asin(sinp)

		# Yaw (z-axis rotation)
		siny_cosp = 2.0 * (w * z + x * y)
		cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(siny_cosp, cosy_cosp)

		return roll, pitch, yaw


class JetbotAvoider(Node):
	"""ROS 2 node for JetBot navigation using Potential Fields with LIDAR avoidance."""

	def __init__(self) -> None:
		super().__init__('jetbot_potential_fields')

		# ---------------- Parameters ----------------
		# Goal in global frame (map/odom)
		self.declare_parameter('goal_x', 1.0)
		self.declare_parameter('goal_y', 0.0)
		self.declare_parameter('goal_tolerance', 0.10)  # meters

		# Frames
		self.declare_parameter('global_frame', 'odom')
		self.declare_parameter('base_frame', 'base_link')

		# Speeds
		self.declare_parameter('max_linear_speed', 0.25)
		self.declare_parameter('max_angular_speed', 1.2)
		self.declare_parameter('min_linear_speed', 0.06)  # sesgo para evitar quedarnos girando
		self.declare_parameter('heading_cruise_angle', 0.7)  # rad, ~40°: debajo de esto forzamos v>=min si despejado
		self.declare_parameter('front_cruise_clearance', 0.5)  # m, despeje para aplicar v_min
		self.declare_parameter('heading_smoothing_alpha', 0.3)  # filtro exponencial en heading

		# Potential fields gains
		self.declare_parameter('k_att', 1.0)  # attractive gain
		self.declare_parameter('k_rep', 0.4)  # repulsive gain
		self.declare_parameter('repulsive_radius', 0.8)  # influence distance (m)
		self.declare_parameter('min_range_clip', 0.05)  # clip too-small ranges

		# Safety/advanced heuristics (kept from advanced avoider spirit)
		self.declare_parameter('front_stop_threshold', 0.25)
		self.declare_parameter('search_turn_speed', 0.6)  # rad/s when searching

		# Wall-follow / local minima handling (small, cluttered spaces)
		self.declare_parameter('wall_follow_enter_front', 0.35)  # enter wall-follow if front < this
		self.declare_parameter('wall_follow_exit_front', 0.7)   # exit wall-follow when front > this
		self.declare_parameter('wall_follow_side', 'auto')       # auto | left | right
		self.declare_parameter('wall_distance', 0.35)            # desired distance to wall (m)
		self.declare_parameter('v_wall', 0.10)                   # linear speed in wall-follow
		self.declare_parameter('w_wall_gain', 1.0)               # angular gain in wall-follow
		self.declare_parameter('progress_timeout', 5.0)          # seconds without improving
		self.declare_parameter('progress_epsilon', 0.05)         # meters improvement threshold

		# Resolve params
		self._goal_x = float(self.get_parameter('goal_x').get_parameter_value().double_value)
		self._goal_y = float(self.get_parameter('goal_y').get_parameter_value().double_value)
		self._tol = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
		self._global_frame = str(self.get_parameter('global_frame').get_parameter_value().string_value or 'odom')
		self._base_frame = str(self.get_parameter('base_frame').get_parameter_value().string_value or 'base_link')
		self._v_max = float(self.get_parameter('max_linear_speed').get_parameter_value().double_value)
		self._w_max = float(self.get_parameter('max_angular_speed').get_parameter_value().double_value)
		self._v_min = float(self.get_parameter('min_linear_speed').get_parameter_value().double_value)
		self._heading_cruise = float(self.get_parameter('heading_cruise_angle').get_parameter_value().double_value)
		self._front_cruise_clearance = float(self.get_parameter('front_cruise_clearance').get_parameter_value().double_value)
		self._heading_alpha = float(self.get_parameter('heading_smoothing_alpha').get_parameter_value().double_value)
		self._k_att = float(self.get_parameter('k_att').get_parameter_value().double_value)
		self._k_rep = float(self.get_parameter('k_rep').get_parameter_value().double_value)
		self._rho0 = float(self.get_parameter('repulsive_radius').get_parameter_value().double_value)
		self._min_clip = float(self.get_parameter('min_range_clip').get_parameter_value().double_value)
		self._front_stop = float(self.get_parameter('front_stop_threshold').get_parameter_value().double_value)
		self._search_w = float(self.get_parameter('search_turn_speed').get_parameter_value().double_value)
		self._wf_enter = float(self.get_parameter('wall_follow_enter_front').get_parameter_value().double_value)
		self._wf_exit = float(self.get_parameter('wall_follow_exit_front').get_parameter_value().double_value)
		self._wf_side_pref = str(self.get_parameter('wall_follow_side').get_parameter_value().string_value or 'auto')
		self._wf_dist = float(self.get_parameter('wall_distance').get_parameter_value().double_value)
		self._v_wall = float(self.get_parameter('v_wall').get_parameter_value().double_value)
		self._w_wall_gain = float(self.get_parameter('w_wall_gain').get_parameter_value().double_value)
		self._prog_timeout = float(self.get_parameter('progress_timeout').get_parameter_value().double_value)
		self._prog_eps = float(self.get_parameter('progress_epsilon').get_parameter_value().double_value)

		# ---------------- Publishers / Subscribers ----------------
		qos_cmd = QoSProfile(
			depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
		)
		self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_cmd)
		self._scan_sub = self.create_subscription(
			LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
		)

		# ---------------- TF2 (pose in global frame) ----------------
		self._tf_buffer: Buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
		self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

		# ---------------- State ----------------
		self._last_scan: Optional[LaserScan] = None
		self._arrived: bool = False
		self._last_tf_warn_sec = 0.0
		self._heading_ema: Optional[float] = None  # filtro de heading para reducir oscilaciones
		# FSM de navegación
		self._mode: str = 'seek'  # 'seek' | 'wall'
		self._wall_side: int = 1  # +1=left, -1=right
		self._wall_start_dist: float = float('inf')
		# Seguimiento de progreso
		self._last_goal_dist: float = float('inf')
		self._last_progress_time: float = self._now_sec()
		self._heading_ema: Optional[float] = None  # filtro de heading para reducir oscilaciones

		# Control loop ~20 Hz
		self._timer = self.create_timer(0.05, self._control_loop)

		self.get_logger().info(
			f'Potential Fields listo -> goal=({self._goal_x:.2f}, {self._goal_y:.2f}) '
			f'frames: {self._global_frame}->{self._base_frame}, vmax={self._v_max:.2f}, wmax={self._w_max:.2f}'
		)

	# ------------------------- Callbacks -------------------------------------
	def _on_scan(self, msg: LaserScan) -> None:
		self._last_scan = msg

	# ------------------------- Helpers ---------------------------------------
	def _now_sec(self) -> float:
		return float(self.get_clock().now().nanoseconds) / 1e9

	@staticmethod
	def _finite_values(values: List[float]) -> List[float]:
		return [x for x in values if x is not None and not math.isinf(x) and not math.isnan(x)]

	def _front_distance(self) -> Optional[float]:
		if self._last_scan is None:
			return None
		ranges = list(self._last_scan.ranges)
		if not ranges:
			return None
		# Take a small frontal sector (first/last 15 samples)
		sector = ranges[:15] + ranges[-15:]
		vals = self._finite_values([max(r, self._min_clip) for r in sector])
		return min(vals) if vals else None

	def _side_distance(self, side: int) -> Optional[float]:
		"""Approx. distance to obstacle on side: +1 left sector, -1 right sector."""
		if self._last_scan is None:
			return None
		ranges = list(self._last_scan.ranges)
		n = len(ranges)
		if n == 0:
			return None
		# Define sectors ~60° on each side
		k = max(5, n // 12)
		if side >= 0:
			sector = ranges[n//2 : n//2 + k]  # left side (assuming CCW increasing)
		else:
			sector = ranges[n//2 - k : n//2]  # right side
		vals = self._finite_values([max(r, self._min_clip) for r in sector])
		if not vals:
			return None
		return sum(vals) / len(vals)

	def _goal_direction_base(self, rx: float, ry: float, yaw: float) -> float:
		"""Angle in base frame toward goal (radians)."""
		dx_g = self._goal_x - rx
		dy_g = self._goal_y - ry
		d_ang_global = math.atan2(dy_g, dx_g)
		# rotate to base frame (-yaw)
		ang_base = self._wrap_pi(d_ang_global - yaw)
		return ang_base

	@staticmethod
	def _wrap_pi(a: float) -> float:
		"""Wrap angle to [-pi, pi]."""
		while a > math.pi:
			a -= 2.0 * math.pi
		while a < -math.pi:
			a += 2.0 * math.pi
		return a

	def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
		"""Return (x, y, yaw) of base_frame in global_frame."""
		try:
			tf = self._tf_buffer.lookup_transform(
				self._global_frame, self._base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
			)
		except Exception:
			return None
		tx = tf.transform.translation.x
		ty = tf.transform.translation.y
		q = tf.transform.rotation
		yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
		return float(tx), float(ty), float(yaw)

	def _compute_attractive(self, rx: float, ry: float) -> Tuple[float, float]:
		# Vector from robot to goal in global frame
		dx_g = self._goal_x - rx
		dy_g = self._goal_y - ry
		# Attractive force in global frame
		fx_g = self._k_att * dx_g
		fy_g = self._k_att * dy_g
		return fx_g, fy_g

	def _compute_repulsive(self) -> Tuple[float, float]:
		"""Repulsive vector in base frame (x-forward, y-left)."""
		if self._last_scan is None:
			return 0.0, 0.0
		scan = self._last_scan
		fx_b = 0.0
		fy_b = 0.0
		# Iterate a decimated set of rays for performance
		n = len(scan.ranges)
		if n == 0 or scan.angle_increment == 0.0:
			return 0.0, 0.0
		step = max(1, n // 90)  # ~2 deg resolution
		rho0 = max(self._rho0, 1e-3)
		for i in range(0, n, step):
			r = scan.ranges[i]
			if r is None or math.isnan(r) or math.isinf(r):
				continue
			r = max(r, self._min_clip)
			if r > rho0:
				continue
			# Angle of this ray in base frame
			ang = scan.angle_min + i * scan.angle_increment
			# Obstacle position in base frame
			ox = r * math.cos(ang)
			oy = r * math.sin(ang)
			# Direction away from obstacle to robot (normalized)
			norm = math.hypot(ox, oy)
			if norm < 1e-6:
				continue
			# From artificial potential theory (Khatib): grad with (1/r - 1/rho0)/r^2 term
			scale = self._k_rep * ((1.0 / r) - (1.0 / rho0)) * (1.0 / (r * r))
			# Unit vector from obstacle to robot is (-ox, -oy)/norm
			fx_b += scale * (-ox / norm)
			fy_b += scale * (-oy / norm)
		return fx_b, fy_b

	@staticmethod
	def _rotate_global_to_base(fx_g: float, fy_g: float, yaw: float) -> Tuple[float, float]:
		# Base frame x-forward, y-left; rotation by -yaw
		cos_y = math.cos(-yaw)
		sin_y = math.sin(-yaw)
		fx_b = cos_y * fx_g - sin_y * fy_g
		fy_b = sin_y * fx_g + cos_y * fy_g
		return fx_b, fy_b

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
			pass

	def destroy_node(self) -> bool:  # type: ignore[override]
		self.stop_robot()
		return super().destroy_node()

	# ----------------------- Control loop ------------------------------------
	def _control_loop(self) -> None:
		# If arrived, keep stopped
		if self._arrived:
			self._publish(0.0, 0.0)
			return

		# Need scan and pose
		pose = self._lookup_robot_pose()
		if pose is None:
			# No TF yet; gentle search rotation as a fallback (log throttled)
			now = self._now_sec()
			if now - self._last_tf_warn_sec > 2.0:
				self.get_logger().warn('Esperando TF de pose; girando para sincronizar...')
				self._last_tf_warn_sec = now
			self._publish(0.0, self._search_w)
			return
		rx, ry, yaw = pose

		# Goal check
		dx = self._goal_x - rx
		dy = self._goal_y - ry
		dist_goal = math.hypot(dx, dy)
		if dist_goal <= self._tol:
			if not self._arrived:
				self.get_logger().info('Objetivo alcanzado. Deteniendo robot.')
			self._arrived = True
			self._publish(0.0, 0.0)
			return

		# Attractive (global), then rotate to base frame
		fx_att_g, fy_att_g = self._compute_attractive(rx, ry)
		fx_att_b, fy_att_b = self._rotate_global_to_base(fx_att_g, fy_att_g, yaw)

		# Repulsive (base)
		fx_rep_b, fy_rep_b = self._compute_repulsive()

		# Resultant in base
		fx_b = fx_att_b + fx_rep_b
		fy_b = fy_att_b + fy_rep_b

		# Convert forces to a heading and speeds with FSM for small, cluttered spaces
		heading_goal = self._goal_direction_base(rx, ry, yaw)
		heading_raw = math.atan2(fy_b, fx_b)
		mag = math.hypot(fx_b, fy_b)

		# Smooth heading
		if self._heading_ema is None:
			self._heading_ema = heading_raw
		else:
			err = self._wrap_pi(heading_raw - self._heading_ema)
			self._heading_ema = self._wrap_pi(self._heading_ema + self._heading_alpha * err)
		heading_pf = self._heading_ema

		d_front = self._front_distance()
		front_clear = (d_front is None) or (d_front >= self._front_cruise_clearance)

		# Progress monitor
		improved = (self._last_goal_dist - dist_goal) > self._prog_eps
		if improved:
			self._last_goal_dist = dist_goal
			self._last_progress_time = self._now_sec()
		else:
			# keep last values; possibly trigger wall-follow below
			pass

		# Mode transitions
		if self._mode == 'seek':
			enter = (d_front is not None and d_front < self._wf_enter) or \
					 (not front_clear and abs(heading_pf) > self._heading_cruise) or \
					 ((self._now_sec() - self._last_progress_time) > self._prog_timeout and dist_goal > self._tol)
			if enter:
				# choose side: prefer side with more space, or user preference
				if self._wf_side_pref in ('left', 'right'):
					self._wall_side = 1 if self._wf_side_pref == 'left' else -1
				else:
					left_d = self._side_distance(+1) or 0.0
					right_d = self._side_distance(-1) or 0.0
					self._wall_side = 1 if left_d >= right_d else -1
				self._mode = 'wall'
				self._wall_start_dist = dist_goal
		else:  # wall-follow
			# Control to keep distance to wall on selected side
			d_side = self._side_distance(self._wall_side)
			if d_side is None:
				d_side = float('inf')
			err = max(-1.0, min(1.0, (self._wf_dist - d_side)))  # positive if too far -> steer toward wall
			# target heading: ±90° plus small correction based on distance error
			heading_target = self._wall_side * (math.pi/2.0) + self._w_wall_gain * err
			# Exit condition: corridor clear toward goal and improved position
			exit_ok = (d_front is None or d_front > self._wf_exit) and abs(heading_goal) < 0.5 and (dist_goal < self._wall_start_dist - 0.1)
			if exit_ok:
				self._mode = 'seek'
				heading_pf = heading_goal
			else:
				heading_pf = heading_target

		# Angular velocity
		k_w = 1.2  # a bit calmer in tight spaces
		w = max(-self._w_max, min(self._w_max, k_w * heading_pf))

		# Linear velocity
		forward = mag * math.cos(heading_pf)
		v_scale_goal = max(0.2, min(1.0, dist_goal / max(self._tol, 1e-3)))
		v = max(0.0, min(self._v_max, forward)) * v_scale_goal
		if front_clear and abs(heading_pf) <= self._heading_cruise:
			v = max(v, min(self._v_min, self._v_max))
		if self._mode == 'wall':
			v = min(v, self._v_wall)

		# Safety stop if immediate obstacle
		if d_front is not None and d_front < self._front_stop:
			v = 0.0
			w = max(-self._w_max, min(self._w_max, k_w * (heading_pf + math.copysign(0.6, -heading_pf))))

		self._publish(v, w)


def main(args=None) -> None:
	rclpy.init(args=args)
	node = JetbotAvoider()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.get_logger().info('Apagando nodo PF: enviando stop a /cmd_vel.')
		node.stop_robot()
		time.sleep(0.05)
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

