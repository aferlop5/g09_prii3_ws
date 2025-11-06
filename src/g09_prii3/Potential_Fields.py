import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import time

# Minimal local implementation for yaw extraction from quaternion to avoid external deps here
def euler_from_quaternion(q):
    x, y, z, w = q
    # roll and pitch not needed; compute yaw directly
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return 0.0, 0.0, yaw

class PotentialFieldsNavigator(Node):
    def __init__(self):
        super().__init__('potential_fields_nav')
        self.declare_parameter('k_att', 1.0)
        # Ajustes más "apurados": menor alcance y menor ganancia repulsiva por defecto
        self.declare_parameter('k_rep', 0.32)
        self.declare_parameter('d0_rep', 0.55)
        self.declare_parameter('max_lin_vel', 0.3)
        self.declare_parameter('max_ang_vel', 1.0)
        self.declare_parameter('escape_gain', 0.2)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        # Parámetros nuevos
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('odom_topic', '/odom')
        # Ganancias de control
        self.declare_parameter('ang_gain', 1.5)
        self.declare_parameter('lin_gain', 1.0)
        # Escalado de velocidad cerca de obstáculos (min ratio)
        self.declare_parameter('slowdown_min_scale', 0.2)
        # Ponderación angular del repulsivo y suavizado de comandos
        self.declare_parameter('front_weight_deg', 80.0)
        self.declare_parameter('rep_scale_side', 0.42)
        self.declare_parameter('smooth_alpha', 0.4)
        # Detección de estancamiento (tiempo sin progreso hacia la meta)
        self.declare_parameter('stuck_timeout', 3.0)
        # Fallback para atascos: seguir la mayor apertura (Follow-The-Gap)
        self.declare_parameter('use_gap_follow', True)
        self.declare_parameter('gap_clear_threshold', 0.55)  # m; si None usa d0_rep
        self.declare_parameter('gap_min_width_deg', 12.0)
        self.declare_parameter('gap_prefer_goal_weight', 0.6)  # [0..1] 1=todo objetivo, 0=solo apertura más grande
        self.declare_parameter('recovery_mode', 'spin+gap')  # 'spin' | 'gap' | 'spin+gap'
        self.declare_parameter('recovery_gap_duration', 3.0)
        # Seguimiento de paredes (wall-follow) como fallback adicional
        self.declare_parameter('use_wall_follow', True)
        self.declare_parameter('wall_side', 'auto')  # 'auto' | 'left' | 'right'
        self.declare_parameter('wall_distance', 0.38)
        self.declare_parameter('wall_kp', 1.2)
        self.declare_parameter('wall_lin_vel', 0.28)
        self.declare_parameter('wall_timeout', 6.0)
        # Cambio de pared en esquinas
        self.declare_parameter('wall_front_switch_thresh', 0.45)
        self.declare_parameter('wall_switch_margin', 0.07)
        self.declare_parameter('wall_switch_cooldown', 1.5)

        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.d0 = self.get_parameter('d0_rep').value
        self.v_max = self.get_parameter('max_lin_vel').value
        self.w_max = self.get_parameter('max_ang_vel').value
        self.escape_gain = self.get_parameter('escape_gain').value

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.ang_gain = self.get_parameter('ang_gain').value
        self.lin_gain = self.get_parameter('lin_gain').value
        self.slowdown_min_scale = self.get_parameter('slowdown_min_scale').value
        self.front_weight_deg = self.get_parameter('front_weight_deg').value
        self.rep_scale_side = self.get_parameter('rep_scale_side').value
        self.smooth_alpha = max(0.0, min(1.0, self.get_parameter('smooth_alpha').value))
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.use_gap_follow = bool(self.get_parameter('use_gap_follow').value)
        self.gap_clear_threshold = self.get_parameter('gap_clear_threshold').value
        if self.gap_clear_threshold is None:
            self.gap_clear_threshold = self.d0
        self.gap_min_width_deg = float(self.get_parameter('gap_min_width_deg').value)
        self.gap_prefer_goal_weight = float(self.get_parameter('gap_prefer_goal_weight').value)
        self.recovery_mode = str(self.get_parameter('recovery_mode').value or 'gap').lower()
        self.recovery_gap_duration = float(self.get_parameter('recovery_gap_duration').value)
        self.use_wall_follow = bool(self.get_parameter('use_wall_follow').value)
        self.wall_side = str(self.get_parameter('wall_side').value or 'auto').lower()
        if self.wall_side not in ('auto','left','right'):
            self.wall_side = 'auto'
        self.wall_distance = float(self.get_parameter('wall_distance').value)
        self.wall_kp = float(self.get_parameter('wall_kp').value)
        self.wall_lin_vel = float(self.get_parameter('wall_lin_vel').value)
        self.wall_timeout = float(self.get_parameter('wall_timeout').value)
        self.wall_front_switch_thresh = float(self.get_parameter('wall_front_switch_thresh').value)
        self.wall_switch_margin = float(self.get_parameter('wall_switch_margin').value)
        self.wall_switch_cooldown = float(self.get_parameter('wall_switch_cooldown').value)

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_scan = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.reached_goal_logged = False
        # Estados para suavizado y recuperación
        self.v_filt = 0.0
        self.w_filt = 0.0
        self.in_recovery = False
        self.recovery_end_time = 0.0
        self.last_goal_dist = float('inf')
        self.last_progress_time = self.get_clock().now().nanoseconds * 1e-9
        self.in_gap_follow = False
        self.gap_follow_end_time = 0.0
        self.in_wall_follow = False
        self.wall_follow_end_time = 0.0
        self.wall_follow_side = 'left'
        self.last_wall_switch_time = 0.0

        self.create_timer(0.05, self.control_loop)

    # Public stop helper to ensure robot halts on shutdown/interruption
    def stop_robot(self):
        try:
            zero = Twist()
            zero.linear.x = 0.0
            zero.angular.z = 0.0
            self.pub_cmd.publish(zero)
        except Exception:
            # Best-effort during teardown
            pass

    # Ensure a stop is sent when the node is destroyed (extra safety)
    def destroy_node(self):  # type: ignore[override]
        try:
            self.stop_robot()
        finally:
            return super().destroy_node()

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x = float(pos.x)
        self.robot_y = float(pos.y)
        q = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(q)
        self.robot_yaw = float(yaw)
        self.odom_received = True

    def compute_force_vector(self):
        # Si aún no hay láser, aplicamos solo fuerza atractiva hacia la meta
        if self.last_scan is None:
            if self.odom_received:
                dx_w = self.goal_x - self.robot_x
                dy_w = self.goal_y - self.robot_y
                cy = math.cos(self.robot_yaw)
                sy = math.sin(self.robot_yaw)
                dx = dx_w * cy + dy_w * sy
                dy = -dx_w * sy + dy_w * cy
            else:
                dx = self.goal_x
                dy = self.goal_y
            return self.k_att * np.array([dx, dy])

        scan = self.last_scan
        # Construye los ángulos con el mismo tamaño que ranges para evitar desajustes
        ranges = np.array(scan.ranges)
        num = ranges.shape[0]
        angles = scan.angle_min + np.arange(num) * scan.angle_increment
        # Filtrar valores inválidos
        valid = np.isfinite(ranges) & (ranges > 0.0)
        angles = angles[valid]
        ranges = ranges[valid]

        # vector resultante
        F = np.array([0.0, 0.0])

        # campo atractivo hacia meta (usar odometría si está disponible)
        if self.odom_received:
            dx_w = self.goal_x - self.robot_x
            dy_w = self.goal_y - self.robot_y
            # Transformar a frame del robot (rotación -yaw)
            cy = math.cos(self.robot_yaw)
            sy = math.sin(self.robot_yaw)
            dx = dx_w * cy + dy_w * sy
            dy = -dx_w * sy + dy_w * cy
        else:
            # Sin odometría aún: suponer robot en el origen con yaw=0
            dx = self.goal_x
            dy = self.goal_y
        # Fuerza atractiva con saturación de norma para evitar excesos
        att_vec = np.array([dx, dy])
        att_norm = np.linalg.norm(att_vec)
        if att_norm > 1e-6:
            att_vec = att_vec / att_norm * min(att_norm, 1.0)  # saturación de atracción
        F_att = self.k_att * att_vec
        F += F_att

        # Campo repulsivo con muestreo por sectores y ponderación angular
        if ranges.size > 0:
            # Definir bins y usar percentil para estabilizar
            rep_bins = 36  # ~10 grados por sector si FOV ~360
            p = 30.0       # percentil 30 para evitar outliers
            a_min = float(angles.min())
            a_max = float(angles.max())
            bin_edges = np.linspace(a_min, a_max, rep_bins + 1)
            front_rad = math.radians(self.front_weight_deg)
            for i in range(rep_bins):
                a0 = bin_edges[i]
                a1 = bin_edges[i + 1]
                mask = (angles >= a0) & (angles < a1)
                if not np.any(mask):
                    continue
                r_bin = ranges[mask]
                # Filtrar por debajo de d0 para contribución
                r_eff = np.percentile(r_bin, p)
                theta = 0.5 * (a0 + a1)
                if r_eff < self.d0:
                    # Ponderación angular: más peso en el frontal
                    w_ang = 1.0
                    if abs(theta) <= front_rad:
                        w_ang = max(0.0, math.cos(theta))  # [0..1] en frontal
                    else:
                        w_ang = self.rep_scale_side * max(0.0, math.cos(theta))

                    if w_ang <= 0.0:
                        continue

                    magnitude = self.k_rep * (1.0 / r_eff - 1.0 / self.d0) * (1.0 / (r_eff * r_eff))
                    magnitude *= w_ang
                    ux = -math.cos(theta)
                    uy = -math.sin(theta)
                    F += magnitude * np.array([ux, uy])

        # Si estamos cerca de la meta, anular fuerzas para evitar oscilaciones
        if self.odom_received:
            dist_goal = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
            if dist_goal <= self.goal_tolerance:
                F = np.array([0.0, 0.0])
                if not self.reached_goal_logged:
                    self.get_logger().info(f"Objetivo alcanzado (dist={dist_goal:.3f} m). Parando.")
                    self.reached_goal_logged = True
                return F
            else:
                self.reached_goal_logged = False

        # si F es muy pequeño → posible mínimo local, aplicar fuerza de escape
        normF = np.linalg.norm(F)
        if normF < 1e-3:
            # aplica una pequeña perturbación (rotación aleatoria) para escapar
            angle = np.random.uniform(-math.pi, math.pi)
            F_escape = self.escape_gain * np.array([math.cos(angle), math.sin(angle)])
            F += F_escape

        return F

    def control_loop(self):
        F = self.compute_force_vector()
        # convertir a polar
        fx, fy = F
        angle = math.atan2(fy, fx)
        magnitude = math.hypot(fx, fy)

        # Ganancia angular y saturación
        w = max(-self.w_max, min(self.w_max, self.ang_gain * angle))

        # Escalado de velocidad lineal por orientación
        angle_scale = 1.0
        if abs(angle) > math.pi / 6:
            angle_scale = 0.3

        # Escalado por proximidad a obstáculos
        obstacle_scale = 1.0
        if self.last_scan is not None:
            ranges = np.array(self.last_scan.ranges)
            valid = np.isfinite(ranges) & (ranges > 0.0)
            if np.any(valid):
                rmin = float(np.min(ranges[valid]))
                if rmin < self.d0 and self.d0 > 1e-6:
                    obstacle_scale = max(self.slowdown_min_scale, rmin / self.d0)

        # velocidad lineal con ganancia, escalados y saturación
        v_cmd = self.lin_gain * magnitude * angle_scale * obstacle_scale
        v = min(self.v_max, v_cmd)

        # si el ángulo es grande, reducir la velocidad lineal para priorizar giro
        if abs(angle) > math.pi / 6:
            v *= 0.3

        # Detección de estancamiento y recuperación (spin-in-place)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.odom_received:
            dist_goal = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
            # progreso si disminuye la distancia de forma apreciable
            if dist_goal < self.last_goal_dist - 0.01:
                self.last_progress_time = now
                self.last_goal_dist = dist_goal
            # activar recuperación si no hay progreso por stuck_timeout y no estamos en tolerancia
            if (now - self.last_progress_time > self.stuck_timeout) and (dist_goal > self.goal_tolerance):
                self.in_recovery = True
                self.recovery_end_time = now + 1.5  # segundos de giro
                # Elegir dirección de giro según espacio libre
                turn_dir = 1.0
                if self.last_scan is not None:
                    ranges = np.array(self.last_scan.ranges)
                    valid = np.isfinite(ranges) & (ranges > 0.0)
                    if np.any(valid):
                        n = ranges.shape[0]
                        left = ranges[int(n*0.5):] if n > 0 else []
                        right = ranges[:int(n*0.5)] if n > 0 else []
                        mean_left = float(np.nanmean(left)) if len(left) > 0 else 0.0
                        mean_right = float(np.nanmean(right)) if len(right) > 0 else 0.0
                        turn_dir = 1.0 if mean_left > mean_right else -1.0
                w = turn_dir * min(self.w_max, 0.8 * self.w_max)
                v = 0.0
                self.last_progress_time = now  # evita rearmado continuo

        if self.in_recovery:
            if now >= self.recovery_end_time:
                self.in_recovery = False

        # Suavizado de comandos (low-pass)
        # Gap-follow fallback si estamos en recuperación o sin progreso
        if self.use_gap_follow and (self.in_recovery or (now - self.last_progress_time > self.stuck_timeout)):
            if not self.in_gap_follow:
                self.in_gap_follow = True
                self.gap_follow_end_time = now + max(1.0, self.recovery_gap_duration)
            # Elegir un ángulo destino basado en aperturas del láser
            gap_ok, target_heading = self._select_gap_heading()
            if gap_ok:
                # Comandar para orientarse al centro de la apertura y avanzar suave
                # Mezclar con objetivo según peso para no perder dirección global
                goal_heading = angle  # ya es el ángulo del vector resultante
                mixed = self._mix_angles(goal_heading, target_heading, self.gap_prefer_goal_weight)
                # Control simple: priorizar giro si desalineado
                heading_err = self._angle_diff(mixed, 0.0)
                w = max(-self.w_max, min(self.w_max, self.ang_gain * heading_err))
                v = self.v_max * (0.4 if abs(heading_err) > math.pi/6 else 0.6)
                # Suavizado de comandos (low-pass)
                self.v_filt = self.smooth_alpha * v + (1.0 - self.smooth_alpha) * self.v_filt
                self.w_filt = self.smooth_alpha * w + (1.0 - self.smooth_alpha) * self.w_filt
                cmd = Twist()
                cmd.linear.x = float(self.v_filt)
                cmd.angular.z = float(self.w_filt)
                self.pub_cmd.publish(cmd)
                if now >= self.gap_follow_end_time:
                    self.in_gap_follow = False
                return
            else:
                # Si no hay aperturas útiles, intentar wall-follow si está activado
                if self.use_wall_follow:
                    if not self.in_wall_follow:
                        self.in_wall_follow = True
                        self.wall_follow_end_time = now + max(2.0, self.wall_timeout)
                        # Elegir lado: auto = según signo del heading a la meta (positivo → izquierda)
                        self.wall_follow_side = self._choose_wall_side()
                    v, w = self._wall_follow_control()
                    # suavizado
                    self.v_filt = self.smooth_alpha * v + (1.0 - self.smooth_alpha) * self.v_filt
                    self.w_filt = self.smooth_alpha * w + (1.0 - self.smooth_alpha) * self.w_filt
                    cmd = Twist()
                    cmd.linear.x = float(self.v_filt)
                    cmd.angular.z = float(self.w_filt)
                    self.pub_cmd.publish(cmd)
                    if now >= self.wall_follow_end_time:
                        self.in_wall_follow = False
                    return
                # Si wall-follow no está activo, dependiendo del modo hacer spin
                if self.recovery_mode in ('spin', 'spin+gap'):
                    w = min(self.w_max, 0.8 * self.w_max)
                    v = 0.0
                    self.v_filt = self.smooth_alpha * v + (1.0 - self.smooth_alpha) * self.v_filt
                    self.w_filt = self.smooth_alpha * w + (1.0 - self.smooth_alpha) * self.w_filt
                    cmd = Twist()
                    cmd.linear.x = float(self.v_filt)
                    cmd.angular.z = float(self.w_filt)
                    self.pub_cmd.publish(cmd)
                    return
                # si recovery_mode='gap' continuar con control normal (sin return)
        self.v_filt = self.smooth_alpha * v + (1.0 - self.smooth_alpha) * self.v_filt
        self.w_filt = self.smooth_alpha * w + (1.0 - self.smooth_alpha) * self.w_filt

        cmd = Twist()
        cmd.linear.x = float(self.v_filt)
        cmd.angular.z = float(self.w_filt)
        self.pub_cmd.publish(cmd)

    def update_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.get_logger().info(f"Nuevo objetivo: ({x:.2f}, {y:.2f})")
    # -------------------- Gap-follow helpers --------------------
    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Return smallest signed angle a-b in [-pi, pi]."""
        d = (a - b + math.pi) % (2 * math.pi) - math.pi
        return d

    @staticmethod
    def _mix_angles(a: float, b: float, wa: float) -> float:
        """Weighted mix of two angles. wa in [0..1] weights angle a vs b."""
        # Convert to vectors to avoid wrap issues
        va = np.array([math.cos(a), math.sin(a)])
        vb = np.array([math.cos(b), math.sin(b)])
        v = wa * va + (1.0 - wa) * vb
        if np.linalg.norm(v) < 1e-9:
            return 0.0
        return math.atan2(v[1], v[0])

    def _select_gap_heading(self):
        """Detect useful gaps in LIDAR and return (ok, heading_rad) for the chosen gap.
        Heading is in robot frame (0 forward, +left)."""
        scan = self.last_scan
        if scan is None or len(scan.ranges) == 0 or scan.angle_increment == 0.0:
            return False, 0.0
        ranges = np.array(scan.ranges)
        valid = np.isfinite(ranges) & (ranges > 0.0)
        if not np.any(valid):
            return False, 0.0
        ranges = ranges.copy()
        # Clip invalid to a small value so they count as blocked
        ranges[~valid] = 0.0
        n = ranges.shape[0]
        thr = float(self.gap_clear_threshold)
        free = ranges > thr
        # Find contiguous free segments
        gaps = []  # list of (i0, i1, width, center_idx)
        i = 0
        while i < n:
            if free[i]:
                start = i
                while i < n and free[i]:
                    i += 1
                end = i - 1
                width = end - start + 1
                gaps.append((start, end, width, start + width // 2))
            i += 1
        if not gaps:
            return False, 0.0
        # Convert indices to angles and filter by min angular width
        min_w = max(1, int(round(math.radians(self.gap_min_width_deg) / max(1e-9, scan.angle_increment))))
        cand = []  # (score, center_angle, width)
        # Determine desired goal heading (angle of attractive vector in robot frame)
        if self.odom_received:
            dx_w = self.goal_x - self.robot_x
            dy_w = self.goal_y - self.robot_y
            cy = math.cos(self.robot_yaw)
            sy = math.sin(self.robot_yaw)
            dx = dx_w * cy + dy_w * sy
            dy = -dx_w * sy + dy_w * cy
            goal_heading = math.atan2(dy, dx)
        else:
            goal_heading = 0.0
        for (i0, i1, w, ic) in gaps:
            if w < min_w:
                continue
            a_center = scan.angle_min + ic * scan.angle_increment
            # Score: blend of width (clearance proxy) and closeness to goal heading
            width_score = w / n
            ang_err = abs(self._angle_diff(goal_heading, a_center))
            ang_score = 1.0 - min(1.0, ang_err / math.pi)  # 1 best when aligned
            score = (1.0 - self.gap_prefer_goal_weight) * width_score + self.gap_prefer_goal_weight * ang_score
            cand.append((score, a_center, w))
        if not cand:
            return False, 0.0
        cand.sort(key=lambda x: x[0], reverse=True)
        return True, cand[0][1]

    # -------------------- Wall-follow helpers --------------------
    def _choose_wall_side(self) -> str:
        # Si hay odometría y un heading claro a la meta, usa su signo
        if self.odom_received:
            dx_w = self.goal_x - self.robot_x
            dy_w = self.goal_y - self.robot_y
            cy = math.cos(self.robot_yaw)
            sy = math.sin(self.robot_yaw)
            dx = dx_w * cy + dy_w * sy
            dy = -dx_w * sy + dy_w * cy
            goal_heading = math.atan2(dy, dx)
            if self.wall_side == 'auto':
                return 'left' if goal_heading >= 0.0 else 'right'
        if self.wall_side in ('left','right'):
            return self.wall_side
        return 'left'

    def _sector_values_by_deg(self, min_deg: float, max_deg: float):
        scan = self.last_scan
        if scan is None:
            return []
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return []
        a_min = scan.angle_min
        a_inc = scan.angle_increment
        lo = math.radians(min_deg)
        hi = math.radians(max_deg)
        def ang_to_idx(a: float) -> int:
            i = int(round((a - a_min) / a_inc))
            return max(0, min(n - 1, i))
        i0 = ang_to_idx(lo)
        i1 = ang_to_idx(hi)
        if i0 > i1:
            i0, i1 = i1, i0
        vals = [r for r in list(self.last_scan.ranges)[i0:i1+1] if r is not None and math.isfinite(r) and r > 0.0]
        return vals

    @staticmethod
    def _percentile_list(vals, p: float):
        if not vals:
            return None
        s = sorted(vals)
        k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
        return s[k]

    def _side_distances(self):
        # Sectores laterales ~60-120 grados
        left_vals = self._sector_values_by_deg(60.0, 120.0)
        right_vals = self._sector_values_by_deg(-120.0, -60.0)
        left = self._percentile_list(left_vals, 30.0) if left_vals else None
        right = self._percentile_list(right_vals, 30.0) if right_vals else None
        return left, right

    def _front_min(self):
        vals = self._sector_values_by_deg(-20.0, 20.0)
        if not vals:
            return None
        return min(vals) if vals else None

    def _wall_follow_control(self):
        # Control proporcional sencillo para mantener distancia a la pared seleccionada
        left, right = self._side_distances()
        side = self.wall_follow_side
        desired = self.wall_distance
        v = self.wall_lin_vel
        w = 0.0
        front = self._front_min()
        now = self.get_clock().now().nanoseconds * 1e-9
        # Detección de esquina y cambio de pared si procede
        corner_thresh = max(0.3, min(self.wall_front_switch_thresh, max(0.3, self.d0)))
        if self.use_wall_follow and front is not None and front < corner_thresh:
            preferred = side
            if left is not None and right is not None:
                if left + self.wall_switch_margin < right:
                    preferred = 'left'
                elif right + self.wall_switch_margin < left:
                    preferred = 'right'
            elif left is not None:
                preferred = 'left'
            elif right is not None:
                preferred = 'right'
            if preferred != side and (now - self.last_wall_switch_time) > self.wall_switch_cooldown:
                self.wall_follow_side = preferred
                self.last_wall_switch_time = now
                self.get_logger().info(f"Esquina detectada — cambiando a pared {preferred}.")
                side = preferred
        # Evitar colisión frontal
        if front is not None and front < max(0.3, 0.7 * self.d0):
            v *= 0.3
            w += 0.8 * self.w_max * (-1.0 if side == 'right' else 1.0)
        # Control lateral
        if side == 'left' and left is not None:
            e = desired - left  # positivo → demasiado lejos de la pared → girar a la izquierda
            w += self.wall_kp * e
        elif side == 'right' and right is not None:
            e = desired - right  # positivo → lejos de la pared → girar a la derecha (w negativo)
            w -= self.wall_kp * e
        # Limitar
        w = max(-self.w_max, min(self.w_max, w))
        v = min(self.v_max, max(0.0, v))
        return v, w

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldsNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown/interrupt, stop the robot explicitly
        node.get_logger().info('Apagando nodo de campos potenciales: enviando stop a /cmd_vel.')
        node.stop_robot()
        # tiny delay to allow message to flush
        time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
