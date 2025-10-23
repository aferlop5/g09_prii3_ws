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
        self.declare_parameter('k_rep', 0.3)
        self.declare_parameter('d0_rep', 0.5)
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
        self.declare_parameter('slowdown_min_scale', 0.4)
        # Ponderación angular del repulsivo y suavizado de comandos
        self.declare_parameter('front_weight_deg', 60.0)
        self.declare_parameter('rep_scale_side', 0.3)
        self.declare_parameter('smooth_alpha', 0.3)
        # Detección de estancamiento (tiempo sin progreso hacia la meta)
        self.declare_parameter('stuck_timeout', 5.0)

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
