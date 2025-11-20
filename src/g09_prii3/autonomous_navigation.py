#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time
import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class AutoNav(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Lista de puntos a los que queremos ir (x, y, yaw en radianes)
        self.waypoints = [
            (3.10, 2.0, math.pi/2),
            (-3.0, 6.0, math.pi),
            (3.10, 15.0, math.pi/2),
            (4.0, 15.0, 0.0),
            (-3.0, 15.0, math.pi)
        ]

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor de Navigation2...')

        self.get_logger().info('Servidor de Navigation2 activo')
        self.send_next_goal(0)

    def send_next_goal(self, idx):
        if idx >= len(self.waypoints):
            self.get_logger().info('Todos los puntos alcanzados. Terminando.')
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[idx]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'Enviando goal {idx+1}: x={x}, y={y}, yaw={yaw}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, idx))

    def goal_response_callback(self, future, idx):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rechazado')
            self.send_next_goal(idx + 1)
            return

        self.get_logger().info('Goal aceptado')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.get_result_callback(future, idx))

    def get_result_callback(self, future, idx):
        result = future.result().result
        self.get_logger().info(f'Goal {idx+1} alcanzado!')
        time.sleep(1.0)
        self.send_next_goal(idx + 1)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
