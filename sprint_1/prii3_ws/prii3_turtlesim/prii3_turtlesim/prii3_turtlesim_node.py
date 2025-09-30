#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleNine(Node):
    def __init__(self):
        super().__init__('turtle_nine')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.create_timer(1.0, self.start_drawing)
        self.drawn = False

    def start_drawing(self):
        if self.drawn:
            return
        self.drawn = True
        self.get_logger().info("Dibujando el número 9")

        def move(distance, speed=1.0):
            msg = Twist()
            msg.linear.x = speed
            t0 = time.time()
            duration = distance / speed
            while time.time() - t0 < duration:
                self.pub.publish(msg)
                time.sleep(0.05)
            msg.linear.x = 0.0
            self.pub.publish(msg)

        def turn(angle, angular_speed=1.0):
            msg = Twist()
            msg.angular.z = angular_speed if angle > 0 else -angular_speed
            t0 = time.time()
            duration = abs(angle) / angular_speed
            while time.time() - t0 < duration:
                self.pub.publish(msg)
                time.sleep(0.05)
            msg.angular.z = 0.0
            self.pub.publish(msg)

        # --- Secuencia para dibujar un 9---
        # 1. Línea vertical izquierda
        turn(math.pi/2)

        move(4.0)

        # 2. Giro 90° a la derecha -> línea superior
        turn(math.pi/2)

        # 3. Línea horizontal superior
        move(1.5)

        # 4. Giro 90° a la derecha -> línea vertical derecha
        turn(math.pi/2)

        # 5. Línea vertical descendente de la cabeza
        move(1.5)

        # 6. Giro 90° a la izquierda -> cerrar horizontal superior
        turn(math.pi/2)

        # 7. Línea horizontal de cierre (hacia la izquierda)
        move(2.0)

        self.get_logger().info("Número 9 dibujado y cerrado correctamente.")

def main():
    rclpy.init()
    node = TurtleNine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

