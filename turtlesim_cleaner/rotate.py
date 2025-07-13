#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleRotator(Node):
    def __init__(self):
        super().__init__('robot_cleaner_rotator')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_msg = Twist()

    def rotate(self, speed_deg, angle_deg, clockwise):
        # Convert degrees to radians
        angular_speed = math.radians(speed_deg)
        relative_angle = math.radians(angle_deg)

        # No linear movement
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0

        # Set angular velocity
        if clockwise:
            self.vel_msg.angular.z = -abs(angular_speed)
        else:
            self.vel_msg.angular.z = abs(angular_speed)

        # Start time
        t0 = time.time()
        current_angle = 0.0

        while current_angle < relative_angle:
            self.publisher_.publish(self.vel_msg)
            t1 = time.time()
            current_angle = angular_speed * (t1 - t0)
            time.sleep(0.1)  # avoid publishing too fast

        # Stop rotation
        self.vel_msg.angular.z = 0.0
        self.publisher_.publish(self.vel_msg)
        self.get_logger().info("Rotation complete.")

def main(args=None):
    rclpy.init(args=args)
    rotator = TurtleRotator()

    print("Let's rotate your robot (ROS 2)")
    try:
        speed = float(input("Input your speed (degrees/sec): "))
        angle = float(input("Type your angle (degrees): "))
        clockwise = input("Clockwise? (y/n): ").strip().lower() == 'y'

        rotator.rotate(speed, angle, clockwise)
    except ValueError:
        print("Invalid input. Please enter numeric values.")

    rotator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
