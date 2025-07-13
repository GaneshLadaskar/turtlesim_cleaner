#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('robot_cleaner')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_msg = Twist()

    def move(self, speed, distance, is_forward):
        if is_forward:
            self.vel_msg.linear.x = abs(speed)
        else:
            self.vel_msg.linear.x = -abs(speed)
        
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        t0 = time.time()
        current_distance = 0.0

        while current_distance < distance:
            self.velocity_publisher.publish(self.vel_msg)
            t1 = time.time()
            current_distance = speed * (t1 - t0)
            time.sleep(0.1)  # add sleep to avoid spamming

        # Stop the turtle after moving
        self.vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(self.vel_msg)
        self.get_logger().info('Movement complete.')

def main(args=None):
    rclpy.init(args=args)
    mover = TurtleBotMover()

    print("Let's move your robot (ROS 2)")
    try:
        speed = float(input("Input your speed: "))
        distance = float(input("Type your distance: "))
        is_forward = input("Forward? (y/n): ").strip().lower() == 'y'

        mover.move(speed, distance, is_forward)
    except ValueError:
        print("Invalid input. Please enter numeric values.")

    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
