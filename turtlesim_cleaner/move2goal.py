#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        self.pose = Pose()
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.goal_pose = Pose()
        self.distance_tolerance = 0.5
        self.reached_goal = False

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def get_distance(self, goal_x, goal_y):
        return sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))

    def timer_callback(self):
        if self.reached_goal:
            return

        vel_msg = Twist()

        distance = self.get_distance(self.goal_pose.x, self.goal_pose.y)
        if distance >= self.distance_tolerance:
            # Proportional controller
            vel_msg.linear.x = 1.5 * distance
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 4.0 * (atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x) - self.pose.theta)
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.reached_goal = True
            self.get_logger().info("Goal reached!")

        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_bot = TurtleBotController()

    print("Let's move your robot to a goal position (ROS 2)")
    try:
        x = float(input("Set your x goal: "))
        y = float(input("Set your y goal: "))
        tol = float(input("Set your distance tolerance: "))

        turtle_bot.goal_pose.x = x
        turtle_bot.goal_pose.y = y
        turtle_bot.distance_tolerance = tol

        rclpy.spin(turtle_bot)

    except ValueError:
        print("Invalid input. Please enter numeric values.")
        turtle_bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
