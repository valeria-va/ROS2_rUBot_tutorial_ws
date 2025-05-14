#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

class MoveTurtleNode(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.move_turtle)
        self.linear_velocity = 1.0
        self.angular_velocity = 0.0
        self.distance = 10.0
        self.vel_msg = Twist()

    def pose_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.get_logger().info(f'Robot X = {msg.x:.2f}\t Robot Y = {msg.y:.2f}')

    def move_turtle(self):
        self.vel_msg.linear.x = self.linear_velocity
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = self.angular_velocity

        if self.robot_x >= self.distance or self.robot_y >= self.distance:
            self.get_logger().info("Robot hits a wall")
            self.get_logger().warn("Stopping robot")
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self.publisher_.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    move_turtle_node = MoveTurtleNode()
    rclpy.spin(move_turtle_node)
    move_turtle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()