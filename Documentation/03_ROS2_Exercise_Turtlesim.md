# **ROS2 Exercise Turtlesim**

The objective of this section is to solve the exercise.

**Exercise:**

Create a new package "ros2_move_turtle" to control the movement of the previous Turtlesim robot.

![](./Images/02_ROS2_tutorial/02_move_turtle.png)

The program functionality will be based on 2 nodes:
- The "/turtlesim" node we have already practice in last section
- A new "/move_turtle" node that:
    - subscribes to the "/turtle1/Pose" topic
    - Publish to the "/turtle1/cmd_vel" topic a message Twist
    - if the running time is greater to 10s, the robot stops
    - if the position in x direction of the robot is greater than 10m, the robot stops

**Solution**

- Create the proper package:
````bash
cd src
ros2 pkg create --build-type ament_python --license Apache-2.0 ros2_move_turtle --dependencies rclpy turtlesim geometry_msgs
````
- Compile
````bash
cd ..
colcon build
````
- Create the new "/move_turtle" node: move_turtle.py
````bash#!/usr/bin/env python3
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
        self.linear_velocity = 0.2
        self.angular_velocity = 0.0
        self.distance = 7.0
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
````
- Modify the "setup.py":
````python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_move_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'move_turtle_exec = ros2_move_turtle.move_turtle:main',
        ],
    },
)
````
- Create the "move_turtle.launch.py" file:
````python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'),
        Node(
            package='ros2_move_turtle',
            executable='move_turtle_exec',
            output='screen'),
    ])
    ````
- Compile
- Launch:
````xml
ros2 launch ros2_move_turtle move_turtle.launch.py
````
