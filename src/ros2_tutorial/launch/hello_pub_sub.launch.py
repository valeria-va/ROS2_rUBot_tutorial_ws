from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_tutorial',
            executable='publisher_node',
            output='screen'),
        Node(
            package='ros2_tutorial',
            executable='subscriber_node',
            output='screen'),
    ])