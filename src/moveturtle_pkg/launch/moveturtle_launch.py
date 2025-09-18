from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveturtle_pkg',
            executable='moveturtle_exec',
            output='screen'),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen',)
            
            ])



