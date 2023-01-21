from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="ros2_tutorial",
        executable="publisher_hello.py",
        name="simple_publisher",
    )

    subscriber_node = Node(
        package="ros2_tutorial",
        executable="subscriber_hello",
        name="simple_subscriber",
    )

    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    return ld