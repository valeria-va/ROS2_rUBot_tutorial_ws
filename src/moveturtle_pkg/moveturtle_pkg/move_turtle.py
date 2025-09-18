import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):

    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, pose_msg):
        twist = Twist()
        if abs(pose_msg.x) >= 7.0 or abs(pose_msg.y) >= 7.0:            
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing: linear.x={twist.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
