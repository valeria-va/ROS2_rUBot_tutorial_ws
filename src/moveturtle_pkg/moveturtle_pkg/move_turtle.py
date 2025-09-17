import rclpy # import the Node module from ROS2 Python library
from rclpy.node import Node
from geometry_msgs.msg import Twist #Import Twist and Pose from turtlesim
from turtlesim.msg import Pose

class move_turtle(Node):
	def __init__(self):
		super().__init__('monitor_and_move')
		
		self.cmd_vel_publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
		
		self.subscription = self.create_subscription( #Suscription to read pose
		Pose,
		'/turtle1/pose',
		self.pose_callback, #function to run
		10
		)
		
		def pose_callback(self, pose):
		twist = Twist()
		
		if pose.x > 7 or pose.y > 7:
		twist.linear.x = 0.0
		twist.angular.z = 0.0
		else:
		twist.linear.x = 1.0
		twist.angular.z = 0.0

		self.cmd_vel_publisher.publish(twist)

def main(args=None):
	# initialize the ROS communication
	rclpy.init(args=args)
	move_turtle_node = move_turtle()
	rclpy.spin(move_turtle_node)
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	move_turtle_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main() #call the main function
