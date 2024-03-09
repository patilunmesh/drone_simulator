import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Header
from visualization_msgs.msg import Marker, MarkerArray

class CBFPublisher(Node):
	def __init__(self):
		super().__init__("CBFPublisher")

		self.declare_parameter("my_id", 1)
		self.declare_parameter("safety_radius", 0.7)
		self.myid = self.get_parameter("my_id").get_parameter_value().integer_value
		self.safety_radius = self.get_parameter("safety_radius").get_parameter_value().double_value

		# Publishers
		self.cbf_val_publisher = self.create_publisher(Float32MultiArray, 'cbf_val', 1)

		# subscribers
		self.subscription_markers = self.create_subscription(
			MarkerArray, "obstacles", self.marker_array_callback, 1)
		self.subscription_pose = self.create_subscription(
			Odometry, "/drone" + str(self.myid) + "/true_pose", self.cbf_callback, 1)

		self.obstacles = []

	def marker_array_callback(self, msg):
		self.get_logger().info("got obstacles")
		self.obstacles = msg.markers

	def cbf_callback(self, msg):
		try:
			drone_pos = msg.pose.pose.position
			cbf_vals = []
			for obstacle in self.obstacles:
				obstacle_pos = obstacle.pose.position
				obstacle_radius = obstacle.scale.x / 2.0 + self.safety_radius
				h = self.calculate_cbf(drone_pos, obstacle_pos, obstacle_radius)
				cbf_vals.append(h)
			if len(cbf_vals) > 0:
				cbf_msg = Float32MultiArray()
				cbf_msg.data = cbf_vals
				self.cbf_val_publisher.publish(cbf_msg)
				self.get_logger().info("publishing cbf values")
		except Exception as e:
			self.get_logger().info("cannot compute CBF, got error message {}".format(e))

	@staticmethod
	def calculate_cbf(position1, position2, separation):
		dx = position1.x - position2.x
		dy = position1.y - position2.y
		dz = position1.z - position2.z
		h = (dx**2 + dy**2 + dz**2) - separation**2
		return h


def main(args=None):
	rclpy.init(args=args)
	cbf_publisher = CBFPublisher()
	rclpy.spin(cbf_publisher)
	cbf_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()