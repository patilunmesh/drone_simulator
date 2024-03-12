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

		self.timer_period = 1 / 10
		self.timer = self.create_timer(self.timer_period, self.pub_cbf_vals_callback)

		# Publishers
		self.cbf_val_publisher = self.create_publisher(Float32MultiArray, "/drone" + str(self.myid) + "/cbf_val", 1)
		self.cbf_vals = []

		# subscribers
		self.subscription_markers = self.create_subscription(
			MarkerArray, "obstacles", self.marker_array_callback, 1)
		self.subscription_pose = self.create_subscription(
			Odometry, "/drone" + str(self.myid) + "/true_pose", self.save_cbf_vals_callback, 1)

		self.obstacles = []

	def marker_array_callback(self, msg):
		# self.get_logger().info("got obstacles")
		self.obstacles = msg.markers

	def pub_cbf_vals_callback(self):
		cbf_msg = Float32MultiArray()
		cbf_msg.data = self.cbf_vals
		self.cbf_val_publisher.publish(cbf_msg)
		# self.get_logger().info("publishing cbf values")


	def save_cbf_vals_callback(self, msg):
		try:
			drone_pos = msg.pose.pose.position
			cbf_vals = []
			for obstacle in self.obstacles:
				obstacle_pos = obstacle.pose.position
				obstacle_radius = obstacle.scale.x / 2.0 + self.safety_radius
				h, hx_deriv, hy_deriv, hz_deriv = self.calculate_cbf(drone_pos, obstacle_pos, obstacle_radius)
				cbf_vals.extend([h, hx_deriv, hy_deriv, hz_deriv])
			self.cbf_vals = cbf_vals
		except Exception as e:
			self.get_logger().info("cannot compute CBF, got error message {}".format(e))

	@staticmethod
	def calculate_cbf(position1, position2, separation):
		dx = position1.x - position2.x
		dy = position1.y - position2.y
		dz = position1.z - position2.z
		h = (dx**2 + dy**2 + dz**2) - separation**2
		hx_deriv = 2*dx
		hy_deriv = 2*dy
		hz_deriv = 2*dz
		return h, hx_deriv, hy_deriv, hz_deriv


def main(args=None):
	rclpy.init(args=args)
	cbf_publisher = CBFPublisher()
	rclpy.spin(cbf_publisher)
	cbf_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()