from tutorial_interfaces.msg import RemoteID
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class RemoteIdBroadcast(Node):
    '''
    Publishes remote ID of the drone as a broadcast.
    Subscribes to true pose of the drone.
    '''
    def __init__(self):
        super().__init__("remote_id_broadcast")
        self.declare_parameter("my_id", 0)
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value
        self.subscription_pose = self.create_subscription(
            Odometry,
            '/drone'+str(self.myid)+'/true_pose',
            self.pose_callback,
            1
        )
        self.publisher_remote_id = self.create_publisher(RemoteID, "Remote_id", 10)
        
    def pose_callback(self, msg):
        remote_id = RemoteID()
        remote_id.id = self.myid
        remote_id.lattitude = msg.pose.pose.position.x
        remote_id.longitude = msg.pose.pose.position.y
        remote_id.altitude = msg.pose.pose.position.z
        remote_id.horizontal_speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        remote_id.vertical_speed = msg.twist.twist.linear.z
        remote_id.heading = np.arctan2(msg.twist.twist.linear.y, msg.twist.twist.linear.x)
        self.publisher_remote_id.publish(remote_id)
        