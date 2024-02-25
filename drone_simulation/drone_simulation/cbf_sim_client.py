import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from drone_simulation.utils import drone_visualizer
from drone_simulation.utils import l2_norm

from tutorial_interfaces.srv import ResetSim

"""
A ROS2 + python based node acts as a client to the drone_gym service

"""


class DroneCBFClient(Node):
    def __init__(self):
        super().__init__("droneCBFclient")
        # parameters
        self.declare_parameter("my_id", 0)
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value

        self.cli = self.create_client(ResetSim, "/drone" + str(self.myid) + "/ResetSim")
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = ResetSim.Request()

    def send_request(self, cbf_param=0.0):  # tested
        self.req.reset = True
        self.req.cbf_param = cbf_param
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    client = DroneCBFClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
