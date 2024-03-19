"""
A client node for interfacing with the CBF simulator in `cbf_sim.py`
Allows for resetting the simulation and setting new CBF parameters.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node

from tutorial_interfaces.srv import ResetSim


class DroneCBFClient(Node):
    def __init__(self):
        super().__init__("droneCBFclient")
        # Parameters
        self.declare_parameter("my_id", 1)
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value

        # Client for the Reset Service
        self.cli = self.create_client(ResetSim, "/drone" + str(self.myid) + "/ResetSim")
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = ResetSim.Request()

    def send_request(self, cbf_param=15.0):
        self.req.reset = True
        self.req.cbf_param = cbf_param
        self.future = self.cli.call_async(self.req)


def main(args=None):
    """
    Sends service request after user keyboard input for new CBF parameter.
    """
    rclpy.init(args=args)
    cbf_param = input("Set new CBF parameter (float): ")
    try:
        cbf_param = float(cbf_param)
    except:
        print("Cannot set value {}. Using default value".format(cbf_param))
        cbf_param = 15.0
    client = DroneCBFClient()
    client.send_request(cbf_param=cbf_param)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
