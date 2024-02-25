import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from drone_simulation.utils import drone_visualizer
from drone_simulation.utils import l2_norm
from tutorial_interfaces.srv import DroneSim

"""
A ROS2 + python based node acts as a client to the drone_gym service

"""


class DroneClient(Node):
    def __init__(self):
        super().__init__("drone_client")
        # parameters
        self.declare_parameter("start_pose", [0, 0, 0])
        self.declare_parameter("my_id", 0)
        startpos = (
            self.get_parameter("start_pose").get_parameter_value().integer_array_value
        )
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value

        self.cli = self.create_client(DroneSim, "/drone" + str(self.myid) + "/DroneSim")
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = DroneSim.Request()

    def send_request(self, step):  # tested
        self.req.step = step
        self.req.victim_state = [0.0, 0.0, -10.0]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    finish = False
    client_ = DroneClient()
    max_itr = 10000
    itr = 0
    while not finish:
        client_.send_request(step=True)
        rclpy.spin_until_future_complete(client_, client_.future)
        time.sleep(0.15)
        itr += 1
        if itr > max_itr:
            print("max_itr reached", response.reward)
            break  # itr based terminal condition
        # goal state based terminal condition
        response = client_.future.result()
        finish = response.done
        if finish:
            print("goal_reached", response.reward)
            break
    client_.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
