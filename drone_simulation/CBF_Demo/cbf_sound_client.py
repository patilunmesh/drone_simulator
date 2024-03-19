"""
A client node for interfacing with the CBF Publisher sound service in `cbf_publisher.py`
Allows for toggling on and off the sound
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from drone_simulation.utils import drone_visualizer
from drone_simulation.utils import l2_norm

from tutorial_interfaces.srv import Sound


class DroneCBFSoundClient(Node):
    def __init__(self):
        super().__init__("droneCBFSoundclient")
        # parameters
        self.declare_parameter("my_id", 1)
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value

        self.cli = self.create_client(Sound, "/drone" + str(self.myid) + "/sound")
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = Sound.Request()

    def send_request(self, sound_on):
        self.req.sound_on = sound_on
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    sound_on = input("Toggle Sound (1 for on / other for off): ")
    try:
        sound_on = int(sound_on)
        if sound_on == 1:
            sound_on = True
        else:
            sound_on = False
        client = DroneCBFSoundClient()
        client.send_request(sound_on=sound_on)
        client.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(e)
        print("Cannot set value {}".format(sound_on))


if __name__ == "__main__":
    main()
