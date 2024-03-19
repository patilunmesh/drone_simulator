"""
Node to compute and publish CBF values for the given drone id and safety radius.
Additionally, if `sound_on` parameter is set to 1, will sonify the CBF values by mapping them to pitches
and playing them via SineWave objects through the audio driver of the computer the node is running on.
"""

import time
import numpy as np
from pysinewave import (
    SineWave,
)  # sudo apt-get install libportaudio2, then pip install pysinewave
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Header
from visualization_msgs.msg import Marker, MarkerArray

from tutorial_interfaces.srv import Sound


class CBFPublisher(Node):
    def __init__(self):
        super().__init__("CBFPublisher")

        self.declare_parameter("my_id", 1)  # drone id
        self.declare_parameter("safety_radius", 0.7)  # safety radius for computing CBF
        self.declare_parameter("sound_on", 0)  # 0 to turn off sound, 1 to turn on sound
        self.declare_parameter(
            "sound_on_upper_bound", 50
        )  # Upper bound on CBF values for lowest pitch
        self.declare_parameter(
            "sound_on_lower_bound", 5
        )  # Lower bound on CBF values for highest pitch
        self.declare_parameter(
            "sound_pitch_freq", 0
        )  # 0 to round to nearest pitch, 1 to use raw frequency
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value
        self.safety_radius = (
            self.get_parameter("safety_radius").get_parameter_value().double_value
        )

        self.sound_on = (
            self.get_parameter("sound_on").get_parameter_value().integer_value
        )
        self.sound_upper_bound = (
            self.get_parameter("sound_on_upper_bound")
            .get_parameter_value()
            .integer_value
        )
        self.sound_lower_bound = (
            self.get_parameter("sound_on_lower_bound")
            .get_parameter_value()
            .integer_value
        )
        self.pitch_freq = (
            self.get_parameter("sound_pitch_freq").get_parameter_value().integer_value
        )
        self.sinewave = SineWave(pitch=0, pitch_per_second=10)
        self.playing = False

        self.timer_period = 1 / 10
        self.timer = self.create_timer(self.timer_period, self.pub_cbf_vals_callback)

        # Publishers
        self.cbf_val_publisher = self.create_publisher(
            Float32MultiArray, "/drone" + str(self.myid) + "/cbf_val", 1
        )
        self.cbf_vals = []

        # subscribers
        self.subscription_markers = self.create_subscription(
            MarkerArray, "obstacles", self.marker_array_callback, 1
        )
        self.subscription_pose = self.create_subscription(
            Odometry,
            "/drone" + str(self.myid) + "/true_pose",
            self.save_cbf_vals_callback,
            1,
        )

        # Sound on Sound Off service
        self.sound_service = self.create_service(
            Sound, "/drone" + str(self.myid) + "/sound", self.sound_callback
        )

        self.obstacles = []

    def marker_array_callback(self, msg):
        """
        save the obstacles published on the `obstacles` topic
        """
        self.obstacles = msg.markers

    def pub_cbf_vals_callback(self):
        """
        Publish the computed CBF values
        """
        cbf_msg = Float32MultiArray()
        cbf_msg.data = self.cbf_vals
        self.cbf_val_publisher.publish(cbf_msg)

    def save_cbf_vals_callback(self, msg):
        """
        Computes CBF values for the obstacles.
        Plays sound based on the CBF values if sound enabled.
        """
        try:
            drone_pos = msg.pose.pose.position
            cbf_vals = []
            for obstacle in self.obstacles:
                obstacle_pos = obstacle.pose.position
                obstacle_radius = obstacle.scale.x / 2.0 + self.safety_radius
                h, hx_deriv, hy_deriv, hz_deriv = self.calculate_cbf(
                    drone_pos, obstacle_pos, obstacle_radius
                )
                cbf_vals.extend([h, hx_deriv, hy_deriv, hz_deriv])
            self.cbf_vals = cbf_vals

            if self.sound_on == 1:
                return self.playSound(h)
        except Exception as e:
            self.get_logger().info("cannot compute CBF, got error message {}".format(e))

    @staticmethod
    def calculate_cbf(position1, position2, separation):
        """
        Calculate the CBF value (h) and the partial derivatives (hx_deriv, hy_deriv, hz_deriv)
        """
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        dz = position1.z - position2.z
        h = (dx**2 + dy**2 + dz**2) - separation**2
        hx_deriv = 2 * dx
        hy_deriv = 2 * dy
        hz_deriv = 2 * dz
        return h, hx_deriv, hy_deriv, hz_deriv

    def sound_callback(self, request, response):
        self.get_logger().info("Toggling Sound")

        if request.sound_on:
            self.sound_on = 1
        else:
            self.sound_on = 0
            self.sinewave.stop()
            self.playing = False

        response.done = True
        return response

    def playSound(self, h):
        """
        Play a sound based on the CBF value h
        """
        self.transformPitch(h)
        if not self.playing:
            self.sinewave.play()
            self.playing = True
        else:
            self.transformPitch(h)
        time.sleep(3)

    def transformPitch(self, h):
        """
        Transform the CBF value h to frequency/pitch values
        """
        # higher CBF values map to lower pitches
        # lower CBF values map to higher pitches
        high_freq = 523.25
        low_freq = 261.63
        slope = (high_freq - low_freq) / (
            self.sound_lower_bound - self.sound_upper_bound
        )
        freq_output = low_freq + slope * (h - self.sound_upper_bound)
        if self.pitch_freq == 0:
            note_output = self.mapToNote(freq_output)
            self.sinewave.set_pitch(note_output)
        else:
            self.sinewave.set_frequency(freq_output)

    @staticmethod
    def mapToNote(freq):
        """
        Map ranges of raw frequency value to a single pitch value
        """
        if freq <= 261.63:
            note = 0  # C1
        elif 261.63 < freq <= 277.18:
            note = 1  # C#/Db
        elif 277.18 < freq <= 293.66:
            note = 2  # D
        elif 293.66 < freq <= 311.13:
            note = 3  # D#/Eb
        elif 311.13 < freq <= 329.63:
            note = 4  # E
        elif 329.63 < freq <= 349.23:
            note = 5  # F
        elif 349.23 < freq <= 369.99:
            note = 6  # F#/Gb
        elif 369.99 < freq <= 392.00:
            note = 7  # G
        elif 392.00 < freq <= 415.30:
            note = 8  # G#/Ab
        elif 415.30 < freq <= 440.00:
            note = 9  # A
        elif 440.00 < freq <= 466.16:
            note = 10  # A#/Bb
        elif 466.16 < freq <= 493.88:
            note = 11  # B
        else:
            note = 12  # C2
        return note


def main(args=None):
    rclpy.init(args=args)
    cbf_publisher = CBFPublisher()
    rclpy.spin(cbf_publisher)
    cbf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
