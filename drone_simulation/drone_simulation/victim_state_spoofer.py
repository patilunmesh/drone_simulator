import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from drone_simulation.future_pred import (
    cone_visualizer,
    drone_visualizer,
    markov_prediction,
)
from drone_simulation.utils import cosine_between_vectors, l2_norm
from tutorial_interfaces.msg import RemoteID


class Spoofed_state_estimator(Node):
    def __init__(self):
        super().__init__("spoofed_state_estimator")
        self.declare_parameter("my_id", 2)
        self.declare_parameter("start_pose", [3, -3, 0])
        self.startpos = (
            self.get_parameter("start_pose").get_parameter_value().integer_array_value
        )
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value
        self.target_id = 1
        self.my_state = np.array([0.0, 0.0, 0.0])  # x, y, z

        # publishers
        self.desired_victim_publisher = self.create_publisher(
            Marker, "/desired_victim", 1
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/drone" + str(self.myid) + "/true_pose", 1
        )
        self.drone_publisher = self.create_publisher(
            Marker, "/drone" + str(self.myid) + "/drone", 1
        )
        self.obstacle_publisher = self.create_publisher(MarkerArray, "obstacles", 1)
        self.local_traj_publish = self.create_publisher(
            Path, "/drone" + str(self.myid) + "/local_traj", 1
        )

        # subscriber
        self.local_traj_sub = self.create_subscription(
            Path,
            "/drone" + str(self.myid) + "/phantom_local_traj",
            self.victim_path_callback,
            1,
        )  # subscribes phantom path and publishes physical copy
        self.target_rid_sub = self.create_subscription(
            RemoteID,
            "/drone" + str(self.target_id) + "/Remote_id",
            self.target_rid_callback,
            1,
        )
        self.victim_rid_sub = self.create_subscription(
            RemoteID,
            "/drone" + str(self.myid) + "/Remote_id",
            self.victim_rid_callback,
            1,
        )
        # subscribe to drone marker from RL herder
        # self.drone_marker_herder = self.create_subscription(MarkerArray, '/marker_desired', self.drone_marker_herder_callback, 1)

        self.counter = 0
        self.victim_rid_recieved = False
        self.target_loc = np.array([0.0, 0.0, 0.0])
        self.exp_target_delta = np.array(self.startpos) - np.array([0, 0, 3])
        self.victim_rid_loc = np.array(self.startpos)
        self.attackers_desired_state = np.array([0.0, 0.0, 0.0])
        self.desired_goal = np.array([3.0, 3.0, 3.0])
        self.attackers_estimate = np.array(
            self.startpos
        )  # attacker's estimate of the physical pose of the victim
        # snooze
        self.timer_counter = 140
        self.last_called = 0

        # intermittent control
        self.control_state = True  # True means control is with the attacker, False means control is with the victim
        self.interval = np.random.choice([5, 10, 20, 50, 100])
        self.iters = self.interval

    def target_rid_callback(self, msg):
        self.target_loc = np.array([msg.lattitude, msg.longitude, msg.altitude])
        self.attackers_desired_state = self.herder(self.target_loc, msg)
        self.timer_counter += 1
        # uncomment to test guiding beacon
        # desired_marker = cone_visualizer(self.attackers_desired_state[0], self.attackers_desired_state[1], self.attackers_desired_state[2], 0.0)
        # self.desired_victim_publisher.publish(desired_marker)
        # arr = MarkerArray()
        # arr.markers.append(desired_marker)
        # self.obstacle_publisher.publish(arr)

        # no attack behaviour #tested
        # new_loc =  self.attackers_estimate
        # random attack #tested
        # new_loc= self.attackers_estimate + np.array([3.0, 3.0, -3.0])

        # permanent control
        delt = self.attackers_estimate - self.attackers_desired_state
        new_loc = self.victim_rid_loc
        if (self.timer_counter - self.last_called) > 150:
            if np.linalg.norm(delt) < 0.3:
                self.last_called = self.timer_counter
            else:
                new_loc = self.victim_rid_loc - 0.05 * delt

        # intermittent control
        # case1
        # success_rate = 0.99 #probability of success of spoofing
        # p = np.random.uniform(0, 1)
        # if p < success_rate:
        #     new_loc =  self.victim_rid_loc - 0.05*(self.attackers_estimate - self.attackers_desired_state)
        # else:
        #     new_loc =  self.victim_rid_loc

        # case2
        # loss of control for certain intervals of time
        # if self.control_state:
        #     #attacker has control for this interval
        #     if self.iters > 1:
        #         self.iters -= 1
        #         new_loc =  self.victim_rid_loc - 0.05*(self.attackers_estimate - self.attackers_desired_state)
        #     else:
        #         self.control_state = False
        #         self.interval = np.random.choice([10, 20, 50, 100])
        #         self.iters = self.interval

        # if not self.control_state:
        #     #attacker looses control for this interval
        #     if self.iters > 1:
        #         self.iters -= 1
        #         new_loc =  self.victim_rid_loc
        #     else:
        #         self.control_state = True
        #         self.interval = np.random.choice([10, 20, 50, 100])
        #         self.iters = self.interval
        #         new_loc =  self.victim_rid_loc - 0.05*(self.attackers_estimate - self.attackers_desired_state)

        new_pose = PoseStamped()
        new_pose.header = msg.header
        new_pose.pose.position.x = float(new_loc[0])
        new_pose.pose.position.y = float(new_loc[1])
        new_pose.pose.position.z = float(new_loc[2])
        if self.victim_rid_recieved:
            self.pose_publisher.publish(new_pose)
            self.victim_rid_recieved = False
        # time.sleep(0.15)

    def victim_path_callback(self, msg):
        # takes victim path and updates it with attackers estimate
        victim_path = msg.poses
        new_path = Path()
        new_path.header = msg.header
        for pose in victim_path:
            new_pose = PoseStamped()
            new_pose.header = pose.header
            delta = self.attackers_estimate - self.victim_rid_loc
            new_pose.pose.position.x = pose.pose.position.x + delta[0]
            new_pose.pose.position.y = pose.pose.position.y + delta[1]
            new_pose.pose.position.z = pose.pose.position.z + delta[2]
            new_path.poses.append(new_pose)
        self.local_traj_publish.publish(new_path)

    def victim_rid_callback(self, msg):
        # print("victim_rid_callback")
        self.victim_rid_loc = np.array(
            [float(msg.lattitude), float(msg.longitude), float(msg.altitude)]
        )
        self.victim_rid_recieved = True
        # estimate physical coordintes using remote ID velocity information #tested
        vh, theta, vz = msg.horizontal_speed, msg.heading, msg.vertical_speed
        vx, vy = vh * np.cos(theta), vh * np.sin(theta)
        self.attackers_estimate = self.attackers_estimate + np.array([vx, vy, vz]) * 0.1
        drone_marker = drone_visualizer(
            self.attackers_estimate[0],
            self.attackers_estimate[1],
            self.attackers_estimate[2],
        )
        self.drone_publisher.publish(drone_marker)
        # m = MarkerArray()
        # m.markers.append(drone_marker)
        # self.obstacle_publisher.publish(m)

    def herder(self, target_loc, msg):
        delT = 2.0
        if l2_norm(target_loc, self.desired_goal) < 2:
            print("##################################")
            print("Attack succesfull")
            print("##################################")
        elif target_loc[-1] > self.desired_goal[-1]:
            print("##################################")
            print("Attack Failed, drone crossed altitude")
            print("##################################")
        xt, yt, zt = target_loc[0], target_loc[1], target_loc[2]
        if (
            abs(xt - self.desired_goal[0]) < 1.0
            and abs(yt - self.desired_goal[1]) < 1.0
        ):
            return np.array([xt + 3, yt + 3, zt + 2 + (self.myid) * 0.5])
        heading, v_hor, vz = msg.heading, msg.horizontal_speed, msg.vertical_speed
        # if abs(v_hor) < 0.1: return np.array([xt+3, yt+3, zt+3])
        vx = v_hor * math.cos(heading)
        vy = v_hor * math.sin(heading)
        nx, ny, nz = xt + vx * delT, yt + vy * delT, zt + vz * delT
        # try 8 locations around nx, ny, nz in x-y plane with 45 degree diff
        minangle = cosine_between_vectors(target_loc, self.desired_goal, [nx, ny, nz])
        minind = -1
        minangles = []
        for i in range(8):
            x, y = nx + math.cos(i * math.pi / 4), ny + math.sin(i * math.pi / 4)
            newx, newy, newz, _ = markov_prediction(msg, delT, [x, y, nz])
            # get score of this new location
            angle1 = cosine_between_vectors(
                target_loc, self.desired_goal, [newx, newy, newz]
            )
            # if only one victim we need minimum angle
            # if angle1 < minangle:
            #     minangle = angle1
            #     minind = i
            # for multiple victims, we need to sort the angles and pick the one with minimum angle
            # then pick second minimum for second victim and so on
            if angle1 < minangle:
                minangles.append((angle1, i))
            # sort based on angles and move corresponding index
        if len(minangles) > (self.myid - 1):
            minangles.sort(key=lambda x: x[0])
            minind = minangles[self.myid - 1][
                1
            ]  # myid is zero for target and 1, 2, etc for victims

        if minind == -1:
            return np.array([xt + 3, yt + 3, zt + 2 + (self.myid) * 0.5])
        else:
            x, y = nx + math.cos(minind * math.pi / 4), ny + math.sin(
                minind * math.pi / 4
            )
            return np.array([x, y, nz + (self.myid) * 0.5])

    # def drone_marker_herder_callback(self, msg):
    #     for marker in msg.markers:
    #         i = 0
    #         self.attackers_desired_state = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
    #         break


def main(args=None):
    rclpy.init(args=args)
    pose_estimator = Spoofed_state_estimator()
    rclpy.spin(pose_estimator)
    pose_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
