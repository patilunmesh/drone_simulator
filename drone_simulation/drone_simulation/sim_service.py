"""
This code takes input start pose, goal pose and time. These states are passed to a trajectory generator.
The trajectory generator is based on the rapid motion primitive generation paper.
The generated trajectory is then sent to controller as a set of waypoints.
LQR controller is used to generate the control sequence which is rotor velocities for a quadrotor.
The control output, and current state is then carried over to a state estimation module.

Difference from sim.py: this one is designed for RL setup. it acts as a service.
This ros service called by RL_herder and a reward signal, next state is expected.
This acts as a step() function in gym.
"""

# todo: continuity in replanning state (velocity and acceleration)

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

from drone_simulation.LQR import LQR
from drone_simulation.trajectoryGen import trajectoryGenie
from drone_simulation.utils import (
    drone_visualizer,
    local_traj_maker,
    trajectory_maker,
    update_time,
)
from tutorial_interfaces.msg import Obstacles
from tutorial_interfaces.srv import DroneSim


class DroneGym(Node):
    def __init__(self):
        super().__init__("drone_gym")

        # parameters
        self.declare_parameter("start_pose", [0, 0, 0])
        self.declare_parameter("my_id", 0)
        startpos = (
            self.get_parameter("start_pose").get_parameter_value().integer_array_value
        )
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value
        self.target_id = 1
        # publishers
        self.drone_publisher = self.create_publisher(Marker, "drone", 1)
        self.pose_publisher = self.create_publisher(Odometry, "true_pose", 1)
        # self.obstacle_publisher = self.create_publisher(MarkerArray, 'obstacles', 10)
        self.global_traj_publisher = self.create_publisher(Path, "global_traj", 1)
        self.local_traj_publisher = self.create_publisher(Path, "local_traj", 1)

        # subscribers
        self.detected_obst_subscriber = self.create_subscription(
            Obstacles,
            "/drone" + str(self.myid) + "/near_obstacles",
            self.obst_callback,
            1,
        )
        self.timer_period = 1 / 10  # seconds
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # service
        self.sim_service = self.create_service(
            DroneSim, "/drone" + str(self.myid) + "/DroneSim", self.step_update_callback
        )
        self.i = 0

        # Define the duration:
        self.Tf = 10  # in seconds
        numPlotPoints = 100 * self.Tf  # temporal resolution
        self.time = np.linspace(0, self.Tf, numPlotPoints)
        # Define the trajectory starting state:
        pos0 = startpos  # position
        vel0 = [0, 0, 0]  # velocity
        acc0 = [0, 0, 0]  # acceleration

        # Define the goal state:
        self.posf = [pos0[0] + 10, pos0[1], pos0[2] + 30]  # position
        self.velf = [0, 0, 0]  # velocity
        self.accf = [0, 9.81, 0]  # acceleration

        # Rapid trajectory generator
        traj, px, py, pz = trajectoryGenie(
            pos0, vel0, acc0, self.posf, self.velf, self.accf, self.Tf, numPlotPoints
        )
        self.traj_marker = trajectory_maker(px, py, pz)
        x_nl = LQR(px, py, pz, self.Tf + 2, numPlotPoints)
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.tru_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # obstacle flags and data
        self.victim_data = [0.0, 0.0, 0.0]
        self.collision_flag = False
        self.replanning = False
        self.obstacle_data = []
        self.t0 = self.get_clock().now()
        # to snooze over reactive ness of obstacle avoidance
        self.timer_counter = 0
        self.last_called = 0
        # self.avoided_obstacle = []

    def step_update_callback(self, request, response):
        # print("requested update ", self.myid)
        if self.myid == self.target_id:
            self.victim_data = request.victim_state
        sec, nanosec = update_time(self.t0, 0.01, self.i)
        if self.i < len(self.time) - 1:
            x, y, z = (
                self.ax[self.i],
                self.ay[self.i],
                self.az[self.i],
            ) 
            vx_ = (self.ax[self.i + 1] - self.ax[self.i]) / (self.timer_period)
            vy_ = (self.ay[self.i + 1] - self.ay[self.i]) / (self.timer_period)
            vz_ = (self.az[self.i + 1] - self.az[self.i]) / (self.timer_period)
            rol, pit, ya = self.roll[self.i], self.pitch[self.i], self.yaw[self.i]
            drone_marker, pos = drone_visualizer(x, y, z, rol, pit, ya)
            pos.twist.twist.linear.x, pos.twist.twist.linear.y, pos.twist.twist.linear.z = vx_, vy_, vz_
            local_traj_marker = local_traj_maker(self.ax, self.ay, self.az, self.i)
            self.tru_pose = x, y, z, rol, pit, ya
            # all publishers
            self.drone_publisher.publish(drone_marker)
            self.pose_publisher.publish(pos)
            self.global_traj_publisher.publish(self.traj_marker)
            self.local_traj_publisher.publish(local_traj_marker)

        self.timer_counter += 1
        if not self.collision_flag:
            self.i += 1

        if self.collision_flag:
            print("collision found!! Replanning now")
            self.replanner(local_traj_marker)
            self.collision_flag = False
        response.state = self.tru_pose
        response.done = False
        if self.tru_pose[2] >= (self.posf[-1] - 0.1):
            print("reached non-desired goal")
            response.done = True
        if self.i >= len(self.time) - 2:
            print("reached desired goal")
            response.done = True
        response.reward = self.get_reward()
        return response

    def obst_callback(self, msg):
        obstacles = msg.data
        if (
            len(obstacles) > 0 and self.i > 4
        ):  # first five points should be obstacle free
            print("total obstacles:  ", len(obstacles))
            self.collision_flag = True
            for obst in obstacles:
                print("obstacle data: ", obst)
                [r, x, y, z, ind, od_id] = (
                    obst.r,
                    obst.x,
                    obst.y,
                    obst.z,
                    obst.path_id,
                    obst.obstacle_id,
                )  # od_id is obstacle id: 0 for non-drone, 1,2,3.. for drone
            self.obstacle_data.append([r, x, y, z, ind - 1, od_id])
        else:
            self.collision_flag = False
            self.obstacle_data = []

    def get_reward(self):
        [ox, oy, oz] = self.victim_data
        dist = (
            abs(ox - self.tru_pose[0])
            + abs(oy - self.tru_pose[1])
            + abs(oz - self.tru_pose[2])
        )
        if dist < 10:
            r = 1 / (self.tru_pose[2] + 0.1)
        else:
            r = -1.0
        return r

    def deflector(self, drone_pt, ind):
        obst_pt = (
            self.obstacle_data[ind][1],
            self.obstacle_data[ind][2],
            self.obstacle_data[ind][3],
        )

        r = self.obstacle_data[ind][0] / 1.0

        if drone_pt.x == obst_pt[0]:
            newx = drone_pt.x
        else:
            dir_x = abs(drone_pt.x - obst_pt[0]) / (drone_pt.x - obst_pt[0])
            newx = drone_pt.x + dir_x / max(abs(drone_pt.x - obst_pt[0]), r)
        if drone_pt.y == obst_pt[1]:
            newy = drone_pt.y
        else:
            dir_y = abs(drone_pt.y - obst_pt[1]) / (drone_pt.y - obst_pt[1])
            newy = drone_pt.y + dir_y / max(abs(drone_pt.y - obst_pt[1]), r)
        # dir_z = abs(drone_pt.z - obst_pt[2])/(drone_pt.z - obst_pt[2])

        # newz = drone_pt.z + dir_z /max(abs(drone_pt.z - obst_pt[2]), r)
        # print(drone_pt, newx, newy, newz)
        newz = drone_pt.z

        return newx, newy, newz

    def replanner(self, local_traj):
        self.replanning = True
        self.last_called = self.timer_counter
        [lx, ly, lz, phi0, theta0, psi0] = self.tru_pose
        if len(self.obstacle_data) == 1:
            minind = len(local_traj.poses)
            for i in range(len(self.obstacle_data)):
                # avoid the closest obstacle #TODO: improve this
                if self.obstacle_data[i][4] < minind:
                    minind = self.obstacle_data[i][4]
            if minind > len(local_traj.poses) - 2:
                return
            drone_pt = local_traj.poses[minind].pose.position
            newx, newy, newz = self.deflector(drone_pt, i)
            newTf = int(((self.Tf * 100) - self.i + 200) / 100)
            traj1, px1, py1, pz1 = trajectoryGenie(
                [lx, ly, lz],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [newx, newy, newz],
                self.velf,
                self.accf,
                1,
                100,
            )
            traj2, px2, py2, pz2 = trajectoryGenie(
                [newx, newy, newz],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                self.posf,
                self.velf,
                self.accf,
                newTf,
                100 * (newTf),
            )
            px = np.concatenate((px1, px2))
            py = np.concatenate((py1, py2))
            pz = np.concatenate((pz1, pz2))

        elif len(self.obstacle_data) == 2:
            # multiple obstacle avoidance strategy
            # see how close obstacles are, if too close then average out
            # if far away then avoid both
            diff = abs(self.obstacle_data[0][4] - self.obstacle_data[1][4])
            faraway = False if diff < 8 else True
            closer_id = min(self.obstacle_data[0][4], self.obstacle_data[1][4])
            cl_ind = 0
            if self.obstacle_data[0][4] >= self.obstacle_data[1][4]:
                cl_ind = 1
            if closer_id > len(local_traj.poses) - 2:
                return
            drone_pt = local_traj.poses[closer_id].pose.position
            newx, newy, newz = self.deflector(drone_pt, cl_ind)
            # get second obstacle avoidance point
            fr_ind = 1 - cl_ind
            far_id = self.obstacle_data[fr_ind][4]
            if far_id > len(local_traj.poses) - 2:
                return
            drone_pt = local_traj.poses[far_id].pose.position
            newx2, newy2, newz2 = self.deflector(drone_pt, fr_ind)
            if not faraway:
                newTf = int(((self.Tf * 100) - self.i + 200) / 100)
                traj1, px1, py1, pz1 = trajectoryGenie(
                    [lx, ly, lz],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [newx, newy, newz],
                    self.velf,
                    self.accf,
                    1,
                    100,
                )
                traj2, px2, py2, pz2 = trajectoryGenie(
                    [newx, newy, newz],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    self.posf,
                    self.velf,
                    self.accf,
                    newTf,
                    100 * (newTf),
                )
                px = np.concatenate((px1, px2))
                py = np.concatenate((py1, py2))
                pz = np.concatenate((pz1, pz2))
            else:
                # far away so avoid both
                newTf = int(((self.Tf * 100) - self.i + 200) / 100)
                traj1, px1, py1, pz1 = trajectoryGenie(
                    [lx, ly, lz],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [newx, newy, newz],
                    self.velf,
                    self.accf,
                    1,
                    100,
                )
                traj1_, px1_, py1_, pz1_ = trajectoryGenie(
                    [newx, newy, newz],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [newx2, newy2, newz2],
                    self.velf,
                    self.accf,
                    1,
                    100,
                )
                traj2, px2, py2, pz2 = trajectoryGenie(
                    [newx2, newy2, newz2],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    self.posf,
                    self.velf,
                    self.accf,
                    newTf,
                    100 * (newTf),
                )
                px = np.concatenate((px1, px1_, px2))
                py = np.concatenate((py1, py1_, py2))
                pz = np.concatenate((pz1, pz1_, pz2))

        else:
            return

        x_nl = LQR(px, py, pz, newTf + 12, (newTf) * 100, phi0, theta0, psi0)
        # print("LQR solved")
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.i = 0
        self.time = np.linspace(0, newTf, newTf * 100)
        print("replanning finished!")
        self.traj_marker = trajectory_maker(px, py, pz)


def main(args=None):
    rclpy.init(args=args)
    drone_gym = DroneGym()
    rclpy.spin(drone_gym)
    drone_gym.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
