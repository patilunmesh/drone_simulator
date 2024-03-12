"""
Sim summary:
This code takes input start pose, goal pose and time. These states are passed to a trajectory generator.
The trajectory generator is based on the rapid motion primitive generation paper.
The generated trajectory is then sent to controller as a set of waypoints.
# TODO: CHANGE THIS GENERATED TRAJECTORY TO ONE THAT IS CBF BASED
LQR controller is used to generate the control sequence which is rotor velocities for a quadrotor.
The control output, and current state is then carried over to a state estimation module.

"""

import time
import numpy as np
from scipy.optimize import minimize, LinearConstraint
import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

from drone_simulation.LQR import LQR
from drone_simulation.trajectoryGen import trajectoryGenie, trajectoryGenieAccel
from drone_simulation.utils import (
    drone_visualizer,
    local_traj_maker,
    trajectory_maker,
    update_time,
)

from tutorial_interfaces.srv import ResetSim

class DroneCBFSim(Node):
    def __init__(self):
        super().__init__("DroneCBFSim")

        # parameters
        self.declare_parameter("start_pose", [0, 0, 0])
        self.declare_parameter("my_id", 0)
        self.declare_parameter("cbf_param", 1)
        startpos = (
            self.get_parameter("start_pose").get_parameter_value().integer_array_value
        )
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value
        self.cbf_param = self.get_parameter("cbf_param").get_parameter_value().integer_value

        # publishers
        self.drone_publisher = self.create_publisher(Marker, "drone", 1)
        self.pose_publisher = self.create_publisher(Odometry, "true_pose", 1)
        self.global_traj_publisher = self.create_publisher(Path, "global_traj", 1)
        self.local_traj_publisher = self.create_publisher(Path, "local_traj", 1)
        
        # subscribers
        self.detected_obst_subscriber = self.create_subscription(
            Float32MultiArray, "near_obstacles", self.obst_callback, 1
        )
        self.cbf_subscriber = self.create_subscription(
            Float32MultiArray, "/drone" + str(self.myid) + "/cbf_val", self.cbf_callback, 1
        )
        self.cbf_vals = []
        self.cbf_warn = False

        self.timer_period = 1 / 10  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # reset service used to send this drone back to starting position
        self.reset_service = self.create_service(ResetSim, "/drone" + str(self.myid) + "/ResetSim", self.reset_callback)

        self.i = 0

        # Define the duration of simulation:
        self.Tf = 5  # in seconds
        self.numPlotPoints = 100 * self.Tf  # temporal resolution
        self.time = np.linspace(0, self.Tf, self.numPlotPoints)
        # Define the trajectory starting state:
        self.pos0 = startpos  # position
        self.vel0 = [0, 0, 0]  # velocity
        self.acc0 = [0, 0, 0]  # acceleration

        # Define the goal state:
        # self.posf = [self.pos0[0] + 10, self.pos0[1], self.pos0[2] + 10]  # position
        self.posf = [1.5, 4, 9]  # position
        self.velf = [0, 0, 0]  # velocity
        self.accf = [0, 9.81, 0]  # acceleration

        # Rapid trajectory generator
        # self.traj, self.px, self.py, self.pz = trajectoryGenie(
        #     self.pos0, self.vel0, self.acc0, self.posf, self.velf, self.accf, self.Tf, self.numPlotPoints
        # )
        self.traj, self.px, self.py, self.pz, self.accel = trajectoryGenieAccel(
            self.pos0, self.vel0, self.acc0, self.posf, self.velf, self.accf, self.Tf, self.numPlotPoints
        )
        self.traj_marker = trajectory_maker(self.px, self.py, self.pz)
        self.tru_state = [self.pos0[0], self.pos0[1], self.pos0[1], 0.0, 0.0, 0.0]

        #solving control sequence beforhand
        x_nl = LQR(self.px, self.py, self.pz, self.Tf + 2, self.numPlotPoints)
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.tru_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # obstacle flags and data
        self.collision_flag = False
        self.replanning = False
        self.obstacle_data = []
        self.t0 = self.get_clock().now()

    def timer_callback(self):
        sec, nanosec = update_time(self.t0, 0.01, self.i)
        head = Header()
        head.stamp.sec, head.stamp.nanosec = sec, nanosec
        if self.i < len(self.time) - 1:    
            x = self.ax[self.i]
            y = self.ay[self.i]
            z = self.az[self.i]
            vx_ = (self.ax[self.i + 1] - self.ax[self.i]) / (self.timer_period)
            vy_ = (self.ay[self.i + 1] - self.ay[self.i]) / (self.timer_period)
            vz_ = (self.az[self.i + 1] - self.az[self.i]) / (self.timer_period)
            rol, pit, ya = self.roll[self.i], self.pitch[self.i], self.yaw[self.i]
            drone_marker, pos = drone_visualizer(x, y, z, rol, pit, ya)
            pos.twist.twist.linear.x, pos.twist.twist.linear.y, pos.twist.twist.linear.z = vx_, vy_, vz_
            local_traj_marker = local_traj_maker(self.ax, self.ay, self.az, self.i)
            self.tru_pose = x, y, z, rol, pit, ya
            self.tru_state = [x, y, z, vx_, vy_, vz_]
            # all publishers
            self.drone_publisher.publish(drone_marker)
            self.pose_publisher.publish(pos)

            self.global_traj_publisher.publish(self.traj_marker)
            self.local_traj_publisher.publish(local_traj_marker)
        else:
            print('simulation finished, please reset')

        if not self.collision_flag:
            self.i += 1

        if self.collision_flag:
            print("collision found!! replanning begins")
            print(self.cbf_vals)
            self.replanner(local_traj_marker)
            self.collision_flag = False

    def obst_callback(self, msg):
        obstacles = msg.data
        if (len(obstacles) > 0 and self.i > 4):  
            # first five points should be obstacle free
            self.collision_flag = True
            [r, x, y, z, ind] = obstacles
            self.obstacle_data = [r, x, y, z, ind - 1]
        else:
            self.collision_flag = False

    def cbf_callback(self, msg):
        cbf_vals = msg.data
        new_cbf_vals = []
        c = 0
        cbf_vals_obs = []
        while c < len(cbf_vals):
            cbf_vals_obs.append(cbf_vals[c])
            if len(cbf_vals_obs) == 4:
                new_cbf_vals.append(list(cbf_vals_obs))
                cbf_vals_obs = []
            c += 1
        self.cbf_vals = new_cbf_vals

    def reset_callback(self, request, response):
        self.get_logger().info('Getting Request to Reset Sim')
        self.i = 0

        if request.cbf_param != self.cbf_param:
            # TODO: redo trajectory
            pass
        
        x_nl = LQR(self.px, self.py, self.pz, self.Tf + 2, self.numPlotPoints)
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.tru_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.collision_flag = False
        self.replanning = False
        self.obstacle_data = []
        self.t0 = self.get_clock().now()

        self.timer.destroy()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        response.done = True
        return response

    def replanner(self, local_traj):
        '''
        This function can be used to customize the obstacle avoidance feature
        '''
        self.replanning = True
        
        # get new position using CBF controller
        safe_state = self.keep_safe()
        newx = safe_state[0]
        newy = safe_state[1]
        newz = safe_state[1]

        [lx, ly, lz, phi0, theta0, psi0] = self.tru_pose
        drone_pt = local_traj.poses[int(self.obstacle_data[-1])].pose.position  # get position at index of trajectory where they intersect
        # obst_pt = self.obstacle_data[1], self.obstacle_data[2], self.obstacle_data[3]
        # r = self.obstacle_data[0] / 1.0
        # if drone_pt.x == obst_pt[0]:
        #     newx = drone_pt.x
        # else:
        #     dir_x = abs(drone_pt.x - obst_pt[0]) / (drone_pt.x - obst_pt[0])
        #     newx = drone_pt.x + dir_x / max(abs(drone_pt.x - obst_pt[0]), r)
        # if drone_pt.y == obst_pt[1]:
        #     newy = drone_pt.y
        # else:
        #     dir_y = abs(drone_pt.y - obst_pt[1]) / (drone_pt.y - obst_pt[1])
        #     newy = drone_pt.y + dir_y / max(abs(drone_pt.y - obst_pt[1]), r)

        newTf = int(((self.Tf * 100) - self.i + 200) / 100)
        traj1, px1, py1, pz1 = trajectoryGenie(
            [lx, ly, lz],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            # [newx, newy, drone_pt.z],
            [newx, newy, newz],
            self.velf,
            self.accf,
            1,
            100,
        )
        traj2, px2, py2, pz2 = trajectoryGenie(
            # [newx, newy, drone_pt.z],
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
        x_nl = LQR(px, py, pz, newTf + 12, (newTf) * 100, phi0, theta0, psi0)
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.i = 0
        self.time = np.linspace(0, newTf, newTf * 100)
        print("replanning finished!")
        print(self.cbf_vals)
        self.traj_marker = trajectory_maker(px, py, pz)


    def keep_safe(self):
        dt = self.timer_period
        f = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        g = np.array([
            [0.5*dt**2, 0, 0],
            [0, 0.5*dt**2, 0],
            [0, 0, 0.5*dt**2],
            [dt, 0, 0],
            [0, dt, 0],
            [0, 0, dt],
        ])

        
        A = []
        b = []
        for val in self.cbf_vals:
            h = val[0]
            h_deriv = [val[1], val[2], val[3], 0, 0, 0]
            Lfh = np.dot(h_deriv, np.dot(f, self.tru_state))
            Lgh = np.dot(h_deriv, g)
            A.append(Lgh)
            b.append(Lfh + self.cbf_param * h)

        constraints = LinearConstraint(A, lb=b, ub=np.inf)
        def min_func(u):
            return np.linalg.norm(u - self.accel[self.i])**2
        u0 = self.accel[self.i]
        try:
            res = minimize(min_func, u0, constraints=constraints)
            u_safe = res.x
        except Exception as e:
            print('could not solve for safe inputs {}'.format(e))
            u_safe = self.accel[self.i]

        # the ideal new state to keep it safe
        new_state = np.dot(f, self.tru_state) + np.dot(g, u_safe)

        return new_state


rclpy.init(args=None)
Drone_sim = DroneCBFSim()
time.sleep(0.15)
rclpy.spin(Drone_sim)
Drone_sim.destroy_node()
rclpy.shutdown()
