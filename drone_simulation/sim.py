'''
This code takes input start pose, goal pose and time. These states are passed to a trajectory generator.
The trajectory generator is based on the rapid motion primitive generation paper.
The generated trajectory is then sent to controller as a set of waypoints.
LQR controller is used to generate the control sequence which is rotor velocities for a quadrotor.
The control output, and current state is then carried over to a state estimation module.

'''

#todo: continuity in replanning state (velocity and acceleration)

import rclpy
from rclpy.node import Node
import numpy as np
from drone_simulation.trajectoryGen import trajectoryGenie, plotter
from drone_simulation.LQR import LQR
from drone_simulation.utils import drone_visualizer, trajectory_maker
from drone_simulation.utils import local_traj_maker
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

from visualization_msgs.msg import Marker
from tutorial_interfaces.msg import RemoteID
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

class Drone_sim(Node):

    def __init__(self):
        super().__init__('Drone_sim')

        #parameters
        self.declare_parameter('start_pose', [0, 0, 0])
        self.declare_parameter('my_id', 0)
        startpos = self.get_parameter('start_pose').get_parameter_value().integer_array_value
        self.myid = self.get_parameter('my_id').get_parameter_value().integer_value
        
        #publishers
        self.rid_publisher = self.create_publisher(RemoteID, 'Remote_id', 10)
        self.drone_publisher = self.create_publisher(Marker, 'drone', 1)
        self.pose_publisher = self.create_publisher(PoseStamped, 'true_pose', 1)
        #self.obstacle_publisher = self.create_publisher(MarkerArray, 'obstacles', 10)
        self.global_traj_publisher = self.create_publisher(Path, 'global_traj', 1 )
        self.local_traj_publisher = self.create_publisher(Path, 'local_traj', 1)

        #subscribers
        self.detected_obst_subscriber = self.create_subscription(Float32MultiArray, 'near_obstacles', self.obst_callback, 1)

        self.timer_period = 1/20 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.i = 0
        
        # Define the duration:
        self.Tf = 5 #in seconds
        numPlotPoints = 100*self.Tf #temporal resolution
        self.time = np.linspace(0, self.Tf, numPlotPoints)
        # Define the trajectory starting state:
        pos0 = startpos #position
        vel0 = [0, 0, 0] #velocity
        acc0 = [0, 0, 0] #acceleration

        # Define the goal state:
        self.posf = [pos0[0], pos0[1], pos0[2] + 15]  # position
        self.velf = [0, 0, 0]  # velocity
        self.accf = [0, 9.81, 0]  # acceleration

        # Rapid trajectory generator
        traj, px, py, pz = trajectoryGenie(pos0, vel0, acc0, self.posf, self.velf, self.accf, self.Tf, numPlotPoints)
        self.traj_marker = trajectory_maker(px, py, pz)
        x_nl = LQR(px, py, pz, self.Tf+2, numPlotPoints)
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.tru_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #obstacle flags and data
        self.collision_flag = False
        self.replanning = False
        self.obstacle_data = []      
    
    def timer_callback(self):
        k = RemoteID()
        k.sysid = self.myid
        t = self.get_clock().now()
        k.header.stamp = t.to_msg()
        if self.i < len(self.time)-1:
            k.lattitude, k.longitude, k.altitude = self.ax[self.i], self.ay[self.i], self.az[self.i]#self.traj.get_position(self.time[self.i])
            #vx, vy, k.vertical_speed = self.traj.get_velocity(self.time[self.i])
            vx_ = (self.ax[self.i +1] - self.ax[self.i])/(self.timer_period)
            vy_ = (self.ay[self.i +1] - self.ay[self.i])/(self.timer_period)
            k.vertical_speed = (self.az[self.i +1] - self.az[self.i])/(self.timer_period)
            k.horizontal_speed = np.hypot(vx_, vy_)
            k.heading = np.arctan2(vy_, vx_)
            rol, pit, ya = self.roll[self.i], self.pitch[self.i], self.yaw[self.i]
            drone_marker, pos = drone_visualizer(k.lattitude, k.longitude, k.altitude, rol, pit, ya)
            local_traj_marker = local_traj_maker(self.ax, self.ay, self.az, self.i)
            self.tru_pose = k.lattitude, k.longitude, k.altitude, rol, pit, ya
            #all publishers
            self.drone_publisher.publish(drone_marker)
            self.pose_publisher.publish(pos)
            self.rid_publisher.publish(k)
            
            self.global_traj_publisher.publish(self.traj_marker)
            self.local_traj_publisher.publish(local_traj_marker)
        if not self.collision_flag:
            self.i += 1
            
        if self.collision_flag:
            print("collision found!! replanning begins")
            self.replanner(local_traj_marker)
            self.collision_flag = False
                  

    def obst_callback(self, msg):
        obstacles = msg.data
        if len(obstacles) > 0 and self.i > 4: #first five points should be obstacle free
            self.collision_flag = True
            [r, x, y, z, ind] = obstacles
            self.obstacle_data = [r, x, y, z, ind-1]
        else:
            self.collision_flag = False


    def replanner(self, local_traj):
        self.replanning = True
        [lx, ly, lz, phi0, theta0, psi0] = self.tru_pose
        drone_pt = local_traj.poses[int(self.obstacle_data[-1])].pose.position
        obst_pt = self.obstacle_data[1], self.obstacle_data[2], self.obstacle_data[3]
        r = self.obstacle_data[0]/1.8
        print(obst_pt)
        if drone_pt.x == obst_pt[0]: newx = drone_pt.x
        else:
            dir_x = abs(drone_pt.x - obst_pt[0])/(drone_pt.x - obst_pt[0])
            newx = drone_pt.x + dir_x /max(abs(drone_pt.x - obst_pt[0]), r)
        if drone_pt.y == obst_pt[1]: newy = drone_pt.y
        else:
            dir_y = abs(drone_pt.y - obst_pt[1])/(drone_pt.y - obst_pt[1])
            newy = drone_pt.y + dir_y /max(abs(drone_pt.y - obst_pt[1]), r)
        
        #dir_z = abs(drone_pt.z - obst_pt[2])/(drone_pt.z - obst_pt[2])
        
        #newz = drone_pt.z + dir_z /max(abs(drone_pt.z - obst_pt[2]), r)
        #print(drone_pt, newx, newy, newz)
        newTf = int(((self.Tf * 100) - self.i + 150)/100)
        traj1, px1, py1, pz1 = trajectoryGenie([lx, ly, lz], [0., 0., 0.], [0., 0., 0.0], [newx, newy, drone_pt.z], self.velf, self.accf, 1, 100)
        traj2, px2, py2, pz2 = trajectoryGenie([newx, newy, drone_pt.z], [0., 0., 0.], [0., 0., 0.0], self.posf, self.velf, self.accf, newTf, 100*(newTf))
        px = np.concatenate((px1, px2))
        py = np.concatenate((py1, py2))
        pz = np.concatenate((pz1, pz2))
        x_nl = LQR(px, py, pz, newTf+12, (newTf)*100, phi0, theta0, psi0)
        #print("LQR solved")
        self.ax, self.ay, self.az = x_nl[:, 0], x_nl[:, 2], x_nl[:, 4]
        self.roll, self.pitch, self.yaw = x_nl[:, 6], x_nl[:, 8], x_nl[:, 10]
        self.i = 0
        self.time = np.linspace(0, newTf, newTf*100)
        print("replanning finished!")
        self.traj_marker = trajectory_maker(px, py, pz)
        
rclpy.init(args=None)
Drone_sim = Drone_sim()
rclpy.spin(Drone_sim)
Drone_sim.destroy_node()
rclpy.shutdown()