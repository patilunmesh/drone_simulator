import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import math

def drone_visualizer(x, y, z, r, p, yaw, victim=False):
    #creates drone marker and drone true position
    drone_marker = Marker()
    pos =Odometry()
    drone_marker.header.frame_id = '/drone'
    pos.header = drone_marker.header
    drone_marker.id = 0
    drone_marker.type = Marker.MESH_RESOURCE
    drone_marker.action = Marker.ADD

    drone_marker.scale.x = 1.2
    drone_marker.scale.y = 1.2
    drone_marker.scale.z = 1.2

    drone_marker.color.r = 0.6
    drone_marker.color.g = 0.6
    drone_marker.color.b = 0.6
    drone_marker.color.a = 1.0

    if victim:
        drone_marker.color.r = 1.0
        drone_marker.color.g = 0.0
        drone_marker.color.b = 0.0
        drone_marker.color.a = 0.7


    drone_marker.pose.position.x = x
    drone_marker.pose.position.y = y
    drone_marker.pose.position.z = z
    pos.pose.pose.position.x, pos.pose.pose.position.y, pos.pose.pose.position.z = x, y, z

    q = quaternion_from_euler(r, p, yaw)
    drone_marker.pose.orientation.x = q[0]
    drone_marker.pose.orientation.y = q[1]
    drone_marker.pose.orientation.z = q[2]
    drone_marker.pose.orientation.w = q[3]
    pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w = q[0], q[1], q[2], q[3]

    drone_marker.mesh_resource = "package://drone_simulation/drone600.stl"
    return drone_marker, pos

def trajectory_maker(px, py, pz):
    traj = Path()
    traj.header.frame_id = '/drone'
    for i in range(len(px)):
        pose = PoseStamped()
        pose.pose.position.x = px[i]
        pose.pose.position.y = py[i]
        pose.pose.position.z = pz[i]
        traj.poses.append(pose)

    return traj

def local_traj_maker(xs, ys, zs, ind, local_path_length=30):
    traj = Path()
    traj.header.frame_id = '/drone'
    if ind+local_path_length < len(xs):

       for i in range(ind, ind + local_path_length):
           pose = PoseStamped()
           pose.pose.position.x = xs[i]
           pose.pose.position.y = ys[i]
           pose.pose.position.z = zs[i]
           traj.poses.append(pose)
    return traj

def to_sec(t):
    sec, nanosec = t.seconds_nanoseconds()
    s = (sec + nanosec/1e9)
    return s

def header_to_sec(header):
        s = (header.stamp.sec + header.stamp.nanosec/1e9)
        return s

def update_time(t, step, i):
    s = to_sec(t)
    s_ = (s + i*step)*1e9
    sec = int(s_//1e9)
    nanosec = int(s_%1e9)
    return sec, nanosec

def cosine_between_vectors(base, v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    base = np.array(base)
    dot_prod = (np.dot((v1 - base),(v2 - base)))/(np.linalg.norm(v1 - base) * np.linalg.norm(v2 - base))
    return np.arccos(dot_prod)

def l2_norm(v1, v2):
    return np.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2 + (v1[2] - v2[2])**2)

def get_successors( A, x, y, z, step):
    #aplying actions to target
    match A:
        case 0: #stay
            return x, y, z
        case 1:#right
            return x+step, y, z
        case 2:#left
            return x-step, y ,z
        case 3:#up
            return x, y, z+step
        case 4:#down
            return x, y, z-step
        case 5: #front
            return x, y+step, z
        case 6: #back
            return x, y-step, z

def save_numpy_array(file_name, array):
    np.save(file_name, array)
    print("Saved to ", file_name)

def load_numpy_array(file_name):
    array = np.load(file_name)
    return array

def xy_to_grid(x, y, w):
    ind = y * (w) + x
    return ind


def grid_to_xy(ind, w):
    y = int(ind / w)
    x = ind - y * w
    return x, y

def obst_processor(self, msg):
    obstacles = msg.data
    if len(obstacles) > 0:
        [r, x, y, z, ind] = obstacles
        obst_loc = [x, y, z, r]
    return obst_loc

def velocity_calc(heading, v_hor):
    # 1 second time horizon
    correction = 0.0
    vx = v_hor * math.cos(heading - correction)
    vy = v_hor * math.sin(heading - correction)
    return vx, vy

def to_sec_(header):
    s = header.stamp.sec + header.stamp.nanosec / 1e9
    return s

def cone_visualizer(x, y, z, r=0.6):
    # creates cone marker and shows predicted drone position
    drone_marker = Marker()
    drone_marker.header.frame_id = "/drone"
    drone_marker.id = 1
    drone_marker.type = Marker.MESH_RESOURCE
    drone_marker.action = Marker.ADD

    drone_marker.scale.x = 1.0
    drone_marker.scale.y = 1.0
    drone_marker.scale.z = 1.0

    drone_marker.color.r = 0.6
    drone_marker.color.g = 0.6
    drone_marker.color.b = 0.6
    drone_marker.color.a = 1.0

    drone_marker.pose.position.x = x
    drone_marker.pose.position.y = y
    drone_marker.pose.position.z = z
    drone_marker.mesh_resource = "package://drone_simulation/cone.stl"
    return drone_marker


def sphere_visualizer_xyz(x, y, z):
    # creates cone marker and shows predicted drone position
    r = 0.5
    marker = Marker()
    marker.ns = "obstacles"
    marker.id = 0
    marker.header.frame_id = "drone"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale.x = 2 * r
    marker.scale.y = 2 * r
    marker.scale.z = 2 * r
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    return marker


def drone_visualizer_xyz(x, y, z):
    # creates drone marker and drone true position
    drone_marker = Marker()
    drone_marker.header.frame_id = "/drone"
    drone_marker.id = 2
    drone_marker.type = Marker.MESH_RESOURCE
    drone_marker.action = Marker.ADD

    drone_marker.scale.x = 1.0
    drone_marker.scale.y = 1.0
    drone_marker.scale.z = 1.0

    drone_marker.color.r = 0.8
    drone_marker.color.g = 0.2
    drone_marker.color.b = 0.2
    drone_marker.color.a = 1.0

    drone_marker.pose.position.x = x
    drone_marker.pose.position.y = y
    drone_marker.pose.position.z = z
    drone_marker.mesh_resource = "package://drone_simulation/drone600.stl"
    return drone_marker

