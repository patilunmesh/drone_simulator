from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

from visualization_msgs.msg import Marker
from tutorial_interfaces.msg import RemoteID
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


def drone_visualizer(x, y, z, r, p, yaw):
    #creates drone marker and drone true position
    drone_marker = Marker()
    pos =PoseStamped()
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

    drone_marker.pose.position.x = x
    drone_marker.pose.position.y = y
    drone_marker.pose.position.z = z
    pos.pose.position.x, pos.pose.position.y, pos.pose.position.z = x, y, z

    q = quaternion_from_euler(r, p, yaw)
    drone_marker.pose.orientation.x = q[0]
    drone_marker.pose.orientation.y = q[1]
    drone_marker.pose.orientation.z = q[2]
    drone_marker.pose.orientation.w = q[3]
    pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z, pos.pose.orientation.w = q[0], q[1], q[2], q[3]

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

def local_traj_maker(xs, ys, zs, ind):
    traj = Path()
    traj.header.frame_id = '/drone'
    local_path_length = 60
    if ind+local_path_length < len(xs):
       
       for i in range(ind, ind + local_path_length):
           pose = PoseStamped()
           pose.pose.position.x = xs[i]
           pose.pose.position.y = ys[i]
           pose.pose.position.z = zs[i]
           traj.poses.append(pose)
    return traj

           
