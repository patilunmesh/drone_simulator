"""
Subscribes to obstacles
and publishes only those obstacles that are
intersecting with drone trajectory
"""

from functools import partial

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from tutorial_interfaces.msg import Obstacle, Obstacles, RemoteID


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.declare_parameter("N_robots", 1)
        self.declare_parameter("my_id", 0)
        total_drones = (
            self.get_parameter("N_robots").get_parameter_value().integer_value
        )
        self.myid = self.get_parameter("my_id").get_parameter_value().integer_value

        # looking at other drones as obstacles true pose
        # for i in range(total_drones):
        #     if i+1 != self.myid:
        #         self.subscription_pose = self.create_subscription(
        #                 PoseStamped,
        #                 '/drone'+str(i+1)+'/true_pose',
        #                 partial(self.pose_callback, index=i),
        #                 1
        #             )
        # looking at other drones as obstacles from remote ID
        for i in range(total_drones):
            if i + 1 != self.myid:
                self.subscription_pose = self.create_subscription(
                    Marker,
                    "/drone" + str(i + 1) + "/drone",
                    partial(self.physical_drone_callback, index=i),
                    1,
                )

        self.subscription_pose  # prevent unused variable warning
        self.other_drone_poses = [[None, None, None, None]] * total_drones

        # self.subscription_markers = self.create_subscription(
        #     MarkerArray,
        #     'obstacles',
        #     self.marker_array_callback,
        #     1
        # )
        self.subscription_pose = self.create_subscription(
            Path, "local_traj", self.path_callback, 1
        )
        self.publisher_near_obstacles = self.create_publisher(
            Obstacles, "near_obstacles", 1
        )
        self.obstacles = []
        self.safety_radius = 0.7

    # def pose_callback(self, msg, index):
    #     #actual pose subscriptions
    #     self.other_drone_poses[index][0] = round(msg.pose.position.x, 2)
    #     self.other_drone_poses[index][1] = round(msg.pose.position.y, 2)
    #     self.other_drone_poses[index][2] = round(msg.pose.position.z, 2)

    def physical_drone_callback(self, msg, index):
        # no rid dependancy
        self.other_drone_poses[index][0] = round(msg.pose.position.x, 2)
        self.other_drone_poses[index][1] = round(msg.pose.position.y, 2)
        self.other_drone_poses[index][2] = round(msg.pose.position.z, 2)
        self.other_drone_poses[index][3] = round(msg.scale.x / 2.0, 2)

    # def marker_array_callback(self, msg):
    #     self.obstacles = msg.markers

    def path_callback(self, msg):
        robot_poses = msg
        self.publish_near_obstacles(robot_poses, msg.header)

    def publish_near_obstacles(self, robot_poses, header):
        # if robot_poses is None or not self.obstacles:
        #     return

        near_obs = Obstacles()
        near_obstacles = []
        robot_positions = robot_poses.poses
        ind = 0  # index of the pose
        for pose_number, robot_position in enumerate(robot_positions):
            ind += 1
            # for obstacle in self.obstacles:
            #     obstacle_position = obstacle.pose.position
            #     obstacle_radius = obstacle.scale.x + self.safety_radius

            #     distance = self.calculate_distance(robot_position.pose.position, obstacle_position)
            #     if distance <= obstacle_radius:
            #         obst = Obstacle()
            #         obst.r = float(obstacle_radius-self.safety_radius)
            #         obst.x =  obstacle_position.x
            #         obst.y =  obstacle_position.y
            #         obst.z =  obstacle_position.z
            #         obst.path_id =  ind
            #         obst.obstacle_id =  0 #id of the non drone obstacle
            #         near_obstacles.append(obst)
            if len(near_obstacles) > 1:
                break
            ###add other drone obstacles only on a shorter horizon
            # if 10 < pose_number < 40:
            for id, other_drone in enumerate(self.other_drone_poses):
                if id != self.myid - 1:
                    if other_drone[0] is not None:
                        obstacle_radius = float(other_drone[3]) + self.safety_radius
                        distance = self.calculate_distance_custom(
                            robot_position.pose.position, other_drone
                        )
                        if distance <= obstacle_radius:
                            obst = Obstacle()
                            obst.r = float(obstacle_radius - self.safety_radius)
                            obst.x = float(other_drone[0])
                            obst.y = float(other_drone[1])
                            obst.z = float(other_drone[2])
                            obst.path_id = ind
                            obst.obstacle_id = id + 1  # id of the non drone obstacle
                            near_obstacles.append(obst)
                    if len(near_obstacles) > 1:
                        break

        if len(near_obstacles) > 0:
            # print(" ###################### ", len(near_obstacles))
            near_obs.data = near_obstacles
            near_obs.header = header
            self.publisher_near_obstacles.publish(near_obs)

    @staticmethod
    def calculate_distance(position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        dz = position1.z - position2.z
        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        return distance

    @staticmethod
    def calculate_distance_custom(position1, other_drone):
        dx = position1.x - other_drone[0]
        dy = position1.y - other_drone[1]
        dz = position1.z - other_drone[2]
        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        return distance


def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
