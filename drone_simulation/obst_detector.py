'''
Subscribes to obstacles
and publishes only those obstacles that are 
intersecting with drone trajectory
'''

import rclpy
from rclpy.node import Node
from functools import partial
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from tutorial_interfaces.msg import RemoteID

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('N_robots', 1)
        self.declare_parameter('my_id', 0)
        total_drones = self.get_parameter('N_robots').get_parameter_value().integer_value
        self.myid = self.get_parameter('my_id').get_parameter_value().integer_value
        
        #looking at other drones as obstacles true pose
        # for i in range(total_drones):
        #     if i+1 != self.myid:
        #         self.subscription_pose = self.create_subscription(
        #                 PoseStamped,
        #                 '/drone'+str(i+1)+'/true_pose',
        #                 partial(self.pose_callback, index=i),
        #                 1
        #             )
        #looking at other drones as obstacles from remote ID
        for i in range(total_drones):
            if i+1 != self.myid:
                self.subscription_pose = self.create_subscription(
                        RemoteID,
                        '/drone'+str(i+1)+'/Remote_id',
                        partial(self.rid_callback, index=i),
                        1
                    )
                
        self.subscription_pose  # prevent unused variable warning
        self.other_drone_poses = [[None, None, None]]*total_drones

        self.subscription_markers = self.create_subscription(
            MarkerArray,
            'obstacles',
            self.marker_array_callback,
            1
        )
        self.subscription_pose = self.create_subscription(
            Path,
            'local_traj',
            self.path_callback,
            1
        )
        self.publisher_near_obstacles = self.create_publisher(
            Float32MultiArray,
            'near_obstacles',
            1
        )
        self.obstacles = []
        self.safety_radius = 0.7

    
    # def pose_callback(self, msg, index):
    #     #actual pose subscriptions
    #     self.other_drone_poses[index][0] = round(msg.pose.position.x, 2)
    #     self.other_drone_poses[index][1] = round(msg.pose.position.y, 2)
    #     self.other_drone_poses[index][2] = round(msg.pose.position.z, 2)

    def rid_callback(self, msg, index):
        #need to covert to UTM if actual GPS used
        self.other_drone_poses[index][0] = round(msg.lattitude, 2)
        self.other_drone_poses[index][1] = round(msg.longitude, 2)
        self.other_drone_poses[index][2] = round(msg.altitude, 2)
        
    def marker_array_callback(self, msg):
        self.obstacles = msg.markers
        

    def path_callback(self, msg):
        robot_poses = msg
        self.publish_near_obstacles(robot_poses)

    def publish_near_obstacles(self, robot_poses):
        # if robot_poses is None or not self.obstacles:
        #     return

        near_obstacles = []
        robot_positions = robot_poses.poses
        ind = 0.0
        for pose_number, robot_position in enumerate(robot_positions):
            ind += 1.0
            for obstacle in self.obstacles:
                obstacle_position = obstacle.pose.position
                obstacle_radius = obstacle.scale.x/2.0 + self.safety_radius

                distance = self.calculate_distance(robot_position.pose.position, obstacle_position)
                if distance <= obstacle_radius:
                    obstacle_info = [
                        obstacle_radius-self.safety_radius,
                        obstacle_position.x,
                        obstacle_position.y,
                        obstacle_position.z,
                        ind
                    ]
                    near_obstacles.extend(obstacle_info)
            if len(near_obstacles) > 0:
                break
            ###add other drone obstacles only on a shorter horizon
            if 15 < pose_number < 30:
                for id, other_drone in enumerate(self.other_drone_poses):
                    if id != self.myid:
                        if other_drone[0] is not None:
                            distance = self.calculate_distance_custom(robot_position.pose.position, other_drone)
                            if distance <= 2.1: #2.1 is drone size
                                obstacle_info = [
                                    2.1,
                                    float(other_drone[0]),
                                    float(other_drone[1]),
                                    float(other_drone[2]),
                                    ind
                                ]
                                near_obstacles.extend(obstacle_info)
                        if len(near_obstacles) > 0:
                            break

        if len(near_obstacles) > 0:
            near_obstacles_msg = Float32MultiArray()
            near_obstacles_msg.data = near_obstacles
            self.publisher_near_obstacles.publish(near_obstacles_msg)

    @staticmethod
    def calculate_distance(position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        dz = position1.z - position2.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance
    
    @staticmethod
    def calculate_distance_custom(position1, other_drone):
        dx = position1.x - other_drone[0]
        dy = position1.y - other_drone[1]
        dz = position1.z - other_drone[2]
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
