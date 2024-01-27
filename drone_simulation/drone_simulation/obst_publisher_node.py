import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import math

#tested
class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_publisher")
        self.obstacle_publisher = self.create_publisher(MarkerArray, "obstacles", 1)
        self.timer = self.create_timer(1.0, self.timer_callback2)

        spheres = [
            {"x": 1.5, "y": 4.0, "z": 6.0, "radius": 2.0}
            # {'x': 3.0, 'y': 4.0, 'z': 5.0, 'radius': 0.5},
            # {'x': 1.0, 'y': 6.0, 'z': 8.0, 'radius': 1.8},
            # {'x': 0.0, 'y': 0.0, 'z': 6.0, 'radius': 1.9}
        ]
        self.markerarr = MarkerArray()
        index = 0
        for sphere in spheres:
            marker = Marker()
            marker.ns = "obstacles"
            marker.id = index
            marker.header.frame_id = "drone"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = sphere["x"]
            marker.pose.position.y = sphere["y"]
            marker.pose.position.z = sphere["z"]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * sphere["radius"]
            marker.scale.y = 2 * sphere["radius"]
            marker.scale.z = 2 * sphere["radius"]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            self.markerarr.markers.append(marker)
            index += 1

    def timer_callback2(self):
        self.obstacle_publisher.publish(self.markerarr)

#tested
class DynamicObstaclePublisher(Node):
    def __init__(self, orbit_radius, angular_velocity):
        super().__init__("dynamic_obstacle_publisher")
        self.obstacle_publisher = self.create_publisher(MarkerArray, "obstacles", 1)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.orbit_radius = orbit_radius
        self.angular_velocity = angular_velocity
        self.time = 0.0

    def timer_callback(self):
        self.time += 1.0

        # Update the dynamic obstacles' positions
        marker_array = self.update_dynamic_obstacles()

        # Publish the dynamic obstacles
        self.obstacle_publisher.publish(marker_array)

    def update_dynamic_obstacles(self):
        marker_array = MarkerArray()

        # Number of dynamic obstacles
        num_obstacles = 3

        for index in range(num_obstacles):
            marker = Marker()
            marker.ns = "dynamic_obstacles"
            marker.id = index
            marker.header.frame_id = "drone"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Update the position based on circular orbit
            angle = self.time * self.angular_velocity + (2 * math.pi / num_obstacles) * index
            marker.pose.position.x = self.orbit_radius * math.cos(angle)
            marker.pose.position.y = self.orbit_radius * math.sin(angle)
            marker.pose.position.z = 2.0  # Adjust the height based on your scenario

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 2.0  # Diameter of the sphere
            marker.scale.y = 2.0
            marker.scale.z = 2.0

            marker.color.r = 0.0  # Color can be adjusted
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

        return marker_array

def main(args=None):
    rclpy.init(args=args)
    dynamic_obstacle_ = False
    try:
        # Specify the orbit radius and angular velocity
        if dynamic_obstacle_:
            orbit_radius = 5.0
            angular_velocity = 0.05
            dynamic_obstacle_publisher = DynamicObstaclePublisher(orbit_radius, angular_velocity)
            rclpy.spin(dynamic_obstacle_publisher)
        else:
            obstacle_publisher = ObstaclePublisher()
            rclpy.spin(obstacle_publisher)
    except Exception as e:
        print("Error during node initialization: ", str(e))
    finally:
        dynamic_obstacle_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
