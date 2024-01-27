import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class MultiDroneGpsSpoofingNode(Node):
    def __init__(self, target_drone_id, neighbor_range):
        super().__init__('multi_drone_gps_spoofing_node')

        # Parameters
        self.target_drone_id = target_drone_id
        self.neighbor_range = neighbor_range
        self.target_lat = 10.0   # Latitude of the target drone
        self.target_lon = 20.0   # Longitude of the target drone

        # Create a publisher to publish the spoofed GPS data
        self.spoofed_publisher = self.create_publisher(NavSatFix, '/drone/spoofed_gps_data', 10)

        # Publish cmd_vel messages to simulate the effect of the spoofing attack
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Create subscribers to listen to the GPS data from all drones
        self.gps_subscribers = {}
        for drone_id in range(1, 11):  # Assuming drones IDs from 1 to 10
            topic = f'/drone/{drone_id}/gps_data'
            self.gps_subscribers[drone_id] = self.create_subscription(
                NavSatFix,
                topic,
                self.gps_callback,
                10
            )

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Calculate Euclidean distance between two GPS coordinates
        R = 6371000.0  # Earth radius in meters

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = math.sin(dlat / 2.0) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def gps_callback(self, msg):
        drone_id = int(msg.header.frame_id)  # Assuming drone ID is embedded in the frame_id
        try:
            # Modify the GPS data only for the target drone and its neighbors
            if (
                drone_id == self.target_drone_id or
                self.calculate_distance(msg.latitude, msg.longitude, self.target_lat, self.target_lon) <= self.neighbor_range
            ):
                # Apply the spoofing attack
                new_lat, new_lon, new_alt = self.gps_attack_model(msg.latitude, msg.longitude, msg.altitude)

                # Calculate deltas
                delta_x = new_lat - msg.latitude
                delta_y = new_lon - msg.longitude
                delta_z = new_alt - msg.altitude

                # Create spoofed GPS message
                spoofed_msg = NavSatFix()
                spoofed_msg.header = msg.header
                spoofed_msg.latitude = new_lat
                spoofed_msg.longitude = new_lon
                spoofed_msg.altitude = new_alt

                # Publish the spoofed GPS data
                self.spoofed_publisher.publish(spoofed_msg)
                #self.get_logger().info(f"Spoofed GPS data published for drone {drone_id}")

                # Simulate the effect of the spoofing attack
                self.effect_gps_attack(delta_x, delta_y, delta_z, t=6.0)

        except Exception as e:
            self.get_logger().error(f"Error in GPS callback for drone {drone_id}: {str(e)}")

    def effect_gps_attack(self, dx, dy, dz, t=1.0):
        try:
            # Calculate drift velocity as a difference between the current and spoofed GPS data
            drift_vel = Twist()
            drift_vel.linear.x = -dx / t
            drift_vel.linear.y = -dy / t
            drift_vel.linear.z = -dz / t

            # Publish the drift velocity for t seconds
            rate = self.create_rate(t)
            for _ in range(int(t)):
                self.cmd_vel_publisher.publish(drift_vel)
                rate.sleep()

            # Stop the effect
            drift_vel2 = Twist()
            self.cmd_vel_publisher.publish(drift_vel2)

        except Exception as e:
            self.get_logger().error(f"Error in effect_gps_attack: {str(e)}")

    def gps_attack_model(self, lat, lon, alt):
        # Input: original lat, lon, alt
        # Output: spoofed lat, lon, alt
        # This function implements the attack model
        # For simplicity, let's add a fixed offset to the latitude and longitude
        new_lat = lat + 0.001
        new_lon = lon + 0.001
        new_alt = alt
        return new_lat, new_lon, new_alt


def main(args=None):
    rclpy.init(args=args)

    try:
        # Specify the target drone ID and the neighbor range
        target_drone_id = 5
        neighbor_range = 100  # Assuming distance in meters
        multi_drone_gps_spoofing_node = MultiDroneGpsSpoofingNode(target_drone_id, neighbor_range)
        rclpy.spin(multi_drone_gps_spoofing_node)
    except Exception as e:
        print("Error during node initialization: ", str(e))
    finally:
        multi_drone_gps_spoofing_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
