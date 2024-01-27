import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

class GpsSpoofingNode(Node):
    def __init__(self):
        super().__init__('gps_spoofing_node')

        # Create a publisher to publish the spoofed GPS data
        self.spoofed_publisher = self.create_publisher(NavSatFix, '/drone/spoofed_gps_data', 10)

        # Publish cmd_vel messages to simulate the effect of the spoofing attack
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Create a subscriber to listen to the original GPS data
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps_data',
            self.gps_callback,
            10
        )

        # Parameterization
        self.spoofing_offset = 6.0
        self.attack_duration = 6.0
        self.rate = 1.0  # Default rate

    def gps_callback(self, msg):
        try:
            # Modify the GPS data here as per spoofing strategy
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
            #self.get_logger().info("Spoofed GPS data published: %s", spoofed_msg)

            # Simulate the effect of the spoofing attack
            self.effect_gps_attack(delta_x, delta_y, delta_z, self.attack_duration)

        except Exception as e:
            self.get_logger().error("Error in GPS callback: %s", str(e))

    def effect_gps_attack(self, dx, dy, dz, duration):
        try:
            # Calculate drift velocity
            drift_vel = Twist()
            drift_vel.linear.x = -dx / duration
            drift_vel.linear.y = -dy / duration
            drift_vel.linear.z = -dz / duration

            # Publish the drift velocity for duration seconds
            rate = self.create_rate(self.rate)
            for _ in range(int(duration)):
                self.cmd_vel_publisher.publish(drift_vel)
                rate.sleep()

            # Stop the effect
            drift_vel2 = Twist()
            self.cmd_vel_publisher.publish(drift_vel2)

        except Exception as e:
            self.get_logger().error("Error in effect_gps_attack: %s", str(e))

    def realistic_effect_gps_attack(self, dx, dy, dz, duration=1.0):
        # Implement realistic conditions for rejecting the spoofing attack
        simulate_effect = abs(dx) <= 0.1 and abs(dy) <= 0.1 and abs(dz) <= 0.1

        # Add more conditions here

        if simulate_effect:
            self.effect_gps_attack(dx, dy, dz, duration)

    def gps_attack_model(self, lat, lon, alt):
        # Implement the attack model
        new_lat = lat + self.spoofing_offset
        new_lon = lon + self.spoofing_offset
        new_alt = alt
        return new_lat, new_lon, new_alt


def main(args=None):
    rclpy.init(args=args)

    try:
        spoofing_node = GpsSpoofingNode()
        rclpy.spin(spoofing_node)
    except Exception as e:
        print("Error during node initialization: ", str(e))
    finally:
        spoofing_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
