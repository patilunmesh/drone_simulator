import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import random
#tested with gazebo plugin sjtu_drone
class ImuSpoofingNode(Node):
    def __init__(self):
        super().__init__('imu_spoofing_node')

        # Create a publisher to publish the spoofed IMU data
        self.spoofed_publisher = self.create_publisher(Imu, '/drone/spoofed_imu_data', 10)

        # Publish cmd_vel messages to simulate the effect of the spoofing attack
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Create a subscriber to listen to the original IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/drone/imu/out',
            self.imu_callback,
            10
        )

        # Parameterization
        self.spoofing_offset = 0.1  # Modify based on your spoofing strategy
        self.attack_fail_probability = 0.1 #probability of spoofing attack failure
        self.attack_duration = 6.0
        self.rate = 1.0  # Default rate

    def imu_callback(self, msg):
        try:
            # Modify the IMU data here as per spoofing strategy
            new_msg = self.imu_attack_model(msg)

            # Calculate deltas
            delta_ang_vel_x = new_msg.angular_velocity.x - msg.angular_velocity.x
            delta_ang_vel_y = new_msg.angular_velocity.y - msg.angular_velocity.y
            delta_ang_vel_z = new_msg.angular_velocity.z - msg.angular_velocity.z

            # Publish the spoofed IMU data
            self.spoofed_publisher.publish(new_msg)
            #self.get_logger().info("Spoofed IMU data published: %s", new_msg)

            # Simulate the effect of the spoofing attack
            self.effect_imu_attack(delta_ang_vel_x, delta_ang_vel_y, delta_ang_vel_z, self.attack_duration)

        except Exception as e:
            self.get_logger().error("Error in IMU callback: %s", str(e))

    def effect_imu_attack(self, d_ang_vel_x, d_ang_vel_y, d_ang_vel_z, duration):
        try:
            # Calculate drift angular velocity
            #a PID based effect model can be implemented here
            drift_ang_vel = Twist()
            drift_ang_vel.angular.x = -d_ang_vel_x / duration
            drift_ang_vel.angular.y = -d_ang_vel_y / duration
            drift_ang_vel.angular.z = -d_ang_vel_z / duration

            # Publish the drift angular velocity for duration seconds
            rate = self.create_rate(self.rate)
            for _ in range(int(duration)):
                self.cmd_vel_publisher.publish(drift_ang_vel)
                rate.sleep()

            # Stop the effect
            drift_ang_vel2 = Twist()
            self.cmd_vel_publisher.publish(drift_ang_vel2)

        except Exception as e:
            self.get_logger().error("Error in effect_imu_attack: %s", str(e))


    def realistic_imu_attack_model(self, msg):
        #the litearature suggests that the imu spoofing attack could be intermittent
        #and the spoofing offset could be noisy

        # Intermittent attack model
        epsilon = self.attack_fail_probability
        # random number between 0 and 1
        r = random.random()
        if r > epsilon:
            new_msg = self.imu_attack_model(msg)
        else:
            new_msg = msg

        # Noisy attack model
        # msg.angular_velocity.x += random.uniform(-self.spoofing_offset, self.spoofing_offset)
        # msg.angular_velocity.y += random.uniform(-self.spoofing_offset, self.spoofing_offset)
        # msg.angular_velocity.z += random.uniform(-self.spoofing_offset, self.spoofing_offset)
        # new_msg = self.imu_attack_model(msg)
        
        return new_msg
        


    def imu_attack_model(self, msg):
        # Implement the attack model
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.angular_velocity.x = msg.angular_velocity.x + self.spoofing_offset
        new_msg.angular_velocity.y = msg.angular_velocity.y + self.spoofing_offset
        new_msg.angular_velocity.z = msg.angular_velocity.z + self.spoofing_offset
        # Add more modifications as needed
        return new_msg


def main(args=None):
    rclpy.init(args=args)

    try:
        imu_spoofing_node = ImuSpoofingNode()
        rclpy.spin(imu_spoofing_node)
    except Exception as e:
        print("Error during node initialization: ", str(e))
    finally:
        imu_spoofing_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
