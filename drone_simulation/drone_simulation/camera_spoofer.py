import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
#this node is not tested yet
class CameraSpoofingNode(Node):
    def __init__(self, spoofed_image_path):
        super().__init__('camera_spoofing_node')

        # Create a publisher to publish the spoofed camera image
        self.spoofed_publisher = self.create_publisher(Image, 'drone/camera/spoofed_image', 10)

        # Create a subscriber to listen to the original camera image
        self.subscription = self.create_subscription(
            Image,
            'drone/camera/image_raw',
            self.camera_callback,
            10
        )

        # Parameters
        self.spoofed_image_path = spoofed_image_path
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        try:
            # Replace the camera image with the spoofed image
            spoofed_image = self.load_spoofed_image()
            spoofed_msg = self.bridge.cv2_to_imgmsg(spoofed_image, encoding="bgr8")
            spoofed_msg.header = msg.header

            # Publish the spoofed camera image
            self.spoofed_publisher.publish(spoofed_msg)
            self.get_logger().info("Spoofed camera image published")

        except Exception as e:
            self.get_logger().error("Error in camera callback: %s", str(e))

    def load_spoofed_image(self):
        # Load the user-specified spoofed image
        spoofed_image = cv2.imread(self.spoofed_image_path)

        if spoofed_image is None:
            raise FileNotFoundError(f"Specified image '{self.spoofed_image_path}' not found.")

        return spoofed_image


def main(args=None):
    rclpy.init(args=args)

    try:
        # Specify the path to the image you want to use for spoofing
        spoofed_image_path = '/path/to/your/spoofed/image.jpg'
        camera_spoofing_node = CameraSpoofingNode(spoofed_image_path)
        rclpy.spin(camera_spoofing_node)
    except Exception as e:
        print("Error during node initialization: ", str(e))
    finally:
        camera_spoofing_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
