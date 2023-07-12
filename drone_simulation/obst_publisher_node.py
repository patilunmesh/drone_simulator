import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.obstacle_publisher = self.create_publisher(MarkerArray, 'obstacles', 1)
        self.timer = self.create_timer(1.0, self.timer_callback2)
        
        spheres = [
            {'x': 1.5, 'y': 4.0, 'z': 6.0, 'radius': 1.5},
            {'x': 3.0, 'y': 4.0, 'z': 5.0, 'radius': 0.5},
            {'x': 1.0, 'y': 6.0, 'z': 8.0, 'radius': 1.8},
            {'x': 0.0, 'y': 0.0, 'z': 6.0, 'radius': 1.9}
        ]
        self.markerarr = MarkerArray()
        index = 0
        for sphere in spheres:
            marker = Marker()
            marker.ns = 'obstacles'
            marker.id = index
            marker.header.frame_id = 'drone'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = sphere['x']
            marker.pose.position.y = sphere['y']
            marker.pose.position.z = sphere['z']
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * sphere['radius']
            marker.scale.y = 2 * sphere['radius']
            marker.scale.z = 2 * sphere['radius']
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            self.markerarr.markers.append(marker)
            index += 1



    def timer_callback2(self):
        self.obstacle_publisher.publish(self.markerarr)  
    

def main(args=None):
    rclpy.init(args=args)
    obstacle_publisher = ObstaclePublisher()
    rclpy.spin(obstacle_publisher)
    obstacle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
