import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

class GPSMarkerPublisher(Node):
    def __init__(self):
        super().__init__('gps_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'gps_marker_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Or your specific frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gps"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0  # Set your GPS coordinate X
        marker.pose.position.y = 0.0  # Set your GPS coordinate Y
        marker.pose.position.z = 0.1  # Slightly above the ground
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0  # Initial size, you can animate this for the "ping"
        marker.scale.y = 1.0
        marker.scale.z = 0.1  # Flat circle
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = Duration(sec=1)  # How long the marker lasts before disappearing

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    gps_marker_publisher = GPSMarkerPublisher()
    rclpy.spin(gps_marker_publisher)
    gps_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
