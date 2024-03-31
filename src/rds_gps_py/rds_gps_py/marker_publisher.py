import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
from builtin_interfaces.msg import Duration
import numpy as np  # Make sure to install numpy if you haven't

class GPSMarkerPublisher(Node):
    def __init__(self):
        super().__init__('gps_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'gps_marker_topic', 10)
        # Subscribe to the /fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.navsatfix_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_covariance = 1.0  # Default covariance if not provided

    def navsatfix_callback(self, msg):
        # Use the position covariance to determine the marker size
        # Here, we simply take the average of the diagonal elements in the covariance matrix (variances)
        # Covariance matrix is represented as a flat array in row-major order, first 3 elements are the diagonal
        covariance_lat = msg.position_covariance[0]
        covariance_long = msg.position_covariance[3]
        self.last_covariance = (covariance_long+covariance_lat)*4

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust to your coordinate frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gps"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0  # Update with your logic if needed
        marker.pose.position.y = 0.0  # Update with your logic if needed
        marker.pose.position.z = 0.1  # Slightly above the ground or sea level
        marker.pose.orientation.w = 1.0
        scale = self.last_covariance  # Scale the marker based on covariance
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = 0.1  # Keep a fixed height for the marker
        marker.color.a = 0.5  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = Duration(sec=1)  # How long the marker lasts before disappearing

        self.publisher_.publish(marker)

    def on_timer(self):
        # Call the publish marker function periodically
        self.publish_marker()

def main(args=None):
    rclpy.init(args=args)
    gps_marker_publisher = GPSMarkerPublisher()
    # Create a timer to call the publish_marker function, adjust the period to suit your needs
    timer_period = 1  # seconds
    gps_marker_publisher.create_timer(timer_period, gps_marker_publisher.on_timer)
    rclpy.spin(gps_marker_publisher)
    gps_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()