#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class HUDOverlayNode : public rclcpp::Node {
public:
  HUDOverlayNode() : Node("hud_overlay_node"){

  }

    void init(){
        image_transport::ImageTransport it(shared_from_this());
        sub_ = it.subscribe("/camera/image_raw/compressed", 1, &HUDOverlayNode::imageCallback, this);
        pub_ = it.advertise("/hud_overalay", 1);
    }
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

      // Draw your HUD on the frame
      // Example: draw a simple line
      cv::line(frame, cv::Point(10, 10), cv::Point(100, 100), CV_RGB(255, 0, 0), 2);
    
      // Convert back to ROS message and publish
      sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      pub_.publish(msg_out);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HUDOverlayNode>();
  node->init();
  rclcpp::spin(std::make_shared<HUDOverlayNode>());
  rclcpp::shutdown();
  return 0;
}
