#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rds_msgs/msg/vehicle_interface.hpp>
#include <sstream>

class HUDOverlayNode : public rclcpp::Node {
public:
  HUDOverlayNode() : Node("rds_hud_overlay_node"){

  }

    void init(){
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(1);
            // Set transport hints for compression
    image_transport::TransportHints hints(this, "compressed");

        image_transport::ImageTransport it(shared_from_this());
        hud_sub_ = it.subscribe("/camera/image_raw", 1, &HUDOverlayNode::imageCallback, this, &hints);
        hud_pub_ = it.advertise("/hud_overalay", 1);
        RCLCPP_INFO(this->get_logger(), "meow");
        hw_control_subscriber_ = this->create_subscription<rds_msgs::msg::VehicleInterface>(
        "/vehicle_1/command", 4, std::bind(&HUDOverlayNode::commandCallback, this, std::placeholders::_1));
        cv::namedWindow("RDS_HUD", cv::WINDOW_NORMAL);

    }
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Rect rect(0, frame.rows - status_bar_height, frame.cols, status_bar_height);

      cv::Mat roi = frame(rect);
      //frame.copyTo(rect);
      // Draw your HUD on the frame
      // Example: draw a simple line
      cv::Mat overlay = roi.clone();
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << vehicle_1_current_command.gas_pedal; // Set precision to 2 decimal places
      std::string gas_pedal_string = oss.str();
      cv::rectangle(overlay, cv::Point(0, 0), cv::Point(frame.cols, frame.rows - status_bar_height), cv::Scalar(0, 0, 0), -1); 
      cv::addWeighted(overlay, 0.4, roi, 1 - 0.4, 0, roi);      



      cv::putText(frame, "GAS", cv::Point(20, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255,255,255), 3);
      cv::line(frame, cv::Point(0, frame.rows - status_bar_height), cv::Point(frame.cols, frame.rows - status_bar_height), CV_RGB(0, 0, 0), 4);
      cv::putText(frame, gas_pedal_string, cv::Point(300, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 255, 0), 2);

      
      cv::imshow("RDS_HUD", frame);
      cv::waitKey(1);
      // Convert back to ROS message and publish
      sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      hud_pub_.publish(msg_out);
       // RCLCPP_INFO(this->get_logger(), "meow");

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  
  }
  }

  void commandCallback(const rds_msgs::msg::VehicleInterface::ConstSharedPtr& msg){
     vehicle_1_current_command =* msg;
  }
  int status_bar_height = 150;
  float gas_pedal;
  rds_msgs::msg::VehicleInterface vehicle_1_current_command;
  
  image_transport::Subscriber hud_sub_;
  image_transport::Publisher hud_pub_;
  rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr hw_control_subscriber_;
  
  };

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HUDOverlayNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
