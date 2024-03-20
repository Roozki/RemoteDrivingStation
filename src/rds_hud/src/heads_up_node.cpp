#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rds_msgs/msg/vehicle_interface.hpp>
#include <rds_msgs/msg/vehicle_status.hpp>
#include <rds_msgs/msg/hud_manage.hpp>
#include <sstream>

#define GEAR_REVERSE -2
#define GEAR_PARK -1
#define GEAR_NEUTRAL 0
#define GEAR_DRIVE 1
#define NET_ERR -15
#define NUM_NUMS 10 //num nums

#define _W 1080 //width
#define _H 1920 //height

#define PI 3.14159265359

class HUDOverlayNode : public rclcpp::Node {
public:
  HUDOverlayNode() : Node("rds_hud_overlay_node"){

  }
    cv::Mat frame;

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
        vehicle_1_control_subscriber_ = this->create_subscription<rds_msgs::msg::VehicleInterface>(
        "/vehicle_1/command", 4, std::bind(&HUDOverlayNode::commandCallback, this, std::placeholders::_1));
        vehicle_1_status_subscriber_ = this->create_subscription<rds_msgs::msg::VehicleStatus>(
          "/vehicle_1/status", 4, std::bind(&HUDOverlayNode::statusCallback, this, std::placeholders::_1));
        cv::namedWindow("RDS_HUD", cv::WINDOW_NORMAL);
    // Set the mouse callback function
    cv::setMouseCallback("RDS_HUD", HUDOverlayNode::onMouse, this);

    }
private:

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Rect rect(0, frame.rows - status_bar_height, frame.cols, status_bar_height);

      cv::Mat roi = frame(rect);
      //frame.copyTo(rect);
      // Draw your HUD on the frame
      // Example: draw a simple line
      cv::Mat overlay = roi.clone();
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(0) << vehicle_1_current_command.gas_pedal; // Set precision to 0 decimal places
      std::string gas_pedal_string = oss.str();
      cv::rectangle(overlay, cv::Point(0, 0), cv::Point(frame.cols, frame.rows - status_bar_height), cv::Scalar(0, 0, 0), -1); 
      cv::addWeighted(overlay, 0.4, roi, 1 - 0.4, 0, roi);      
      
      switch (vehicle_1_current_command.gear)
      {
      case GEAR_REVERSE:
        current_gear = "R"; //reverse
        break;
      case GEAR_PARK:
        current_gear = "P"; //Park
        break;
      case GEAR_NEUTRAL:
        current_gear = "N";
        break;
      case GEAR_DRIVE:
      if(vehicle_1_current_command.manual){ //TODO SWiTCH TO STATUS
        current_gear = "1";
      } else{
        current_gear = "D";
      }
        break;
      case NET_ERR: //could just change to converting gear number to 
        current_gear = "NET ERR";
        break;
      default:
        current_gear = std::to_string(vehicle_1_current_command.gear);
        break;
      }
      //TODO add signals, hazards
      if (msg->header.stamp.nanosec > 500000000){ //blinking
      vehicle_1_current_command.lights.resize(5);

    if (vehicle_1_current_command.lights[0] == 1){
        //left signal
    cv::Point leftArrowStart(frame.cols * 0.05, frame.rows / 2);
    cv::Point leftArrowEnd(frame.cols * 0.10, frame.rows / 2);


    
    cv::arrowedLine(frame, leftArrowEnd, leftArrowStart, cv::Scalar(20, 255, 20), 2, 3, 0, 2);
      }

      if (vehicle_1_current_command.lights[1] == 1){

    cv::Point rightArrowStart(frame.cols * 0.90, frame.rows / 2);
    cv::Point rightArrowEnd(frame.cols * 0.95, frame.rows / 2);
     cv::arrowedLine(frame, rightArrowStart, rightArrowEnd, cv::Scalar(20, 255, 20), 2, 3, 0, 2);

        }

      }
      //cv::putText(frame, "GEAR", cv::Point(10, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 3);
      cv::putText(frame, current_gear, cv::Point(190, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,255), 3);
      cv::line(frame, cv::Point(0, frame.rows - status_bar_height), cv::Point(frame.cols, frame.rows - status_bar_height), CV_RGB(0, 0, 0), 4);
      cv::putText(frame, gas_pedal_string + "km/h", cv::Point(890, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);
     // cv::putText(frame, , cv::Point(950, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,255), 3);
      int spedometer_radius = 300;
      cv::Point spedometerLineStart(frame.cols/2, frame.rows/2);
      cv::Point spedometerLineEnd((frame.cols/2) - spedometer_radius, frame.rows/2);
      //@TODO Need to add progress bar for gas
      //add kmph
      //add gear
        int sped_increment = 10;//40/NUM_NUMS;
        int max_sped = sped_increment*NUM_NUMS;
      for(int i = 0; i < NUM_NUMS; i++){
        float spedometer_draw_angle = (i*sped_increment * PI) / max_sped;
      std::string sped_num = std::to_string(i*sped_increment); //+"km/h";
      cv::putText(frame, sped_num, cv::Point(((spedometer_radius)*(-1)*cos(spedometer_draw_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius)*(-1)*sin(spedometer_draw_angle)) + spedometerLineEnd.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1);

      }
      float spedometer_angle = (vehicle_1_current_command.gas_pedal * PI) / max_sped;

      spedometerLineEnd.x = ((spedometer_radius)*(-1)*cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius;
      spedometerLineEnd.y = ((spedometer_radius)*(-1)*sin(spedometer_angle)) + spedometerLineEnd.y;

      cv::line(frame, spedometerLineStart, spedometerLineEnd, cv::Scalar(255, 0, 0), 3);
    

      //DEBUG - show mouse position (helps to know where to put stuff)
      cv::putText(frame, mousePointText, cv::Point(mouseX + 10, mouseY + 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
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
       //  vehicle_1_current_command.lights.resize(5);

  }
  void statusCallback(const rds_msgs::msg::VehicleStatus::ConstSharedPtr& msg){
     vehicle_1_current_status =* msg;
  }
  void hudManagmentCallback(const rds_msgs::msg::HudManage::ConstSharedPtr& msg){
    status = msg->status;

  }
  // Callback function for mouse events
static void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_MOUSEMOVE) {
            HUDOverlayNode* HudClassInstance = static_cast<HUDOverlayNode*>(userdata); // Cast userdata back to MyClass*
            HudClassInstance->refreshMousePosition(x, y); // Use the instance to call a non-static member function
       
        // Copy the image to avoid drawing over the original one
        // Prepare the text to display

        // Put the text on the image copy

    }
}
void refreshMousePosition(int x, int y){
       mousePointText = "X: " + std::to_string(x) + ", Y: " + std::to_string(y);

        // Choose a point to position the text, ensuring it's within the window bounds
       mouseX = std::min(x, _W - 200); // Adjust 200 based on the text width
        mouseY = std::min(y, _H - 20);  // Adjust 20 based on the text height

}
  //Display Values
  int status_bar_height = 150;
  //sfloat current_gas_pedal;
  rds_msgs::msg::VehicleInterface vehicle_1_current_command;
  rds_msgs::msg::VehicleStatus vehicle_1_current_status;
  std::string current_gear = "NET_ERR";

  //Vehicle type (manual or Automatic)
  std::string manual_or_automatic;

  //Managment
  int status;


  //debug, mouse stuff
  std::string mousePointText;
  int mouseX;
  int mouseY;

  
  image_transport::Subscriber hud_sub_;
  image_transport::Publisher hud_pub_;
  rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr vehicle_1_control_subscriber_;
  rclcpp::Subscription<rds_msgs::msg::VehicleStatus>::SharedPtr vehicle_1_status_subscriber_;
  
  };

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HUDOverlayNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
