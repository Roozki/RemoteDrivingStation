#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/freetype.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rds_msgs/msg/vehicle_interface.hpp>
#include <rds_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/header.hpp>
#include <rds_msgs/msg/hud_manage.hpp>
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <mutex>
#include <heads_up_definitions.h>
#include <SFML/Audio.hpp>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>



class HUDOverlayNode : public rclcpp::Node
{
public:
  HUDOverlayNode() : Node("rds_hud_overlay_node")
  {
  }
  struct HUD_Struct
  {
    bool authorized = false;
    bool initiated = false;
    int state = 0;
    int init_stage = 0;
    int init_i = 0;
    int32_t init_sec;
    uint32_t curr_nanosec;
    int blank = 10;
    float vehicle_latency = NETWORK_ERROR;
  };
  HUD_Struct hud;
  cv::Mat frame;
  cv::Mat rear_frame;
  void drawHud();

  void init()
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(1);
    // Set transport hints for compression
    image_transport::TransportHints hints(this, "compressed");

    image_transport::ImageTransport it(shared_from_this());
    //    image_transport::ImageTransport it(shared_from_this());
    //! ----------------------------------------//
    //!                 TOPICS                  //
    //! ----------------------------------------//

    hud_sub_ = it.subscribe("/vehicle_1/main_feed/image_raw", 1, &HUDOverlayNode::imageCallback, this, &hints);
    hud_pub_ = it.advertise("/hud_overalay", 1);
    rearview_sub = it.subscribe("/vehicle_1/rear_feed/image_raw", 1, &HUDOverlayNode::rearImageCallback, this, &hints);
    RCLCPP_INFO(this->get_logger(), "meow");
    vehicle_1_control_subscriber_ = this->create_subscription<rds_msgs::msg::VehicleInterface>(
        "/vehicle_1/command", 4, std::bind(&HUDOverlayNode::commandCallback, this, std::placeholders::_1));
    vehicle_1_status_subscriber_ = this->create_subscription<rds_msgs::msg::VehicleStatus>(
        "/vehicle_1/status", 4, std::bind(&HUDOverlayNode::statusCallback, this, std::placeholders::_1));
    cv::namedWindow("RDS_HUD", cv::WINDOW_NORMAL);
    // Set the mouse callback function
    cv::setMouseCallback("RDS_HUD", HUDOverlayNode::onMouse, this);
    ft2 = cv::freetype::createFreeType2(); // for more fonts

  

    //fancyPantsStartup();
    playSound("AI_engine_up.wav");
  }
  void playSound(const std::string& soundFileName) {
        sf::SoundBuffer buffer;
        if (!buffer.loadFromFile(sound_path + soundFileName)) {
            std::cerr << "Failed to load sound file: " << soundFileName << std::endl;
            return;
        }else {
            std::cerr << "Load Success: " << soundFileName << std::endl;
          
        }


        speaker.setBuffer(buffer);
        speaker.setVolume(50);
        speaker.play();

        sf::sleep(sf::milliseconds(1000));
       
        // You might want to wait for the sound to finish or manage sounds differently
        // depending on your application's needs
    }

private:
        sf::Sound speaker;
  sf::SoundBuffer sfxBuffer;
std::string package_share_directory = ament_index_cpp::get_package_share_directory("rds_hud");
std::string sound_path = package_share_directory + "/sounds/";

  // Create a pointer to the FreeType2 library
  cv::Ptr<cv::freetype::FreeType2> ft2;
  int latency_refresh_count = 0;

  //!----------------------------
  //!   DRIVE LINE
  //!------------------------------
  void drawDriveLine(cv::Mat &img, cv::Point start, cv::Point control, cv::Point end, cv::Scalar color, int thickness)
  {
    int steering_thickness = 5;
    int rpm_thickness = 3;
    cv::Scalar steering_colour = cv::Scalar(0,0,0);
    cv::Scalar rpm_colour = cv::Scalar(255, 255, 255);
    // quadratic breazear... chatgpt
    //also draws accell
    const int numDriveLinePoints = 100;
    std::vector<cv::Point> driveLinePoints;
    double rpm_line_percent = ((vehicle_1_current_command.gas_pedal *(VEHICLE_MAX_THROTTLE)) / static_cast<double>(numDriveLinePoints));

    for (int i = 0; i <= numDriveLinePoints; i++)
    {
      double t = i / (1.0 * static_cast<double>(numDriveLinePoints));
      double a = pow((1.0 - t), 2.0);
      double b = 2.0 * t * (1.0 - t);
      double c = pow(t, 2.0);



      cv::Point pt;
      pt.x = static_cast<int>(a * start.x + b * control.x + c * end.x);
      pt.y = static_cast<int>(a * start.y + b * control.y + c * end.y);
      driveLinePoints.push_back(pt);
    }

    for (size_t i = 1; i < driveLinePoints.size(); i++)
    {
      
      cv::line(img, driveLinePoints[i - 1], driveLinePoints[i], color, steering_thickness);
     // if(i <= rpm_line_percent){
      cv::line(img, driveLinePoints[i - 1], driveLinePoints[i], color, rpm_thickness);
      //}

    }
  } // thanks chatgpt


  void fancyPantsStartup()
  {
    // TODO swipe open camera feed, add gps start, net speed
    //  cv::Rect rect(0, 0, 1920, 1080);
    cv::Mat blackScreen(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));

    // frame = ;
    int mid_cols = blackScreen.cols / 2;
    int mid_rows = blackScreen.rows / 2;
    cv::putText(blackScreen, "PRESS ANY KEY TO START...", cv::Point(300, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(150, 150, 150), 2);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(0);
    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);

    for (int i = 20; i < (blackScreen.cols / 4); i++)
    {
      // cv::putText(frame, ".", cv::Point((frame.cols /2) - 500 + i*35, frame.rows /2), cv::FONT_HERSHEY_SIMPLEX, 7, cv::Scalar(155, 155, 155), 8);
      int width = static_cast<int>(pow(i / 100.0, 8)); // Ensure proper casting and division

      // Ensure width does not exceed frame.cols
      // width = std::min(width, frame.cols);

      cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(width, blackScreen.rows), cv::Scalar(155, 155, 155), -1);
      cv::putText(blackScreen, "RDS", cv::Point(mid_cols - 470, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 11, cv::Scalar(0, 0, 0), 10);
      // cv::waitKey(5);
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(1);
    }
    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
    cv::putText(blackScreen, "RDS", cv::Point(mid_cols - 470, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 11, cv::Scalar(155, 155, 155), 10);
    //// cv::waitKey(100);
    //// cv::imshow("RDS_HUD", blackScreen);
    cv::putText(blackScreen, "drive", cv::Point(mid_cols - 480, mid_rows + 200), cv::FONT_HERSHEY_SIMPLEX, 7, cv::Scalar(15, 15, 255), 6, cv::FONT_ITALIC);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(1500);
    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);

    for (int i = 40; i < (blackScreen.cols / 4); i++)
    {
      int _progress = static_cast<int>(pow(i / 100.0, 10));
      cv::rectangle(blackScreen, cv::Point(0, 310), cv::Point(_progress, 450), cv::Scalar(200, 200, 200), -1);
      cv::putText(blackScreen, "HEADS UP DISPLAY STARTUP", cv::Point(200, 420), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 0, 0), 3, 16);
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(1);
    }
    // cv::rectangle(blackScreen, cv::Point(0,0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
    cv::putText(blackScreen, "network: ", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(105, 105, 105), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(500);
    std::string network_status_msg = "STANDBY";
    cv::Scalar network_status_colour = cv::Scalar(150, 150, 0);
    cv::putText(blackScreen, "network: ", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 235), 3);

    cv::imshow("RDS_HUD", blackScreen);
    for (int i = 0; i < 20; i++)
    {

      cv::rectangle(blackScreen, cv::Point(499, 650), cv::Point(900, 750), cv::Scalar(0, 0, 0), -1);
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(70);
      cv::putText(blackScreen, network_status_msg, cv::Point(500, 700), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(80);
    }
    //! network check
    // cv::waitKey(100);
    if (networkCheck() == NETWORK_OK)
    {
      network_status_msg = "ONLINE, LATENCY: " + std::to_string(static_cast<int>(hud.vehicle_latency)) + "ms";
      network_status_colour = cv::Scalar(10, 255, 10);
    }
    else
    {
      network_status_msg = "OFFLINE!!";
      network_status_colour = cv::Scalar(10, 10, 255);
    }

    cv::rectangle(blackScreen, cv::Point(499, 650), cv::Point(900, 750), cv::Scalar(0, 0, 0), -1);

    cv::putText(blackScreen, network_status_msg, cv::Point(500, 700), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(1000);

    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
    if (networkCheck() == NETWORK_OK)
    {
      cv::putText(blackScreen, "CLIENT READY, REQUESTING AUTHORIZATION...", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
    }
    else if(OFFLINE_MODE){
      cv::putText(blackScreen, "CLIENT IN OFFLINE MODE", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    }
    else
    {
      cv::putText(blackScreen, "CLIENT FAILURE... womp womp", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    }
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(1000);
    while (!hud.authorized)
    {
      // rclcpp::spin_some(HUDOverlayNode);
      cv::waitKey(50);
      hud.authorized = true;
      // TODO authorize hub
      //! figure out how to spin
    }
    cv::putText(blackScreen, "CLIENT AUTHORIZED", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(1000);
  }

  int networkCheck()
  {
    std::string ping_string;

    char _buffer[128];
    FILE *_cmd_out_pipe = popen("ping -c 1 8.8.8.8 | grep 'time=' | awk '{print $7}' | cut -d'=' -f2", "r"); // bash is OP
    if (fgets(_buffer, 128, _cmd_out_pipe) != NULL)
    {
      ping_string += _buffer;
      size_t _endpos = ping_string.find_last_not_of("\t\r\n");
      ping_string = ping_string.substr(0, _endpos + 1);
      hud.vehicle_latency = std::stoi(ping_string);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Network Error!!");
      return NETWORK_ERROR;
    }
    if (!hud.initiated)
    {
      std::string output = "Network Test Success! latency: " + ping_string;
      RCLCPP_WARN(this->get_logger(), output.c_str());
    }

    return NETWORK_OK;
  }
  void rearImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (hud.initiated)
    {
      rear_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
  }
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
        std::unique_lock<std::mutex> lock(image_mutex_);

    last_frame = msg->header;
    hud.authorized = true;
    if (hud.authorized)
    {
     try
     {
        last_frame_ = *msg; // Assume copy assignment is defined
        //last_frame
        hud.initiated = true;

        
      }
     catch (cv_bridge::Exception &e)
     {
       RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }

    lock.unlock();
  }
  std_msgs::msg::Header last_frame;
  sensor_msgs::msg::Image last_frame_;


  // void blackout(cv::mat frame, int ms_delay){

  //     cv::rectangle(frame, cv::Point(0,0), cv::Point(frame.cols, frame.rows), cv::Scalar(0, 0, 0), -1);
  //     cv::imshow("RDS_HUD", frame);
  //   cv::waitKey(ms_delay);

  // }
  void commandCallback(const rds_msgs::msg::VehicleInterface::ConstSharedPtr &msg)
  {
    vehicle_1_current_command = *msg;
    //  vehicle_1_current_command.lights.resize(5);
  }
  void statusCallback(const rds_msgs::msg::VehicleStatus::ConstSharedPtr &msg)
  {
    vehicle_1_current_status = *msg;
  }
  void hudManagmentCallback(const rds_msgs::msg::HudManage::ConstSharedPtr &msg)
  {
    status = msg->status;
  }
  // Callback function for mouse events
  static void onMouse(int event, int x, int y, int flags, void *userdata)
  {
    if (event == cv::EVENT_MOUSEMOVE)
    {
      HUDOverlayNode *HudClassInstance = static_cast<HUDOverlayNode *>(userdata); // Cast userdata back to MyClass*
      HudClassInstance->refreshMousePosition(x, y);                               // Use the instance to call a non-static member function

      // Copy the image to avoid drawing over the original one
      // Prepare the text to display

      // Put the text on the image copy
    }
  }
  void refreshMousePosition(int x, int y)
  {
    mousePointText = "X: " + std::to_string(x) + ", Y: " + std::to_string(y);

    // Choose a point to position the text, ensuring it's within the window bounds
    mouseX = std::min(x, _W - 200); // Adjust 200 based on the text width
    mouseY = std::min(y, _H - 20);  // Adjust 20 based on the text height
  }
  
  // Display Values
  int status_bar_height = 150;
  // sfloat current_gas_pedal;
  rds_msgs::msg::VehicleInterface vehicle_1_current_command;
  rds_msgs::msg::VehicleStatus vehicle_1_current_status;

  std::string current_gear = "NET_ERR";

  // Vehicle type (manual or Automatic)
  std::string manual_or_automatic;

  // Managment
  int status;

  // debug, mouse stuff
  std::string mousePointText;
  int mouseX;
  int mouseY;

  image_transport::Subscriber hud_sub_;
  image_transport::Subscriber rearview_sub;
  image_transport::Publisher hud_pub_;
  rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr vehicle_1_control_subscriber_;
  rclcpp::Subscription<rds_msgs::msg::VehicleStatus>::SharedPtr vehicle_1_status_subscriber_;

  std::mutex image_mutex_;
};