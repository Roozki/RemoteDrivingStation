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
#include <std_msgs/msg/string.hpp>
#include <rds_msgs/msg/hud_manage.hpp>
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <mutex>
#include <heads_up_definitions.h>
#include <SFML/Audio.hpp>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>



class HUDOverlayNode : public rclcpp::Node
{
public:
  HUDOverlayNode() : Node("rds_hud_overlay_node")
  {
  }
  struct HUD_Struct
  {
    bool fancyPantsDone = false;
    int systems_online = 0;
    bool authorized = false;
    bool initiated = false;
    bool ready = false;
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
  cv::Mat blackScreen;
  cv::Mat resizedRearView;

  void drawHud();

  void init()
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos.keep_last(1);
    auto sound_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sound_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sound_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    sound_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    sound_qos.keep_last(10);
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
    sound_pubber = this->create_publisher<std_msgs::msg::String>("/speaker/command", qos);
    
    gnss_subber = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 1, std::bind(&HUDOverlayNode::gnssCallback, this, std::placeholders::_1));

   
    cv::namedWindow("RDS_HUD", cv::WINDOW_NORMAL);
    // Set the mouse callback function
    cv::setMouseCallback("RDS_HUD", HUDOverlayNode::onMouse, this);
    ft2 = cv::freetype::createFreeType2(); // for more fonts

  

    ////fancyPantsStartup();
  }
  //!---------------------------------------------------------//
  //!                       FANCY PANTS                       //
  //!---------------------------------------------------------//
  void fancyPantsStartup()
  {
    rclcpp::Rate tempRate(50);
    // TODO swipe open camera feed, add gps start, net speed
    //  cv::Rect rect(0, 0, 1920, 1080);
    blackScreen = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));


    // frame = ;
    int mid_cols = blackScreen.cols / 2;
    int mid_rows = blackScreen.rows / 2;
    cv::putText(blackScreen, "PRESS ANY KEY TO START...", cv::Point(300, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(150, 150, 150), 2);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(0);

    ////cv::waitKey(100);
    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
    cv::imshow("RDS_HUD", blackScreen);
    sendSoundCommand("bleep.wav");
    cv::waitKey(10);
    playMP4();
    // for (int i = 20; i < (blackScreen.cols / 4); i++)
    // {
      //   // cv::putText(frame, ".", cv::Point((frame.cols /2) - 500 + i*35, frame.rows /2), cv::FONT_HERSHEY_SIMPLEX, 7, cv::Scalar(155, 155, 155), 8);
      //   int width = static_cast<int>(pow(i / 100.0, 8)); // Ensure proper casting and division

      //   // Ensure width does not exceed frame.cols
      //   // width = std::min(width, frame.cols);

      //   cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(width, blackScreen.rows), cv::Scalar(155, 155, 155), -1);
      //   cv::putText(blackScreen, "RDS", cv::Point(mid_cols - 470, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 11, cv::Scalar(0, 0, 0), 10);
      //   // cv::waitKey(5);
      //   cv::imshow("RDS_HUD", blackScreen);
      //   cv::waitKey(1);
    // }
    ////cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
    ////cv::putText(blackScreen, "RDS", cv::Point(mid_cols - 470, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 11, cv::Scalar(155, 155, 155), 10);
    //// cv::waitKey(100);
    //// cv::imshow("RDS_HUD", blackScreen);
    ////cv::putText(blackScreen, "drive", cv::Point(mid_cols - 480, mid_rows + 200), cv::FONT_HERSHEY_SIMPLEX, 7, cv::Scalar(15, 15, 255), 6, cv::FONT_ITALIC);
   // drawPicture("rdslogo.png", mid_rows, mid_cols);
    ////cv::imshow("RDS_HUD", blackScreen);
    ////cv::waitKey(1500);
    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
      sendSoundCommand("AI_engine_up.wav");

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
    cv::waitKey(50);
    cv::putText(blackScreen, "gps: ", cv::Point(200, 800), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(105, 105, 105), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(50);
    cv::putText(blackScreen, "car: ", cv::Point(200, 900), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(105, 105, 105), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(50);
    cv::putText(blackScreen, "control: ", cv::Point(200, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(105, 105, 105), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(50);
    std::string network_status_msg = "STANDBY";
    std::string gps_status_msg = "STANDBY";
    std::string car_status_msg = "STANDBY";
    std::string control_status_msg = "STANDBY";
   
    cv::Scalar gps_status_colour = cv::Scalar(150, 150, 0);
    cv::Scalar car_status_colour = cv::Scalar(150, 150, 0);
    cv::Scalar control_status_colour = cv::Scalar(150, 150, 0);
    cv::Scalar network_status_colour = cv::Scalar(150, 150, 0);
    cv::putText(blackScreen, "network: ", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 235), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(100);

    cv::putText(blackScreen, "gps: ", cv::Point(200, 800), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 235), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(100);
    cv::putText(blackScreen, "car: ", cv::Point(200, 900), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 235), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(100);
    cv::putText(blackScreen, "control: ", cv::Point(200, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 235), 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(100);
    for (int i = 0; i < 10; i++)
    {
      
      cv::rectangle(blackScreen, cv::Point(600, 650), cv::Point(1079, 1450), cv::Scalar(0, 0, 0), -1);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(50);
      
      cv::putText(blackScreen, network_status_msg, cv::Point(600, 700), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, gps_status_msg, cv::Point(600, 800), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, car_status_msg, cv::Point(600, 900), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, control_status_msg, cv::Point(600, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(60);
    }
    //! network check
    // cv::waitKey(100);
    if (networkCheck() == NETWORK_OK)
    {
      hud.systems_online++;
      network_status_msg = "ONLINE, LATENCY: " + std::to_string(static_cast<int>(hud.vehicle_latency)) + "ms";
      network_status_colour = cv::Scalar(10, 255, 10);
    }
    else
    {
      network_status_msg = "OFFLINE!!";
      network_status_colour = cv::Scalar(10, 10, 255);
    }


    cv::rectangle(blackScreen, cv::Point(599, 650), cv::Point(900, 750), cv::Scalar(0, 0, 0), -1);

    cv::putText(blackScreen, network_status_msg, cv::Point(500, 700), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(10);
        for (int i = 0; i < 10; i++)
    {
      
      cv::rectangle(blackScreen, cv::Point(600, 750), cv::Point(1080, 1450), cv::Scalar(0, 0, 0), -1);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(50);
      
      //cv::putText(blackScreen, network_status_msg, cv::Point(500, 700), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, gps_status_msg, cv::Point(600, 800), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, car_status_msg, cv::Point(600, 900), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, control_status_msg, cv::Point(600, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(60);
    }
    //! gps check
    
    if (vehicle_1_current_gnss.status.service == GPS_OK)
    {
      hud.systems_online++;
      gps_status_msg = "SATELLITES FIXED";
      gps_status_colour = cv::Scalar(10, 255, 10);
    }
    else
    {
      gps_status_msg = "OFFLINE";
      gps_status_colour = cv::Scalar(10, 10, 255);
    }
    cv::rectangle(blackScreen, cv::Point(599, 750), cv::Point(1080, 850), cv::Scalar(0, 0, 0), -1);
    cv::putText(blackScreen, gps_status_msg, cv::Point(600, 800), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(10);
    for (int i = 0; i < 5; i++)
    {
      
      cv::rectangle(blackScreen, cv::Point(600, 850), cv::Point(1080, 1450), cv::Scalar(0, 0, 0), -1);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(50);
      
      cv::putText(blackScreen, car_status_msg, cv::Point(600, 900), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      cv::putText(blackScreen, control_status_msg, cv::Point(600, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(60);
    }
        //! controls check
    
    if (vehicle_1_current_status.online == true)
    {
      hud.systems_online++; 

      car_status_msg = "CAR CONNECTED";
      car_status_colour = cv::Scalar(10, 255, 10);
    }
    else
    {
      car_status_msg = "OFFLINE";
      car_status_colour = cv::Scalar(10, 10, 255);
    }
    cv::rectangle(blackScreen, cv::Point(599, 850), cv::Point(1080, 950), cv::Scalar(0, 0, 0), -1);

        cv::putText(blackScreen, car_status_msg, cv::Point(600, 900), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(10);
        for (int i = 0; i < 5; i++)
    {
      
      cv::rectangle(blackScreen, cv::Point(600, 950), cv::Point(1080, 1450), cv::Scalar(0, 0, 0), -1);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(50);
      
      cv::putText(blackScreen, control_status_msg, cv::Point(600, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);
      
      
      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(60);
    }
    hud.systems_online++; 
    hud.systems_online++;
    cv::rectangle(blackScreen, cv::Point(599, 950), cv::Point(1080, 1050), cv::Scalar(0, 0, 0), -1);

      control_status_msg = "READY";
      control_status_colour = cv::Scalar(10, 10, 255);
      cv::putText(blackScreen, control_status_msg, cv::Point(600, 1000), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(235, 235, 0), 3);

      cv::imshow("RDS_HUD", blackScreen);
      cv::waitKey(180);




//!final start messages
    cv::rectangle(blackScreen, cv::Point(0, 0), cv::Point(blackScreen.cols, blackScreen.rows), cv::Scalar(0, 0, 0), -1);
    if (hud.systems_online > 3)
    {
      sendSoundCommand("AI_welcome.wav");
      cv::waitKey(10);
      cv::putText(blackScreen, "WELCOME DRIVER", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
    

    }
    else if(OFFLINE_MODE){
      cv::putText(blackScreen, "CLIENT IN OFFLINE MODE", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
      sendSoundCommand("AI_welcome_attention.wav");
    }
    else if (hud.systems_online <=3 && networkCheck() == NETWORK_OK){
      cv::putText(blackScreen, "WELCOME DRIVER", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
      sendSoundCommand("AI_welcome_attention.wav");

    }
    else
    {
      cv::putText(blackScreen, "SYSTEM RESTART REQUIRED", cv::Point(200, 400), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
      sendSoundCommand("AI_system_failure.wav");

    }
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(1200);
    while (!hud.authorized)
    {
      // rclcpp::spin_some(HUDOverlayNode);
      cv::waitKey(50);
      hud.authorized = true;

      // TODO authorize hub
      //! figure out how to spin
    }
    cv::waitKey(400);
    if(hud.systems_online > 3){
      
      cv::putText(blackScreen, "ALL SYSTEMS ONLINE", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);
    }else if(networkCheck() == NETWORK_OK){
      network_status_colour = cv::Scalar(0, 200, 200);
      cv::putText(blackScreen, "SOME SYSTEMS NEED ATTENTION", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);

    }else{
            cv::putText(blackScreen, "ALL SYSTEMS OFFLINE", cv::Point(200, 700), cv::FONT_HERSHEY_SIMPLEX, 2, network_status_colour, 3);

    }
    cv::imshow("RDS_HUD", blackScreen);
    cv::waitKey(2500);
    hud.fancyPantsDone = true;
  }


private:

void sendSoundCommand(std::string file){
  std_msgs::msg::String outmsg;
  outmsg.data = file;
  sound_pubber->publish(outmsg);
}


std::string package_share_directory = ament_index_cpp::get_package_share_directory("rds_hud");
std::string img_path = package_share_directory + "/images/";
std::string mp4_path = package_share_directory + "/videos/";
  // Create a pointer to the FreeType2 library
  cv::Ptr<cv::freetype::FreeType2> ft2;
  int latency_refresh_count = 0;

  //!----------------------------
  //!   DRIVE LINE
  //!------------------------------
  void drawDriveLine(cv::Mat &img, cv::Point start, cv::Point control, cv::Point end, cv::Scalar color, int thickness)
  {
    int steering_thickness = 7;
    int rpm_thickness = 5;
    cv::Scalar steering_colour = cv::Scalar(0,0,0);
    cv::Scalar rpm_colour = cv::Scalar(255, 255, 255);
    // quadratic breazear... chatgpt
    //also draws accell
    const int numDriveLinePoints = 100;
    std::vector<cv::Point> driveLinePoints;
    double rpm_line_percent = (vehicle_1_current_command.gas_pedal * static_cast<double>(numDriveLinePoints));

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
      //drawTransparentLine(frame, driveLinePoints[i - 1], driveLinePoints[i], steering_colour, steering_thickness, 0.4);

      cv::line(img, driveLinePoints[i - 1], driveLinePoints[i], steering_colour, steering_thickness);
      if(i <= rpm_line_percent){
      //drawTransparentLine(frame, driveLinePoints[i - 1], driveLinePoints[i], rpm_colour, rpm_thickness, 0.2);
  
      cv::line(img, driveLinePoints[i - 1], driveLinePoints[i], rpm_colour, rpm_thickness);
      }

    }
  } // thanks chatgpt

  void drawPicture(std::string file, int x, int y){
        // Load the PNG image
        std::string file_path = img_path + file;
    cv::Mat pngImage = cv::imread(file_path, cv::IMREAD_UNCHANGED);

    // Ensure both images are loaded
    if(blackScreen.empty() || frame.empty() || pngImage.empty()) {
        std::cout << "Could not load one of the images!" << std::endl;
        ////return -1;
    }
      // Separate the PNG image into BGR and alpha channels
    std::vector<cv::Mat> pngChannels(4);
    cv::split(pngImage, pngChannels);
    cv::Mat bgr[3] = { pngChannels[0], pngChannels[1], pngChannels[2] };
    cv::Mat colorImage;
    cv::merge(bgr, 3, colorImage); // Merge the BGR channels back to an image
    cv::Mat alphaImage = pngChannels[3]; // Alpha channel

    // Select the region in the main image where the PNG will be placed
    ////int x = 100; // Top-left x-coordinate of where the PNG image is to be placed
    ////int y = 50; // Top-left y-coordinate of where the PNG image is to be placed
    cv::Rect roi(x, y, pngImage.cols, pngImage.rows);

    // Place the PNG image over the main image
    for (int i = 0; i < pngImage.rows; ++i) {
        for (int j = 0; j < pngImage.cols; ++j) {
            if (alphaImage.at<uchar>(i, j) > 0) { // Check if the pixel is not transparent
                if(hud.initiated){
                  frame.at<cv::Vec3b>(y + i, x + j) = colorImage.at<cv::Vec3b>(i, j);
                }else{
                  blackScreen.at<cv::Vec3b>(y + i, x + j) = colorImage.at<cv::Vec3b>(i, j);
                }
            }
        }
    }
  }
  void playMP4(){
       // Path to the MP4 video file
    std::string videoPath = mp4_path + "/startupigen.mp4";

    // Open the video file
    cv::VideoCapture capture(videoPath);
    if (!capture.isOpened()) {
        std::cerr << "Error opening video file!" << std::endl;
    }

    // Get the frames per second (fps) of the video
    double fps = capture.get(cv::CAP_PROP_FPS);
    int delay = 1000 / fps; // Delay between each frame in ms

    //cv::Mat frame;
    while (true) {
        // Read the current frame
        bool success = capture.read(frame);
        if (!success) {
            std::cout << "Reached the end of the video or failed to read a frame." << std::endl;
            break; // Exit the loop if no more frames or if there's an error
        }

        // Display the frame
        cv::imshow("RDS_HUD", frame);

        // Wait for 'delay' ms or until a key is pressed; if 'Esc' is pressed, break out of the loop
        if (cv::waitKey(delay) == 27) {
            break;
        }
    }

    // Release the video capture object
    capture.release();
  }
void drawTransparentLine(cv::Mat& image, cv::Point pt1, cv::Point pt2, cv::Scalar color, int thickness, double alpha) {
    // Create a temporary RGBA image with the same dimensions as the original image
    cv::Mat rgbaImage(image.size(), CV_8UC4, cv::Scalar(0,0,0,0));

    // Draw the line on the RGBA image
    cv::line(rgbaImage, pt1, pt2, cv::Scalar(color[0], color[1], color[2], alpha * 255), thickness);

    // Split the RGBA image into separate channels (including alpha)
    cv::Mat channels[4];
    cv::split(rgbaImage, channels);

    // Create a mask from the alpha channel
    cv::Mat mask = channels[3];

    // Copy the line to the original image using the mask
    rgbaImage.copyTo(image, mask);
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
      last_rear_frame_ = *msg;
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
        hud.ready = true;

        
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
  sensor_msgs::msg::Image last_rear_frame_;


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
  void gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg){
    vehicle_1_current_gnss = *msg;
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

void drawHazardsSign(cv::Mat &image, const cv::Point &position, bool isActive) {
    // Define the rectangle dimensions and properties
    cv::Rect rect(position, cv::Size(180, 50)); // Adjust size as needed
    int thickness = 2; // Thickness of the rectangle border

    // Define colors
    cv::Scalar rectColor(255, 255, 255); // White rectangle
    cv::Scalar textColorOn(255, 255, 255); // Red border color
    cv::Scalar borderColorOff(100, 100, 100); // grey border color
    cv::Scalar textColorOff(128, 128, 128); // Gray text for "off" state
    cv::Scalar borderColorOn(0, 255, 0); // Green text for "on" state

    // Draw the rectangle with border
    cv::rectangle(image, rect, isActive ? borderColorOn : borderColorOff, thickness);
    //cv::rectangle(image, rect, rectColor, cv::FILLED, cv::LINE_8, thickness);

    // Set text properties
    std::string text = "HAZARDS";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.7;
    int textThickness = 2;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, textThickness, &baseline);

    // Calculate text position to center it within the rectangle
    cv::Point textOrg(rect.x + (rect.width - textSize.width) / 2,
                      rect.y + (rect.height + textSize.height) / 2);

    // Draw the text
    cv::putText(image, text, textOrg, fontFace, fontScale, isActive ? textColorOn : textColorOff, textThickness);
}
void drawSignalStatus(cv::Mat &image, const cv::Point &position, bool isLeftActive, bool isRightActive) {
    // Base position for the "SIGNAL" text and L/R indicators
    cv::Point basePosition = position + cv::Point(0, 0); // Adjust the vertical offset as needed

    // Define text properties
    std::string signalText = "SIGNAL";
    std::string leftText = "<";
    std::string rightText = ">";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.7;
    double SigfontScale = 1.3;
    int textThickness = 2;
    int SigtextThickness = 3;

    cv::Scalar activeColor(0, 255, 0); // Green for active
    cv::Scalar inactiveColor(128, 128, 128); // Gray for inactive

    // Draw the "SIGNAL" text
    cv::putText(image, signalText, basePosition, fontFace, fontScale, inactiveColor, textThickness);

    // Calculate offsets for L and R indicators based on text size
    int baseline = 0;
    cv::Size textSizeL = cv::getTextSize(leftText, fontFace, SigfontScale, SigtextThickness, &baseline);
    cv::Size textSizeR = cv::getTextSize(rightText, fontFace, SigfontScale, SigtextThickness, &baseline);

    // Positions for L and R indicators
    cv::Point leftPosition = basePosition + cv::Point(-textSizeL.width + 10, textSizeL.height + 15); // Adjust spacing as needed
    cv::Point rightPosition = basePosition + cv::Point(textSizeR.width + 30, textSizeR.height + 15);

    // Draw L and R indicators
    cv::putText(image, leftText, leftPosition, fontFace, fontScale, isLeftActive ? activeColor : inactiveColor, textThickness);
    cv::putText(image, rightText, rightPosition, fontFace, fontScale, isRightActive ? activeColor : inactiveColor, textThickness);
}



  
  // Display Values
  int status_bar_height = 150;
  // sfloat current_gas_pedal;
  rds_msgs::msg::VehicleInterface vehicle_1_current_command;
  rds_msgs::msg::VehicleStatus vehicle_1_current_status;
  sensor_msgs::msg::NavSatFix vehicle_1_current_gnss;

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sound_pubber;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subber;
  std::mutex image_mutex_;
};