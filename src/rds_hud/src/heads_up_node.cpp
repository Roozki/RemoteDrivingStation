#include <heads_up_node.h>
#include <heads_up_definitions.h>


void HUDOverlayNode::drawHud(){
        std::unique_lock<std::mutex> lock(image_mutex_);

        frame = cv_bridge::toCvCopy(last_frame_, "bgr8")->image;
        int mid_cols = frame.cols / 2;
        int mid_rows = frame.rows / 2;

        cv::Scalar gear_colour(255, 255, 255);

        if (hud.initiated)
        {
          latency_refresh_count++;
          if (latency_refresh_count > 30)
          {
            latency_refresh_count = 0;
            networkCheck();
          }
          //! ----------------------------------------//
          //!               DRIVE LINE                //
          //! ----------------------------------------//

          cv::Point driveLineStart(frame.cols / 2, frame.rows - 100); // Start at the bottom center
          cv::Point driveLineControl(frame.cols / 2, 3* frame.rows / 4.5);
          
     
          double driveLineRadians = (vehicle_1_current_command.steering_angle*70.0) * CV_PI / 180.0;
          int driveLineEndOffset = std::tan(driveLineRadians)*100;
          // Draw the curve
          cv::Point driveLineEnd(driveLineStart.x + driveLineEndOffset*2, frame.rows/3 + (abs(driveLineEndOffset)));            // End at the top center

          drawDriveLine(frame, driveLineStart, driveLineControl, driveLineEnd, cv::Scalar(0, 0, 0), 2);

          std::string latencyString = std::to_string(static_cast<int>(hud.vehicle_latency));
          cv::Rect rect(0, frame.rows - status_bar_height, frame.cols, status_bar_height);

          cv::Mat roi = frame(rect);
          // frame.copyTo(rect);
          //  Draw your HUD on the frame
          //  Example: draw a simple line
          cv::Mat overlay = roi.clone();
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(0) << vehicle_1_current_command.gas_pedal; // Set precision to 0 decimal places
          std::string gas_pedal_string = oss.str();
          cv::rectangle(overlay, cv::Point(0, 0), cv::Point(frame.cols, frame.rows - status_bar_height), cv::Scalar(0, 0, 0), -1);
          cv::addWeighted(overlay, 0.4, roi, 1 - 0.4, 0, roi);
          switch (vehicle_1_current_command.gear)
          {
          case GEAR_REVERSE:
            current_gear = "R"; // reverse
            gear_colour = cv::Scalar(10, 10, 200);
            break;
          case GEAR_PARK:
            current_gear = "P"; // Park
            gear_colour = cv::Scalar(10, 200, 200);
            break;
          case GEAR_NEUTRAL:
            current_gear = "N";
            gear_colour = cv::Scalar(200, 200, 200);

            break;
          case GEAR_DRIVE:
            if (vehicle_1_current_command.manual)
            { // TODO SWiTCH TO STATUS
              current_gear = "1";
            }
            else
            {
              current_gear = "D";
              gear_colour = cv::Scalar(20, 200, 20);
            }
            break;
          case NET_ERR: // could just change to converting gear number to
            current_gear = "NET ERR";
            break;
          default:
            current_gear = std::to_string(vehicle_1_current_command.gear);
            break;
          }
          // TODO add hazards

          //! ----------------------------------------//
          //!                 SIGNIALS                //
          //! ----------------------------------------//
          vehicle_1_current_command.lights.resize(5);

          if (vehicle_1_current_command.left_signal == 1)
          {
            // #left signal

            cv::Point leftArrowStart(frame.cols * 0.055, frame.rows / 2);
            cv::Point leftArrowEnd(frame.cols * 0.05, frame.rows / 2);
            if (last_frame.stamp.nanosec > 300000000)
            { // blinking
              if (last_frame.stamp.nanosec > 600000000)
              { // wtf code am i writing
                ////cv::arrowedLine(frame, rightArrowEnd, rightArrowStart, cv::Scalar(20, 255, 20), 2, 3, 0, 2);
                // do nothing
              }
              else
              {
                cv::arrowedLine(frame, leftArrowStart, leftArrowEnd, cv::Scalar(20, 205, 20), 10, 3, 0, 20);
              }
            }
            else
            {
              cv::arrowedLine(frame, leftArrowStart, leftArrowEnd, cv::Scalar(255, 255, 255), 10, 3, 0, 20);
            }
          }
          else if (vehicle_1_current_command.right_signal == 1)
          {
            cv::Point rightArrowStart(frame.cols * 0.945, frame.rows / 2);
            cv::Point rightArrowEnd(frame.cols * 0.95, frame.rows / 2);
            if (last_frame.stamp.nanosec > 300000000)
            { // blinking
              if (last_frame.stamp.nanosec > 600000000)
              { // wtf code am i writing
                ////cv::arrowedLine(frame, rightArrowEnd, rightArrowStart, cv::Scalar(20, 255, 20), 2, 3, 0, 2);
                // do nothing
              }
              else
              {
                cv::arrowedLine(frame, rightArrowStart, rightArrowEnd, cv::Scalar(20, 205, 20), 10, 8, 0, 20);
              }
            }
            else
            {

              cv::arrowedLine(frame, rightArrowStart, rightArrowEnd, cv::Scalar(255, 255, 255), 10, 8, 0, 20);
            }
          }
          //! ----------------------------------------//
          //!                 GEARS                   //
          //! ----------------------------------------//
          // cv::putText(frame, "GEAR", cv::Point(10, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 3);

          cv::putText(frame, latencyString + "ms", cv::Point(frame.cols - 200, frame.rows - status_bar_height / 10), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(200, 200, 0), 3);

          cv::putText(frame, current_gear, cv::Point(190, frame.rows - status_bar_height / 10), cv::FONT_HERSHEY_SIMPLEX, 5, gear_colour, 3);
          cv::line(frame, cv::Point(0, frame.rows - status_bar_height), cv::Point(frame.cols, frame.rows - status_bar_height), CV_RGB(0, 0, 0), 4);
          cv::putText(frame, gas_pedal_string + "km/h", cv::Point(900, 1040), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2);
          // cv::putText(frame, , cv::Point(950, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,255), 3);
          int spedometer_radius = 100;
          cv::Point spedometerLineStart(frame.cols / 2, frame.rows - 20);
          cv::Point spedometerLineEnd((frame.cols / 2) - spedometer_radius, frame.rows - 20);
          //@TODO Need to add progress bar for gas
          // add kmph
          // add gear
          int sped_increment = 10; // 40/NUM_NUMS;
          int max_sped = sped_increment * NUM_NUMS;
          float spedometer_angle = ((0) * sped_increment * PI) / max_sped;
          cv::putText(frame, "0", cv::Point(((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius) * (-1) * sin(spedometer_angle)) + spedometerLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          std::string sped_num;
          for (int i = 1; i < NUM_NUMS; i++)
          {
            sped_num = std::to_string(i * sped_increment); //+"km/h";
            spedometer_angle = ((i)*sped_increment * PI) / max_sped;
            cv::putText(frame, sped_num, cv::Point(((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius) * (-1) * sin(spedometer_angle)) + spedometerLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          }
          sped_num = "100"; //+"km/h";

          cv::putText(frame, sped_num, cv::Point(((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius) * (-1) * sin(PI)) + spedometerLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);

          spedometer_angle = (vehicle_1_current_command.gas_pedal * PI) / max_sped;

          spedometerLineEnd.x = ((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius;
          spedometerLineEnd.y = ((spedometer_radius) * (-1) * sin(spedometer_angle)) + spedometerLineEnd.y;

          cv::line(frame, spedometerLineStart, spedometerLineEnd, cv::Scalar(255, 0, 0), 3);

          // DEBUG - show mouse position (helps to know where to put stuff)
          cv::putText(frame, mousePointText, cv::Point(mouseX + 10, mouseY + 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        }
        else
        {
          //! ----------------------------------------//
          //!                 HUD INIT                //
          //! ----------------------------------------//
        switch (hud.state){
          case 0:
            cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.rows, frame.cols), cv::Scalar(0, 0, 0), -1);
            cv::imshow("RDS_HUD", frame);
            cv::waitKey(400);
            cv::putText(frame, "CAM ONLINE", cv::Point(mid_cols - 300, mid_rows), cv::FONT_HERSHEY_SIMPLEX, 4, cv::Scalar(200, 200, 200), 7);
            cv::imshow("RDS_HUD", frame);
            cv::waitKey(1000);
            hud.state++;
            break;
          case 1:
            if(hud.init_i < 60){
              hud.init_i++;
              cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.rows/2 - (frame.rows/30)*hud.init_i, frame.cols), cv::Scalar(0, 0, 0), -1);
              cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.rows/2 + (frame.rows/30)*hud.init_i, frame.cols), cv::Scalar(0, 0, 0), -1);
            }else{
              hud.state++;
            }
            break;
          default:
            hud.initiated = true;
            break;
        }

         
        }
        
        cv::imshow("RDS_HUD", frame);
        cv::waitKey(1);
        // Convert back to ROS message and publish
        // sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        // hud_pub_.publish(msg_out);
        // RCLCPP_INFO(this->get_logger(), "meow");
        lock.unlock();
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<HUDOverlayNode>();
  node->init();
  rclcpp::Rate hud_refresh_rate(40);
  while (rclcpp::ok()){
    if(node->hud.initiated){
      RCLCPP_ERROR(node->get_logger(), "frame success");
    node->drawHud();

    }else{
      RCLCPP_ERROR(node->get_logger(), "empty frame");

    }
    rclcpp::spin_some(node);
    hud_refresh_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
