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
          if (latency_refresh_count > 90)
          {
            latency_refresh_count = 0;
            networkCheck();
          }
           //! ----------------------------------------//
          //!             REARVIEW CAM                //
          //! ----------------------------------------//
          //! + reverse change
          int rear_view_width = 400;
          int rear_view_height = 200;
          if(last_rear_frame_.width == NULL){
        cv::Mat placeholderFrame(rear_view_width, rear_view_height, CV_8UC3, cv::Scalar(0, 0, 0));  // Adjust the size as needed

        // Set the text properties
        std::string rear_cam_offline_text = "REAR CAM OFFLINE";
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        cv::Scalar textColor(255, 255, 255);  // White text

        // Get the text size to center the text on the placeholder frame
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(rear_cam_offline_text, fontFace, fontScale, thickness, &baseline);
        cv::Point textOrg((placeholderFrame.cols - textSize.width) / 2, (placeholderFrame.rows + textSize.height) / 2);

        // Draw the text on the placeholder frame
        cv::putText(placeholderFrame, rear_cam_offline_text, textOrg, fontFace, fontScale, textColor, thickness);
        placeholderFrame.copyTo(rear_frame);
          }else{
          rear_frame = cv_bridge::toCvCopy(last_rear_frame_, "bgr8")->image;



          }
          if(current_gear == "R"){
            cv::Mat temp_buff_frame = frame.clone();;
            // cv::resize(rear_frame, temp_buff_frame, cv::Size(frame.cols, frame.rows));
            // cv::resize(frame, rear_frame, cv::Size(rear_view_width, rear_view_height));
            // cv::resize(temp_buff_frame, frame, cv::Size(rear_view_width, rear_view_height));
            // // Swap the feeds in reverse

            // Resize the rear frame to fill the main frame
            cv::resize(rear_frame, frame, frame.size()); 

            // Resize the original main frame (now stored in temp) to fit the rearview mirror size and position
            cv::Mat smallMainFrame;
            cv::resize(temp_buff_frame, smallMainFrame, cv::Size(rear_view_width, rear_view_height)); // Ensure rear_view_width and rear_view_height are defined

            // Create a black border for the smaller main frame, to maintain the "mirror" effect
            cv::copyMakeBorder(smallMainFrame, smallMainFrame, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            // Define the position of the smaller main frame on the new main frame (which now shows the rear view)
            cv::Rect small_main_roi(cv::Point(mid_cols - rear_view_width/2, 20), smallMainFrame.size()); // Adjust the position as needed

            // Place the smaller main frame onto the new main frame
            smallMainFrame.copyTo(frame(small_main_roi));
          } else {
                     // Resize the rearview frame to fit it as a "mirror" on the main frame
          cv::resize(rear_frame, resizedRearView, cv::Size(rear_view_width, rear_view_height));  // Adjust size as needed

        // Create a black border for the rearview "mirror"
          cv::copyMakeBorder(resizedRearView, resizedRearView, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        // Define where to overlay the rearview frame on the main frame
          cv::Rect rear_roi(cv::Point(mid_cols - rear_view_width/2, 20), resizedRearView.size());  // Adjust position as needed

        // Overlay the rearview frame onto the main frame
          resizedRearView.copyTo(frame(rear_roi));
          }

          //! ----------------------------------------//
          //!               DRIVE LINE                //
          //! ----------------------------------------//

          cv::Point driveLineStart(frame.cols / 2, frame.rows - status_bar_height); // Start at the bottom center
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
          oss.clear();
          oss << std::fixed << std::setprecision(0) << (vehicle_1_current_status.velocity *(36/10)); // Set precision to 0 decimal places
          std::string velocity_string = oss.str();

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
          //!           STATUS INDICATORS             //
          //! ----------------------------------------//

          
          drawHazardsSign(frame, cv::Point(1400, frame.rows - status_bar_height + 20), vehicle_1_current_command.hazards);
          drawSignalStatus(frame, cv::Point(400, frame.rows - status_bar_height + 30), vehicle_1_current_command.left_signal, vehicle_1_current_command.right_signal);


          //! ----------------------------------------//
          //!                 SIGNIALS                //
          //! ----------------------------------------//
          vehicle_1_current_command.lights.resize(5);

          if (vehicle_1_current_command.left_signal == 1 || vehicle_1_current_command.hazards)
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
          if (vehicle_1_current_command.right_signal == 1 || vehicle_1_current_command.hazards)
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
          //!                 NETWORK                 //
          //! ----------------------------------------//
          cv::putText(frame, "NET: " + latencyString + "ms", cv::Point(frame.cols - 350, frame.rows - status_bar_height / 10), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(200, 200, 200), 3);

          //! ----------------------------------------//
          //!                 GEARS                   //
          //! ----------------------------------------//
          // cv::putText(frame, "GEAR", cv::Point(10, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 3);


          cv::putText(frame, current_gear, cv::Point(190, frame.rows - status_bar_height / 10), cv::FONT_HERSHEY_SIMPLEX, 5, gear_colour, 3);
          cv::line(frame, cv::Point(0, frame.rows - status_bar_height), cv::Point(frame.cols, frame.rows - status_bar_height), CV_RGB(0, 0, 0), 4);
        
          //! ----------------------------------------//
          //!             SPEDOMETER + RPM            //
          //! ----------------------------------------//
        
          cv::putText(frame, velocity_string + "km/h", cv::Point(597, 1006), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2);
          cv::putText(frame, gas_pedal_string + "rev/min", cv::Point(1242, 1006), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 2);

          // cv::putText(frame, , cv::Point(950, frame.rows - status_bar_height/2), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,255), 3);
          int spedometer_radius = 100;
          int rpm_radius = 100;

          cv::Point spedometerLineStart(frame.cols / 3, frame.rows - 20);
          cv::Point spedometerLineEnd((frame.cols / 3) - spedometer_radius, frame.rows - 20);
          cv::Point rpmLineStart(2*frame.cols / 3, frame.rows - 20);
          cv::Point rpmLineEnd((2*frame.cols / 3) - rpm_radius, frame.rows - 20);
          //@TODO Need to add progress bar for gas
          // add kmph
          // add gear
          int rpm_increment = 10; // 40/NUM_NUMS;
          int max_rpm = rpm_increment * NUM_RPM_NUMS;
          int sped_increment = 10; // 40/NUM_NUMS;
          int max_sped = sped_increment * NUM_NUMS;
          float rpm_angle = ((0) * rpm_increment * PI) / max_rpm;

          float spedometer_angle = ((0) * sped_increment * PI) / max_sped;
          cv::putText(frame, "0", cv::Point(((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius) * (-1) * sin(spedometer_angle)) + spedometerLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          cv::putText(frame, "0", cv::Point(((rpm_radius) * (-1) * cos(rpm_angle)) + rpmLineEnd.x + rpm_radius, ((rpm_radius) * (-1) * sin(rpm_angle)) + rpmLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          
          std::string sped_num;
          std::string rpm_num;

          for (int i = 1; i < NUM_NUMS; i++)
          {
            sped_num = std::to_string(i * sped_increment); //+"km/h";
            spedometer_angle = ((i)*sped_increment * PI) / max_sped;
            cv::putText(frame, sped_num, cv::Point(((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius) * (-1) * sin(spedometer_angle)) + spedometerLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          }
          for (int i = 1; i < NUM_RPM_NUMS; i++)
          {
            rpm_num = std::to_string(i * rpm_increment); //+ "rev/min";
            rpm_angle = ((i)*rpm_increment * PI) / max_rpm;
            cv::putText(frame, rpm_num, cv::Point(((rpm_radius) * (-1) * cos(rpm_angle)) + rpmLineEnd.x + rpm_radius, ((rpm_radius) * (-1) * sin(rpm_angle)) + rpmLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          }
          sped_num = "100"; //+"km/h";
          rpm_num = "100"; //+"km/h";


          cv::putText(frame, sped_num, cv::Point(((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius, ((spedometer_radius) * (-1) * sin(PI)) + spedometerLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);
          cv::putText(frame, rpm_num, cv::Point(((rpm_radius) * (-1) * cos(rpm_angle)) + rpmLineEnd.x + rpm_radius, ((rpm_radius) * (-1) * sin(PI)) + rpmLineEnd.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 2);

          rpm_angle = (vehicle_1_current_command.gas_pedal * PI);
          spedometer_angle = ((vehicle_1_current_status.velocity / max_sped) * PI);


          spedometerLineEnd.x = ((spedometer_radius) * (-1) * cos(spedometer_angle)) + spedometerLineEnd.x + spedometer_radius;
          spedometerLineEnd.y = ((spedometer_radius) * (-1) * sin(spedometer_angle)) + spedometerLineEnd.y;
         
          rpmLineEnd.x = ((rpm_radius) * (-1) * cos(rpm_angle)) + rpmLineEnd.x + rpm_radius;
          rpmLineEnd.y = ((rpm_radius) * (-1) * sin(rpm_angle)) + rpmLineEnd.y;

          cv::line(frame, spedometerLineStart, spedometerLineEnd, cv::Scalar(255, 0, 0), 3);
          cv::line(frame, rpmLineStart, rpmLineEnd, cv::Scalar(0, 0, 220), 3);


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


            hud.state++;
            break;
          case 1:
            if(hud.init_i < 100){
              hud.init_i = 100;
            }
            for (int i = 0; i < 6; i++){
            if(hud.init_i < (frame.cols/6.5)){
              hud.init_i++;
               int temp_width = static_cast<int>(pow(hud.init_i / 100.0, 8)); // Ensure proper casting and division
              cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols - temp_width, frame.rows), cv::Scalar(0, 0, 0), -1);
              ////cv::rectangle(frame, cv::Point(0, 0), cv::Point(frame.cols/2 + (frame.cols/30)*hud.init_i, frame.rows), cv::Scalar(0, 0, 0), -1);
          //    cv::rectangle(frame, cv::Point(0, mid_rows - 100), cv::Point(frame.cols, mid_rows + 40), cv::Scalar(0, 0, 0), -1);
        //    cv::putText(frame, "CAM ONLINE", cv::Point(mid_cols - 350, mid_rows + 20), cv::FONT_HERSHEY_SIMPLEX, 4, cv::Scalar(220, 220, 220), 7);

            }
            }
            
            if (hud.init_i >= frame.cols/6.5){
              hud.state++;
              hud.initiated = true;

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
    for (int i = 0; i < 40; i++){
    hud_refresh_rate.sleep();
    rclcpp::spin_some(node);
    }

    if(node->hud.fancyPantsDone != 1){
     node->fancyPantsStartup();
     //node->hud.fancyPantsDone = 1;
    }
  while (rclcpp::ok()){
 
    if(node->hud.ready){
     ///// RCLCPP_ERROR(node->get_logger(), "frame success");
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
