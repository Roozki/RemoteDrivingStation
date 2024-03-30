#include "rds_hw_interfaces/car_hardware_interface.h"

VehicleInterface::VehicleInterface() : Node("VehicleInterfaceNode"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    feedback_publisher = this->create_publisher<rds_msgs::msg::VehicleStatus>("/vehicle_1/status", qos);
        command_subscriber = this->create_subscription<rds_msgs::msg::VehicleInterface>(
            "/vehicle_1/command", 10, std::bind(&VehicleInterface::CommandCallback, this, std::placeholders::_1));

//TODO add net ping
        if(!SIMULATE){
        esp32.setPort(port);
        esp32.open();
    }
    double period = 1.0/COMM_POLL_RATE;

    //! set serial rx on a quick polling timer
    timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),std::bind(&VehicleInterface::serialRx, this));

}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleInterface>();
    



    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}


void VehicleInterface::CommandCallback(const rds_msgs::msg::VehicleInterface::SharedPtr msg){
    curr_vehicle_cmd.steering_angle = msg->steering_angle;
    curr_vehicle_cmd.gas_pedal = msg->gas_pedal;
    curr_vehicle_cmd.brake_pedal = msg->brake_pedal;
    curr_vehicle_cmd.lights.resize(NUM_LIGHTS);
    for(int i = 0; i < NUM_LIGHTS; i++){
        curr_vehicle_cmd.lights[i] = msg->lights[i];
    }

}


void VehicleInterface::serialTx(){
   char tx_msg[TX_UART_BUFF]; 
   sprintf(tx_msg, "$C(%0.2f, %0.2f, %i, %i, %i, %i, %i)\n", curr_vehicle_cmd.steering_angle, curr_vehicle_cmd.gas_pedal - curr_vehicle_cmd.brake_pedal, curr_vehicle_cmd.lights[0], curr_vehicle_cmd.lights[1], curr_vehicle_cmd.lights[2], curr_vehicle_cmd.lights[3], curr_vehicle_cmd.lights[4]);
   esp32.write(tx_msg);
   RCLCPP_ERROR(this->get_logger(), "Sent via serial: %s", tx_msg);
   esp32.flushOutput();
}

void VehicleInterface::parseUart(std::string buffer){
    float checksum;
 // Removing the '$R(' at the beginning and the ')\n' at the end
    std::string content = buffer.substr(3, buffer.size() - 5);

    // Using stringstream to parse the content
    std::stringstream ss(content);
    char separator; // To consume the commas and parentheses

    // Extracting the values
    ss >> checksum >> separator >> vehicle.right_sonar >> separator >> vehicle.left_sonar;

    rds_msgs::msg::VehicleStatus outMsg;
    outMsg.left_sonar = vehicle.right_sonar;
    outMsg.right_sonar = vehicle.left_sonar;
            if(vehicle.initiated != 1){
            vehicle.initiated = 1;
        }
    outMsg.online = true;
    feedback_publisher->publish(outMsg);
    
}

void VehicleInterface::serialRx(){
    //rclcpp::Rate loop_rate(50);
    std::string next_char = "";
    std::string buffer = "";
    int timeoutCounter = 0;
    //zephyrComm.teensy.flushInput();
   if (esp32.available() > 0){
       // ROS_WARN("Reading");

        //timeoutCounter ++;
       // next_char = teensy.read(); 
        buffer = esp32.read(RX_UART_BUFF);
        RCLCPP_WARN(this->get_logger(), "%s", buffer.c_str());
        // if(next_char == "\n" || next_char == "\r" || next_char == "\0"){
        //     timeoutCounter = RX_UART_BUFF;
        // }
        if (buffer.size() > 0){
        if(buffer.find("Car Ready") != std::string::npos){
       // fresh_rx_angle = true;
     }else if(buffer.find("C") != std::string::npos){
        parseUart(buffer);

     }


   }


    
     

// if (buffer.size() > 0){
//         if(buffer.find("Arm Ready") != std::string::npos){
//         homed = true;
//        // fresh_rx_angle = true;
//      }else if(buffer.find("my_angleP") != std::string::npos){
//         parseArmAngleUart(buffer);
//      }


//    }
        //sleep(1);
    }
    }

/*

    TX: $C(1, 2, 3, 4, 5, 6, 7)\n
    1: steering angle   -- float, 0.2 -> -1 to 1
    2: linear velocity  -- float, 0.2 -> -1 to 1
    3: left_signal      -- int,   1 on (flashing), 0 off. 
    4: right signal     -- same as above
    5: hazards          -- same as signals
    6: front lights     -- 1 on, 0 off
    7: brake lights     -- 1 on, 0 off
    RX:
      command success

    RX:
    1: left_sonar:  1, object detected, 0 clear
    2: right_sonar: 1, object detected, 0 clear



*/