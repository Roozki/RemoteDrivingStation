#include "rds_hw_interfaces/car_hardware_interface.h"



VehicleInterface::VehicleInterface() : Node("VehicleInterfaceNode"){
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    feedback_publisher = this->create_publisher<rds_msgs::msg::VehicleStatus>("/vehicle_1/status", qos);
        command_subscriber = this->create_subscription<rds_msgs::msg::VehicleInterface>(
            "/vehicle_1/command", 10, std::bind(&VehicleInterface::CommandCallback, this, std::placeholders::_1));
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


void VehicleInterface::sendCommand(){
   char tx_msg[TX_UART_BUFF];
   sprintf(tx_msg, "$C(%0.2f, %0.2f, %i, %i, %i, %i)\n", curr_vehicle_cmd.steering_angle, curr_vehicle_cmd.gas_pedal - curr_vehicle_cmd.brake_pedal, curr_vehicle_cmd.lights[0], curr_vehicle_cmd.lights[1], curr_vehicle_cmd.lights[2], curr_vehicle_cmd.lights[3], curr_vehicle_cmd.lights[4]);

   esp32.write(tx_msg);
   RCLCPP_ERROR(this->get_logger(), "Sent via serial: %s", tx_msg);
   esp32.flushOutput();
}

