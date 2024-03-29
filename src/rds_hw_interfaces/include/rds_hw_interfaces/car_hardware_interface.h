#include <rclcpp/rclcpp.hpp>

#include <rds_msgs/msg/vehicle_interface.hpp>
#include <rds_msgs/msg/vehicle_status.hpp>
#include <serial/serial.h>

#define NUM_LIGHTS 5
#define TX_UART_BUFF 128



class VehicleInterface : public rclcpp::Node{
    public:
        VehicleInterface();
        rds_msgs::msg::VehicleInterface curr_vehicle_cmd;


    private:
    //Subscriber callbacks
    void CommandCallback(const rds_msgs::msg::VehicleInterface::SharedPtr msg);

    //Subscribers
    rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr command_subscriber;

    //Pubslishers
    rclcpp::Publisher<rds_msgs::msg::VehicleStatus>::SharedPtr feedback_publisher;

    //Serial Handling
    void sendCommand();

    unsigned long baud = 115200;
    std::string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";

    serial::Serial esp32;
    serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second

};
