#include <rclcpp/rclcpp.hpp>

#include <rds_msgs/msg/vehicle_interface.hpp>
#include <rds_msgs/msg/vehicle_status.hpp>
#include <serial/serial.h>
#include <thread>
#include <chrono>

#define NUM_LIGHTS 5
#define TX_UART_BUFF 128
#define RX_UART_BUFF 128
#define SIMULATE true

//!serial stuff
#define COMM_POLL_RATE 20


class VehicleInterface : public rclcpp::Node{
    public:
        VehicleInterface();
        rds_msgs::msg::VehicleInterface curr_vehicle_cmd;
        void serialRx();

    struct VehicleHW{
        bool initiated = false;
        bool manual = false;
        float steeringAngle = 0.0;
        float accel = 0.0;
        int current_gear = 0;
        int left_sonar = 0;
        int right_sonar = 0;
    };


    private:
    VehicleHW vehicle;
    //Subscriber callbacks
    void CommandCallback(const rds_msgs::msg::VehicleInterface::SharedPtr msg);
    //std::string port = "/dev/ttyACM0";
    unsigned long baud = 115200;
    //! set serial port
    std::string port = "/dev/serial/by-id/...";

    serial::Serial esp32;
    serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second


    //Subscribers
    rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr command_subscriber;

    //Pubslishers
    rclcpp::Publisher<rds_msgs::msg::VehicleStatus>::SharedPtr feedback_publisher;

    //Serial Handling
    void serialTx();

    void parseUart(std::string buffer);
    std::thread serialRxThread;

    rclcpp::TimerBase::SharedPtr timer_;


};
