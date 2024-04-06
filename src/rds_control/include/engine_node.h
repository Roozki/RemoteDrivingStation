#include "rclcpp/rclcpp.hpp"
#include "rds_msgs/msg/vehicle_interface.hpp"
#include "rds_msgs/msg/engine_interface.hpp"

class EngineNode : public rclcpp::Node {
public:
    EngnineNode() : Node("engine_control") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).transient_local();
        command_publisher_ = this->create_publisher<rds_msgs::msg::VehicleInterface>("/vehicle_1/command", qos);
        
        double period = 1.0/CONTROL_RATE;
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        driver_subscriber_ = this->create_subscription<rds_msgs::msg::EngineInterface>(
            "/vehicle_1/engine/command", 3, std::bind(&EngineNode::engineCallback, this, std::placeholders::_1));

    
    }

private:
    float curr_rpm = 0;
    int curr_gear = 0;

};


//void EngineNode
