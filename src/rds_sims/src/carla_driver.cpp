//this bypasses autoware auto

#define CONTROL_RATE 60.0
#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

class ManualControlNode : public rclcpp::Node {
public:
    ManualControlNode() : Node("manual_control") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        double period = 1.0/CONTROL_RATE;
        timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        
    }

    void send_command(float steering_angle, float speed, float acceleration, float jerk) {
        autoware_auto_control_msgs::msg::AckermannControlCommand msg;
        msg.lateral.steering_tire_angle = steering_angle;
        msg.longitudinal.speed = speed;
        msg.longitudinal.acceleration = acceleration;
        msg.longitudinal.jerk = jerk;

        command_publisher_->publish(msg);
    }

    void test_send(){
        send_command(0.5, 1.0, 1.0, 0.5);
        // rclcpp::logger

    }

private:
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualControlNode>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}