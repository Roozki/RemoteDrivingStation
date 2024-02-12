#include "g29_control.h"

#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "sensor_msgs/msg/joy.hpp"



class ManualControlNode : public rclcpp::Node {
public:
    ManualControlNode() : Node("g29_control") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        command_publisher_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", qos);
        gear_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
        
        double period = 1.0/CONTROL_RATE;
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ManualControlNode::joy_callback, this, std::placeholders::_1));
    }

    void send_command(float steering_angle, float speed, float acceleration, float jerk) {
        autoware_auto_control_msgs::msg::AckermannControlCommand msg;
        msg.lateral.steering_tire_angle = steering_angle;
        msg.longitudinal.speed = speed;
        msg.longitudinal.acceleration = acceleration;
        msg.longitudinal.jerk = jerk;
        command_publisher_->publish(msg);
        
    }

    void send_gear_command(int gear){
        autoware_auto_vehicle_msgs::msg::GearCommand msg;
        msg.command = gear;
        gear_publisher_->publish(msg);
    }

    

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
    int gears[4] = {GEAR_PARKING, GEAR_REVERSE, GEAR_NEUTRAL, GEAR_1};
    int current_gear = 0;
    int prev_paddleR = 0;
    int prev_paddleL = 0;

    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        float steering_angle = msg->axes[0];
        float speed = 99.0 - (msg->axes[3] + 1)*100;
        float acceleration = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
        float jerk = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;
        int paddleR = msg->buttons[4];
        int paddleL = msg->buttons[5];
        
        if(paddleL == 0){
            prev_paddleL = 0;
        }
        if(paddleR == 0){
            prev_paddleR = 0;
        }

        if(paddleR){
        if(prev_paddleR != paddleR){
            prev_paddleR = paddleR;
            current_gear++;
        }
        }else if(paddleL){
        if(prev_paddleL != paddleL){
            prev_paddleL = paddleL;
            current_gear--;
        }
        }
        if(current_gear < 0){
            current_gear = 0;
        }

        if(current_gear > 4){
            current_gear = 4;
        }

        send_command(steering_angle, speed, acceleration, jerk);
        send_gear_command(gears[current_gear]);
    }
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr g29_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualControlNode>();

    // Example: send a command with a steering angle of 0.5 rad and speed of 1.0 m/s

    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}