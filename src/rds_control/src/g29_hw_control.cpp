//this bypasses autoware auto
#include "g29_control.h"


#include "rclcpp/rclcpp.hpp"
#include "rds_msgs/msg/vehicle_interface.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback_array.hpp"

#define g29 true 


class ManualControlNode : public rclcpp::Node {
public:
    ManualControlNode() : Node("g29_control") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).transient_local();
        command_publisher_ = this->create_publisher<rds_msgs::msg::VehicleInterface>("/vehicle_1/command", qos);
        
        double period = 1.0/CONTROL_RATE;
        // timer_ = this->create_wall_timer(
        // std::chrono::duration<double>(period),std::bind(&ManualControlNode::test_send, this));
        g29_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ManualControlNode::joy_callback, this, std::placeholders::_1));
    }

    void send_command(float steering_angle, float speed, float acceleration, float jerk, int gear) {
        // rds_msgs::msg::VehicleInterface msg;
        // msg.lights.resize(NUM_LIGHTS);
        // msg.steering_angle = steering_angle;
        // msg.gas_pedal = acceleration;
        // msg.gear = gear;

        
    }
    

    // void test_send(){
    //     send_command(0.5, 1.0, 1.0, 0.5);
    //     // rclcpp::logger

    // }

private:
    struct Vehicle{
        bool manual = false;
        bool engine_running = false;
    };
    Vehicle vehicle;
    int gears[4] = {GEAR_REVERSE, GEAR_PARKING, GEAR_NEUTRAL, GEAR_1};
    int current_gear = 0;
    int prev_paddleR = 0;
    int prev_paddleL = 0;
    //int prev_lights[]
    //lights -- i found it easier to just fully split them up. should make a struct or smt 
    int  front_lights = 0;
    bool prev_front_lights = false;
    bool curr_front_lights = false;

    int brake_lights = 0;
    bool prev_brake_lights = false;
    bool curr_brake_lights = false;

    int prev_signalL = 0;
    bool signalL = false;
    bool curr_signalL = false;

    int prev_signalR = 0;
    bool signalR = false;
    bool curr_signalR = false;

    bool prev_hazards = 0;
    bool hazards = false;
    bool curr_hazards = false;
    rclcpp::Publisher<rds_msgs::msg::VehicleInterface>::SharedPtr command_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr vibe_publisher_;

    void publishFeedback()
    {
        auto message = sensor_msgs::msg::JoyFeedbackArray();
        auto feedback = sensor_msgs::msg::JoyFeedback();

        // Configure the feedback for vibration
        feedback.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
        feedback.id = 0; // Assuming 0 is the ID for the vibration motor
        feedback.intensity = 1.0; // Max intensity

        message.array.push_back(feedback);

        // Optionally, add more feedback commands here, such as for LEDs or buzzers

        vibe_publisher_->publish(message);
    }


    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        rds_msgs::msg::VehicleInterface vehicle_msg;

        float steering_angle;
        float speed;
        float gas_pedal;
        float brake_pedal;
        float clutch_pedal;
        float jerk;


        int lights[NUM_LIGHTS];
        vehicle_msg.lights.resize(NUM_LIGHTS);
        
        int paddleR = 0;
        int paddleL = 0;
        bool park_button = false;
        bool engine_pwr_button = false;
        if(g29){
            paddleR = msg->buttons[4];
            paddleL = msg->buttons[5];
            // lights[0] = msg->buttons[13];
            // lights[1] = msg->buttons[15];
            // lights[2] = msg->buttons[2];
            // lights[3] = msg->buttons[3];
            // lights[4] = msg->buttons[2];
            curr_front_lights = msg->buttons[1]; 
            curr_hazards = msg->buttons[3];
            curr_signalL = msg->buttons[11]; //1
            curr_signalR = msg->buttons[10]; //2
            engine_pwr_button = msg->buttons[23];
            park_button = msg->buttons[24];

        } else { //use joy_linux not joy
        paddleR = msg->buttons[4];
        paddleL = msg->buttons[5];
            //lights[0] = msg->buttons[3];
            //lights[1] = msg->buttons[1];
            lights[2] = msg->buttons[2];
            lights[3] = msg->buttons[8];
            lights[4] = msg->buttons[7];

            curr_hazards = msg->buttons[8];
            curr_signalL = msg->buttons[3];
            curr_signalR = msg->buttons[1];
            
        }

        if(paddleL == 0){
            prev_paddleL = 0;
        }
        if(paddleR == 0){
            prev_paddleR = 0;
        }
        
       // if()
      
        // if(prev_signalL != lights[0]){
        //     prev_signalL = lights[0];

        //       if(lights[0] == 1){
        //     vehicle_msg.lights[1] = 1;
        //     vehicle_msg.lights[0] = 0;
        //     lights[1] = 0;
        // }}
        
      
        // if(prev_signalR != lights[1]){
        //     prev_signalR = lights[1];

        //       if(lights[1] == 1){
        //     vehicle_msg.lights[0] = 1;
        //     vehicle_msg.lights[1] = 0;
        //     lights[0] = 0;
        //     RCLCPP_ERROR(this->get_logger(), "test");
        // }}
            if(g29){ //16 900
        steering_angle = -msg->axes[0];
        //speed = 99.0 - (msg->axes[3] + 1)*100;
        gas_pedal = (msg->axes[2] + 1.0)/2.0;//// - (msg->axes[3] + 1)*20;
        brake_pedal = (msg->axes[3] + 1.0)/2.0;
        clutch_pedal = (msg->axes[1] + 1.0)/2.0;
       //// jerk = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;

    }else{
        steering_angle = msg->axes[0];
     //   speed = 99.0 - (msg->axes[4] + 1)*100;
        gas_pedal = (-msg->axes[5] + 1.0)/2.0;//// - (msg->axes[5] + 1)*20;
        brake_pedal = (-msg->axes[2] + 1.0)/2.0;

        ////jerk = (msg->axes[2] + 1)*20 - (msg->axes[3] + 1)*20;

    }
        //! ----------------------------------------//
        //!                 LIGHTS                  //
        //! ----------------------------------------//

        if(curr_signalL){
            if(curr_signalL != prev_signalL){
                signalL = !signalL;
            }
        }
        
        if(curr_signalR){
            if(curr_signalR != prev_signalR){
                signalR = !signalR;
            }
        }
        if(curr_hazards){
            if(curr_hazards != prev_hazards){
                hazards = !hazards;
            }
        }

        prev_hazards = curr_hazards;
        prev_signalL = curr_signalL;
        prev_signalR = curr_signalR;

        if(signalR && !signalL){
            lights[0] = 1;
        }else{
            lights[0] = 0;
        }
        if(signalL && !signalR){
            lights[1] = 1;
        }else{
            lights[1] = 0;
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
        
        if(current_gear != GEAR_PARKING)
            if(current_gear < GEAR_REVERSE){
                current_gear = GEAR_REVERSE;
            }
        int max_gear = 5;
        if(!vehicle.manual){
            max_gear = 1;
        }
        if(current_gear > max_gear){
            current_gear = max_gear;
        }
        if(!vehicle.engine_running){
            current_gear = GEAR_PARKING;
            if(brake_pedal > 0.5 && engine_pwr_button){
                vehicle.engine_running = true;
                current_gear = GEAR_PARKING;
            }
        }
        if(park_button){
            current_gear = GEAR_PARKING;
        }


 
        if (gas_pedal < 0){
            gas_pedal = 0;
        }
        //! fillup msg
         for(int i = 0; i < NUM_LIGHTS - 1; i++){
            if(lights[i] == 1){
            vehicle_msg.lights[i] = 1;
            
            //publishFeedback();
            } else{
                vehicle_msg.lights[i] = 0;
            }
        }
     //   
       // vehicle_msg.lights = lights;
        vehicle_msg.left_signal = signalL;
        vehicle_msg.right_signal = signalR;
        vehicle_msg.steering_angle = steering_angle;
        vehicle_msg.gas_pedal = gas_pedal;
        vehicle_msg.brake_pedal = brake_pedal;
        if(brake_pedal > 0.0){
            vehicle_msg.rear_lights = true;
        } else {
            vehicle_msg.rear_lights = false;
        }
        vehicle_msg.engine_running = vehicle.engine_running;
        vehicle_msg.clutch = clutch_pedal;
        vehicle_msg.gear = current_gear;
        vehicle_msg.hazards = hazards;
        command_publisher_->publish(vehicle_msg);

        //send_command(steering_angle, speed, acceleration, jerk, current_gear);
        //send_gear_command(gears[current_gear]);
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