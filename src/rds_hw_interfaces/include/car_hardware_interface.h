#include <rds_msgs/msg/VehicleInterface.h>
#include <rclcpp/rclcpp.h>




class VehicleInterface : public rclcpp::Node{
    public:
        VehicleInterface();
    


    private:
    //Subscriber callbacks
    void CommandCallback(const rds_msgs::msg::VehicleInterface::SharedPtr msg);

    //Subscribers
    rclcpp::Subscription<rds_msgs::msg::VehicleInterface>::SharedPtr command_subscriber;

    //Pubslishers
    rclcpp::publisher<rds_msgs::msg::VehicleStatus>::SharedPtr feedback_publisher;
}