#include<rds_hw_interfaces/car_hardware_interface.h"




int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleInterface>();

}