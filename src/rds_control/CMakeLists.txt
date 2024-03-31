cmake_minimum_required(VERSION 3.8)
project(rds_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(rds_msgs REQUIRED)
#find_package(autoware_auto_vehicle_msgs REQUIRED)
#find_package(autoware_auto_control_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include/)

#add_executable(g29_control src/g29_control.cpp include/g29_control.h)
#ament_target_dependencies(g29_control rclcpp autoware_auto_control_msgs autoware_auto_vehicle_msgs joy)

add_executable(g29_hw_control src/g29_hw_control.cpp include/g29_control.h)
ament_target_dependencies(g29_hw_control rclcpp rds_msgs joy)




install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS 
 # g29_control
  g29_hw_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
