cmake_minimum_required(VERSION 3.5)
project(rds_gui)

# Find packages
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(autoware_auto_vehicle_msgs REQUIRED)
#find_package(autoware_auto_control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rds_msgs REQUIRED)


file(GLOB QWT_INCLUDE_DIR "/usr/include/qwt")
file(GLOB QWT_LIBRARIES "/usr/lib/libqwt*")
set(CMAKE_AUTOMOC ON) 
# Include Qt and ROS2 directories
include_directories(include/${PROJECT_NAME} ${rclcpp_INCLUDE_DIRS} ${QWT_INCLUDE_DIR})

# Tell CMake to wrap the UI file to a header
qt5_wrap_ui(UI_HEADERS ui/mainwindow.ui)

# Add executable
add_executable(rds_qt src/mainwindow.cpp src/main.cpp include/${PROJECT_NAME}/mainwindow.h include/${PROJECT_NAME}/ui_mainwindow.h ${UI_HEADERS})
target_link_libraries(rds_qt Qt5::Widgets ${rclcpp_LIBRARIES} ${QWT_LIBRARIES}  )
ament_target_dependencies(rds_qt sensor_msgs rds_msgs)

# Install rules, etc.
install(TARGETS 
rds_qt
DESTINATION 
lib/${PROJECT_NAME}
)

ament_package()
