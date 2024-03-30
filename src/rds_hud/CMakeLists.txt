cmake_minimum_required(VERSION 3.8)
project(rds_hud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rds_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include) 


add_executable(hud_node src/heads_up_node.cpp include/heads_up_node.h include/heads_up_definitions.h)
ament_target_dependencies(hud_node rclcpp image_transport cv_bridge rds_msgs ament_index_cpp std_msgs)
target_link_libraries(hud_node ${OpenCV_LIBS} sfml-system sfml-audio)  # Link against the OpenCV libraries
install(TARGETS
hud_node
DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY 
    videos/
  DESTINATION share/${PROJECT_NAME}/videos
)
install(
  DIRECTORY 
    images/
  DESTINATION share/${PROJECT_NAME}/images
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
