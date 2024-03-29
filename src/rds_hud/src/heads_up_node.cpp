#include <heads_up_node.h>
#include <heads_up_definitions.h>



int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HUDOverlayNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
