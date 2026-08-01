#include "shared_controller/shared_controller.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SharedController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}