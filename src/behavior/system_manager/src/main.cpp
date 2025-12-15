#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "system_manager/node.hpp" 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord_control::SystemManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}