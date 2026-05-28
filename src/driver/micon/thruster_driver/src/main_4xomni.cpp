#include "thruster_driver/node_4xomni.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::thruster_driver::Thruster4xOmniNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
