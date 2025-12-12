#include <rclcpp/rclcpp.hpp>
#include "drogger_bt/node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drogger_bt::DroggerDriver>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}