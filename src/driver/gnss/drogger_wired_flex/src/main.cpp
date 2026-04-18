#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "drogger_wired_flex/node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drogger_wired_flex::DroggerWiredFlexNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
