#include "bms/node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::bms::BmsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
