#include "thruster_driver/node_dif2wheel.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::thruster_driver::ThrusterDif2WheelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
