#include "dutyed_tf_pub_with_disturbance/sim_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::sim::SimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
