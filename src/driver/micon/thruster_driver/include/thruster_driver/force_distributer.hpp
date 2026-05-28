#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>

namespace njord
{
namespace thruster_driver
{

class ForceDistributerNode : public rclcpp::Node
{
public:
  ForceDistributerNode();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  std::array<std::array<float, 2>, 4> thruster_dirs_{};
  std::array<std::array<float, 2>, 4> thruster_pos_{};
};

}  // namespace thruster_driver
}  // namespace njord
