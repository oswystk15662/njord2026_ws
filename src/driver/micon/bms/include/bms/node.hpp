#ifndef BMS_NODE_HPP_
#define BMS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <can_msgs/msg/frame.hpp>

#include <array>
#include <string>

namespace njord
{
namespace bms
{

class BmsNode : public rclcpp::Node
{
public:
  explicit BmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void mrosCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void canCallback(const can_msgs::msg::Frame::SharedPtr msg);

  void publishCells(const std::array<float, 4> & cells);
  static std::string toLower(std::string value);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_mros_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_cells_;

  std::string transport_mode_;
  std::uint32_t can_rx_id_;

  bool can_enabled_;
  bool mros_enabled_;
};

}  // namespace bms
}  // namespace njord

#endif  // BMS_NODE_HPP_
