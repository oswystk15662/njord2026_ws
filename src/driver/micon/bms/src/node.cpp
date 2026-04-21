#include "bms/node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>

namespace njord
{
namespace bms
{

BmsNode::BmsNode(const rclcpp::NodeOptions & options)
: Node("bms_node", options)
{
  transport_mode_ = toLower(this->declare_parameter<std::string>("transport_mode", "mros_usb"));
  can_rx_id_ = static_cast<std::uint32_t>(this->declare_parameter<int>("can_rx_id", 0x401));

  const std::string mros_rx_topic = this->declare_parameter<std::string>("topics.mros_rx", "bms/raw_cells");
  const std::string can_rx_topic = this->declare_parameter<std::string>("topics.can_rx", "bms/can_rx");
  const std::string output_topic = this->declare_parameter<std::string>("topics.output", "bms/cell_voltages");

  can_enabled_ = (transport_mode_ == "can" || transport_mode_ == "both");
  mros_enabled_ = (transport_mode_ == "mros_usb" || transport_mode_ == "both");

  if (!can_enabled_ && !mros_enabled_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid transport_mode='%s'. Falling back to mros_usb.",
      transport_mode_.c_str());
    transport_mode_ = "mros_usb";
    mros_enabled_ = true;
  }

  pub_cells_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic, 10);

  if (mros_enabled_) {
    sub_mros_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      mros_rx_topic,
      10,
      std::bind(&BmsNode::mrosCallback, this, std::placeholders::_1));
  }

  if (can_enabled_) {
    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
      can_rx_topic,
      50,
      std::bind(&BmsNode::canCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "bms node started. transport_mode=%s",
    transport_mode_.c_str());
}

void BmsNode::mrosCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 4U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "mros bms message size=%zu. Expected 4.",
      msg->data.size());
    return;
  }

  std::array<float, 4> cells{
    msg->data[0],
    msg->data[1],
    msg->data[2],
    msg->data[3]};
  publishCells(cells);
}

void BmsNode::canCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->id != can_rx_id_) {
    return;
  }

  if (msg->dlc < 8U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "CAN bms frame dlc=%u. Expected >= 8.", msg->dlc);
    return;
  }

  std::array<float, 4> cells{};
  for (std::size_t i = 0; i < 4; ++i) {
    const std::uint16_t mv = static_cast<std::uint16_t>(msg->data[i * 2]) |
      (static_cast<std::uint16_t>(msg->data[i * 2 + 1]) << 8);
    cells[i] = static_cast<float>(mv) / 1000.0F;
  }

  publishCells(cells);
}

void BmsNode::publishCells(const std::array<float, 4> & cells)
{
  std_msgs::msg::Float32MultiArray out;
  out.data = {cells[0], cells[1], cells[2], cells[3]};
  pub_cells_->publish(out);
}

std::string BmsNode::toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

}  // namespace bms
}  // namespace njord

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord::bms::BmsNode)
