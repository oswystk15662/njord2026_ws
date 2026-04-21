#include "thruster_driver/node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

namespace njord
{
namespace thruster_driver
{

namespace
{
double clampNorm(double value)
{
  return std::clamp(value, -1.0, 1.0);
}
}  // namespace

ThrusterDriverNode::ThrusterDriverNode(const rclcpp::NodeOptions & options)
: Node("thruster_driver_node", options)
{
  input_mode_ = toLower(this->declare_parameter<std::string>("input_mode", "cmd_vel"));
  transport_mode_ = toLower(this->declare_parameter<std::string>("transport_mode", "mros_usb"));

  max_linear_x_ = this->declare_parameter<double>("max_linear_x", 1.0);
  max_angular_z_ = this->declare_parameter<double>("max_angular_z", 1.0);
  angular_coeff_ = this->declare_parameter<double>("angular_coeff", 0.5);

  duty_resolution_ = this->declare_parameter<int>("duty_resolution", 1000);

  left_reverse_ = this->declare_parameter<bool>("left_reverse", false);
  right_reverse_ = this->declare_parameter<bool>("right_reverse", false);

  u16_neutral_ = this->declare_parameter<int>("u16_neutral", 1000);
  u16_span_ = this->declare_parameter<int>("u16_span", 1000);

  left_can_id_ = this->declare_parameter<int>("left_can_id", 0x301);
  right_can_id_ = this->declare_parameter<int>("right_can_id", 0x302);

  const std::string cmd_vel_topic = this->declare_parameter<std::string>("topics.cmd_vel", "cmd_vel");
  const std::string duty_array_topic = this->declare_parameter<std::string>("topics.duty_array", "thruster_command");
  const std::string left_mros_topic = this->declare_parameter<std::string>("topics.left_mros_cmd", "left_esp32/cmd_duty_u16");
  const std::string right_mros_topic = this->declare_parameter<std::string>("topics.right_mros_cmd", "right_esp32/cmd_duty_u16");
  const std::string left_can_topic = this->declare_parameter<std::string>("topics.left_can_tx", "left_esp32/can_tx");
  const std::string right_can_topic = this->declare_parameter<std::string>("topics.right_can_tx", "right_esp32/can_tx");

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

  if (mros_enabled_) {
    pub_left_mros_ = this->create_publisher<std_msgs::msg::UInt16>(left_mros_topic, 10);
    pub_right_mros_ = this->create_publisher<std_msgs::msg::UInt16>(right_mros_topic, 10);
  }

  if (can_enabled_) {
    pub_left_can_ = this->create_publisher<can_msgs::msg::Frame>(left_can_topic, 10);
    pub_right_can_ = this->create_publisher<can_msgs::msg::Frame>(right_can_topic, 10);
  }

  if (input_mode_ == "cmd_vel") {
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      std::bind(&ThrusterDriverNode::cmdVelCallback, this, std::placeholders::_1));
  } else if (input_mode_ == "duty_array") {
    sub_duty_array_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      duty_array_topic,
      10,
      std::bind(&ThrusterDriverNode::dutyArrayCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid input_mode='%s'. Falling back to cmd_vel.",
      input_mode_.c_str());
    input_mode_ = "cmd_vel";
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      std::bind(&ThrusterDriverNode::cmdVelCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "thruster_driver started. input_mode=%s, transport_mode=%s",
    input_mode_.c_str(), transport_mode_.c_str());
}

void ThrusterDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double surge = clampNorm(msg->linear.x / std::max(1e-6, max_linear_x_));
  const double yaw = clampNorm((angular_coeff_ * msg->angular.z) / std::max(1e-6, max_angular_z_));

  double left = clampNorm(surge - yaw);
  double right = clampNorm(surge + yaw);

  if (left_reverse_) {
    left *= -1.0;
  }
  if (right_reverse_) {
    right *= -1.0;
  }

  publishCommands(left, right);
}

void ThrusterDriverNode::dutyArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received duty_array with size=%zu. Expected at least 2.",
      msg->data.size());
    return;
  }

  const double resolution = std::max(1, duty_resolution_);
  double left = clampNorm(static_cast<double>(msg->data[0]) / resolution);
  double right = clampNorm(static_cast<double>(msg->data[1]) / resolution);

  if (left_reverse_) {
    left *= -1.0;
  }
  if (right_reverse_) {
    right *= -1.0;
  }

  publishCommands(left, right);
}

void ThrusterDriverNode::publishCommands(double left_norm, double right_norm)
{
  const std::uint16_t left_u16 = toUint16Command(left_norm);
  const std::uint16_t right_u16 = toUint16Command(right_norm);

  if (mros_enabled_) {
    std_msgs::msg::UInt16 left_msg;
    left_msg.data = left_u16;
    std_msgs::msg::UInt16 right_msg;
    right_msg.data = right_u16;

    pub_left_mros_->publish(left_msg);
    pub_right_mros_->publish(right_msg);
  }

  if (can_enabled_) {
    can_msgs::msg::Frame left_frame;
    left_frame.id = static_cast<std::uint32_t>(left_can_id_);
    left_frame.is_rtr = false;
    left_frame.is_extended = false;
    left_frame.is_error = false;
    left_frame.dlc = 2;
    left_frame.data[0] = static_cast<std::uint8_t>(left_u16 & 0xFFU);
    left_frame.data[1] = static_cast<std::uint8_t>((left_u16 >> 8) & 0xFFU);

    can_msgs::msg::Frame right_frame;
    right_frame.id = static_cast<std::uint32_t>(right_can_id_);
    right_frame.is_rtr = false;
    right_frame.is_extended = false;
    right_frame.is_error = false;
    right_frame.dlc = 2;
    right_frame.data[0] = static_cast<std::uint8_t>(right_u16 & 0xFFU);
    right_frame.data[1] = static_cast<std::uint8_t>((right_u16 >> 8) & 0xFFU);

    pub_left_can_->publish(left_frame);
    pub_right_can_->publish(right_frame);
  }
}

std::uint16_t ThrusterDriverNode::toUint16Command(double normalized) const
{
  const double cmd = static_cast<double>(u16_neutral_) + clampNorm(normalized) * static_cast<double>(u16_span_);
  const int cmd_i = static_cast<int>(std::lround(cmd));
  return static_cast<std::uint16_t>(std::clamp(cmd_i, 0, 65535));
}

std::string ThrusterDriverNode::toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

}  // namespace thruster_driver
}  // namespace njord

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord::thruster_driver::ThrusterDriverNode)
