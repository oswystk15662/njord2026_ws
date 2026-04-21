#ifndef THRUSTER_DRIVER_NODE_HPP_
#define THRUSTER_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <can_msgs/msg/frame.hpp>

#include <string>

namespace njord
{
namespace thruster_driver
{

class ThrusterDriverNode : public rclcpp::Node
{
public:
  explicit ThrusterDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void dutyArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

  void publishCommands(double left_norm, double right_norm);
  std::uint16_t toUint16Command(double normalized) const;
  static std::string toLower(std::string value);

  // Input subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_duty_array_;

  // mROS(USB) outputs (one ESP32 -> one ESC -> one UInt16)
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_left_mros_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_right_mros_;

  // CAN outputs
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_left_can_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_right_can_;

  std::string input_mode_;
  std::string transport_mode_;

  // cmd_vel based mixing
  double max_linear_x_;
  double max_angular_z_;
  double angular_coeff_;

  // kinematics duty-array scaling
  int duty_resolution_;

  bool left_reverse_;
  bool right_reverse_;

  // Encoding to UInt16 for ESP32
  int u16_neutral_;
  int u16_span_;

  int left_can_id_;
  int right_can_id_;

  bool can_enabled_;
  bool mros_enabled_;
};

}  // namespace thruster_driver
}  // namespace njord

#endif  // THRUSTER_DRIVER_NODE_HPP_
