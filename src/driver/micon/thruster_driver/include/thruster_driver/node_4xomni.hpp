#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <memory>
#include <string>
#include <vector>

#include "thruster_driver/dynamics_model_4xomni.hpp"
#include "thruster_driver/software_pwm.hpp"

namespace njord
{
namespace thruster_driver
{

class Thruster4xOmniNode : public rclcpp::Node
{
public:
  Thruster4xOmniNode();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void feedback_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void feedback_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_timer_callback();

  double clamp(double value, double min_value, double max_value) const;
  double apply_deadzone(double value, double deadzone) const;
  double apply_static_map(double value, std::size_t wheel_index) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_feedback_twist_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_feedback_odometry_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_current_duty_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_model_state_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<std::unique_ptr<SoftPwmMotor>> motors_;
  std::unique_ptr<DynamicsModel> dynamics_model_;

  double ref_linear_x_{0.0};
  double ref_linear_y_{0.0};
  double ref_angular_z_{0.0};

  double feedback_linear_x_{0.0};
  double feedback_linear_y_{0.0};
  double feedback_angular_z_{0.0};

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_feedback_time_;
  rclcpp::Time last_control_time_;

  bool use_feedback_{false};
  bool feedback_use_odometry_{true};
  bool stop_on_feedback_timeout_{true};

  double control_rate_hz_{100.0};
  double feedback_timeout_sec_{0.5};
  double watchdog_timeout_sec_{0.5};

  double max_linear_x_{1.0};
  double max_linear_y_{1.0};
  double max_angular_z_{1.0};

  double kp_surge_{0.0};
  double kp_sway_{0.0};
  double kp_yaw_{0.0};

  double deadzone_pos_{0.04};
  double deadzone_neg_{0.04};
  std::vector<double> wheel_forward_gains_{};
  std::vector<double> wheel_reverse_gains_{};
  std::vector<double> wheel_offsets_{};

  double max_thrust_delta_per_sec_{2.0};
  std::vector<double> prev_wheel_cmds_{};
};

}  // namespace thruster_driver
}  // namespace njord
