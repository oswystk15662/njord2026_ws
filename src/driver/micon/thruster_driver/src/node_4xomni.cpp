#include "thruster_driver/node_4xomni.hpp"

#include "thruster_driver/mmg_model_4xomni.hpp"

#include <algorithm>
#include <stdexcept>

namespace njord
{
namespace thruster_driver
{

Thruster4xOmniNode::Thruster4xOmniNode()
: Node("thruster_driver_4xomni_node")
{
  const std::string chip_path = this->declare_parameter("hardware.gpio_chip", std::string("/dev/gpiochip4"));
  const double pwm_frequency = this->declare_parameter("hardware.pwm_frequency", 50.0);

  const std::vector<int64_t> dir_pins_i64 =
    this->declare_parameter("hardware.wheels.dir_pins", std::vector<int64_t>{26, 16});
  const std::vector<int64_t> pwm_pins_i64 =
    this->declare_parameter("hardware.wheels.pwm_pins", std::vector<int64_t>{13, 12});

  if (dir_pins_i64.size() != pwm_pins_i64.size() || dir_pins_i64.empty()) {
    throw std::runtime_error("hardware.wheels.dir_pins and pwm_pins must have the same non-zero length");
  }

  std::vector<int> dir_pins(dir_pins_i64.begin(), dir_pins_i64.end());
  std::vector<int> pwm_pins(pwm_pins_i64.begin(), pwm_pins_i64.end());

  motors_.reserve(dir_pins.size());
  for (std::size_t i = 0; i < dir_pins.size(); ++i) {
    motors_.push_back(std::make_unique<SoftPwmMotor>(chip_path, dir_pins[i], pwm_pins[i], pwm_frequency));
  }

  use_feedback_ = this->declare_parameter("control.use_feedback", false);
  feedback_use_odometry_ = this->declare_parameter("control.feedback_use_odometry", true);
  stop_on_feedback_timeout_ = this->declare_parameter("control.stop_on_feedback_timeout", true);
  control_rate_hz_ = this->declare_parameter("control.rate_hz", 100.0);
  feedback_timeout_sec_ = this->declare_parameter("control.feedback_timeout_sec", 0.5);
  watchdog_timeout_sec_ = this->declare_parameter("safety.watchdog_timeout_sec", 0.5);

  max_linear_x_ = this->declare_parameter("input_scaling.max_linear_x", 1.0);
  max_linear_y_ = this->declare_parameter("input_scaling.max_linear_y", 1.0);
  max_angular_z_ = this->declare_parameter("input_scaling.max_angular_z", 1.0);

  kp_surge_ = this->declare_parameter("control.feedback_kp.surge", 0.0);
  kp_sway_ = this->declare_parameter("control.feedback_kp.sway", 0.0);
  kp_yaw_ = this->declare_parameter("control.feedback_kp.yaw", 0.0);

  deadzone_pos_ = this->declare_parameter("static_map.deadzone_pos", 0.04);
  deadzone_neg_ = this->declare_parameter("static_map.deadzone_neg", 0.04);

  wheel_forward_gains_ = this->declare_parameter(
    "static_map.wheels.forward_gain", std::vector<double>(motors_.size(), 1.0));
  wheel_reverse_gains_ = this->declare_parameter(
    "static_map.wheels.reverse_gain", std::vector<double>(motors_.size(), 1.0));
  wheel_offsets_ = this->declare_parameter(
    "static_map.wheels.offset", std::vector<double>(motors_.size(), 0.0));

  if (wheel_forward_gains_.size() != motors_.size() ||
      wheel_reverse_gains_.size() != motors_.size() ||
      wheel_offsets_.size() != motors_.size()) {
    throw std::runtime_error("static_map.wheels.* vector size must match wheel count");
  }

  max_thrust_delta_per_sec_ = this->declare_parameter("safety.max_thrust_delta_per_sec", 2.0);

  dynamics_model_ = std::make_unique<MmgOmniModel>(*this);
  prev_wheel_cmds_.assign(motors_.size(), 0.0);

  const std::string cmd_vel_topic = this->declare_parameter("topics.cmd_vel", std::string("cmd_vel"));
  const std::string feedback_twist_topic =
    this->declare_parameter("topics.feedback_twist", std::string("feedback_twist"));
  const std::string feedback_odometry_topic =
    this->declare_parameter("topics.feedback_odometry", std::string("/localization/odometry/filtered"));

  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10,
    std::bind(&Thruster4xOmniNode::cmd_vel_callback, this, std::placeholders::_1));

  if (feedback_use_odometry_) {
    sub_feedback_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      feedback_odometry_topic, 10,
      std::bind(&Thruster4xOmniNode::feedback_odometry_callback, this, std::placeholders::_1));
  } else {
    sub_feedback_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      feedback_twist_topic, 10,
      std::bind(&Thruster4xOmniNode::feedback_twist_callback, this, std::placeholders::_1));
  }

  pub_current_duty_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/debug/current_duty", 10);
  pub_model_state_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/debug/model_state", 10);

  last_cmd_time_ = this->now();
  last_feedback_time_ = this->now();
  last_control_time_ = this->now();

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Thruster4xOmniNode::control_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "thruster_driver_4xomni ready. wheels=%zu model=%s feedback=%s",
    motors_.size(), dynamics_model_->name().c_str(), use_feedback_ ? "on" : "off");
}

void Thruster4xOmniNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  ref_linear_x_ = msg->linear.x;
  ref_linear_y_ = msg->linear.y;
  ref_angular_z_ = msg->angular.z;
  last_cmd_time_ = this->now();
}

void Thruster4xOmniNode::feedback_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  feedback_linear_x_ = msg->twist.linear.x;
  feedback_linear_y_ = msg->twist.linear.y;
  feedback_angular_z_ = msg->twist.angular.z;
  last_feedback_time_ = this->now();
}

void Thruster4xOmniNode::feedback_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  feedback_linear_x_ = msg->twist.twist.linear.x;
  feedback_linear_y_ = msg->twist.twist.linear.y;
  feedback_angular_z_ = msg->twist.twist.angular.z;
  last_feedback_time_ = this->now();
}

void Thruster4xOmniNode::control_timer_callback()
{
  const auto now = this->now();
  const double dt = std::max(1e-3, (now - last_control_time_).seconds());
  last_control_time_ = now;

  const bool cmd_timeout = (now - last_cmd_time_).seconds() > watchdog_timeout_sec_;
  const bool feedback_timeout = (now - last_feedback_time_).seconds() > feedback_timeout_sec_;

  double surge_cmd = 0.0;
  double sway_cmd = 0.0;
  double yaw_cmd = 0.0;

  if (!cmd_timeout) {
    surge_cmd = clamp(ref_linear_x_ / std::max(1e-6, max_linear_x_), -1.0, 1.0);
    sway_cmd = clamp(ref_linear_y_ / std::max(1e-6, max_linear_y_), -1.0, 1.0);
    yaw_cmd = clamp(ref_angular_z_ / std::max(1e-6, max_angular_z_), -1.0, 1.0);
  }

  bool force_reset = cmd_timeout;

  if (use_feedback_) {
    if (feedback_timeout && stop_on_feedback_timeout_) {
      surge_cmd = 0.0;
      sway_cmd = 0.0;
      yaw_cmd = 0.0;
      force_reset = true;
    } else {
      const double meas_surge = clamp(feedback_linear_x_ / std::max(1e-6, max_linear_x_), -1.0, 1.0);
      const double meas_sway = clamp(feedback_linear_y_ / std::max(1e-6, max_linear_y_), -1.0, 1.0);
      const double meas_yaw = clamp(feedback_angular_z_ / std::max(1e-6, max_angular_z_), -1.0, 1.0);

      surge_cmd = clamp(surge_cmd + kp_surge_ * (surge_cmd - meas_surge), -1.0, 1.0);
      sway_cmd = clamp(sway_cmd + kp_sway_ * (sway_cmd - meas_sway), -1.0, 1.0);
      yaw_cmd = clamp(yaw_cmd + kp_yaw_ * (yaw_cmd - meas_yaw), -1.0, 1.0);
    }
  }

  ModelInput model_input;
  model_input.surge_cmd = surge_cmd;
  model_input.sway_cmd = sway_cmd;
  model_input.yaw_cmd = yaw_cmd;
  model_input.dt = dt;
  model_input.reset_requested = force_reset;

  const ModelOutput model_out = dynamics_model_->step(model_input);

  std::vector<double> duties = model_out.wheel_duties;
  if (duties.size() != motors_.size()) {
    duties.assign(motors_.size(), 0.0);
  }

  const double max_delta = std::max(0.0, max_thrust_delta_per_sec_) * dt;
  for (std::size_t i = 0; i < duties.size(); ++i) {
    double cmd = apply_static_map(duties[i], i);
    cmd = prev_wheel_cmds_[i] + clamp(cmd - prev_wheel_cmds_[i], -max_delta, max_delta);
    prev_wheel_cmds_[i] = cmd;
    motors_[i]->set_speed(cmd);
    duties[i] = cmd;
  }

  std_msgs::msg::Float32MultiArray duty_msg;
  duty_msg.data.reserve(duties.size());
  for (double d : duties) {
    duty_msg.data.push_back(static_cast<float>(d));
  }
  pub_current_duty_->publish(duty_msg);

  std_msgs::msg::Float32MultiArray state_msg;
  state_msg.data = {
    static_cast<float>(model_out.state.x),
    static_cast<float>(model_out.state.y),
    static_cast<float>(model_out.state.yaw),
    static_cast<float>(model_out.state.x_dot),
    static_cast<float>(model_out.state.y_dot),
    static_cast<float>(model_out.state.yaw_dot)};
  pub_model_state_->publish(state_msg);
}

double Thruster4xOmniNode::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

double Thruster4xOmniNode::apply_deadzone(double value, double deadzone) const
{
  const double dz = std::max(0.0, deadzone);
  const double abs_value = std::abs(value);
  if (abs_value <= dz) {
    return 0.0;
  }
  return std::copysign(clamp(abs_value, 0.0, 1.0), value);
}

double Thruster4xOmniNode::apply_static_map(double value, std::size_t wheel_index) const
{
  double mapped = value;
  if (mapped >= 0.0) {
    mapped = apply_deadzone(mapped, deadzone_pos_);
    mapped *= wheel_forward_gains_[wheel_index];
  } else {
    mapped = apply_deadzone(mapped, deadzone_neg_);
    mapped *= wheel_reverse_gains_[wheel_index];
  }

  mapped += wheel_offsets_[wheel_index];
  return clamp(mapped, -1.0, 1.0);
}

}  // namespace thruster_driver
}  // namespace njord
