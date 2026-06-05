#include "thruster_driver/node_dif2wheel.hpp"

#include <algorithm>
#include <chrono>

namespace njord
{
namespace thruster_driver
{

ThrusterDif2WheelNode::ThrusterDif2WheelNode()
: Node("node_dif2wheel"),
  last_cmd_time_(this->now()),
  last_feedback_time_(this->now()),
  last_control_time_(this->now())
{
  const std::string gpio_chip_path = this->declare_parameter("gpio_chip", "/dev/gpiochip4");
  const int left_dir_pin = this->declare_parameter("left_dir_pin", 26);
  const int left_pwm_pin = this->declare_parameter("left_pwm_pin", 13);
  const int right_dir_pin = this->declare_parameter("right_dir_pin", 16);
  const int right_pwm_pin = this->declare_parameter("right_pwm_pin", 12);
  const double pwm_frequency_hz = this->declare_parameter("pwm_frequency", 50.0);

  max_linear_x_ = this->declare_parameter("input_scaling.max_linear_x", 1.0);
  max_angular_z_ = this->declare_parameter("input_scaling.max_angular_z", 1.0);
  angular_coeff_ = this->declare_parameter("angular_coeff", 0.5);

  control_rate_hz_ = this->declare_parameter("control.rate_hz", 50.0);
  use_feedback_ = this->declare_parameter("control.use_feedback", false);
  feedback_use_odometry_ = this->declare_parameter("control.feedback_use_odometry", true);
  stop_on_feedback_timeout_ = this->declare_parameter("control.stop_on_feedback_timeout", true);
  feedback_timeout_sec_ = this->declare_parameter("control.feedback_timeout_sec", 0.5);
  watchdog_timeout_sec_ = this->declare_parameter("safety.watchdog_timeout_sec", 0.5);

  left_motor_ = std::make_unique<HardwarePWM>(
    gpio_chip_path, left_dir_pin, left_pwm_pin, pwm_frequency_hz);
  right_motor_ = std::make_unique<HardwarePWM>(
    gpio_chip_path, right_dir_pin, right_pwm_pin, pwm_frequency_hz);

  VelocityController::PIDGains pid_gains;
  pid_gains.surge_kp = this->declare_parameter("control.base.surge_kp", 0.5);
  pid_gains.surge_ki = this->declare_parameter("control.base.surge_ki", 0.0);
  pid_gains.yaw_kp = this->declare_parameter("control.base.yaw_kp", 0.5);
  pid_gains.yaw_ki = this->declare_parameter("control.base.yaw_ki", 0.0);

  VelocityController::DOBConfig dob_config;
  dob_config.enable = this->declare_parameter("control.dob.enable", false);
  dob_config.observer_gain = this->declare_parameter("control.dob.observer_gain", 1.0);
  dob_config.filter_tau = this->declare_parameter("control.dob.filter_tau_sec", 0.1);
  dob_config.use_nominal_model = this->declare_parameter("control.dob.use_nominal_model", true);

  velocity_controller_ = std::make_unique<VelocityController>(pid_gains, dob_config);

  if (dob_config.enable && dob_config.use_nominal_model) {
    VelocityController::NominalMMGModel nominal_mmg;
    nominal_mmg.mass = this->declare_parameter("control.mmg.mass", 3.1);
    nominal_mmg.iz_eff =
      this->declare_parameter("control.mmg.iz", 0.2) +
      this->declare_parameter("control.mmg.added_inertia_jz", 0.0);
    nominal_mmg.x_u = this->declare_parameter("control.mmg.damping.linear.surge", 0.0);
    nominal_mmg.y_v = this->declare_parameter("control.mmg.damping.linear.sway", 0.0);
    nominal_mmg.n_r = this->declare_parameter("control.mmg.damping.linear.yaw", 0.0);
    nominal_mmg.x_uu = this->declare_parameter("control.mmg.damping.quadratic.surge", 0.0);
    nominal_mmg.y_vv = this->declare_parameter("control.mmg.damping.quadratic.sway", 0.0);
    nominal_mmg.n_rr = this->declare_parameter("control.mmg.damping.quadratic.yaw", 0.5);

    velocity_controller_->enable_nominal_dob(nominal_mmg);
  }

  mmg_model_ = std::make_unique<MmgDynamics>(*this);

  ThrustAllocator::Config thrust_config;
  thrust_config.thruster_spacing =
    this->get_parameter("control.mmg_2wheeled.thruster_spacing").as_double();
  thrust_config.thrust_coeff =
    this->get_parameter("control.mmg_2wheeled.thrust_coeff").as_double();
  thrust_config.max_force = this->declare_parameter("control.safety.max_force", 1.0);
  thrust_allocator_ = std::make_unique<ThrustAllocator>(thrust_config);

  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_smoothed", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_callback(msg); });

  if (feedback_use_odometry_) {
    sub_feedback_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odometry/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        feedback_odometry_callback(msg);
      });
  }

  pub_current_force_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/debug/current_force", 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { control_timer_callback(); });

  RCLCPP_INFO(
    this->get_logger(),
    "ThrusterDif2WheelNode initialized: rate=%.1fHz, use_feedback=%s, watchdog=%.2fs",
    control_rate_hz_, use_feedback_ ? "true" : "false", watchdog_timeout_sec_);
}

ThrusterDif2WheelNode::~ThrusterDif2WheelNode()
{
  if (left_motor_) {
    left_motor_->emergency_stop();
  }
  if (right_motor_) {
    right_motor_->emergency_stop();
  }
}

void ThrusterDif2WheelNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  ref_linear_x_ = msg->linear.x;
  ref_angular_z_ = msg->angular.z;
  last_cmd_time_ = this->now();
}

void ThrusterDif2WheelNode::feedback_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  meas_linear_x_ = msg->twist.twist.linear.x;
  meas_angular_z_ = msg->twist.twist.angular.z;
  last_feedback_time_ = this->now();
}

void ThrusterDif2WheelNode::control_timer_callback()
{
  const auto now = this->now();
  const double dt = (now - last_control_time_).seconds();
  last_control_time_ = now;

  const bool cmd_timeout = (now - last_cmd_time_).seconds() > watchdog_timeout_sec_;
  const bool feedback_timeout = (now - last_feedback_time_).seconds() > feedback_timeout_sec_;

  double ref_surge = 0.0;
  double ref_yaw = 0.0;

  if (!cmd_timeout) {
    ref_surge = clamp(ref_linear_x_ / std::max(1e-6, max_linear_x_), -1.0, 1.0);
    ref_yaw = clamp(
      (angular_coeff_ * ref_angular_z_) / std::max(1e-6, max_angular_z_), -1.0, 1.0);
  }

  double meas_surge = 0.0;
  double meas_yaw = 0.0;

  if (use_feedback_) {
    if (feedback_timeout && stop_on_feedback_timeout_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Feedback timeout! Commanding zero.");
      ref_surge = 0.0;
      ref_yaw = 0.0;
      velocity_controller_->reset();
    } else {
      meas_surge = clamp(meas_linear_x_ / std::max(1e-6, max_linear_x_), -1.0, 1.0);
      meas_yaw = clamp(meas_angular_z_ / std::max(1e-6, max_angular_z_), -1.0, 1.0);
    }
  }

  const auto tau_cmd = velocity_controller_->compute(ref_surge, ref_yaw, meas_surge, meas_yaw, dt);

  auto force_cmd = thrust_allocator_->allocate(tau_cmd);
  force_cmd = ThrustAllocator::clamp_force(force_cmd);

  if (force_cmd.size() >= 2) {
    left_motor_->set_force(force_cmd[0]);
    right_motor_->set_force(force_cmd[1]);

    mmg_model_->step(force_cmd[0], force_cmd[1], dt);

    std_msgs::msg::Float32MultiArray force_msg;
    force_msg.data = {static_cast<float>(force_cmd[0]), static_cast<float>(force_cmd[1])};
    pub_current_force_->publish(force_msg);
  }
}

double ThrusterDif2WheelNode::clamp(double value, double min_val, double max_val) const
{
  return std::max(min_val, std::min(value, max_val));
}

}  // namespace thruster_driver
}  // namespace njord
