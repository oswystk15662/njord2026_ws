#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <memory>

#include "thruster_driver/hardware_pwm.hpp"
#include "thruster_driver/mmg_dynamics.hpp"
#include "thruster_driver/thrust_allocator.hpp"
#include "thruster_driver/velocity_controller.hpp"

namespace njord
{
namespace thruster_driver
{

class ThrusterDif2WheelNode : public rclcpp::Node
{
public:
  ThrusterDif2WheelNode();
  ~ThrusterDif2WheelNode() override;

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void feedback_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_timer_callback();

  double clamp(double value, double min_val, double max_val) const;

  std::unique_ptr<HardwarePWM> left_motor_;
  std::unique_ptr<HardwarePWM> right_motor_;

  std::unique_ptr<VelocityController> velocity_controller_;
  std::unique_ptr<ThrustAllocator> thrust_allocator_;
  std::unique_ptr<MmgDynamics> mmg_model_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_feedback_odometry_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_current_force_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_feedback_time_;
  rclcpp::Time last_control_time_;

  bool use_feedback_{false};
  bool feedback_use_odometry_{true};
  bool stop_on_feedback_timeout_{true};

  double control_rate_hz_{50.0};
  double feedback_timeout_sec_{0.5};
  double watchdog_timeout_sec_{0.5};

  double max_linear_x_{1.0};
  double max_angular_z_{1.0};
  double angular_coeff_{0.5};

  double ref_linear_x_{0.0};
  double ref_angular_z_{0.0};
  double meas_linear_x_{0.0};
  double meas_angular_z_{0.0};
};

}  // namespace thruster_driver
}  // namespace njord
