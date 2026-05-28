#include "thruster_driver/thrust_allocator.hpp"

#include <algorithm>
#include <cmath>

namespace njord
{
namespace thruster_driver
{

ThrustAllocator::ThrustAllocator(const Config & config)
: config_(config)
{
}

ThrustAllocator::ThrustAllocator(rclcpp::Node & node)
{
  config_.thruster_spacing = node.declare_parameter("control.mmg_2wheeled.thruster_spacing", 0.5);
  config_.thrust_coeff = node.declare_parameter("control.mmg_2wheeled.thrust_coeff", 1.0);
  config_.max_duty = node.declare_parameter("control.safety.max_duty", 1.0);
}

std::vector<double> ThrustAllocator::allocate(const std::vector<double> & tau_cmd)
{
  if (tau_cmd.size() < 2) {
    return {0.0, 0.0};
  }

  const double x_cmd = tau_cmd[0];
  const double n_cmd = tau_cmd.size() >= 3 ? tau_cmd[2] : (tau_cmd.size() > 1 ? tau_cmd[1] : 0.0);

  const double spacing_inv = std::max(1e-6, config_.thruster_spacing);
  const double f_left = x_cmd - n_cmd * 2.0 / spacing_inv;
  const double f_right = x_cmd + n_cmd * 2.0 / spacing_inv;

  auto force_to_duty = [this](double force) -> double {
    if (std::abs(force) < 1e-6) {
      return 0.0;
    }
    const double sign = force > 0.0 ? 1.0 : -1.0;
    const double abs_force = std::abs(force);
    const double abs_duty = std::sqrt(abs_force / std::max(1e-6, config_.thrust_coeff));
    return clamp(abs_duty * sign, -config_.max_duty, config_.max_duty);
  };

  return {force_to_duty(f_left), force_to_duty(f_right)};
}

std::vector<double> ThrustAllocator::clamp_duty(const std::vector<double> & duty)
{
  std::vector<double> clamped = duty;
  for (auto & d : clamped) {
    d = std::max(-1.0, std::min(d, 1.0));
  }
  return clamped;
}

double ThrustAllocator::clamp(double value, double min_val, double max_val) const
{
  return std::max(min_val, std::min(value, max_val));
}

}  // namespace thruster_driver
}  // namespace njord
