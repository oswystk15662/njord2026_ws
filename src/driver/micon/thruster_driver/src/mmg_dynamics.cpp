#include "thruster_driver/mmg_dynamics.hpp"

#include <algorithm>
#include <cmath>

namespace njord
{
namespace thruster_driver
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

MmgDynamics::MmgDynamics(const Parameters & params)
: params_(params)
{
}

MmgDynamics::MmgDynamics(rclcpp::Node & node)
{
  params_.mass = node.declare_parameter("control.mmg_2wheeled.mass", 3.1);
  params_.iz = node.declare_parameter("control.mmg_2wheeled.iz", 5.0);
  params_.added_mass_x = node.declare_parameter("control.mmg_2wheeled.added_mass_x", 0.0);
  params_.added_mass_y = node.declare_parameter("control.mmg_2wheeled.added_mass_y", 0.0);
  params_.added_inertia_jz = node.declare_parameter("control.mmg_2wheeled.added_inertia_jz", 0.0);
  params_.x_g = node.declare_parameter("control.mmg_2wheeled.x_g", 0.0);

  params_.x_u = node.declare_parameter("control.mmg_2wheeled.damping.linear.surge", 6.0);
  params_.y_v = node.declare_parameter("control.mmg_2wheeled.damping.linear.sway", 8.0);
  params_.n_r = node.declare_parameter("control.mmg_2wheeled.damping.linear.yaw", 4.0);
  params_.x_uu = node.declare_parameter("control.mmg_2wheeled.damping.quadratic.surge", 3.0);
  params_.y_vv = node.declare_parameter("control.mmg_2wheeled.damping.quadratic.sway", 4.0);
  params_.n_rr = node.declare_parameter("control.mmg_2wheeled.damping.quadratic.yaw", 2.5);

  params_.thrust_coeff = node.declare_parameter("control.mmg_2wheeled.thrust_coeff", 1.0);
  params_.thruster_spacing = node.declare_parameter("control.mmg_2wheeled.thruster_spacing", 0.5);
}

void MmgDynamics::step(double duty_left, double duty_right, double dt)
{
  const double dt_clamped = std::max(1e-4, dt);

  auto duty_to_force = [this](double duty) -> double {
    return params_.thrust_coeff * duty * std::abs(duty);
  };

  const double f_left = duty_to_force(clamp(duty_left, -1.0, 1.0));
  const double f_right = duty_to_force(clamp(duty_right, -1.0, 1.0));

  const double x_force = (f_left + f_right) / 2.0;
  const double n_moment = (f_right - f_left) * params_.thruster_spacing / 2.0;

  x_hydro_ = -params_.x_u * state_.u - params_.x_uu * std::abs(state_.u) * state_.u;
  y_hydro_ = -params_.y_v * state_.v - params_.y_vv * std::abs(state_.v) * state_.v;
  n_hydro_ = -params_.n_r * state_.r - params_.n_rr * std::abs(state_.r) * state_.r;

  const double x_total = x_force + x_hydro_;
  const double y_total = y_hydro_;
  const double n_total = n_moment + n_hydro_;

  const double Mx = std::max(1e-6, params_.mass + params_.added_mass_x);
  const double My = std::max(1e-6, params_.mass + params_.added_mass_y);
  const double Iz_eff = std::max(
    1e-6, params_.iz + params_.x_g * params_.x_g * params_.mass + params_.added_inertia_jz);

  const double bx = x_total + My * state_.v * state_.r + params_.x_g * params_.mass * state_.r * state_.r;
  const double by = y_total - Mx * state_.u * state_.r;
  const double byaw = n_total - params_.x_g * params_.mass * state_.u * state_.r;

  const double u_dot = bx / Mx;

  const double mm = params_.x_g * params_.mass;
  const double det = My * Iz_eff - mm * mm;
  double v_dot = 0.0;
  double r_dot = 0.0;

  if (std::abs(det) < 1e-9) {
    v_dot = by / My;
    r_dot = byaw / Iz_eff;
  } else {
    v_dot = (by * Iz_eff - mm * byaw) / det;
    r_dot = (My * byaw - mm * by) / det;
  }

  state_.u += u_dot * dt_clamped;
  state_.v += v_dot * dt_clamped;
  state_.r += r_dot * dt_clamped;

  state_.yaw = wrap_angle(state_.yaw + state_.r * dt_clamped);
  const double x_dot = std::cos(state_.yaw) * state_.u - std::sin(state_.yaw) * state_.v;
  const double y_dot = std::sin(state_.yaw) * state_.u + std::cos(state_.yaw) * state_.v;
  state_.x += x_dot * dt_clamped;
  state_.y += y_dot * dt_clamped;
}

std::vector<double> MmgDynamics::get_last_hydro_forces() const
{
  return {x_hydro_, y_hydro_, n_hydro_};
}

void MmgDynamics::reset()
{
  state_ = State{};
  x_hydro_ = 0.0;
  y_hydro_ = 0.0;
  n_hydro_ = 0.0;
}

double MmgDynamics::clamp(double value, double min_val, double max_val) const
{
  return std::max(min_val, std::min(value, max_val));
}

double MmgDynamics::wrap_angle(double angle) const
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace thruster_driver
}  // namespace njord
