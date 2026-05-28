#include "thruster_driver/mmg_model_4xomni.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace njord
{
namespace thruster_driver
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

MmgOmniModel::MmgOmniModel(rclcpp::Node & node)
{
  mass_ = node.declare_parameter("control.omni.mass", 15.0);
  iz_ = node.declare_parameter("control.omni.iz", 5.0);

  x_u_ = node.declare_parameter("control.omni.damping.linear.surge", 6.0);
  y_v_ = node.declare_parameter("control.omni.damping.linear.sway", 8.0);
  n_r_ = node.declare_parameter("control.omni.damping.linear.yaw", 4.0);
  x_uu_ = node.declare_parameter("control.omni.damping.quadratic.surge", 3.0);
  y_vv_ = node.declare_parameter("control.omni.damping.quadratic.sway", 4.0);
  n_rr_ = node.declare_parameter("control.omni.damping.quadratic.yaw", 2.5);

  surge_ref_gain_ = node.declare_parameter("control.omni.reference.surge_gain", 1.2);
  sway_ref_gain_ = node.declare_parameter("control.omni.reference.sway_gain", 1.2);
  yaw_ref_gain_ = node.declare_parameter("control.omni.reference.yaw_gain", 1.5);
  surge_p_ = node.declare_parameter("control.omni.control.surge_p", 14.0);
  sway_p_ = node.declare_parameter("control.omni.control.sway_p", 12.0);
  yaw_p_ = node.declare_parameter("control.omni.control.yaw_p", 7.0);

  regularization_lambda_ = std::max(
    1e-9, node.declare_parameter("control.omni.allocation.regularization_lambda", 1e-4));

  wheel_pos_x_ = node.declare_parameter("control.omni.wheels.pos_x", std::vector<double>{});
  wheel_pos_y_ = node.declare_parameter("control.omni.wheels.pos_y", std::vector<double>{});
  wheel_angles_rad_ = node.declare_parameter("control.omni.wheels.angle_rad", std::vector<double>{});
  wheel_force_per_duty_ = node.declare_parameter(
    "control.omni.wheels.force_per_duty", std::vector<double>{});

  const std::size_t n = wheel_angles_rad_.size();
  if (n < 2) {
    throw std::runtime_error("control.omni.wheels.angle_rad must contain at least 2 wheels");
  }
  if (wheel_pos_x_.size() != n || wheel_pos_y_.size() != n) {
    throw std::runtime_error("control.omni.wheels.pos_x/pos_y size must match angle_rad size");
  }
  if (wheel_force_per_duty_.empty()) {
    wheel_force_per_duty_.assign(n, 1.0);
  }
  if (wheel_force_per_duty_.size() != n) {
    throw std::runtime_error("control.omni.wheels.force_per_duty size must match angle_rad size");
  }
}

ModelOutput MmgOmniModel::step(const ModelInput & input)
{
  if (input.reset_requested) {
    reset();
    return ModelOutput{};
  }

  const double dt = std::max(1e-4, input.dt);
  const double u_ref = clamp(input.surge_cmd, -1.0, 1.0) * surge_ref_gain_;
  const double v_ref = clamp(input.sway_cmd, -1.0, 1.0) * sway_ref_gain_;
  const double r_ref = clamp(input.yaw_cmd, -1.0, 1.0) * yaw_ref_gain_;

  const double x_hydro = -x_u_ * u_ - x_uu_ * std::abs(u_) * u_;
  const double y_hydro = -y_v_ * v_ - y_vv_ * std::abs(v_) * v_;
  const double n_hydro = -n_r_ * r_ - n_rr_ * std::abs(r_) * r_;

  const double x_force = surge_p_ * (u_ref - u_) + x_hydro;
  const double y_force = sway_p_ * (v_ref - v_) + y_hydro;
  const double n_moment = yaw_p_ * (r_ref - r_) + n_hydro;

  const double u_dot = (x_force + mass_ * v_ * r_) / std::max(1e-6, mass_);
  const double v_dot = (y_force - mass_ * u_ * r_) / std::max(1e-6, mass_);
  const double r_dot = n_moment / std::max(1e-6, iz_);

  u_ += u_dot * dt;
  v_ += v_dot * dt;
  r_ += r_dot * dt;

  state_.yaw = wrap_angle(state_.yaw + r_ * dt);
  state_.x_dot = std::cos(state_.yaw) * u_ - std::sin(state_.yaw) * v_;
  state_.y_dot = std::sin(state_.yaw) * u_ + std::cos(state_.yaw) * v_;
  state_.yaw_dot = r_;
  state_.x += state_.x_dot * dt;
  state_.y += state_.y_dot * dt;

  const std::size_t n = wheel_angles_rad_.size();
  std::vector<std::vector<double>> a(3, std::vector<double>(n, 0.0));
  for (std::size_t i = 0; i < n; ++i) {
    const double ct = std::cos(wheel_angles_rad_[i]);
    const double st = std::sin(wheel_angles_rad_[i]);
    const double gain = wheel_force_per_duty_[i];
    a[0][i] = gain * ct;
    a[1][i] = gain * st;
    a[2][i] = gain * (wheel_pos_x_[i] * st - wheel_pos_y_[i] * ct);
  }

  const std::vector<double> b = {x_force, y_force, n_moment};
  std::vector<double> duties = solve_regularized_least_squares(a, b, regularization_lambda_);

  double max_abs = 0.0;
  for (double d : duties) {
    max_abs = std::max(max_abs, std::abs(d));
  }
  if (max_abs > 1.0) {
    for (double & d : duties) {
      d /= max_abs;
    }
  }
  for (double & d : duties) {
    d = clamp(d, -1.0, 1.0);
  }

  ModelOutput out;
  out.wheel_duties = duties;
  out.state = state_;
  return out;
}

void MmgOmniModel::reset()
{
  state_ = VesselState{};
  u_ = 0.0;
  v_ = 0.0;
  r_ = 0.0;
}

const VesselState & MmgOmniModel::state() const
{
  return state_;
}

std::string MmgOmniModel::name() const
{
  return "omni";
}

double MmgOmniModel::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

double MmgOmniModel::wrap_angle(double angle) const
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::vector<double> MmgOmniModel::solve_regularized_least_squares(
  const std::vector<std::vector<double>> & a,
  const std::vector<double> & b,
  double lambda) const
{
  const std::size_t m = a.size();
  if (m == 0) {
    return {};
  }
  const std::size_t n = a.front().size();

  std::vector<std::vector<double>> ata(n, std::vector<double>(n, 0.0));
  std::vector<double> atb(n, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      double sum = 0.0;
      for (std::size_t k = 0; k < m; ++k) {
        sum += a[k][i] * a[k][j];
      }
      ata[i][j] = sum;
    }
    ata[i][i] += lambda;

    double rhs = 0.0;
    for (std::size_t k = 0; k < m; ++k) {
      rhs += a[k][i] * b[k];
    }
    atb[i] = rhs;
  }

  for (std::size_t i = 0; i < n; ++i) {
    std::size_t pivot = i;
    double max_abs = std::abs(ata[i][i]);
    for (std::size_t r = i + 1; r < n; ++r) {
      const double candidate = std::abs(ata[r][i]);
      if (candidate > max_abs) {
        max_abs = candidate;
        pivot = r;
      }
    }
    if (pivot != i) {
      std::swap(ata[i], ata[pivot]);
      std::swap(atb[i], atb[pivot]);
    }

    const double diag = ata[i][i];
    if (std::abs(diag) < 1e-12) {
      continue;
    }

    for (std::size_t r = i + 1; r < n; ++r) {
      const double factor = ata[r][i] / diag;
      if (std::abs(factor) < 1e-15) {
        continue;
      }
      for (std::size_t c = i; c < n; ++c) {
        ata[r][c] -= factor * ata[i][c];
      }
      atb[r] -= factor * atb[i];
    }
  }

  std::vector<double> x(n, 0.0);
  for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
    double rhs = atb[static_cast<std::size_t>(i)];
    for (std::size_t c = static_cast<std::size_t>(i) + 1; c < n; ++c) {
      rhs -= ata[static_cast<std::size_t>(i)][c] * x[c];
    }
    const double diag = ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
    if (std::abs(diag) < 1e-12) {
      x[static_cast<std::size_t>(i)] = 0.0;
    } else {
      x[static_cast<std::size_t>(i)] = rhs / diag;
    }
  }

  return x;
}

}  // namespace thruster_driver
}  // namespace njord
