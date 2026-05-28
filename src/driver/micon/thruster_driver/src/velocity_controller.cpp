#include "thruster_driver/velocity_controller.hpp"

#include <algorithm>
#include <cmath>

namespace njord
{
namespace thruster_driver
{

VelocityController::VelocityController(const PIDGains & gains)
: gains_(gains), dob_config_({}), nominal_model_({})
{
}

VelocityController::VelocityController(const PIDGains & gains, const DOBConfig & dob_config)
: gains_(gains), dob_config_(dob_config), nominal_model_({})
{
}

std::vector<double> VelocityController::compute(
  double u_ref, double r_ref, double u_meas, double r_meas, double dt)
{
  const double dt_clamped = std::max(1e-4, dt);

  const double surge_error = u_ref - u_meas;
  const double yaw_error = r_ref - r_meas;

  surge_integral_ += surge_error * dt_clamped;
  yaw_integral_ += yaw_error * dt_clamped;

  surge_integral_ = clamp(surge_integral_, -1.0, 1.0);
  yaw_integral_ = clamp(yaw_integral_, -1.0, 1.0);

  double surge_pi = gains_.surge_kp * surge_error + gains_.surge_ki * surge_integral_;
  double yaw_pi = gains_.yaw_kp * yaw_error + gains_.yaw_ki * yaw_integral_;

  if (dob_config_.enable) {
    if (dob_config_.use_nominal_model) {
      const double u_meas_dot = (u_meas - prev_u_meas_) / dt_clamped;
      const double r_meas_dot = (r_meas - prev_r_meas_) / dt_clamped;

      const double surge_damping = compute_surge_damping(u_meas);
      const double yaw_damping = compute_yaw_damping(r_meas);

      d_surge_hat_ = nominal_model_.mass * u_meas_dot - prev_x_cmd_ + surge_damping;
      d_yaw_hat_ = nominal_model_.iz_eff * r_meas_dot - prev_n_cmd_ + yaw_damping;

      prev_u_meas_ = u_meas;
      prev_r_meas_ = r_meas;
    }

    const double lpf_alpha = dt_clamped / (dob_config_.filter_tau + dt_clamped);
    d_surge_lpf_ = d_surge_lpf_ + lpf_alpha * (d_surge_hat_ - d_surge_lpf_);
    d_yaw_lpf_ = d_yaw_lpf_ + lpf_alpha * (d_yaw_hat_ - d_yaw_lpf_);

    surge_pi += d_surge_lpf_ * dob_config_.observer_gain;
    yaw_pi += d_yaw_lpf_ * dob_config_.observer_gain;
  }

  double x_cmd = clamp(surge_pi, -1.0, 1.0);
  double y_cmd = 0.0;
  double n_cmd = clamp(yaw_pi, -1.0, 1.0);

  prev_x_cmd_ = x_cmd;
  prev_n_cmd_ = n_cmd;
  prev_surge_error_ = surge_error;
  prev_yaw_error_ = yaw_error;

  return {x_cmd, y_cmd, n_cmd};
}

void VelocityController::reset()
{
  surge_integral_ = 0.0;
  yaw_integral_ = 0.0;
  prev_surge_error_ = 0.0;
  prev_yaw_error_ = 0.0;
  surge_d_state_ = 0.0;
  yaw_d_state_ = 0.0;
  d_surge_hat_ = 0.0;
  d_yaw_hat_ = 0.0;
  d_surge_lpf_ = 0.0;
  d_yaw_lpf_ = 0.0;
  prev_u_meas_ = 0.0;
  prev_r_meas_ = 0.0;
  prev_x_cmd_ = 0.0;
  prev_n_cmd_ = 0.0;
}

void VelocityController::enable_nominal_dob(const NominalMMGModel & nominal_model)
{
  nominal_model_ = nominal_model;
  dob_config_.use_nominal_model = true;
  dob_config_.enable = true;
}

void VelocityController::set_disturbance_estimate(double d_surge, double d_yaw)
{
  d_surge_hat_ = d_surge;
  d_yaw_hat_ = d_yaw;
}

std::pair<double, double> VelocityController::get_disturbance_estimate() const
{
  return {d_surge_lpf_, d_yaw_lpf_};
}

double VelocityController::compute_surge_damping(double u) const
{
  return -(nominal_model_.x_u * u + nominal_model_.x_uu * std::abs(u) * u);
}

double VelocityController::compute_yaw_damping(double r) const
{
  return -(nominal_model_.n_r * r + nominal_model_.n_rr * std::abs(r) * r);
}

double VelocityController::clamp(double value, double min_val, double max_val) const
{
  return std::max(min_val, std::min(value, max_val));
}

}  // namespace thruster_driver
}  // namespace njord
