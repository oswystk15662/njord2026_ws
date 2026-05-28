#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace njord
{
namespace thruster_driver
{

class VelocityController
{
public:
  struct PIDGains
  {
    double surge_kp{1.0};
    double surge_ki{0.0};
    double surge_kd{0.0};
    double yaw_kp{1.0};
    double yaw_ki{0.0};
    double yaw_kd{0.0};
  };

  struct DOBConfig
  {
    double observer_gain{1.0};
    double filter_tau{0.1};
    bool enable{false};
    bool use_nominal_model{false};
  };

  struct NominalMMGModel
  {
    double mass{3.1};
    double iz_eff{0.2};
    double x_u{0.0};
    double y_v{0.0};
    double n_r{0.0};
    double x_uu{0.0};
    double y_vv{0.0};
    double n_rr{0.0};
  };

  explicit VelocityController(const PIDGains & gains);
  explicit VelocityController(const PIDGains & gains, const DOBConfig & dob_config);

  std::vector<double> compute(double u_ref, double r_ref, double u_meas, double r_meas, double dt);
  void reset();
  void enable_nominal_dob(const NominalMMGModel & nominal_model);
  void set_disturbance_estimate(double d_surge, double d_yaw);
  std::pair<double, double> get_disturbance_estimate() const;

private:
  PIDGains gains_;
  DOBConfig dob_config_;
  NominalMMGModel nominal_model_;

  double surge_integral_{0.0};
  double yaw_integral_{0.0};
  double prev_surge_error_{0.0};
  double prev_yaw_error_{0.0};
  double surge_d_state_{0.0};
  double yaw_d_state_{0.0};

  double d_surge_hat_{0.0};
  double d_yaw_hat_{0.0};
  double d_surge_lpf_{0.0};
  double d_yaw_lpf_{0.0};

  double prev_u_meas_{0.0};
  double prev_r_meas_{0.0};
  double prev_x_cmd_{0.0};
  double prev_n_cmd_{0.0};

  double compute_surge_damping(double u) const;
  double compute_yaw_damping(double r) const;

  double clamp(double value, double min_val, double max_val) const;
};

}  // namespace thruster_driver
}  // namespace njord
