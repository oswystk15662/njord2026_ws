#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace njord
{
namespace thruster_driver
{

class MmgDynamics
{
public:
  struct State
  {
    double u{0.0};
    double v{0.0};
    double r{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Parameters
  {
    double mass{3.1};
    double iz{5.0};

    double added_mass_x{0.0};
    double added_mass_y{0.0};
    double added_inertia_jz{0.0};
    double x_g{0.0};

    double x_u{6.0};
    double y_v{8.0};
    double n_r{4.0};

    double x_uu{3.0};
    double y_vv{4.0};
    double n_rr{2.5};

    double thrust_coeff{1.0};
    double thruster_spacing{0.5};
  };

  explicit MmgDynamics(const Parameters & params);
  explicit MmgDynamics(rclcpp::Node & node);

  void step(double duty_left, double duty_right, double dt);

  const State & state() const { return state_; }
  std::vector<double> get_last_hydro_forces() const;
  void reset();

private:
  State state_;
  Parameters params_;

  double x_hydro_{0.0};
  double y_hydro_{0.0};
  double n_hydro_{0.0};

  double clamp(double value, double min_val, double max_val) const;
  double wrap_angle(double angle) const;
};

}  // namespace thruster_driver
}  // namespace njord
