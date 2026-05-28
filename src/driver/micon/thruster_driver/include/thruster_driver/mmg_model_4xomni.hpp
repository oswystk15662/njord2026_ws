#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "thruster_driver/dynamics_model_4xomni.hpp"

namespace njord
{
namespace thruster_driver
{

class MmgOmniModel : public DynamicsModel
{
public:
  explicit MmgOmniModel(rclcpp::Node & node);

  ModelOutput step(const ModelInput & input) override;
  void reset() override;
  const VesselState & state() const override;
  std::string name() const override;

private:
  double clamp(double value, double min_value, double max_value) const;
  double wrap_angle(double angle) const;
  std::vector<double> solve_regularized_least_squares(
    const std::vector<std::vector<double>> & a,
    const std::vector<double> & b,
    double lambda) const;

  VesselState state_{};

  double u_{0.0};
  double v_{0.0};
  double r_{0.0};

  double mass_{15.0};
  double iz_{5.0};

  double x_u_{6.0};
  double y_v_{8.0};
  double n_r_{4.0};
  double x_uu_{3.0};
  double y_vv_{4.0};
  double n_rr_{2.5};

  double surge_ref_gain_{1.2};
  double sway_ref_gain_{1.2};
  double yaw_ref_gain_{1.5};
  double surge_p_{14.0};
  double sway_p_{12.0};
  double yaw_p_{7.0};

  double regularization_lambda_{1e-4};

  std::vector<double> wheel_pos_x_{};
  std::vector<double> wheel_pos_y_{};
  std::vector<double> wheel_angles_rad_{};
  std::vector<double> wheel_force_per_duty_{};
};

}  // namespace thruster_driver
}  // namespace njord
