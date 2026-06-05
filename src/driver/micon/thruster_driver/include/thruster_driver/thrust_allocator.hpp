#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace njord
{
namespace thruster_driver
{

class ThrustAllocator
{
public:
  struct Config
  {
    double thruster_spacing{0.5};
    double thrust_coeff{1.0};
    double max_force{1.0};
  };

  explicit ThrustAllocator(const Config & config);
  explicit ThrustAllocator(rclcpp::Node & node);

  std::vector<double> allocate(const std::vector<double> & tau_cmd);
  static std::vector<double> clamp_force(const std::vector<double> & force);

private:
  Config config_;

  double clamp(double value, double min_val, double max_val) const;
};

}  // namespace thruster_driver
}  // namespace njord
