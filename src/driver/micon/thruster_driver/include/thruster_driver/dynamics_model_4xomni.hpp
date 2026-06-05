#pragma once

#include <string>
#include <vector>

namespace njord
{
namespace thruster_driver
{

struct VesselState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double x_dot{0.0};
  double y_dot{0.0};
  double yaw_dot{0.0};
};

struct ModelInput
{
  double surge_cmd{0.0};
  double sway_cmd{0.0};
  double yaw_cmd{0.0};
  double dt{0.01};
  bool reset_requested{false};
};

struct ModelOutput
{
  std::vector<double> wheel_forces{};
  VesselState state{};
};

class DynamicsModel
{
public:
  virtual ~DynamicsModel() = default;

  virtual ModelOutput step(const ModelInput & input) = 0;
  virtual void reset() = 0;
  virtual const VesselState & state() const = 0;
  virtual std::string name() const = 0;
};

}  // namespace thruster_driver
}  // namespace njord
