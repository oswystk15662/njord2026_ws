#ifndef THRUSTER_DRIVER_PID_HPP_
#define THRUSTER_DRIVER_PID_HPP_

#include <algorithm>
#include <cmath>

namespace njord
{
namespace thruster_driver
{

struct PidAxisState
{
  double integral{0.0};
  double previous_error{0.0};
  bool have_previous_error{false};

  void reset()
  {
    integral = 0.0;
    previous_error = 0.0;
    have_previous_error = false;
  }
};

inline double pidStep(
  double kp, double ki, double kd, double error, double dt, double max_output,
  PidAxisState & state)
{
  const double derivative = state.have_previous_error ?
    (error - state.previous_error) / dt : 0.0;
  const double output_without_i = kp * error + kd * derivative;
  const double candidate_integral = state.integral + error * dt;
  const double candidate_output = output_without_i + ki * candidate_integral;

  // Conditional integration prevents windup while the wrench is saturated.
  if (std::abs(candidate_output) <= max_output) {
    state.integral = candidate_integral;
  }
  state.previous_error = error;
  state.have_previous_error = true;
  return std::clamp(output_without_i + ki * state.integral, -max_output, max_output);
}

}  // namespace thruster_driver
}  // namespace njord

#endif  // THRUSTER_DRIVER_PID_HPP_
