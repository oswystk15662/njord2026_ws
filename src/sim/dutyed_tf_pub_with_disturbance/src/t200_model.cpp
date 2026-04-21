#include "dutyed_tf_pub_with_disturbance/t200_model.hpp"

#include <algorithm>
#include <cmath>

namespace njord
{
namespace sim
{

T200Model::T200Model(double max_forward_newton, double max_reverse_newton)
: max_forward_newton_(std::max(0.0, max_forward_newton)),
  max_reverse_newton_(std::max(0.0, max_reverse_newton))
{
}

double T200Model::forceFromDuty(double duty_normalized) const
{
  const double duty = std::clamp(duty_normalized, -1.0, 1.0);
  if (duty >= 0.0) {
    return max_forward_newton_ * duty * std::abs(duty);
  }
  return -max_reverse_newton_ * std::abs(duty) * std::abs(duty);
}

}  // namespace sim
}  // namespace njord
