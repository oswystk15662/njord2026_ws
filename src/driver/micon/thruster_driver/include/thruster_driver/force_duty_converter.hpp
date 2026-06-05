#pragma once

#include <cstddef>
#include <cmath>
#include <vector>

namespace njord
{
namespace thruster_driver
{
class ForceDutyConverter
{
public:
  // force_per_duty: N per normalized-duty-unit (duty in [-1,1])
  explicit ForceDutyConverter(const std::vector<double> & force_per_duty = std::vector<double>{1.0})
  : force_per_duty_(force_per_duty)
  {}

  double forceToDuty(double force, std::size_t index = 0) const
  {
    const double coeff =
      (index < force_per_duty_.size()) ? force_per_duty_[index] :
      (force_per_duty_.empty() ? 1.0 : force_per_duty_.front());
    if (std::abs(coeff) < 1e-12) {
      return 0.0;
    }
    // duty = force / coeff => clamp to [-1,1]
    double duty = force / coeff;
    if (duty > 1.0) duty = 1.0;
    if (duty < -1.0) duty = -1.0;
    return duty;
  }

private:
  std::vector<double> force_per_duty_;
};

}  // namespace thruster_driver
}  // namespace njord
