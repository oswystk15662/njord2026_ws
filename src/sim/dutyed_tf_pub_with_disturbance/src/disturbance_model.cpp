#include "dutyed_tf_pub_with_disturbance/disturbance_model.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace njord
{
namespace sim
{

class DisturbanceModel::Impl
{
public:
  explicit Impl(std::uint32_t seed)
  : mt(seed), normal(0.0, 1.0)
  {
  }

  std::mt19937 mt;
  std::normal_distribution<double> normal;
};

DisturbanceModel::DisturbanceModel(
  double natural_frequency_hz,
  double damping_ratio,
  double sigma_u,
  double sigma_v,
  double sigma_r,
  std::uint32_t seed)
: wn_(6.283185307179586 * std::max(1e-3, natural_frequency_hz)),
  zeta_(std::max(0.05, damping_ratio)),
  impl_(std::make_unique<Impl>(seed))
{
  u_.sigma = std::max(0.0, sigma_u);
  v_.sigma = std::max(0.0, sigma_v);
  r_.sigma = std::max(0.0, sigma_r);
}

DisturbanceModel::~DisturbanceModel() = default;

double DisturbanceModel::stepAxis(double dt_sec, AxisState & axis)
{
  const double noise = axis.sigma * impl_->normal(impl_->mt);
  const double acc = -2.0 * zeta_ * wn_ * axis.rate - (wn_ * wn_) * axis.value + noise;
  axis.rate += acc * dt_sec;
  axis.value += axis.rate * dt_sec;
  return axis.value;
}

DisturbanceAccel DisturbanceModel::step(double dt_sec)
{
  const double dt = std::clamp(dt_sec, 1e-4, 0.1);
  DisturbanceAccel out;
  out.du = stepAxis(dt, u_);
  out.dv = stepAxis(dt, v_);
  out.dr = stepAxis(dt, r_);
  return out;
}

}  // namespace sim
}  // namespace njord
