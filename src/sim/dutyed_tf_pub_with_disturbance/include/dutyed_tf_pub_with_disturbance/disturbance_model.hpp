#ifndef DUTYED_TF_PUB_WITH_DISTURBANCE__DISTURBANCE_MODEL_HPP_
#define DUTYED_TF_PUB_WITH_DISTURBANCE__DISTURBANCE_MODEL_HPP_

#include <cstdint>
#include <memory>

namespace njord
{
namespace sim
{

struct DisturbanceAccel
{
  double du{0.0};
  double dv{0.0};
  double dr{0.0};
};

class DisturbanceModel
{
public:
  DisturbanceModel(
    double natural_frequency_hz,
    double damping_ratio,
    double sigma_u,
    double sigma_v,
    double sigma_r,
    std::uint32_t seed);

  ~DisturbanceModel();

  DisturbanceAccel step(double dt_sec);

  void setMaxMagnitude(double max_u, double max_v, double max_r)
  {
    max_u_ = max_u;
    max_v_ = max_v;
    max_r_ = max_r;
  }

private:
  struct AxisState
  {
    double value{0.0};
    double rate{0.0};
    double sigma{0.0};
  };

  double wn_;
  double zeta_;
  AxisState u_;
  AxisState v_;
  AxisState r_;
  double max_u_{-1.0};
  double max_v_{-1.0};
  double max_r_{-1.0};

  class Impl;
  std::unique_ptr<Impl> impl_;

  double stepAxis(double dt_sec, AxisState & axis);
};

}  // namespace sim
}  // namespace njord

#endif  // DUTYED_TF_PUB_WITH_DISTURBANCE__DISTURBANCE_MODEL_HPP_
