#ifndef DUTYED_TF_PUB_WITH_DISTURBANCE__T200_MODEL_HPP_
#define DUTYED_TF_PUB_WITH_DISTURBANCE__T200_MODEL_HPP_

namespace njord
{
namespace sim
{

class T200Model
{
public:
  T200Model(double max_forward_newton, double max_reverse_newton);

  double forceFromDuty(double duty_normalized) const;

private:
  double max_forward_newton_;
  double max_reverse_newton_;
};

}  // namespace sim
}  // namespace njord

#endif  // DUTYED_TF_PUB_WITH_DISTURBANCE__T200_MODEL_HPP_
