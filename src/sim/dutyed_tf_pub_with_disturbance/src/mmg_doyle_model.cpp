#include "dutyed_tf_pub_with_disturbance/mmg_doyle_model.hpp"

#include <algorithm>
#include <cmath>

namespace njord
{
namespace sim
{

MMGDoyleModel::MMGDoyleModel(const DoyleParams & params)
: params_(params)
{
  params_.mass_kg = std::max(1e-3, params_.mass_kg);
  params_.inertia_z = std::max(1e-3, params_.inertia_z);
}

PlanarAccel MMGDoyleModel::computeAccel(const PlanarState & state, const PlanarInput & input) const
{
  const double u = state.u;
  const double v = state.v;
  const double r = state.r;

  const double xu = params_.lin_drag_u * u + params_.quad_drag_u * std::abs(u) * u;
  const double yv = params_.lin_drag_v * v + params_.quad_drag_v * std::abs(v) * v;
  const double nr = params_.lin_drag_r * r + params_.quad_drag_r * std::abs(r) * r;

  const double coupling_u = params_.mass_kg * v * r + params_.cross_uv * v * std::abs(v);
  const double coupling_v = -params_.mass_kg * u * r;
  const double coupling_r = -params_.cross_ur * u * r - params_.cross_vr * v * r;

  PlanarAccel out;
  out.du = (input.surge_force - xu + coupling_u) / params_.mass_kg;
  out.dv = (0.0 - yv + coupling_v) / params_.mass_kg;
  out.dr = (input.yaw_moment - nr + coupling_r) / params_.inertia_z;
  return out;
}

}  // namespace sim
}  // namespace njord
