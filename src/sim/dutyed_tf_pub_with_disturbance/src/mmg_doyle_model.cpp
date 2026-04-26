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
  params_.I_zG = std::max(1e-3, params_.I_zG);
  params_.J_z = std::max(1e-3, params_.J_z);
}

PlanarAccel MMGDoyleModel::computeAccel(const PlanarState & state, const PlanarInput & input) const
{
  const double u = state.u;
  const double v = state.v;
  const double r = state.r;
  const double U = std::sqrt(u * u + v * v + 1e-6);
  const double v_dash = (v) / U;
  const double r_dash = (r * params_.Lpp) / U;

  const double X_hull = 0.5 * 1025 * params_.Lpp * params_.D * u * U * (params_.R_0);
  const double Y_hull = 0.5 * 1025 * params_.Lpp * params_.D * U * U * (params_.Y_0 + params_.Y_v  * v_dash + params_.Y_r  * r_dash);
  const double N_hull = 0.5 * 1025 * params_.Lpp * params_.Lpp * params_.D * U * U * (params_.N_0 + params_.N_v  * v_dash + params_.N_r  * r_dash);

  const double coupling_u = (params_.mass_kg + params_.m_y) * v * r + params_.x_G * params_.mass_kg * r * r;
  const double coupling_v = -(params_.mass_kg + params_.m_x) * u * r;

  PlanarAccel out;
  out.du = (input.surge_force + X_hull + coupling_u) 
           / (params_.mass_kg + params_.m_x);
  out.dv = ((params_.x_G * params_.x_G) * (params_.mass_kg * params_.mass_kg) * u * r  - (input.yaw_moment + N_hull) * params_.x_G * params_.mass_kg + (input.sway_force + Y_hull + coupling_v) * (params_.I_zG + params_.J_z + (params_.x_G * params_.x_G) * params_.mass_kg)) 
           / ((params_.I_zG + params_.J_z + (params_.x_G * params_.x_G) * params_.mass_kg) * (params_.mass_kg + params_.m_y) - (params_.x_G * params_.x_G) * (params_.mass_kg * params_.mass_kg));
  out.dr = ((input.yaw_moment + N_hull) - params_.x_G * params_.mass_kg * (out.dv + u * r)) 
           / (params_.I_zG + params_.J_z + (params_.x_G * params_.x_G) * params_.mass_kg);
  return out;
}

}  // namespace sim
}  // namespace njord
