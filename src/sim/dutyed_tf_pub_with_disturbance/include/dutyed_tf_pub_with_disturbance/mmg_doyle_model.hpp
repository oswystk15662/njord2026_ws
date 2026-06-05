#ifndef DUTYED_TF_PUB_WITH_DISTURBANCE__MMG_DOYLE_MODEL_HPP_
#define DUTYED_TF_PUB_WITH_DISTURBANCE__MMG_DOYLE_MODEL_HPP_

namespace njord
{
namespace sim
{

struct PlanarState
{
  double u{0.0};
  double v{0.0};
  double r{0.0};
};

struct PlanarInput
{
  double surge_force{0.0};
  double sway_force{0.0};
  double yaw_moment{0.0};
};

struct PlanarAccel
{
  double du{0.0};
  double dv{0.0};
  double dr{0.0};
};

struct DoyleParams
{
  double mass_kg{50.0};
  double inertia_z{15.0};
  double lin_drag_u{20.0};
  double lin_drag_v{60.0};
  double lin_drag_r{20.0};
  double quad_drag_u{10.0};
  double quad_drag_v{90.0};
  double quad_drag_r{30.0};
  double cross_uv{0.0};
  double cross_ur{0.0};
  double cross_vr{0.0};
};

class MMGDoyleModel
{
public:
  explicit MMGDoyleModel(const DoyleParams & params);

  PlanarAccel computeAccel(const PlanarState & state, const PlanarInput & input) const;

private:
  DoyleParams params_;
};

}  // namespace sim
}  // namespace njord

#endif  // DUTYED_TF_PUB_WITH_DISTURBANCE__MMG_DOYLE_MODEL_HPP_
