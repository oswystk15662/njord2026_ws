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
  double Lpp{1.0};  // 船長
  double D{0.25}; // 喫水
  double mass_kg{10.0}; // 全質量
  double I_zG{1.5};  // Z軸回りの慣性モーメント
  double x_G{0.0}; // 浮心長さ方向位置
  // 付加質量
  double m_x{1.0};
  double m_y{10.0};
  double J_z{1.5};
  // 微係数
  double R_0{-0.2};
  double Y_0{0.001};
  double Y_v{-0.2};
  double Y_r{0.02};
  double N_0{0.001};
  double N_v{-0.2};
  double N_r{-0.02};
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
