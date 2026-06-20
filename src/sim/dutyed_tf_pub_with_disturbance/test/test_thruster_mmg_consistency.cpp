#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "dutyed_tf_pub_with_disturbance/mmg_doyle_model.hpp"
#include "dutyed_tf_pub_with_disturbance/thruster_wrench.hpp"
#include "thruster_driver/allocation.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<njord::thruster_driver::ThrusterGeometry> driverGeometry()
{
  return {
    {0.353553, -0.353553, kPi / 4.0, 1.0, false},
    {0.353553, 0.353553, -kPi / 4.0, 1.0, false},
    {-0.353553, -0.353553, 3.0 * kPi / 4.0, 1.0, false},
    {-0.353553, 0.353553, -3.0 * kPi / 4.0, 1.0, false}};
}

std::vector<njord::sim::SimThrusterGeometry> simulatorGeometry()
{
  return {
    {0.353553, -0.353553, kPi / 4.0},
    {0.353553, 0.353553, -kPi / 4.0},
    {-0.353553, -0.353553, 3.0 * kPi / 4.0},
    {-0.353553, 0.353553, -3.0 * kPi / 4.0}};
}

void expectMmgAxisMatchesAllocation(const std::array<double, 3> & requested)
{
  const auto commands = njord::thruster_driver::allocateWrench(
    driverGeometry(), {requested[0], requested[1], requested[2]}, 1e-9);
  const njord::sim::T200Model thrust_model(50.0, 50.0);
  const auto input = njord::sim::dutiesToPlanarInput(commands, simulatorGeometry(), thrust_model);

  const std::array<double, 3> actual = {
    input.surge_force, input.sway_force, input.yaw_moment};
  for (std::size_t axis = 0; axis < actual.size(); ++axis) {
    if (requested[axis] == 0.0) {
      EXPECT_NEAR(actual[axis], 0.0, 1e-8);
    } else {
      EXPECT_GT(actual[axis] * requested[axis], 0.0);
    }
  }

  njord::sim::DoyleParams params;
  params.R_0 = 0.0;
  params.Y_0 = 0.0;
  params.N_0 = 0.0;
  const njord::sim::MMGDoyleModel mmg(params);
  const auto accel = mmg.computeAccel({}, input);
  const std::array<double, 3> acceleration = {accel.du, accel.dv, accel.dr};
  for (std::size_t axis = 0; axis < acceleration.size(); ++axis) {
    if (requested[axis] != 0.0) {
      EXPECT_GT(acceleration[axis] * requested[axis], 0.0);
    }
  }
}
}  // namespace

TEST(ThrusterMmgConsistency, PreservesSurgeSwayAndYawAxes)
{
  expectMmgAxisMatchesAllocation({0.5, 0.0, 0.0});
  expectMmgAxisMatchesAllocation({0.0, 0.5, 0.0});
  expectMmgAxisMatchesAllocation({0.0, 0.0, 0.25});
}
