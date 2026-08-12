#include <gtest/gtest.h>

#include "thruster_driver/pid.hpp"

TEST(ThrusterPid, IntegratesAndDoesNotWindUpAtTheWrenchLimit)
{
  njord::thruster_driver::PidAxisState state;
  EXPECT_DOUBLE_EQ(njord::thruster_driver::pidStep(0.0, 2.0, 0.0, 1.0, 0.5, 10.0, state), 1.0);
  EXPECT_DOUBLE_EQ(njord::thruster_driver::pidStep(0.0, 2.0, 0.0, 1.0, 0.5, 1.0, state), 1.0);
  EXPECT_DOUBLE_EQ(state.integral, 0.5);
}

TEST(ThrusterPid, DifferentiatesVelocityError)
{
  njord::thruster_driver::PidAxisState state;
  EXPECT_DOUBLE_EQ(njord::thruster_driver::pidStep(0.0, 0.0, 3.0, 1.0, 0.5, 10.0, state), 0.0);
  EXPECT_DOUBLE_EQ(njord::thruster_driver::pidStep(0.0, 0.0, 3.0, 2.0, 0.5, 10.0, state), 6.0);
}
