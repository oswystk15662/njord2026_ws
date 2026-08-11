#ifndef SIMPLE_MANUAL__SBUS_JOY_CONVERTER_HPP_
#define SIMPLE_MANUAL__SBUS_JOY_CONVERTER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace simple_manual
{

struct SbusJoyOutput
{
  geometry_msgs::msg::Twist cmd_vel;
  bool soft_emg{false};
  std::string mode{"manual"};
};

double truncate_3(double value);
SbusJoyOutput convert_sbus_joy(
  const sensor_msgs::msg::Joy & joy, const std::vector<double> & offsets);

}  // namespace simple_manual

#endif  // SIMPLE_MANUAL__SBUS_JOY_CONVERTER_HPP_
