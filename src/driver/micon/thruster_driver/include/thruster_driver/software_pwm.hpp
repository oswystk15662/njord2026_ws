#pragma once

#include <gpiod.hpp>

#include <atomic>
#include <string>
#include <thread>

#include <vector>

#include "thruster_driver/force_duty_converter.hpp"

namespace njord
{
namespace thruster_driver
{

class SoftPwmMotor
{
public:
  SoftPwmMotor(
    const std::string & chip_path,
    int dir_pin,
    int pwm_pin,
    double frequency_hz,
    const std::vector<double> & force_per_duty = std::vector<double>{1.0});
  ~SoftPwmMotor();

  void set_speed(double value);
  void set_force(double force);

private:
  void pwm_loop();

  gpiod::chip chip_;
  gpiod::line line_dir_;
  gpiod::line line_pwm_;

  std::thread pwm_thread_;
  std::atomic<bool> stop_thread_;
  std::atomic<double> duty_cycle_{0.0};
  double period_us_{0.0};
  ForceDutyConverter force_duty_converter_;
};

}  // namespace thruster_driver
}  // namespace njord
