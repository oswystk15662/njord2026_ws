#pragma once

#include <gpiod.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace njord
{
namespace thruster_driver
{

class SoftPwmMotor
{
public:
  SoftPwmMotor(const std::string & chip_path, int dir_pin, int pwm_pin, double frequency_hz);
  ~SoftPwmMotor();

  void set_speed(double value);

private:
  void pwm_loop();

  gpiod::chip chip_;
  gpiod::line line_dir_;
  gpiod::line line_pwm_;

  std::thread pwm_thread_;
  std::atomic<bool> stop_thread_;
  std::atomic<double> duty_cycle_{0.0};
  double period_us_{0.0};
};

}  // namespace thruster_driver
}  // namespace njord
