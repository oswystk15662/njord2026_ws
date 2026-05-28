#pragma once

#include <gpiod.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace njord
{
namespace thruster_driver
{

class HardwarePWM
{
public:
  HardwarePWM(const std::string & chip_path, int dir_pin, int pwm_pin, double freq_hz = 50.0);
  ~HardwarePWM();

  void set_speed(double value);
  double get_speed() const { return last_speed_; }
  void emergency_stop();

private:
  void pwm_loop_thread();

  gpiod::chip chip_;
  gpiod::line line_dir_;
  gpiod::line line_pwm_;

  std::thread pwm_thread_;
  std::atomic<bool> stop_flag_;

  std::atomic<double> duty_cycle_{0.0};
  std::atomic<int> direction_{0};
  double last_speed_{0.0};
  double period_us_{0.0};
};

}  // namespace thruster_driver
}  // namespace njord
