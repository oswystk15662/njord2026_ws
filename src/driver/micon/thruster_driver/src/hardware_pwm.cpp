#include "thruster_driver/hardware_pwm.hpp"

#include <algorithm>
#include <chrono>

namespace njord
{
namespace thruster_driver
{

HardwarePWM::HardwarePWM(const std::string & chip_path, int dir_pin, int pwm_pin, double freq_hz)
: stop_flag_(false)
{
  period_us_ = 1000000.0 / std::max(1.0, freq_hz);

  chip_.open(chip_path);
  line_dir_ = chip_.get_line(dir_pin);
  line_pwm_ = chip_.get_line(pwm_pin);

  gpiod::line_request request;
  request.consumer = "thruster_driver";
  request.request_type = gpiod::line_request::DIRECTION_OUTPUT;

  line_dir_.request(request);
  line_pwm_.request(request);

  line_dir_.set_value(0);
  line_pwm_.set_value(0);

  pwm_thread_ = std::thread(&HardwarePWM::pwm_loop_thread, this);
}

HardwarePWM::~HardwarePWM()
{
  stop_flag_ = true;
  if (pwm_thread_.joinable()) {
    pwm_thread_.join();
  }

  line_pwm_.set_value(0);
  line_dir_.set_value(0);
}

void HardwarePWM::set_speed(double value)
{
  value = std::max(-1.0, std::min(value, 1.0));
  last_speed_ = value;

  direction_ = (value >= 0) ? 1 : -1;
  if (std::abs(value) < 1e-6) {
    direction_ = 0;
  }

  line_dir_.set_value(direction_ < 0 ? 1 : 0);
  duty_cycle_ = std::abs(value);
}

void HardwarePWM::emergency_stop()
{
  duty_cycle_ = 0.0;
  direction_ = 0;
  line_dir_.set_value(0);
  line_pwm_.set_value(0);
}

void HardwarePWM::pwm_loop_thread()
{
  while (!stop_flag_) {
    const double current_duty = duty_cycle_.load();
    const double on_time = period_us_ * current_duty;
    const double off_time = period_us_ * (1.0 - current_duty);

    if (current_duty > 1e-6) {
      line_pwm_.set_value(1);
      if (current_duty < 0.9999) {
        std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<int>(on_time)));
      } else {
        std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<int>(period_us_)));
        continue;
      }
    }

    line_pwm_.set_value(0);
    if (current_duty < 0.9999) {
      std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int>(off_time)));
    }
  }
}

}  // namespace thruster_driver
}  // namespace njord
