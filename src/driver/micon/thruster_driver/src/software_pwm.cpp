#include "thruster_driver/software_pwm.hpp"

#include <algorithm>
#include <chrono>

namespace njord
{
namespace thruster_driver
{

SoftPwmMotor::SoftPwmMotor(
  const std::string & chip_path,
  int dir_pin,
  int pwm_pin,
  double frequency_hz,
  const std::vector<double> & force_per_duty)
: stop_thread_(false), force_duty_converter_(force_per_duty)
{
  period_us_ = 1000000.0 / std::max(1.0, frequency_hz);

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

  pwm_thread_ = std::thread(&SoftPwmMotor::pwm_loop, this);
}

SoftPwmMotor::~SoftPwmMotor()
{
  stop_thread_ = true;
  if (pwm_thread_.joinable()) {
    pwm_thread_.join();
  }

  line_pwm_.set_value(0);
  line_dir_.set_value(0);
}

void SoftPwmMotor::set_speed(double value)
{
  const double clamped = std::max(-1.0, std::min(value, 1.0));
  line_dir_.set_value(clamped >= 0.0 ? 0 : 1);
  duty_cycle_.store(std::abs(clamped));
}

void SoftPwmMotor::set_force(double force)
{
  set_speed(force_duty_converter_.forceToDuty(force));
}

void SoftPwmMotor::pwm_loop()
{
  while (!stop_thread_) {
    const double duty = duty_cycle_.load();
    const double on_time = period_us_ * duty;
    const double off_time = period_us_ * (1.0 - duty);

    if (duty > 0.0) {
      line_pwm_.set_value(1);
      if (duty < 1.0) {
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(on_time)));
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(period_us_)));
        continue;
      }
    }

    line_pwm_.set_value(0);
    if (duty < 1.0) {
      std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(off_time)));
    }
  }
}

}  // namespace thruster_driver
}  // namespace njord
