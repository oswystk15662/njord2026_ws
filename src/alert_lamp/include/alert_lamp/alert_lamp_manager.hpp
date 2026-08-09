#pragma once

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "njord_interfaces/msg/control_state.hpp"
#include "njord_interfaces/msg/health_state.hpp"
#include "njord_interfaces/msg/mission_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include "alert_lamp/msg/alert_lamp_command.hpp"
#include "alert_lamp/alert_lamp_state.hpp"
#include "alert_lamp/status_evaluator.hpp"

namespace alert_lamp
{

class AlertLampManager : public rclcpp::Node
{
public:
  explicit AlertLampManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTimer();
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  SystemStatus collectStatus(const rclcpp::Time & now) const;

  StatusEvaluator evaluator_;
  bool control_received_{false};
  bool mission_received_{false};
  bool health_received_{false};
  njord_interfaces::msg::ControlState control_state_{};
  njord_interfaces::msg::MissionStatus mission_status_{};
  njord_interfaces::msg::HealthState health_state_{};
  AlertState last_state_{AlertState::INITIALIZING};
  bool has_last_state_{false};

  rclcpp::Subscription<njord_interfaces::msg::ControlState>::SharedPtr control_sub_;
  rclcpp::Subscription<njord_interfaces::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Subscription<njord_interfaces::msg::HealthState>::SharedPtr health_sub_;
  rclcpp::Publisher<msg::AlertLampCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
  float green_period_{1.0F};
  float yellow_period_{1.0F};
  float red_period_{0.5F};
  float duty_ratio_{0.5F};
};

}  // namespace alert_lamp
