#pragma once

#include <string>

namespace alert_lamp
{

enum class OperatingMode {UNKNOWN, MANUAL, AUTO};
enum class LampColor {OFF, GREEN, YELLOW, RED};
enum class LampPattern {OFF, SOLID, BLINK};
enum class AlertState
{
  INITIALIZING,
  MANUAL_NORMAL,
  AUTO_NORMAL,
  AUTONOMY_NOT_READY,
  GROUND_COMMUNICATION_LOST,
  CRITICAL_FAULT
};

struct SystemStatus
{
  OperatingMode mode{OperatingMode::UNKNOWN};
  bool emergency_stop{false};
  bool driver_alive{false};
  bool high_level_alive{false};
  bool autonomy_alive{false};
  bool localization_alive{false};
  bool manual_control_alive{false};
  bool ground_station_connected{false};
  bool autonomy_ready{false};
  bool localization_stable{false};
  bool rtk_fix{false};
  bool required_sensor_not_ready{false};
  bool critical_driver_failure{false};
  bool critical_diagnostics{false};
  bool state_unknown{true};
};

struct LampDisplay
{
  LampColor color{LampColor::RED};
  LampPattern pattern{LampPattern::BLINK};
  float period{0.5F};
  float duty_ratio{0.5F};
  std::string reason{"initializing"};
};

}  // namespace alert_lamp
