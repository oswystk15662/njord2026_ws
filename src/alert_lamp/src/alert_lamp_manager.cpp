#include "alert_lamp/alert_lamp_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <functional>
#include <sstream>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace alert_lamp
{
namespace
{
std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
  });
  return value;
}

uint8_t colorValue(LampColor color)
{
  return static_cast<uint8_t>(color);
}

uint8_t patternValue(LampPattern pattern)
{
  return static_cast<uint8_t>(pattern);
}

std::string stateName(AlertState state)
{
  switch (state) {
    case AlertState::INITIALIZING: return "INITIALIZING";
    case AlertState::MANUAL_NORMAL: return "MANUAL_NORMAL";
    case AlertState::AUTO_NORMAL: return "AUTO_NORMAL";
    case AlertState::AUTONOMY_NOT_READY: return "AUTONOMY_NOT_READY";
    case AlertState::GROUND_COMMUNICATION_LOST: return "GROUND_COMMUNICATION_LOST";
    case AlertState::CRITICAL_FAULT: return "CRITICAL_FAULT";
  }
  return "UNKNOWN";
}
}  // namespace

AlertLampManager::AlertLampManager(const rclcpp::NodeOptions & options)
: Node("alert_lamp_manager_node", options), updater_(this)
{
  const auto rate = declare_parameter<double>("evaluation_rate_hz", 10.0);
  driver_timeout_sec_ = declare_parameter<double>("heartbeat.driver_timeout_sec", 1.0);
  high_level_timeout_sec_ = declare_parameter<double>("heartbeat.high_level_timeout_sec", 1.0);
  autonomy_timeout_sec_ = declare_parameter<double>("heartbeat.autonomy_timeout_sec", 1.0);
  localization_timeout_sec_ = declare_parameter<double>("heartbeat.localization_timeout_sec", 1.0);
  manual_timeout_sec_ = declare_parameter<double>("heartbeat.manual_control_timeout_sec", 1.0);
  ground_timeout_sec_ = declare_parameter<double>("heartbeat.ground_station_timeout_sec", 2.0);
  localization_message_timeout_sec_ = declare_parameter<double>("localization.message_timeout_sec",
      0.5);
  position_covariance_threshold_ =
    declare_parameter<double>("localization.position_covariance_threshold", 1.0);
  rtk_grace_period_sec_ = declare_parameter<double>("localization.rtk_grace_period_sec", 10.0);
  rtk_covariance_threshold_ = declare_parameter<double>("localization.rtk_covariance_threshold",
      0.01);
  green_period_ = static_cast<float>(declare_parameter<double>("lamp.green_blink_period_sec", 0.1));
  yellow_period_ = static_cast<float>(declare_parameter<double>("lamp.yellow_blink_period_sec",
      0.1));
  red_period_ = static_cast<float>(declare_parameter<double>("lamp.red_blink_period_sec", 0.05));
  duty_ratio_ = static_cast<float>(declare_parameter<double>("lamp.duty_ratio", 0.5));

  const auto generic_sub = [this](const std::string & name, const std::string & topic_key) {
      const auto topic = declare_parameter<std::string>("topics." + topic_key, "");
      const auto type = declare_parameter<std::string>("heartbeat." + topic_key + "_type",
        "std_msgs/msg/Empty");
      if (topic.empty()) {return std::shared_ptr<rclcpp::GenericSubscription>{};}
      return create_generic_subscription(topic, type, rclcpp::QoS(10),
               [this, name](std::shared_ptr<rclcpp::SerializedMessage>) {updateHeartbeat(name);});
    };
  driver_heartbeat_sub_ = generic_sub("driver", "driver_heartbeat");
  high_level_heartbeat_sub_ = generic_sub("high_level", "high_level_heartbeat");
  autonomy_heartbeat_sub_ = generic_sub("autonomy", "autonomy_heartbeat");
  localization_heartbeat_sub_ = generic_sub("localization", "localization_heartbeat");
  manual_heartbeat_sub_ = generic_sub("manual_control", "manual_control_heartbeat");
  ground_heartbeat_sub_ = generic_sub("ground_station", "ground_station_heartbeat");

  mode_sub_ = create_subscription<std_msgs::msg::String>(
    declare_parameter<std::string>("topics.operating_mode", "/system/operating_mode"), 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      const auto value = lower(message->data);
      mode_ = value == "manual" ? OperatingMode::MANUAL : value ==
      "auto" ? OperatingMode::AUTO : OperatingMode::UNKNOWN;
      mode_received_ = true;
    });
  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    declare_parameter<std::string>("topics.emergency_stop", "/safety/emergency_stop"), 10,
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      emergency_stop_ = message->data; emergency_received_ = true;
                                                                                                                      });
  autonomy_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    declare_parameter<std::string>("topics.autonomy_ready", "/autonomy/ready"), 10,
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      autonomy_ready_ = message->data; autonomy_ready_received_ = true;
                                                                                                                           });
  localization_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter<std::string>("topics.localization", "/odometry/filtered/global"), 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      localization_received_ = true;
      last_localization_ = now();
      localization_covariance_ok_ = message->pose.covariance[0] >= 0.0 &&
      message->pose.covariance[0] <= position_covariance_threshold_;
    });
  rtk_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    declare_parameter<std::string>("topics.rtk_status", "/sensor/vehicle_gnss/fix/raw"), 10,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr message) {
      rtk_received_ = true;
      const bool rtk_like = message->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
      message->position_covariance[0] >= 0.0 &&
      message->position_covariance[0] <= rtk_covariance_threshold_;
      if (rtk_like) {last_rtk_fix_ = now();}
    });
  diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    declare_parameter<std::string>("topics.diagnostics", "/diagnostics"), 10,
    [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      critical_diagnostics_ = std::any_of(message->status.begin(), message->status.end(),
      [](const auto & status) {
        return status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                                                                                                    });
    });
  command_pub_ = create_publisher<msg::AlertLampCommand>(
    declare_parameter<std::string>("topics.alert_command", "/alert_lamp/command"), 10);
  updater_.setHardwareID("alert_lamp_manager");
  updater_.add("Alert Lamp Manager", this, &AlertLampManager::updateDiagnostic);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
    std::bind(&AlertLampManager::onTimer, this));
}

void AlertLampManager::updateHeartbeat(const std::string & name)
{
  heartbeats_.update(name, now());
}

SystemStatus AlertLampManager::collectStatus(const rclcpp::Time & current) const
{
  SystemStatus status;
  status.mode = mode_;
  status.emergency_stop = emergency_stop_;
  status.autonomy_ready = autonomy_ready_;
  status.driver_alive = heartbeats_.isAlive("driver", current, driver_timeout_sec_);
  status.high_level_alive = heartbeats_.isAlive("high_level", current, high_level_timeout_sec_);
  status.autonomy_alive = heartbeats_.isAlive("autonomy", current, autonomy_timeout_sec_);
  status.localization_alive = heartbeats_.isAlive("localization", current,
      localization_timeout_sec_);
  status.manual_control_alive = heartbeats_.isAlive("manual_control", current, manual_timeout_sec_);
  status.ground_station_connected = heartbeats_.isAlive("ground_station", current,
      ground_timeout_sec_);
  const bool localization_fresh = localization_received_ &&
    (current - last_localization_).seconds() <= localization_message_timeout_sec_;
  status.localization_stable = status.localization_alive && localization_fresh &&
    localization_covariance_ok_;
  status.rtk_fix = rtk_received_ && (current - last_rtk_fix_).seconds() <= rtk_grace_period_sec_;
  status.required_sensor_not_ready = required_sensor_not_ready_;
  status.critical_driver_failure = critical_driver_failure_;
  status.critical_diagnostics = critical_diagnostics_;
  status.state_unknown = !mode_received_ || !emergency_received_ ||
    (mode_ == OperatingMode::AUTO && !autonomy_ready_received_);
  return status;
}

void AlertLampManager::onTimer()
{
  const auto current = now();
  const auto status = collectStatus(current);
  const auto state = evaluator_.evaluate(status);
  const auto display = evaluator_.displayFor(state, green_period_, yellow_period_, red_period_,
      duty_ratio_, status);
  msg::AlertLampCommand command;
  command.header.stamp = current;
  command.color = colorValue(display.color);
  command.pattern = patternValue(display.pattern);
  command.period = display.period;
  command.duty_ratio = display.duty_ratio;
  command.reason = display.reason;
  command_pub_->publish(command);
  if (!has_last_state_ || state != last_state_) {
    RCLCPP_WARN(get_logger(), "%s -> %s: %s",
        has_last_state_ ? stateName(last_state_).c_str() : "START",
      stateName(state).c_str(), display.reason.c_str());
    last_state_ = state;
    has_last_state_ = true;
  }
  updater_.force_update();
}

void AlertLampManager::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto current = now();
  const auto status = collectStatus(current);
  const auto state = evaluator_.evaluate(status);
  const auto display = evaluator_.displayFor(state, green_period_, yellow_period_, red_period_,
      duty_ratio_, status);
  stat.summary(state == AlertState::CRITICAL_FAULT ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    state == AlertState::MANUAL_NORMAL ||
      state == AlertState::AUTO_NORMAL ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN, display.reason);
  stat.add("alert_state", stateName(state));
  stat.add("reason", display.reason);
  stat.add("driver_heartbeat_age", heartbeats_.age("driver", current));
  stat.add("high_level_heartbeat_age", heartbeats_.age("high_level", current));
  stat.add("autonomy_heartbeat_age", heartbeats_.age("autonomy", current));
  stat.add("localization_heartbeat_age", heartbeats_.age("localization", current));
  stat.add("ground_station_heartbeat_age", heartbeats_.age("ground_station", current));
  stat.add("localization_stable", status.localization_stable);
  stat.add("rtk_fix", status.rtk_fix);
  stat.add("emergency_stop", status.emergency_stop);
}

}  // namespace alert_lamp
