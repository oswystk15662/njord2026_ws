#pragma once

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "alert_lamp/msg/alert_lamp_command.hpp"
#include "alert_lamp/alert_lamp_state.hpp"
#include "alert_lamp/heartbeat_monitor.hpp"
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
  void updateHeartbeat(const std::string & name);
  SystemStatus collectStatus(const rclcpp::Time & now) const;

  HeartbeatMonitor heartbeats_;
  StatusEvaluator evaluator_;
  OperatingMode mode_{OperatingMode::UNKNOWN};
  bool emergency_stop_{false};
  bool autonomy_ready_{false};
  bool required_sensor_not_ready_{false};
  bool critical_driver_failure_{false};
  bool critical_diagnostics_{false};
  bool mode_received_{false};
  bool emergency_received_{false};
  bool autonomy_ready_received_{false};
  bool localization_received_{false};
  bool rtk_received_{false};
  bool localization_covariance_ok_{false};
  rclcpp::Time last_localization_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rtk_fix_{0, 0, RCL_ROS_TIME};
  AlertState last_state_{AlertState::INITIALIZING};
  bool has_last_state_{false};

  std::shared_ptr<rclcpp::GenericSubscription> driver_heartbeat_sub_;
  std::shared_ptr<rclcpp::GenericSubscription> high_level_heartbeat_sub_;
  std::shared_ptr<rclcpp::GenericSubscription> autonomy_heartbeat_sub_;
  std::shared_ptr<rclcpp::GenericSubscription> localization_heartbeat_sub_;
  std::shared_ptr<rclcpp::GenericSubscription> manual_heartbeat_sub_;
  std::shared_ptr<rclcpp::GenericSubscription> ground_heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Publisher<msg::AlertLampCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
  double driver_timeout_sec_{1.0};
  double high_level_timeout_sec_{1.0};
  double autonomy_timeout_sec_{1.0};
  double localization_timeout_sec_{1.0};
  double manual_timeout_sec_{1.0};
  double ground_timeout_sec_{2.0};
  double localization_message_timeout_sec_{0.5};
  double position_covariance_threshold_{1.0};
  double rtk_grace_period_sec_{10.0};
  double rtk_covariance_threshold_{0.01};
  float green_period_{1.0F};
  float yellow_period_{1.0F};
  float red_period_{0.5F};
  float duty_ratio_{0.5F};
};

}  // namespace alert_lamp
