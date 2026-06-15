#include "diagnostic_monitors/topic_heartbeat_monitor.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace njord
{
namespace diagnostic_monitors
{

namespace
{
std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}
}  // namespace

TopicHeartbeatMonitor::TopicHeartbeatMonitor(const rclcpp::NodeOptions & options)
: Node("topic_heartbeat_monitor", options),
  updater_(this),
  mode_(Mode::RequiredFrequency),
  last_message_time_(0, 0, get_clock()->get_clock_type()),
  has_message_(false)
{
  monitor_name_ = declare_parameter<std::string>("monitor_name", get_name());
  topic_name_ = declare_parameter<std::string>("topic", "");
  topic_type_ = declare_parameter<std::string>("topic_type", "");
  mode_text_ = declare_parameter<std::string>("mode", "required_frequency");
  expected_frequency_hz_ = declare_parameter<double>("expected_frequency", 1.0);
  minimum_frequency_hz_ = declare_parameter<double>("minimum_frequency", 0.1);
  timeout_sec_ = declare_parameter<double>("timeout", 5.0);
  stale_timeout_sec_ = declare_parameter<double>("stale_timeout", timeout_sec_ * 3.0);
  diagnostic_period_sec_ = declare_parameter<double>("diagnostic_period", 1.0);
  window_size_ = declare_parameter<int>("window_size", 50);
  qos_depth_ = declare_parameter<int>("qos_depth", 10);
  qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort");
  diagnostic_name_ = declare_parameter<std::string>(
    "diagnostic_name", "topic_heartbeat: " + monitor_name_);
  hardware_id_ = declare_parameter<std::string>("hardware_id", "topic_heartbeat_monitor");

  mode_ = parseMode(mode_text_);
  window_size_ = std::max(2, window_size_);
  minimum_frequency_hz_ = std::max(0.0, minimum_frequency_hz_);
  expected_frequency_hz_ = std::max(0.0, expected_frequency_hz_);
  timeout_sec_ = std::max(0.0, timeout_sec_);
  stale_timeout_sec_ = std::max(timeout_sec_, stale_timeout_sec_);
  diagnostic_period_sec_ = std::max(0.1, diagnostic_period_sec_);

  updater_.setHardwareID(hardware_id_);
  updater_.add(diagnostic_name_, this, &TopicHeartbeatMonitor::updateDiagnostic);

  if (topic_name_.empty() || topic_type_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Both 'topic' and 'topic_type' parameters are required. topic='%s', topic_type='%s'",
      topic_name_.c_str(), topic_type_.c_str());
  } else {
    subscription_ = create_generic_subscription(
      topic_name_,
      topic_type_,
      makeQos(),
      std::bind(&TopicHeartbeatMonitor::onMessage, this, std::placeholders::_1));
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(diagnostic_period_sec_),
    std::bind(&TopicHeartbeatMonitor::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "Monitoring topic '%s' (%s) as '%s', mode=%s",
    topic_name_.c_str(), topic_type_.c_str(), monitor_name_.c_str(), mode_text_.c_str());
}

TopicHeartbeatMonitor::Mode TopicHeartbeatMonitor::parseMode(const std::string & value) const
{
  const std::string lower = toLower(value);
  if (lower == "required_frequency" || lower == "frequency" || lower == "required") {
    return Mode::RequiredFrequency;
  }
  if (lower == "heartbeat_only" || lower == "heartbeat") {
    return Mode::HeartbeatOnly;
  }
  if (lower == "optional") {
    return Mode::Optional;
  }

  RCLCPP_WARN(
    get_logger(),
    "Unknown mode='%s'. Falling back to required_frequency.",
    value.c_str());
  return Mode::RequiredFrequency;
}

rclcpp::QoS TopicHeartbeatMonitor::makeQos() const
{
  rclcpp::QoS qos(std::max(1, qos_depth_));
  const std::string reliability = toLower(qos_reliability_);
  if (reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}

void TopicHeartbeatMonitor::onMessage(std::shared_ptr<rclcpp::SerializedMessage> message)
{
  (void)message;
  const auto now = get_clock()->now();
  last_message_time_ = now;
  has_message_ = true;
  samples_.push_back(now);
  pruneSamples();
}

void TopicHeartbeatMonitor::pruneSamples()
{
  while (samples_.size() > static_cast<std::size_t>(window_size_)) {
    samples_.pop_front();
  }
}

double TopicHeartbeatMonitor::measuredFrequency() const
{
  if (samples_.size() < 2U) {
    return 0.0;
  }
  const double span = (samples_.back() - samples_.front()).seconds();
  if (span <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  return static_cast<double>(samples_.size() - 1U) / span;
}

double TopicHeartbeatMonitor::lastMessageAgeSec()
{
  if (!has_message_) {
    return std::numeric_limits<double>::infinity();
  }
  return (get_clock()->now() - last_message_time_).seconds();
}

void TopicHeartbeatMonitor::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const size_t publishers = count_publishers(topic_name_);
  const double frequency = measuredFrequency();
  const double age = lastMessageAgeSec();

  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "OK";

  if (topic_name_.empty() || topic_type_.empty()) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = "topic or topic_type is not configured";
  } else if (mode_ == Mode::Optional && publishers == 0U && !has_message_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    message = "optional topic has no publisher";
  } else if (publishers == 0U) {
    level = mode_ == Mode::RequiredFrequency ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "no publishers";
  } else if (!has_message_) {
    level = mode_ == Mode::RequiredFrequency ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "publisher exists but no message received";
  } else if (age > stale_timeout_sec_) {
    level = mode_ == Mode::Optional ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = "last message is stale";
  } else if (age > timeout_sec_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "last message timeout exceeded";
  } else if (mode_ == Mode::RequiredFrequency && frequency < minimum_frequency_hz_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "frequency below minimum";
  } else {
    std::ostringstream oss;
    oss.precision(3);
    oss << frequency << " Hz, last message age " << age << " sec";
    message = oss.str();
  }

  stat.summary(level, message);
  stat.add("monitor_name", monitor_name_);
  stat.add("mode", mode_text_);
  stat.add("topic", topic_name_);
  stat.add("topic_type", topic_type_);
  stat.add("publishers", static_cast<int>(publishers));
  stat.add("expected_frequency_hz", expected_frequency_hz_);
  stat.add("minimum_frequency_hz", minimum_frequency_hz_);
  stat.add("measured_frequency_hz", frequency);
  if (std::isfinite(age)) {
    stat.add("last_message_age_sec", age);
  } else {
    stat.add("last_message_age_sec", "never");
  }
  stat.add("timeout_sec", timeout_sec_);
  stat.add("stale_timeout_sec", stale_timeout_sec_);
  stat.add("window_size", window_size_);
  stat.add("qos_reliability", qos_reliability_);
}

void TopicHeartbeatMonitor::onTimer()
{
  updater_.force_update();
}

}  // namespace diagnostic_monitors
}  // namespace njord

RCLCPP_COMPONENTS_REGISTER_NODE(njord::diagnostic_monitors::TopicHeartbeatMonitor)
