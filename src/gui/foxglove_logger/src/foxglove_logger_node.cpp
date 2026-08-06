#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "njord_interfaces/msg/buoy_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double pointToSegmentDistance(double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(px - ax, py - ay);
  }
  const double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / length_squared, 0.0, 1.0);
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

std::string shorten(std::string value, std::size_t max_length)
{
  if (value.size() <= max_length) {
    return value;
  }
  return value.substr(0, max_length - 3U) + "...";
}

const char * diagnosticLevelName(uint8_t level)
{
  switch (level) {
    case diagnostic_msgs::msg::DiagnosticStatus::OK: return "OK";
    case diagnostic_msgs::msg::DiagnosticStatus::WARN: return "WARN";
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR: return "ERROR";
    case diagnostic_msgs::msg::DiagnosticStatus::STALE: return "STALE";
    default: return "UNKNOWN";
  }
}
}  // namespace

class FoxgloveLogger : public rclcpp::Node
{
public:
  FoxgloveLogger()
  : Node("foxglove_logger")
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    const auto cells_topic = declare_parameter<std::string>("cell_voltages_topic", "/bms/cell_voltages");
    const auto temperature_topic = declare_parameter<std::string>("temperature_topic", "/bms/temperature_c");
    const auto buoy_topic = declare_parameter<std::string>("buoy_topic", "/buoy_detections_3d");
    const auto fix_topic = declare_parameter<std::string>("fix_topic", "/sensor/vehicle_gnss/fix/raw");
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/odometry/filtered/global");
    const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
    const auto speed_topic = declare_parameter<std::string>("ground_speed_topic", "/gui/ground_speed_mps");
    const auto control_topic = declare_parameter<std::string>("control_status_topic", "/system/control_status");
    const auto diagnostics_topic = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    const auto log_topic = declare_parameter<std::string>("log_topic", "/foxglove_log");

    log_pub_ = create_publisher<std_msgs::msg::String>(log_topic, 10);

    cells_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(cells_topic, 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {cells_ = msg->data;});
    temperature_sub_ = create_subscription<std_msgs::msg::Float32>(temperature_topic, 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {temperature_c_ = msg->data;});
    buoy_sub_ = create_subscription<njord_interfaces::msg::BuoyDetectionArray>(buoy_topic, 10,
      [this](njord_interfaces::msg::BuoyDetectionArray::SharedPtr msg) {onBuoys(*msg);});
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(fix_topic, 10,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {fix_ = *msg;});
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {odometry_ = *msg;});
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(plan_topic, 10,
      [this](nav_msgs::msg::Path::SharedPtr msg) {plan_ = *msg;});
    speed_sub_ = create_subscription<std_msgs::msg::Float32>(speed_topic, 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {ground_speed_mps_ = msg->data;});
    control_sub_ = create_subscription<std_msgs::msg::String>(control_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr msg) {control_status_ = msg->data;});
    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {onDiagnostics(*msg);});

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publishReport();});
  }

private:
  enum class LogLevel
  {
    kInfo,
    kWarn,
    kError,
  };

  void publishLog(LogLevel level, const std::string & text) const
  {
    std_msgs::msg::String message;
    message.data = text;
    log_pub_->publish(message);

    switch (level) {
      case LogLevel::kInfo:
        RCLCPP_INFO(get_logger(), "%s", text.c_str());
        break;
      case LogLevel::kWarn:
        RCLCPP_WARN(get_logger(), "%s", text.c_str());
        break;
      case LogLevel::kError:
        RCLCPP_ERROR(get_logger(), "%s", text.c_str());
        break;
    }
  }

  void onBuoys(const njord_interfaces::msg::BuoyDetectionArray & msg)
  {
    if (msg.detections.empty()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    buoy_detection_times_.push_back(now);
    last_buoy_count_ = msg.detections.size();
  }

  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    for (const auto & status : msg.status) {
      diagnostics_[status.name] = status;
    }
  }

  std::string buoyText()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto horizon = now - std::chrono::seconds(1);
    while (!buoy_detection_times_.empty() && buoy_detection_times_.front() < horizon) {
      buoy_detection_times_.pop_front();
    }
    if (buoy_detection_times_.empty()) {
      return "NONE (0.0 Hz)";
    }
    std::ostringstream out;
    out << last_buoy_count_ << " detected (" << std::fixed << std::setprecision(1)
        << static_cast<double>(buoy_detection_times_.size()) << " Hz)";
    return out.str();
  }

  std::optional<double> planDeviation() const
  {
    if (!odometry_ || !plan_ || plan_->poses.empty()) {
      return std::nullopt;
    }
    const double x = odometry_->pose.pose.position.x;
    const double y = odometry_->pose.pose.position.y;
    if (plan_->poses.size() == 1U) {
      const auto & point = plan_->poses.front().pose.position;
      return std::hypot(x - point.x, y - point.y);
    }
    double result = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < plan_->poses.size(); ++i) {
      const auto & a = plan_->poses[i - 1].pose.position;
      const auto & b = plan_->poses[i].pose.position;
      result = std::min(result, pointToSegmentDistance(x, y, a.x, a.y, b.x, b.y));
    }
    return result;
  }

  std::optional<double> headingDegrees() const
  {
    if (!odometry_) {
      return std::nullopt;
    }
    const auto & orientation = odometry_->pose.pose.orientation;
    const double sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    double degrees = std::atan2(sin_yaw, cos_yaw) * 180.0 / kPi;
    while (degrees > 180.0) {degrees -= 360.0;}
    while (degrees <= -180.0) {degrees += 360.0;}
    return degrees;
  }

  std::optional<double> speedMps() const
  {
    if (ground_speed_mps_) {
      return ground_speed_mps_;
    }
    if (!odometry_) {
      return std::nullopt;
    }
    const auto & velocity = odometry_->twist.twist.linear;
    return std::hypot(velocity.x, velocity.y);
  }

  void publishReport()
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << "TEL ";
    out << "CELLS=";
    if (cells_) {
      out << '[';
      for (std::size_t i = 0; i < cells_->size(); ++i) {
        if (i != 0U) {out << ',';}
        out << (*cells_)[i];
      }
      out << ']';
    } else {out << "N/A";}
    out << " LIPO=" << (temperature_c_ ? std::to_string(*temperature_c_) + "C" : "N/A");
    out << " BUOY=" << buoyText();
    if (fix_) {
      out << std::setprecision(6) << " LAT=" << fix_->latitude << " LON=" << fix_->longitude;
      out << std::setprecision(2);
    } else {out << " LAT=N/A LON=N/A";}
    const auto heading = headingDegrees();
    out << " HDG=" << (heading ? std::to_string(*heading) + "deg" : "N/A");
    const auto speed = speedMps();
    out << " SOG=" << (speed ? std::to_string(*speed) + "m/s" : "N/A");
    const auto xte = planDeviation();
    out << " PLAN_XTE=" << (xte ? std::to_string(*xte) + "m" : "N/A");
    publishLog(LogLevel::kInfo, out.str());
    publishControlStatus();
    publishDiagnosticSummary();
  }

  void publishControlStatus() const
  {
    const std::string status = control_status_.value_or("N/A");
    if (status == "auto") {
      publishLog(LogLevel::kInfo, "CTRL_MODE=AUTO");
    } else if (status == "emergency_stop") {
      publishLog(LogLevel::kError, "CTRL_MODE=EMERGENCY_STOP");
    } else {
      publishLog(LogLevel::kWarn, "CTRL_MODE=" + status);
    }
  }

  void publishDiagnosticSummary() const
  {
    std::array<int, 4> counts{};
    std::vector<std::string> issues;
    uint8_t highest_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    for (const auto & [name, status] : diagnostics_) {
      if (status.level < counts.size()) {++counts[status.level];}
      highest_level = std::max(highest_level, status.level);
      if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        issues.push_back(std::string(diagnosticLevelName(status.level)) + ":" +
          shorten(name, 24U) + "=" + shorten(status.message, 64U));
      }
    }
    std::ostringstream out;
    out << "DIAG OK=" << counts[0] << " WARN=" << counts[1] << " ERROR=" << counts[2]
        << " STALE=" << counts[3];
    for (const auto & issue : issues) {out << " | " << issue;}
    if (highest_level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      publishLog(LogLevel::kError, out.str());
    } else if (highest_level == diagnostic_msgs::msg::DiagnosticStatus::WARN ||
      highest_level == diagnostic_msgs::msg::DiagnosticStatus::STALE)
    {
      publishLog(LogLevel::kWarn, out.str());
    } else {
      publishLog(LogLevel::kInfo, out.str());
    }
  }

  std::optional<std::vector<float>> cells_;
  std::optional<float> temperature_c_;
  std::optional<sensor_msgs::msg::NavSatFix> fix_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<nav_msgs::msg::Path> plan_;
  std::optional<float> ground_speed_mps_;
  std::optional<std::string> control_status_;
  std::deque<std::chrono::steady_clock::time_point> buoy_detection_times_;
  std::size_t last_buoy_count_{0};
  std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> diagnostics_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cells_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temperature_sub_;
  rclcpp::Subscription<njord_interfaces::msg::BuoyDetectionArray>::SharedPtr buoy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FoxgloveLogger>());
  rclcpp::shutdown();
  return 0;
}
