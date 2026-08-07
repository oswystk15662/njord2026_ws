#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace
{
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kStartLatitudeDegrees = 63.4420936;
constexpr double kStartLongitudeDegrees = 10.4242793;

double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

struct Waypoint
{
  std::string topic_suffix;
  double latitude_degrees;
  double longitude_degrees;
};

Waypoint localWaypoint(const std::string & topic_suffix, double east_meters, double north_meters)
{
  const double latitude = kStartLatitudeDegrees + radiansToDegrees(north_meters / kEarthRadiusMeters);
  const double longitude = kStartLongitudeDegrees + radiansToDegrees(
    east_meters / (kEarthRadiusMeters * std::cos(degreesToRadians(kStartLatitudeDegrees))));
  return {topic_suffix, latitude, longitude};
}
}  // namespace

class NorwayWaypointPublisher : public rclcpp::Node
{
public:
  NorwayWaypointPublisher()
  : Node("norway_waypoint_publisher")
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    // These points are the visualization-only proposal in
    // Docs/norway_task_waypoint_proposal.md.  Values are local ENU offsets from
    // (63.4420936, 10.4242793), except the supplied geographic start and goal.
    const std::vector<Waypoint> waypoints{
      {"start", kStartLatitudeDegrees, kStartLongitudeDegrees},
      localWaypoint("task1/entry", -2.73, -7.52),
      localWaypoint("task1/marker_observe", -7.17, -19.74),
      localWaypoint("task1/decision", -10.93, -30.08),
      localWaypoint("task1/exit", -16.06, -44.17),
      {"task1/goal", 63.4416044, 10.4238816},
      localWaypoint("task2/risk_check_1", -5.12, -14.10),
      localWaypoint("task2/risk_check_2", -11.96, -32.89),
      localWaypoint("task2/risk_check_3", -17.08, -46.99),
      {"task2/goal", 63.4416044, 10.4238816},
      localWaypoint("task3/gate", -8.20, -22.56),
      localWaypoint("task3/left_approach", -7.34, -37.76),
      localWaypoint("task3/left_berth", -9.73, -44.34),
      localWaypoint("task3/right_approach", -18.62, -33.66),
      localWaypoint("task3/right_berth", -21.01, -40.24),
      {"task3/finish", 63.4416044, 10.4238816},
    };

    rclcpp::QoS qos(1);
    for (const auto & waypoint : waypoints) {
      publishers_.emplace_back(
        waypoint,
        create_publisher<sensor_msgs::msg::NavSatFix>(
          "/visualization/norway_waypoints/" + waypoint.topic_suffix,
          qos.transient_local().reliable()));
    }

    publish();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NorwayWaypointPublisher::publish, this));
  }

private:
  void publish()
  {
    const auto stamp = get_clock()->now();
    for (const auto & [waypoint, publisher] : publishers_) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = stamp;
      fix.header.frame_id = "wgs84";
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      fix.latitude = waypoint.latitude_degrees;
      fix.longitude = waypoint.longitude_degrees;
      fix.altitude = 0.0;
      fix.position_covariance = std::array<double, 9>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 4.0};
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      publisher->publish(fix);
    }
  }

  using PublisherEntry = std::pair<Waypoint, rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr>;
  std::vector<PublisherEntry> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NorwayWaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}
