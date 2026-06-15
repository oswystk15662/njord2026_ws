#include <memory>

#include "diagnostic_monitors/topic_heartbeat_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::diagnostic_monitors::TopicHeartbeatMonitor>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
