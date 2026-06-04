#ifndef PATH_MARKER_VISUALIZER__FULL_PATH_PUBLISHER_HPP_
#define PATH_MARKER_VISUALIZER__FULL_PATH_PUBLISHER_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace path_marker_visualizer
{

enum class ColorTransitionType : uint8_t
{
  LINEAR = 0,
  LOGARITHMIC = 1
};

class FullPathPublisher : public rclcpp::Node
{
public:
  explicit FullPathPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~FullPathPublisher() = default;

private:
  void timer_callback();
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  std_msgs::msg::ColorRGBA compute_color(double speed);

  // ROS 2 interfaces
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string odom_frame_;
  std::string base_frame_;
  std::string twist_topic_;
  bool use_velocity_color_;
  double min_velocity_;
  double max_velocity_;
  uint8_t color_transition_type_val_;
  double log_scale_factor_;
  double line_width_;
  double twist_timeout_sec_;

  // Twist data
  geometry_msgs::msg::Twist current_twist_;
  rclcpp::Time last_twist_time_;

  // Path data
  std::vector<geometry_msgs::msg::Point> points_;
  std::vector<std_msgs::msg::ColorRGBA> colors_;
};

}  // namespace path_marker_visualizer

#endif  // PATH_MARKER_VISUALIZER__FULL_PATH_PUBLISHER_HPP_
