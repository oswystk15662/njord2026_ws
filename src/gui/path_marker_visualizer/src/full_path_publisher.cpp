#include "path_marker_visualizer/full_path_publisher.hpp"

using namespace std::chrono_literals;

namespace path_marker_visualizer
{

FullPathPublisher::FullPathPublisher(const rclcpp::NodeOptions & options)
: Node("full_path_publisher", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("twist_topic", "/cmd_vel");
  this->declare_parameter<bool>("use_velocity_color", true);
  this->declare_parameter<double>("min_velocity", 0.0);
  this->declare_parameter<double>("max_velocity", 2.0);
  this->declare_parameter<uint8_t>("color_transition_type", static_cast<uint8_t>(ColorTransitionType::LINEAR));
  this->declare_parameter<double>("log_scale_factor", 9.0);
  this->declare_parameter<double>("line_width", 0.05);
  this->declare_parameter<double>("twist_timeout_sec", 0.5);

  // Get parameters initially
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("twist_topic", twist_topic_);
  this->get_parameter("use_velocity_color", use_velocity_color_);
  this->get_parameter("min_velocity", min_velocity_);
  this->get_parameter("max_velocity", max_velocity_);
  this->get_parameter("color_transition_type", color_transition_type_val_);
  this->get_parameter("log_scale_factor", log_scale_factor_);
  this->get_parameter("line_width", line_width_);
  this->get_parameter("twist_timeout_sec", twist_timeout_sec_);

  publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("full_path_marker", 10);
  
  // TF buffer & listener setup
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Twist subscription
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    twist_topic_, 10,
    std::bind(&FullPathPublisher::twist_callback, this, std::placeholders::_1));

  last_twist_time_ = this->now();

  // 0.1s timer to check position and update path
  timer_ = this->create_wall_timer(
    100ms, std::bind(&FullPathPublisher::timer_callback, this));
}

void FullPathPublisher::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_twist_ = *msg;
  last_twist_time_ = this->now();
}

std_msgs::msg::ColorRGBA FullPathPublisher::compute_color(double speed)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0;

  if (!use_velocity_color_) {
    // Default green color
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    return color;
  }

  // Clamp speed to [min_velocity_, max_velocity_]
  double clamped_speed = std::max(min_velocity_, std::min(speed, max_velocity_));
  double denom = max_velocity_ - min_velocity_;
  double s = 0.0;
  if (denom > 1e-6) {
    s = (clamped_speed - min_velocity_) / denom;
  }

  double t = 0.0;
  if (color_transition_type_val_ == static_cast<uint8_t>(ColorTransitionType::LOGARITHMIC)) {
    // log(1 + k*s) / log(1 + k)
    // Rises rapidly at low speeds
    double k = std::max(0.1, log_scale_factor_);
    t = std::log(1.0 + k * s) / std::log(1.0 + k);
  } else {
    // Linear
    t = s;
  }

  // Low speed: Blue (0, 0, 1)
  // High speed: Red (1, 0, 0)
  // Interpolation
  color.r = static_cast<float>(t);
  color.g = 0.0f;
  color.b = static_cast<float>(1.0 - t);

  return color;
}

void FullPathPublisher::timer_callback()
{
  // Fetch parameters to support dynamic parameter reconfiguration
  this->get_parameter("use_velocity_color", use_velocity_color_);
  this->get_parameter("min_velocity", min_velocity_);
  this->get_parameter("max_velocity", max_velocity_);
  this->get_parameter("color_transition_type", color_transition_type_val_);
  this->get_parameter("log_scale_factor", log_scale_factor_);
  this->get_parameter("line_width", line_width_);
  this->get_parameter("twist_timeout_sec", twist_timeout_sec_);

  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    return;
  }

  geometry_msgs::msg::Point p;
  p.x = t.transform.translation.x;
  p.y = t.transform.translation.y;
  p.z = t.transform.translation.z;

  // Record point if moved at least 5cm from previous point
  if (!points_.empty()) {
    double dist = std::hypot(p.x - points_.back().x, p.y - points_.back().y);
    if (dist < 0.05) {
      return; 
    }
  }

  // Determine current speed
  double speed = 0.0;
  if (use_velocity_color_) {
    double dt_twist = (this->now() - last_twist_time_).seconds();
    if (dt_twist <= twist_timeout_sec_) {
      double vx = current_twist_.linear.x;
      double vy = current_twist_.linear.y;
      double vz = current_twist_.linear.z;
      speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    }
  }

  points_.push_back(p);
  colors_.push_back(compute_color(speed));

  // Build marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = odom_frame_;
  marker.header.stamp = this->now();
  marker.ns = "path_history";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width_;

  if (use_velocity_color_) {
    marker.colors = colors_;
  } else {
    // Single green color
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }

  marker.points = points_;

  publisher_->publish(marker);
}

}  // namespace path_marker_visualizer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_marker_visualizer::FullPathPublisher>());
  rclcpp::shutdown();
  return 0;
}