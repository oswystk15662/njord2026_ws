#include "dutyed_tf_pub_with_disturbance/sim_node.hpp"

#include "dutyed_tf_pub_with_disturbance/thruster_wrench.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace njord
{
namespace sim
{

namespace
{
double clamp(double value, double low, double high)
{
  return std::clamp(value, low, high);
}
}  // namespace

SimNode::SimNode(const rclcpp::NodeOptions & options)
: Node("dutyed_tf_pub_with_disturbance_node", options)
{
  update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 50.0);
  duty_resolution_ = this->declare_parameter<double>("duty_resolution", 1000.0);
  topic_thruster_command_ = this->declare_parameter<std::string>(
    "topic_thruster_command",
    "/thruster_command");
  topic_odom_ = this->declare_parameter<std::string>("topic_odom", "/odom");

  frame_world_ = this->declare_parameter<std::string>("frame_world", "world");
  frame_map_ = this->declare_parameter<std::string>("frame_map", "map");
  frame_odom_ = this->declare_parameter<std::string>("frame_odom", "odom");
  frame_base_link_ = this->declare_parameter<std::string>("frame_base_link", "base_link");
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

  auto map_offset =
    this->declare_parameter<std::vector<double>>("map_offset_xyyaw", {0.0, 0.0, 0.0});
  auto odom_offset = this->declare_parameter<std::vector<double>>(
    "odom_offset_xyyaw", {0.0, 0.0,
      0.0});
  if (map_offset.size() >= 3U) {
    map_offset_x_ = map_offset[0];
    map_offset_y_ = map_offset[1];
    map_offset_yaw_ = map_offset[2];
  }
  if (odom_offset.size() >= 3U) {
    odom_offset_x_ = odom_offset[0];
    odom_offset_y_ = odom_offset[1];
    odom_offset_yaw_ = odom_offset[2];
  }

  const double max_forward = this->declare_parameter<double>("thruster.max_forward_newton", 50.0);
  const double max_reverse = this->declare_parameter<double>("thruster.max_reverse_newton", 40.0);
  half_beam_meter_ = this->declare_parameter<double>("thruster.half_beam_meter", 0.35);
  left_reverse_ = this->declare_parameter<bool>("thruster.left_reverse", false);
  right_reverse_ = this->declare_parameter<bool>("thruster.right_reverse", false);

  DoyleParams mmg;
  mmg.Lpp = this->declare_parameter<double>("mmg.Lpp", 1.0);
  mmg.D = this->declare_parameter<double>("mmg.D", 0.25);
  mmg.mass_kg = this->declare_parameter<double>("mmg.mass_kg", 10.0);
  mmg.I_zG = this->declare_parameter<double>("mmg.I_zG", 1.5);
  mmg.x_G = this->declare_parameter<double>("mmg.x_G", 0.0);
  mmg.m_x = this->declare_parameter<double>("mmg.m_x", 1.0);
  mmg.m_y = this->declare_parameter<double>("mmg.m_y", 10.0);
  mmg.J_z = this->declare_parameter<double>("mmg.J_z", 1.5);
  mmg.R_0 = this->declare_parameter<double>("mmg.R_0", -0.2);
  mmg.Y_0 = this->declare_parameter<double>("mmg.Y_0", 0.001);
  mmg.Y_v = this->declare_parameter<double>("mmg.Y_v", -0.2);
  mmg.Y_r = this->declare_parameter<double>("mmg.Y_r", 0.02);
  mmg.N_0 = this->declare_parameter<double>("mmg.N_0", 0.001);
  mmg.N_v = this->declare_parameter<double>("mmg.N_v", -0.2);
  mmg.N_r = this->declare_parameter<double>("mmg.N_r", -0.02);

  const double disturbance_hz = this->declare_parameter<double>(
    "disturbance.natural_frequency_hz",
    0.18);
  const double disturbance_zeta = this->declare_parameter<double>("disturbance.damping_ratio", 0.8);
  const double disturbance_sigma_u = this->declare_parameter<double>("disturbance.sigma_u", 0.12);
  const double disturbance_sigma_v = this->declare_parameter<double>("disturbance.sigma_v", 0.12);
  const double disturbance_sigma_r = this->declare_parameter<double>("disturbance.sigma_r", 0.04);
  const int disturbance_seed = this->declare_parameter<int>("disturbance.seed", 2026);

  t200_model_ = std::make_unique<T200Model>(max_forward, max_reverse);
  mmg_model_ = std::make_unique<MMGDoyleModel>(mmg);
  disturbance_model_ = std::make_unique<DisturbanceModel>(
    disturbance_hz,
    disturbance_zeta,
    disturbance_sigma_u,
    disturbance_sigma_v,
    disturbance_sigma_r,
    static_cast<std::uint32_t>(std::max(0, disturbance_seed)));

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom_, 10);
  sub_command_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    topic_thruster_command_,
    10,
    std::bind(&SimNode::commandCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  publishStaticTransforms();

  const double dt = 1.0 / std::max(1.0, update_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt),
    std::bind(&SimNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "dutyed_tf_pub_with_disturbance started");
}

void SimNode::commandCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "thruster_command requires at least 2 values [left, right]");
    return;
  }

  const double res = std::max(1.0, duty_resolution_);
  latest_left_duty_ = clamp(static_cast<double>(msg->data[0]) / res, -1.0, 1.0);
  latest_right_duty_ = clamp(static_cast<double>(msg->data[1]) / res, -1.0, 1.0);

  if (left_reverse_) {
    latest_left_duty_ *= -1.0;
  }
  if (right_reverse_) {
    latest_right_duty_ *= -1.0;
  }

  latest_duties_.resize(msg->data.size());
  for (size_t i = 0; i < msg->data.size(); ++i) {
    latest_duties_[i] = clamp(static_cast<double>(msg->data[i]) / res, -1.0, 1.0);
  }
}

void SimNode::publishStaticTransforms()
{
  if (!publish_tf_) {
    return;
  }

  geometry_msgs::msg::TransformStamped world_to_map;
  world_to_map.header.stamp = this->now();
  world_to_map.header.frame_id = frame_world_;
  world_to_map.child_frame_id = frame_map_;
  world_to_map.transform.translation.x = map_offset_x_;
  world_to_map.transform.translation.y = map_offset_y_;
  world_to_map.transform.translation.z = 0.0;
  tf2::Quaternion q_map;
  q_map.setRPY(0.0, 0.0, map_offset_yaw_);
  world_to_map.transform.rotation.x = q_map.x();
  world_to_map.transform.rotation.y = q_map.y();
  world_to_map.transform.rotation.z = q_map.z();
  world_to_map.transform.rotation.w = q_map.w();

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = this->now();
  map_to_odom.header.frame_id = frame_map_;
  map_to_odom.child_frame_id = frame_odom_;
  map_to_odom.transform.translation.x = odom_offset_x_;
  map_to_odom.transform.translation.y = odom_offset_y_;
  map_to_odom.transform.translation.z = 0.0;
  tf2::Quaternion q_odom;
  q_odom.setRPY(0.0, 0.0, odom_offset_yaw_);
  map_to_odom.transform.rotation.x = q_odom.x();
  map_to_odom.transform.rotation.y = q_odom.y();
  map_to_odom.transform.rotation.z = q_odom.z();
  map_to_odom.transform.rotation.w = q_odom.w();

  tf_static_broadcaster_->sendTransform(world_to_map);
  tf_static_broadcaster_->sendTransform(map_to_odom);
}

void SimNode::onTimer()
{
  const auto now = this->now();
  if (first_tick_) {
    first_tick_ = false;
    last_stamp_ = now;
    publishDynamicTfAndOdom(now);
    return;
  }

  const double dt = clamp((now - last_stamp_).seconds(), 1e-4, 0.1);
  last_stamp_ = now;

  PlanarInput input;
  if (latest_duties_.size() >= 4U) {
    const std::vector<double> duties(latest_duties_.begin(), latest_duties_.begin() + 4);
    const std::vector<SimThrusterGeometry> geometry = {
      {0.353553, -0.353553, 0.785398},
      {0.353553, 0.353553, -0.785398},
      {-0.353553, -0.353553, 2.35619},
      {-0.353553, 0.353553, -2.35619}};
    input = dutiesToPlanarInput(duties, geometry, *t200_model_);
  } else {
    const double left_force = t200_model_->forceFromDuty(latest_left_duty_);
    const double right_force = t200_model_->forceFromDuty(latest_right_duty_);
    input.surge_force = left_force + right_force;
    input.sway_force = 0.0;
    input.yaw_moment = (right_force - left_force) * half_beam_meter_;
  }

  PlanarAccel accel = mmg_model_->computeAccel(state_, input);
  const DisturbanceAccel disturbance = disturbance_model_->step(dt);
  accel.du += disturbance.du;
  accel.dv += disturbance.dv;
  accel.dr += disturbance.dr;

  state_.u += accel.du * dt;
  state_.v += accel.dv * dt;
  state_.r += accel.dr * dt;

  yaw_ += state_.r * dt;
  const double c = std::cos(yaw_);
  const double s = std::sin(yaw_);
  x_ += (state_.u * c - state_.v * s) * dt;
  y_ += (state_.u * s + state_.v * c) * dt;

  publishDynamicTfAndOdom(now);
}

void SimNode::publishDynamicTfAndOdom(const rclcpp::Time & stamp)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = frame_odom_;
  tf.child_frame_id = frame_base_link_;
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  if (publish_tf_) {
    tf_broadcaster_->sendTransform(tf);
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_odom_;
  odom.child_frame_id = frame_base_link_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = state_.u;
  odom.twist.twist.linear.y = state_.v;
  odom.twist.twist.angular.z = state_.r;
  pub_odom_->publish(odom);
}

}  // namespace sim
}  // namespace njord
