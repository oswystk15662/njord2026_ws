#ifndef DUTYED_TF_PUB_WITH_DISTURBANCE__SIM_NODE_HPP_
#define DUTYED_TF_PUB_WITH_DISTURBANCE__SIM_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "dutyed_tf_pub_with_disturbance/disturbance_model.hpp"
#include "dutyed_tf_pub_with_disturbance/mmg_doyle_model.hpp"
#include "dutyed_tf_pub_with_disturbance/t200_model.hpp"

namespace njord
{
namespace sim
{

class SimNode : public rclcpp::Node
{
public:
  explicit SimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void commandCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void onTimer();

  void publishStaticTransforms();
  void publishDynamicTfAndOdom(const rclcpp::Time & stamp);

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::string topic_thruster_command_;
  std::string topic_odom_;
  std::string frame_world_;
  std::string frame_map_;
  std::string frame_odom_;
  std::string frame_base_link_;

  double duty_resolution_{1000.0};
  double update_rate_hz_{50.0};
  double half_beam_meter_{0.35};
  bool left_reverse_{false};
  bool right_reverse_{false};
  bool publish_tf_{true};

  double latest_left_duty_{0.0};
  double latest_right_duty_{0.0};
  std::vector<double> latest_duties_;

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  PlanarState state_{};

  std::unique_ptr<T200Model> t200_model_;
  std::unique_ptr<MMGDoyleModel> mmg_model_;
  std::unique_ptr<DisturbanceModel> disturbance_model_;

  rclcpp::Time last_stamp_;
  bool first_tick_{true};

  double map_offset_x_{0.0};
  double map_offset_y_{0.0};
  double map_offset_yaw_{0.0};
  double odom_offset_x_{0.0};
  double odom_offset_y_{0.0};
  double odom_offset_yaw_{0.0};
};

}  // namespace sim
}  // namespace njord

#endif  // DUTYED_TF_PUB_WITH_DISTURBANCE__SIM_NODE_HPP_
