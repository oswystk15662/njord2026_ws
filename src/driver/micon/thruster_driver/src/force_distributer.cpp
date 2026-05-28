#include "thruster_driver/force_distributer.hpp"

#include <cmath>

using std::placeholders::_1;

namespace njord
{
namespace thruster_driver
{

ForceDistributerNode::ForceDistributerNode()
: Node("force_distributer")
{
  pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thrust_newton", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&ForceDistributerNode::joy_callback, this, _1));

  const float inv = 1.0f / std::sqrt(2.0f);
  thruster_dirs_ = {{{inv, inv}, {inv, -inv}, {-inv, -inv}, {-inv, inv}}};
  const float r = 0.4f;
  for (size_t i = 0; i < 4; ++i) {
    thruster_pos_[i][0] = r * thruster_dirs_[i][0];
    thruster_pos_[i][1] = r * thruster_dirs_[i][1];
  }
}

void ForceDistributerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  float axes0 = 0.0f;
  float axes1 = 0.0f;
  if (msg->axes.size() > 0) {
    axes0 = msg->axes[0];
  }
  if (msg->axes.size() > 1) {
    axes1 = msg->axes[1];
  }

  const float Fx = 0.2f * axes1;
  const float Fy = -0.2f * axes0;

  std::array<float, 4> f_raw{{0.0f, 0.0f, 0.0f, 0.0f}};
  float Rx = 0.0f;
  float Ry = 0.0f;
  for (size_t i = 0; i < 4; ++i) {
    const float ux = thruster_dirs_[i][0];
    const float uy = thruster_dirs_[i][1];
    f_raw[i] = Fx * ux + Fy * uy;
    Rx += f_raw[i] * ux;
    Ry += f_raw[i] * uy;
  }

  const float magR = std::hypot(Rx, Ry);
  const float magF = std::hypot(Fx, Fy);
  if (magR > 1e-6f && magF > 1e-6f) {
    const float scale = magF / magR;
    for (auto & v : f_raw) {
      v *= scale;
    }
  }

  std_msgs::msg::Float32MultiArray out;
  out.data.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    out.data[i] = f_raw[i];
  }
  pub_->publish(out);
}

}  // namespace thruster_driver
}  // namespace njord

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<njord::thruster_driver::ForceDistributerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
