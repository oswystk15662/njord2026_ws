#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

namespace manual_with_odom
{

class JoyConverter : public rclcpp::Node
{
public:
  JoyConverter(const rclcpp::NodeOptions & options)
  : Node("joy_converter", options)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoyConverter::joy_cb, this, _1));
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/desired_twist", 10);
    pub_emg_ = this->create_publisher<std_msgs::msg::Bool>("/emg", 10);
    pub_green_ = this->create_publisher<std_msgs::msg::Bool>("/green", 10);
    pub_yellow_ = this->create_publisher<std_msgs::msg::Bool>("/yellow", 10);
    pub_red_ = this->create_publisher<std_msgs::msg::Bool>("/red", 10);
    RCLCPP_INFO(this->get_logger(), "manual_with_odom::joy_converter started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    float axes0 = 0.0f;
    float axes1 = 0.0f;
    if (msg->axes.size() > 0) axes0 = msg->axes[0];
    if (msg->axes.size() > 1) axes1 = msg->axes[1];

    geometry_msgs::msg::Twist t;
    t.linear.x = 0.2f * axes1;    // forward/back -> x
    t.linear.y = -0.2f * axes0;   // left/right inverted -> y
    t.linear.z = 0.0;
    t.angular.x = 0.0; t.angular.y = 0.0; t.angular.z = 0.0;

    pub_twist_->publish(t);

    std_msgs::msg::Bool emg_msg; emg_msg.data = (msg->buttons.size() > 0) ? (msg->buttons[0] != 0) : false;
    std_msgs::msg::Bool green_msg; green_msg.data = (msg->buttons.size() > 2) ? (msg->buttons[2] != 0) : false;
    std_msgs::msg::Bool yellow_msg; yellow_msg.data = (msg->buttons.size() > 3) ? (msg->buttons[3] != 0) : false;
    std_msgs::msg::Bool red_msg; red_msg.data = (msg->buttons.size() > 4) ? (msg->buttons[4] != 0) : false;

    pub_emg_->publish(emg_msg);
    pub_green_->publish(green_msg);
    pub_yellow_->publish(yellow_msg);
    pub_red_->publish(red_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_emg_, pub_green_, pub_yellow_, pub_red_;
};

} // namespace manual_with_odom

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(manual_with_odom::JoyConverter)
