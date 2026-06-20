// joy_converter.cpp
// Convert sensor_msgs::msg::Joy to velocity and operator-control topics.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class JoyConverter : public rclcpp::Node
{
public:
  JoyConverter()
  : Node("joy_converter")
  {
    sub_ =
      this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyConverter::joy_cb, this, _1));
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_emg_ = this->create_publisher<std_msgs::msg::Bool>("/emg", 10);
    pub_green_ = this->create_publisher<std_msgs::msg::Bool>("/green", 10);
    pub_yellow_ = this->create_publisher<std_msgs::msg::Bool>("/yellow", 10);
    pub_red_ = this->create_publisher<std_msgs::msg::Bool>("/red", 10);
    RCLCPP_INFO(this->get_logger(), "joy_converter started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    float axes0 = 0.0f;
    float axes1 = 0.0f;
    if (msg->axes.size() > 0) {axes0 = msg->axes[0];}
    if (msg->axes.size() > 1) {axes1 = msg->axes[1];}

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.2 * axes1;
    cmd_vel.linear.y = 0.2 * axes0;

    // torque from buttons: button index 6 = cw, 7 = ccw. Total magnitude 0.2 N*m
    bool b6 = (msg->buttons.size() > 6) ? (msg->buttons[6] != 0) : false;
    bool b7 = (msg->buttons.size() > 7) ? (msg->buttons[7] != 0) : false;
    cmd_vel.angular.z = 0.2 * (static_cast<double>(b6) - static_cast<double>(b7));
    pub_cmd_vel_->publish(cmd_vel);

    // publish emg and LEDs
    std_msgs::msg::Bool emg_msg;
    emg_msg.data = (msg->buttons.size() > 0) ? (msg->buttons[0] != 1) : true;
    std_msgs::msg::Bool green_msg;
    green_msg.data = (msg->buttons.size() > 1) ? (msg->buttons[1] != 0) : false;
    std_msgs::msg::Bool yellow_msg;
    yellow_msg.data = (msg->buttons.size() > 2) ? (msg->buttons[2] != 0) : false;
    std_msgs::msg::Bool red_msg;
    red_msg.data = (msg->buttons.size() > 3) ? (msg->buttons[3] != 0) : false;

    pub_emg_->publish(emg_msg);
    pub_green_->publish(green_msg);
    pub_yellow_->publish(yellow_msg);
    pub_red_->publish(red_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_emg_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_green_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_yellow_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_red_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
