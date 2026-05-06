#include <memory>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

namespace manual_with_odom
{

class PIDController : public rclcpp::Node
{
public:
  PIDController(const rclcpp::NodeOptions & options)
  : Node("pid_force_controller", options)
  {
    this->declare_parameter<double>("kp", 10.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("max_force", 0.2);

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    max_force_ = this->get_parameter("max_force").as_double();

    sub_des_ = this->create_subscription<geometry_msgs::msg::Twist>("/desired_twist", 10, std::bind(&PIDController::des_cb, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 50, std::bind(&PIDController::odom_cb, this, std::placeholders::_1));

    pub_force_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/force_cmd", 10);

    last_time_ = now();
    timer_ = this->create_wall_timer(50ms, std::bind(&PIDController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "pid_force_controller started (kp=%.2f, ki=%.2f, kd=%.2f, max_force=%.3f)", kp_, ki_, kd_, max_force_);
  }

private:
  void des_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    desired_x_ = msg->linear.x;
    desired_y_ = msg->linear.y;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    // odom provides twist in child_frame (base_link) in frame of odom/map as appropriate
    current_vx_ = msg->twist.twist.linear.x;
    current_vy_ = msg->twist.twist.linear.y;
  }

  void control_loop()
  {
    const auto now_t = now();
    double dt = (now_t - last_time_).seconds();
    if (dt <= 0.0) dt = 1e-3;

    double des_x, des_y, cur_x, cur_y;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      des_x = desired_x_; des_y = desired_y_;
      cur_x = current_vx_; cur_y = current_vy_;
    }

    double err_x = des_x - cur_x;
    double err_y = des_y - cur_y;

    // integrate
    integ_x_ += err_x * dt;
    integ_y_ += err_y * dt;
    // derivative
    double derr_x = (err_x - last_err_x_) / dt;
    double derr_y = (err_y - last_err_y_) / dt;

    double fx = kp_ * err_x + ki_ * integ_x_ + kd_ * derr_x;
    double fy = kp_ * err_y + ki_ * integ_y_ + kd_ * derr_y;

    // clamp per-axis to max_force_
    if (fx > max_force_) fx = max_force_;
    if (fx < -max_force_) fx = -max_force_;
    if (fy > max_force_) fy = max_force_;
    if (fy < -max_force_) fy = -max_force_;

    last_err_x_ = err_x; last_err_y_ = err_y;
    last_time_ = now_t;

    std_msgs::msg::Float32MultiArray out;
    out.data.resize(2);
    out.data[0] = static_cast<float>(fx);
    out.data[1] = static_cast<float>(fy);
    pub_force_->publish(out);
  }

  rclcpp::Time now() { return this->get_clock()->now(); }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_des_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_force_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  double desired_x_{0.0}, desired_y_{0.0};
  double current_vx_{0.0}, current_vy_{0.0};

  double kp_=1.0, ki_=0.0, kd_=0.0;
  double integ_x_{0.0}, integ_y_{0.0};
  double last_err_x_{0.0}, last_err_y_{0.0};
  rclcpp::Time last_time_;
  double max_force_{0.2};
};

} // namespace manual_with_odom

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(manual_with_odom::PIDController)
