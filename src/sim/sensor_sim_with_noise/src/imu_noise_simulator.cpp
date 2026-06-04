#include <random>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

namespace njord
{
namespace sim
{

class ImuNoiseSimulatorNode : public rclcpp::Node
{
public:
  explicit ImuNoiseSimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("imu_noise_simulator", options),
    rng_(std::random_device{}())
  {
    // Retrieve parameters
    imu_orientation_noise_std_ = this->declare_parameter<double>("imu_orientation_noise_std", std::sqrt(0.0012));
    imu_angular_vel_noise_std_ = this->declare_parameter<double>("imu_angular_vel_noise_std", std::sqrt(0.000012));
    imu_linear_accel_noise_std_ = this->declare_parameter<double>("imu_linear_accel_noise_std", std::sqrt(0.012));

    double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 20.0);

    frame_id_imu_ = this->declare_parameter<std::string>("frame_id_imu", "imu_link");

    // Initialize publishers
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/wit/imu", 10);

    // Initialize subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ImuNoiseSimulatorNode::odomCallback, this, std::placeholders::_1));

    // Setup timer
    const double dt = 1.0 / std::max(1.0, publish_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&ImuNoiseSimulatorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "ImuNoiseSimulatorNode started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
    has_odom_ = true;
  }

  void timerCallback()
  {
    if (!has_odom_ || !latest_odom_) {
      return;
    }

    const auto now = this->now();
    const auto & odom = *latest_odom_;

    // Calculate time step dt for differentiation
    double dt = 0.05; // fallback
    if (last_time_.nanoseconds() != 0) {
      dt = (now - last_time_).seconds();
    }
    last_time_ = now;
    if (dt < 1e-4 || dt > 1.0) {
      dt = 0.05;
    }

    // Extract orientation
    tf2::Quaternion q(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Compute acceleration by differentiating body-frame velocities
    double u_curr = odom.twist.twist.linear.x;
    double v_curr = odom.twist.twist.linear.y;
    double r_curr = odom.twist.twist.angular.z;

    double du = 0.0;
    double dv = 0.0;
    if (has_prev_vel_) {
      du = (u_curr - prev_u_) / dt;
      dv = (v_curr - prev_v_) / dt;
    }
    prev_u_ = u_curr;
    prev_v_ = v_curr;
    has_prev_vel_ = true;
    has_prev_vel_ = true;

    // Generate Gaussian Noise
    std::normal_distribution<double> noise_imu_orient(0.0, imu_orientation_noise_std_);
    std::normal_distribution<double> noise_imu_gyro(0.0, imu_angular_vel_noise_std_);
    std::normal_distribution<double> noise_imu_accel(0.0, imu_linear_accel_noise_std_);

    // Publish IMU
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = frame_id_imu_;

    // IMU orientation (noisy yaw, add small noise to roll/pitch too)
    double noisy_roll = roll + noise_imu_orient(rng_);
    double noisy_pitch = pitch + noise_imu_orient(rng_);
    double noisy_yaw = yaw + noise_imu_orient(rng_);
    tf2::Quaternion q_noisy;
    q_noisy.setRPY(noisy_roll, noisy_pitch, noisy_yaw);
    imu_msg.orientation.x = q_noisy.x();
    imu_msg.orientation.y = q_noisy.y();
    imu_msg.orientation.z = q_noisy.z();
    imu_msg.orientation.w = q_noisy.w();

    // IMU angular velocity
    imu_msg.angular_velocity.x = noise_imu_gyro(rng_);
    imu_msg.angular_velocity.y = noise_imu_gyro(rng_);
    imu_msg.angular_velocity.z = r_curr + noise_imu_gyro(rng_);

    // IMU linear acceleration (including gravity on Z)
    imu_msg.linear_acceleration.x = du + noise_imu_accel(rng_);
    imu_msg.linear_acceleration.y = dv + noise_imu_accel(rng_);
    imu_msg.linear_acceleration.z = 9.80665 + noise_imu_accel(rng_);

    // Covariances (WIT901C covariances * 1.2)
    // original: orientation (0.001), gyro (0.00001), accel (0.01)
    for (int i = 0; i < 9; ++i) {
      imu_msg.orientation_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = 0.0012;
    imu_msg.orientation_covariance[4] = 0.0012;
    imu_msg.orientation_covariance[8] = 0.0012;

    imu_msg.angular_velocity_covariance[0] = 0.000012;
    imu_msg.angular_velocity_covariance[4] = 0.000012;
    imu_msg.angular_velocity_covariance[8] = 0.000012;

    imu_msg.linear_acceleration_covariance[0] = 0.012;
    imu_msg.linear_acceleration_covariance[4] = 0.012;
    imu_msg.linear_acceleration_covariance[8] = 0.012;

    pub_imu_->publish(imu_msg);
  }

  // Parameters
  double imu_orientation_noise_std_;
  double imu_angular_vel_noise_std_;
  double imu_linear_accel_noise_std_;
  std::string frame_id_imu_;

  // Publishers & Subscribers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  bool has_odom_{false};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  double prev_u_{0.0};
  double prev_v_{0.0};
  bool has_prev_vel_{false};

  // Random number generator
  std::mt19937 rng_;
};

} // namespace sim
} // namespace njord

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<njord::sim::ImuNoiseSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
