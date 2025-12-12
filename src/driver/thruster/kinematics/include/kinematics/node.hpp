#ifndef KINEMATICS_NODE_HPP_
#define KINEMATICS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vector>
#include <string>
#include <map>

namespace njord
{
namespace kinematics
{

struct ThrusterConfig {
    std::string id;
    double x, y, z;
    double roll, pitch, yaw;
    bool reverse;
    
    // 運動学的な寄与係数 (初期化時に計算)
    double contribution_x;   // 前進力への寄与
    double contribution_yaw; // 旋回力への寄与
};

class KinematicsNode : public rclcpp::Node
{
public:
    explicit KinematicsNode(const rclcpp::NodeOptions & options);
    ~KinematicsNode() = default;

private:
    void init_parameters();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void calculate_contribution(ThrusterConfig & thruster);

    // Subscribers & Publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_duty_;

    // Configs
    int duty_resolution_;
    double max_linear_vel_;
    double max_angular_vel_;
    double linear_gain_;
    double angular_gain_;

    std::vector<std::string> thruster_ids_;
    std::vector<ThrusterConfig> thrusters_;
};

} // namespace kinematics
} // namespace njord

#endif // KINEMATICS_NODE_HPP_