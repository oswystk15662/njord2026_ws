#include "kinematics/node.hpp"
#include <cmath>
#include <algorithm>

namespace njord
{
namespace kinematics
{

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node("kinematics_node", options)
{
    init_parameters();

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&KinematicsNode::cmd_vel_callback, this, std::placeholders::_1));

    // ESCへの指令値 (PWM Duty相当) を配列で出力
    // micon_driver_fd はこれを受け取ってCAN/Serialに流す想定
    pub_duty_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "thruster_command", 10);

    RCLCPP_INFO(this->get_logger(), "Kinematics Node Initialized.");
}

void KinematicsNode::init_parameters()
{
    // 基本パラメータ
    this->declare_parameter("duty_resolution", 1000);
    this->declare_parameter("max_linear_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("linear_gain", 500.0);
    this->declare_parameter("angular_gain", 500.0);
    this->declare_parameter("thruster_ids", std::vector<std::string>({}));

    duty_resolution_ = this->get_parameter("duty_resolution").as_int();
    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    thruster_ids_ = this->get_parameter("thruster_ids").as_string_array();

    if (thruster_ids_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No thruster_ids provided in params.");
    }

    // 各スラスタのPose読み込み
    for (const auto & id : thruster_ids_) {
        ThrusterConfig t;
        t.id = id;
        
        // params: thrusters.<id>.pose
        std::string prefix = "thrusters." + id;
        this->declare_parameter(prefix + ".pose", std::vector<double>{0,0,0,0,0,0});
        this->declare_parameter(prefix + ".reverse", false);

        std::vector<double> pose = this->get_parameter(prefix + ".pose").as_double_array();
        t.reverse = this->get_parameter(prefix + ".reverse").as_bool();

        if (pose.size() == 6) {
            t.x = pose[0]; t.y = pose[1]; t.z = pose[2];
            t.roll = pose[3]; t.pitch = pose[4]; t.yaw = pose[5];
            
            calculate_contribution(t); // 運動学的寄与の計算
            thrusters_.push_back(t);
            
            RCLCPP_INFO(this->get_logger(), 
                "Loaded Thruster [%s]: xyz(%.2f, %.2f, %.2f) contribution(x:%.2f, yaw:%.2f)",
                id.c_str(), t.x, t.y, t.z, t.contribution_x, t.contribution_yaw);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid pose size for thruster %s", id.c_str());
        }
    }
}

void KinematicsNode::calculate_contribution(ThrusterConfig & t)
{
    // 単純化のため、2D平面(x, y, yaw)での寄与のみ計算します
    // スラスタの推力ベクトル F = [cos(yaw), sin(yaw)]
    
    double cos_yaw = std::cos(t.yaw);
    double sin_yaw = std::sin(t.yaw);

    // 1. 前進方向(Surge)への寄与: Fx = F * cos(yaw)
    t.contribution_x = cos_yaw;

    // 2. 旋回方向(Yaw)への寄与: Torque = r x F (2D cross product)
    // Torque = x * Fy - y * Fx
    // Fy = sin(yaw), Fx = cos(yaw)
    t.contribution_yaw = t.x * sin_yaw - t.y * cos_yaw;

    // ※注意: 右舷(y < 0)のスラスタが前進(cos=1)すると、
    // Torque = 0 - (-y)*1 = y > 0 (左回転) ??
    // 右手系では Z軸正方向(左回転)が正。
    // 右スラスタ(y=-0.3)が前進すると、船は左へ回ろうとする(正のトルク)。これで合っています。
}

void KinematicsNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 1. 入力制限
    double linear_cmd = std::clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
    double angular_cmd = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

    std_msgs::msg::Int16MultiArray output_msg;

    // 2. 各スラスタへの配分 (Allocation)
    for (const auto & t : thrusters_) {
        // 基本的なミキシング: (前進指令 * 前進寄与) + (旋回指令 * 旋回寄与 * 旋回ゲイン調整)
        // 旋回寄与(t.contribution_yaw) は "そのスラスタが正転したときに発生するトルク"
        // 逆に言えば、正のトルク(左旋回)をしたければ、正のトルクを生むスラスタを回し、負のトルクを生むスラスタを逆転させる。
        
        // Target Force = LinearCmd * LinearGain
        // Target Torque = AngularCmd * AngularGain
        
        // 簡易的な加算混合制御
        double thrust = (linear_cmd * linear_gain_ * t.contribution_x) + 
                        (angular_cmd * angular_gain_ * t.contribution_yaw);

        // ※ contribution_yaw の符号について:
        // 左スラスタ(y>0) -> contribution_yaw < 0 (右回転モーメント)
        // 左旋回(angular_cmd > 0) したい時、左スラスタは引くべき(thrust < 0)
        // 式: (+cmd) * (-contrib) = -thrust (後進) -> 合っている。
        
        if (t.reverse) {
            thrust *= -1.0;
        }

        // 3. クリップ & 整数化
        int16_t duty = static_cast<int16_t>(std::clamp(thrust, (double)-duty_resolution_, (double)duty_resolution_));
        output_msg.data.push_back(duty);
    }

    pub_duty_->publish(output_msg);
}

} // namespace kinematics
} // namespace njord

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord::kinematics::KinematicsNode)