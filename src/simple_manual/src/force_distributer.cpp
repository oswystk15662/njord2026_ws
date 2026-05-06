// joy2force.cpp
// Component node: converts sensor_msgs::msg::Joy to per-thruster force commands

#include <memory>
#include <array>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace simple_manual
{

class Joy2Force : public rclcpp::Node
{
public:
	explicit Joy2Force(const rclcpp::NodeOptions & options)
	: Node("joy2force", options)
	{
		pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thrust_newton", 10);
		sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"joy", 10,
			std::bind(&Joy2Force::joy_callback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "Joy2Force component started");

		const float inv = 1.0f / std::sqrt(2.0f);
		// thruster unit directions (X-configuration, 45 deg offsets)
		thruster_dirs_ = {{{inv, inv}, {inv, -inv}, {-inv, -inv}, {-inv, inv}}};
		// thruster positions (radius = 0.4 m) measured from vehicle center
		const float r = 0.4f; // meters (400 mm)
		for (size_t i = 0; i < 4; ++i) {
			thruster_pos_[i][0] = r * thruster_dirs_[i][0];
			thruster_pos_[i][1] = r * thruster_dirs_[i][1];
		}
	}

private:
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		float axes0 = 0.0f;
		float axes1 = 0.0f;
		if (msg->axes.size() > 0) axes0 = msg->axes[0];
		if (msg->axes.size() > 1) axes1 = msg->axes[1];

		const float Fx = 0.2f * axes1;    // forward/back -> x
		const float Fy = -0.2f * axes0;   // left/right inverted -> y

		std::array<float, 4> f_raw{{0,0,0,0}};
		float Rx = 0.0f, Ry = 0.0f;
		for (size_t i = 0; i < 4; ++i) {
			const float ux = thruster_dirs_[i][0];
			const float uy = thruster_dirs_[i][1];
			// project desired force onto thruster thrust axis
			f_raw[i] = Fx * ux + Fy * uy;
			Rx += f_raw[i] * ux;
			Ry += f_raw[i] * uy;
		}

		const float magR = std::hypot(Rx, Ry);
		const float magF = std::hypot(Fx, Fy);
		if (magR > 1e-6f && magF > 1e-6f) {
			const float scale = magF / magR;
			for (auto &v : f_raw) v *= scale;
		}

		std_msgs::msg::Float32MultiArray out;
		out.data.resize(4);
		for (size_t i = 0; i < 4; ++i) out.data[i] = f_raw[i];
		pub_->publish(out);
	}

	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
	std::array<std::array<float,2>,4> thruster_dirs_;
	std::array<std::array<float,2>,4> thruster_pos_{}; // meter
};

}  // namespace simple_manual

RCLCPP_COMPONENTS_REGISTER_NODE(simple_manual::Joy2Force)
