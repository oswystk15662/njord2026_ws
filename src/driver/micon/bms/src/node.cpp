#include "bms/node.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <sstream>
#include <string>

namespace njord
{
namespace bms
{

BmsNode::BmsNode(const rclcpp::NodeOptions & options)
: Node("bms_node", options)
{
  const std::string input_topic =
    this->declare_parameter<std::string>("topics.input", "micon/bms_cells");
  const std::string output_topic =
    this->declare_parameter<std::string>("topics.output", "bms/cell_voltages");
  const std::string diagnostics_topic =
    this->declare_parameter<std::string>("topics.diagnostics", "/diagnostics");
  warning_voltage_ = this->declare_parameter<double>("low_voltage.warning", 3.5);
  error_voltage_ = this->declare_parameter<double>("low_voltage.error", 3.3);

  pub_cells_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic, 10);
  pub_diagnostics_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
  sub_cells_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    input_topic, 10, std::bind(&BmsNode::cellsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "bms node started. input=%s output=%s",
    input_topic.c_str(), output_topic.c_str());
}

void BmsNode::cellsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 4U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "BMS cell message size=%zu. Expected 4.",
      msg->data.size());
    return;
  }

  const std::array<float, 4> cells{
    msg->data[0],
    msg->data[1],
    msg->data[2],
    msg->data[3]};
  publishCells(cells);
  publishDiagnostics(cells);
}

void BmsNode::publishCells(const std::array<float, 4> & cells)
{
  std_msgs::msg::Float32MultiArray out;
  out.data = {cells[0], cells[1], cells[2], cells[3]};
  pub_cells_->publish(out);
}

void BmsNode::publishDiagnostics(const std::array<float, 4> & cells)
{
  const auto min_it = std::min_element(cells.begin(), cells.end());
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "bms/cell_voltage";
  status.hardware_id = "micon";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";

  if (*min_it < error_voltage_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "cell voltage below error threshold";
  } else if (*min_it < warning_voltage_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "cell voltage below warning threshold";
  }

  for (size_t i = 0; i < cells.size(); ++i) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "cell" + std::to_string(i + 1) + "_voltage";
    std::ostringstream ss;
    ss << cells[i];
    kv.value = ss.str();
    status.values.push_back(kv);
  }

  diagnostic_msgs::msg::KeyValue min_kv;
  min_kv.key = "min_cell_voltage";
  std::ostringstream min_ss;
  min_ss << *min_it;
  min_kv.value = min_ss.str();
  status.values.push_back(min_kv);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.status.push_back(status);
  pub_diagnostics_->publish(array);
}

}  // namespace bms
}  // namespace njord

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(njord::bms::BmsNode)
