#ifndef MICON_DRIVER_FD__SERIAL_WRITER_HPP_
#define MICON_DRIVER_FD__SERIAL_WRITER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace micon_driver_fd
{

struct Flags
{
  bool emergency{false};
  bool red{false};
  bool yellow{false};
  bool green{false};
};

constexpr size_t kPacketSize = 4 * sizeof(float) + 1;
using Packet = std::array<uint8_t, kPacketSize>;

Packet encode_packet(const std::array<float, 4> & thrust, const Flags & flags);

struct BmsCells
{
  std::array<float, 4> volts{};
};

class RxParser
{
public:
  std::vector<BmsCells> push(const uint8_t * data, size_t size);
  std::vector<BmsCells> push(const std::vector<uint8_t> & data);

private:
  std::vector<uint8_t> buffer_;
};

constexpr uint8_t kRxStart = 0xAA;
constexpr uint8_t kRxTypeBms = 0x01;
constexpr uint8_t kRxBmsPayloadLen = 8;

std::uint16_t crc16_rx_frame(const uint8_t * data, size_t size);
std::vector<uint8_t> encode_bms_rx_frame(const std::array<std::uint16_t, 4> & cells_mv);

class SerialWriter : public rclcpp::Node
{
public:
  explicit SerialWriter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SerialWriter() override;

private:
  void thrust_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void emg_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void red_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void yellow_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void green_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_cb();
  int open_serial(const std::string & device, int baud);
  void rx_loop();
  void close_serial();
  void publish_bms(const BmsCells & cells);

  Flags flags_;
  std::array<float, 4> thrust_{{0.0f, 0.0f, 0.0f, 0.0f}};
  std::mutex mutex_;
  std::mutex fd_mutex_;
  int fd_{-1};
  std::string serial_port_;
  int baud_{115200};
  std::string command_topic_;
  std::atomic_bool running_{false};
  std::thread rx_thread_;
  RxParser rx_parser_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_thrust_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emg_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_red_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_yellow_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_green_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bms_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micon_driver_fd

#endif  // MICON_DRIVER_FD__SERIAL_WRITER_HPP_
