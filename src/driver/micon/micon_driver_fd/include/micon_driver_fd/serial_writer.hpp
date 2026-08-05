#ifndef MICON_DRIVER_FD__SERIAL_WRITER_HPP_
#define MICON_DRIVER_FD__SERIAL_WRITER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace micon_driver_fd
{

struct Flags
{
  bool emergency{false};
  bool red{false};
  bool yellow{false};
  bool green{false};
};

enum class EmergencyStopState : uint8_t
{
  RUNNING = 0,
  SOFT_EMG = 1,
  HARD_EMG = 2,
};

constexpr uint8_t kProtocolVersion = 0x01;
constexpr uint8_t kThrusterCommandType = 0x01;
constexpr size_t kHeaderSize = 5;
constexpr size_t kCrcSize = 2;
constexpr size_t kPayloadSize = 4 * sizeof(float) + 1;
constexpr size_t kRawFrameSize = kHeaderSize + kPayloadSize + kCrcSize;
constexpr size_t kMaxEncodedFrameSize = kRawFrameSize + 1;
constexpr size_t kPacketSize = kMaxEncodedFrameSize + 1;
using Packet = std::vector<uint8_t>;

Packet encode_packet(
  const std::array<float, 4> & thrust,
  const Flags & flags,
  uint16_t sequence = 0);

// Parses one CSV line emitted by Docs/njord_BMS_v0/master.  The first field is
// a timestamp, followed by the four cell voltages in volts.
bool parse_bms_csv_line(const std::string & line, std::array<float, 4> * cells);

class SerialWriter : public rclcpp::Node
{
public:
  explicit SerialWriter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SerialWriter() override;

private:
  void thrust_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void soft_emg_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void red_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void yellow_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void green_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void ground_station_heartbeat_cb(const std_msgs::msg::Empty::SharedPtr msg);
  void timer_cb();
  void update_ground_station_watchdog();
  void read_bms();
  void publish_safety_state();
  int open_serial(const std::string & device, int baud);

  Flags flags_;
  std::array<float, 4> thrust_{{0.0f, 0.0f, 0.0f, 0.0f}};
  std::mutex mutex_;
  uint16_t sequence_{0};
  int fd_{-1};
  std::string serial_port_;
  int baud_{115200};
  std::string command_topic_;
  std::string bms_topic_;
  std::string ground_station_heartbeat_topic_;
  double ground_station_heartbeat_timeout_sec_{0.0};
  std::string serial_rx_buffer_;
  bool soft_emg_{false};
  bool ground_station_heartbeat_received_{false};
  bool ground_station_timeout_emg_{false};
  bool relay_active_{false};
  std::chrono::steady_clock::time_point last_ground_station_heartbeat_{};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_thrust_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_soft_emg_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_red_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_yellow_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_green_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_ground_station_heartbeat_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bms_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_relay_active_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_safety_emergency_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace micon_driver_fd

#endif  // MICON_DRIVER_FD__SERIAL_WRITER_HPP_
