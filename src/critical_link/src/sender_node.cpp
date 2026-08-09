#include "critical_link/io.hpp"
#include "critical_link/joy_codec.hpp"
#include "critical_link/protocol.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"

namespace critical_link
{
namespace
{

uint64_t steady_milliseconds()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint64_t make_session_id()
{
  std::random_device random;
  const uint64_t entropy = (static_cast<uint64_t>(random()) << 32U) ^ random();
  return entropy ^ (steady_milliseconds() << 16U) ^ static_cast<uint64_t>(getpid());
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class CriticalLinkSender : public rclcpp::Node
{
public:
  CriticalLinkSender()
  : Node("critical_link_sender"), session_id_(make_session_id())
  {
    joy_topic_ = declare_parameter<std::string>(
      "joy_input_topic", "/critical_link/input/joy");
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_input_topic", "/critical_link/input/heartbeat");
    const auto configured_source_id = declare_parameter<int64_t>("source_id", 1);
    if (configured_source_id <= 0 || configured_source_id > std::numeric_limits<uint32_t>::max()) {
      throw std::runtime_error("source_id must be an unsigned 32-bit value greater than zero");
    }
    source_id_ = static_cast<uint32_t>(configured_source_id);
    deadman_button_ = declare_parameter<int64_t>("deadman_button", -1);
    takeover_button_ = declare_parameter<int64_t>("takeover_button", -1);
    if (deadman_button_ < -1 || takeover_button_ < -1) {
      throw std::runtime_error("deadman_button and takeover_button must be -1 or a Joy button index");
    }
    serial_device_ = declare_parameter<std::string>("serial_device", "");
    serial_baud_ = declare_parameter<int>("serial_baud", 921600);

    for (const auto & text : declare_parameter<std::vector<std::string>>(
        "udp_paths", std::vector<std::string>{}))
    {
      const auto spec = parse_udp_sender_spec(text);
      if (!spec) {
        throw std::runtime_error(
                "invalid udp_paths entry; expected name|bind_ipv4|destination_ipv4|port: " + text);
      }
      udp_paths_.push_back(std::make_unique<UdpPath>(*spec));
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::SharedPtr message) {send_joy(*message);});
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      heartbeat_topic_, 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        send(StreamId::kGroundHeartbeat, {}, 0U);
      });
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    probe_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
          send(StreamId::kLinkProbe, {}, 0U);
          publish_diagnostics();
    });

    RCLCPP_INFO(
      get_logger(), "critical-link sender source=%u session=%llu udp_paths=%zu serial=%s",
      source_id_,
      static_cast<unsigned long long>(session_id_), udp_paths_.size(),
      serial_device_.empty() ? "disabled" : serial_device_.c_str());
  }

  ~CriticalLinkSender() override
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  struct UdpPath
  {
    explicit UdpPath(UdpSenderSpec input_spec)
    : spec(std::move(input_spec)) {}

    ~UdpPath()
    {
      if (fd >= 0) {
        close(fd);
      }
    }

    bool send_frame(const std::vector<uint8_t> & bytes)
    {
      const auto now = std::chrono::steady_clock::now();
      if (fd < 0) {
        if (now < next_open_attempt) {
          ++failed;
          return false;
        }
        fd = open_udp_sender(spec);
        next_open_attempt = now + std::chrono::seconds(1);
        if (fd < 0) {
          last_errno = errno;
          ++failed;
          return false;
        }
      }
      const ssize_t result = ::send(fd, bytes.data(), bytes.size(), 0);
      if (result == static_cast<ssize_t>(bytes.size())) {
        ++sent;
        return true;
      }
      last_errno = errno;
      ++failed;
      close(fd);
      fd = -1;
      return false;
    }

    UdpSenderSpec spec;
    int fd{-1};
    uint64_t sent{0};
    uint64_t failed{0};
    int last_errno{0};
    std::chrono::steady_clock::time_point next_open_attempt{};
  };

  void send_joy(const sensor_msgs::msg::Joy & joy)
  {
    const auto payload = encode_joy_payload(joy);
    if (!payload) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting Joy with too many fields or non-finite axis");
      return;
    }
    uint16_t flags = 0U;
    const bool control_active = deadman_button_ < 0 ||
      (static_cast<size_t>(deadman_button_) < joy.buttons.size() && joy.buttons[deadman_button_] != 0);
    if (control_active) {
      flags |= kFlagControlActive;
    }
    const bool takeover_pressed = takeover_button_ >= 0 &&
      static_cast<size_t>(takeover_button_) < joy.buttons.size() && joy.buttons[takeover_button_] != 0;
    if (takeover_pressed && !takeover_pressed_) {
      flags |= kFlagTakeoverRequest;
    }
    takeover_pressed_ = takeover_pressed;
    send(StreamId::kJoy, *payload, flags);
  }

  void send(StreamId stream, std::vector<uint8_t> payload, uint16_t flags)
  {
    const size_t index = static_cast<size_t>(stream);
    Frame frame;
    frame.stream = stream;
    frame.flags = flags;
    frame.source_id = source_id_;
    frame.session_id = session_id_;
    frame.sequence = ++sequences_[index];
    frame.source_monotonic_ms = steady_milliseconds();
    frame.payload = std::move(payload);
    const auto bytes = encode_frame(frame);

    for (auto & path : udp_paths_) {
      path->send_frame(bytes);
    }
    send_serial(bytes);
    ++generated_[index];
  }

  void send_serial(const std::vector<uint8_t> & bytes)
  {
    if (serial_device_.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(serial_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (serial_fd_ < 0) {
      if (now < serial_next_open_attempt_) {
        ++serial_failed_;
        return;
      }
      serial_fd_ = open_serial_port(serial_device_, serial_baud_);
      serial_next_open_attempt_ = now + std::chrono::seconds(1);
      if (serial_fd_ < 0) {
        serial_last_errno_ = errno;
        ++serial_failed_;
        return;
      }
    }
    if (write_all(serial_fd_, bytes.data(), bytes.size())) {
      ++serial_sent_;
      return;
    }
    serial_last_errno_ = errno;
    ++serial_failed_;
    close(serial_fd_);
    serial_fd_ = -1;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    for (const auto & path : udp_paths_) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "critical_link/sender/" + path->spec.name;
      status.hardware_id = path->spec.bind_address;
      status.level = path->fd >= 0 ? status.OK : status.WARN;
      status.message = path->fd >= 0 ? "connected UDP path" : "UDP path unavailable";
      status.values.push_back(key_value("sent", std::to_string(path->sent)));
      status.values.push_back(key_value("failed", std::to_string(path->failed)));
      status.values.push_back(key_value("last_errno", std::to_string(path->last_errno)));
      array.status.push_back(std::move(status));
    }
    if (!serial_device_.empty()) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "critical_link/sender/espnow_serial";
      status.hardware_id = serial_device_;
      status.level = serial_fd_ >= 0 ? status.OK : status.WARN;
      status.message = serial_fd_ >= 0 ? "serial path open" : "serial path unavailable";
      status.values.push_back(key_value("sent", std::to_string(serial_sent_)));
      status.values.push_back(key_value("failed", std::to_string(serial_failed_)));
      status.values.push_back(key_value("last_errno", std::to_string(serial_last_errno_)));
      array.status.push_back(std::move(status));
    }
    diagnostics_pub_->publish(array);
  }

  std::string joy_topic_;
  std::string heartbeat_topic_;
  uint32_t source_id_{1};
  int64_t deadman_button_{-1};
  int64_t takeover_button_{-1};
  bool takeover_pressed_{false};
  std::string serial_device_;
  int serial_baud_{921600};
  uint64_t session_id_{0};
  std::array<uint32_t, 4> sequences_{};
  std::array<uint64_t, 4> generated_{};
  std::vector<std::unique_ptr<UdpPath>> udp_paths_;
  std::mutex serial_mutex_;
  int serial_fd_{-1};
  uint64_t serial_sent_{0};
  uint64_t serial_failed_{0};
  int serial_last_errno_{0};
  std::chrono::steady_clock::time_point serial_next_open_attempt_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr probe_timer_;
};

}  // namespace critical_link

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<critical_link::CriticalLinkSender>());
  rclcpp::shutdown();
  return 0;
}
