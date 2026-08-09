#include "critical_link/io.hpp"
#include "critical_link/joy_codec.hpp"
#include "critical_link/protocol.hpp"
#include "critical_link/source_arbiter.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class CriticalLinkReceiver : public rclcpp::Node
{
public:
  CriticalLinkReceiver()
  : Node("critical_link_receiver")
  {
    joy_output_topic_ = declare_parameter<std::string>("joy_output_topic", "/joy");
    heartbeat_output_topic_ = declare_parameter<std::string>(
      "heartbeat_output_topic", "/heartbeat/ground_station");
    serial_device_ = declare_parameter<std::string>("serial_device", "");
    serial_baud_ = declare_parameter<int>("serial_baud", 921600);
    stale_after_sec_ = declare_parameter<double>("diagnostic_stale_after_sec", 3.0);
    const double command_timeout_sec = declare_parameter<double>("source_command_timeout_sec", 0.25);
    const double heartbeat_timeout_sec = declare_parameter<double>("source_heartbeat_timeout_sec", 1.5);
    const double neutral_before_handover_sec = declare_parameter<double>(
      "neutral_before_handover_sec", 0.05);
    const double takeover_request_timeout_sec = declare_parameter<double>(
      "takeover_request_timeout_sec", 0.5);
    source_publish_period_sec_ = declare_parameter<double>("source_publish_period_sec", 0.05);
    heartbeat_publish_period_sec_ = declare_parameter<double>(
      "heartbeat_publish_period_sec", 1.0);
    if (!std::isfinite(stale_after_sec_) || stale_after_sec_ <= 0.0 ||
      !std::isfinite(command_timeout_sec) || command_timeout_sec <= 0.0 ||
      !std::isfinite(heartbeat_timeout_sec) || heartbeat_timeout_sec <= 0.0 ||
      !std::isfinite(neutral_before_handover_sec) || neutral_before_handover_sec < 0.0 ||
      !std::isfinite(takeover_request_timeout_sec) || takeover_request_timeout_sec <= 0.0 ||
      !std::isfinite(source_publish_period_sec_) || source_publish_period_sec_ <= 0.0 ||
      !std::isfinite(heartbeat_publish_period_sec_) || heartbeat_publish_period_sec_ <= 0.0)
    {
      throw std::runtime_error("critical-link arbitration timeouts must be finite and positive");
    }

    std::vector<SourceConfig> source_configs;
    for (const auto & text : declare_parameter<std::vector<std::string>>(
        "source_specs", std::vector<std::string>{"1|primary_ground|100"}))
    {
      const auto source = parse_source_spec(text);
      if (!source) {
        throw std::runtime_error(
                "invalid source_specs entry; expected source_id|name|priority: " + text);
      }
      source_configs.push_back(*source);
    }
    source_arbiter_ = std::make_unique<SourceArbiter>(
      std::move(source_configs),
      SourceArbiterConfig{
        static_cast<uint64_t>(command_timeout_sec * 1000.0),
        static_cast<uint64_t>(heartbeat_timeout_sec * 1000.0),
        static_cast<uint64_t>(neutral_before_handover_sec * 1000.0),
        static_cast<uint64_t>(takeover_request_timeout_sec * 1000.0),
      });

    for (const auto & text : declare_parameter<std::vector<std::string>>(
        "udp_paths", std::vector<std::string>{}))
    {
      const auto spec = parse_udp_receiver_spec(text);
      if (!spec) {
        throw std::runtime_error(
                "invalid udp_paths entry; expected name|bind_ipv4|port: " + text);
      }
      udp_paths_.push_back(std::make_unique<ReceivePath>(*spec));
    }

    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(joy_output_topic_, 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>(heartbeat_output_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    for (auto & path : udp_paths_) {
      path->thread = std::thread([this, path_ptr = path.get()]() {udp_loop(*path_ptr);});
    }
    if (!serial_device_.empty()) {
      serial_thread_ = std::thread([this]() {serial_loop();});
    }
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(source_publish_period_sec_)),
      [this]() {publish_arbitrated_control();});

    RCLCPP_INFO(
      get_logger(), "critical-link receiver udp_paths=%zu serial=%s sources=%zu",
      udp_paths_.size(), serial_device_.empty() ? "disabled" : serial_device_.c_str(),
      source_arbiter_->statuses(steady_milliseconds()).size());
  }

  ~CriticalLinkReceiver() override
  {
    running_ = false;
    for (auto & path : udp_paths_) {
      if (path->thread.joinable()) {
        path->thread.join();
      }
    }
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
  }

private:
  struct ReceivePath
  {
    explicit ReceivePath(UdpReceiverSpec input_spec)
    : spec(std::move(input_spec)) {}

    UdpReceiverSpec spec;
    std::thread thread;
    std::atomic<bool> open{false};
    std::atomic<uint64_t> received{0};
    std::atomic<uint64_t> invalid{0};
    std::atomic<uint64_t> last_receive_ms{0};
    std::atomic<int> last_errno{0};
  };

  void udp_loop(ReceivePath & path)
  {
    std::array<uint8_t, kMaxFrameSize + 1U> buffer{};
    while (running_) {
      const int fd = open_udp_receiver(path.spec);
      if (fd < 0) {
        path.last_errno = errno;
        path.open = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      path.open = true;
      while (running_) {
        const ssize_t size = recv(fd, buffer.data(), buffer.size(), 0);
        if (size > 0) {
          path.last_receive_ms = steady_milliseconds();
          ++path.received;
          if (!handle_datagram(buffer.data(), static_cast<size_t>(size))) {
            ++path.invalid;
          }
        } else if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          path.last_errno = errno;
          break;
        }
      }
      path.open = false;
      close(fd);
    }
  }

  void serial_loop()
  {
    FrameStreamDecoder decoder;
    std::array<uint8_t, 512> buffer{};
    while (running_) {
      const int fd = open_serial_port(serial_device_, serial_baud_);
      if (fd < 0) {
        serial_last_errno_ = errno;
        serial_open_ = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      serial_open_ = true;
      decoder.clear();
      while (running_) {
        const ssize_t size = read(fd, buffer.data(), buffer.size());
        if (size > 0) {
          serial_last_receive_ms_ = steady_milliseconds();
          const auto frames = decoder.push(buffer.data(), static_cast<size_t>(size));
          for (const auto & frame : frames) {
            ++serial_received_;
            handle_frame(frame);
          }
        } else if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          serial_last_errno_ = errno;
          break;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
      serial_open_ = false;
      close(fd);
    }
  }

  bool handle_datagram(const uint8_t * data, size_t size)
  {
    const auto frame = decode_frame(data, size);
    if (!frame) {
      return false;
    }
    handle_frame(*frame);
    return true;
  }

  void handle_frame(const Frame & frame)
  {
    std::optional<sensor_msgs::msg::Joy> joy;
    if (frame.stream == StreamId::kJoy) {
      joy = decode_joy_payload(frame.payload.data(), frame.payload.size());
      if (!joy) {
        ++invalid_payload_;
        return;
      }
    } else if (!frame.payload.empty()) {
      ++invalid_payload_;
      return;
    }

    const uint64_t receive_ms = steady_milliseconds();
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      if (!source_arbiter_->known_source(frame.source_id)) {
        ++unauthorized_source_;
        return;
      }
      if (!gate_.accept(frame.source_id, frame.session_id, frame.stream, frame.sequence)) {
        ++duplicates_or_old_;
        return;
      }
      source_arbiter_->accept(frame, joy, receive_ms);
    }

    if (frame.stream == StreamId::kJoy) {
      ++accepted_joy_;
    } else if (frame.stream == StreamId::kGroundHeartbeat) {
      ++accepted_heartbeat_;
    } else if (frame.stream == StreamId::kLinkProbe) {
      last_probe_ms_ = steady_milliseconds();
    }
  }

  void publish_arbitrated_control()
  {
    const uint64_t now_ms = steady_milliseconds();
    ArbitrationDecision decision;
    std::vector<SourceStatus> source_statuses;
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      decision = source_arbiter_->tick(now_ms);
      source_statuses = source_arbiter_->statuses(now_ms);
    }

    std::string source_name = "neutral";
    if (decision.selected_source) {
      for (const auto & status : source_statuses) {
        if (status.config.id == *decision.selected_source) {
          source_name = status.config.name;
          break;
        }
      }
    }
    decision.joy.header.stamp = now();
    decision.joy.header.frame_id = "critical_link/" + source_name;
    joy_pub_->publish(decision.joy);
    if (decision.selected_source) {
      last_joy_ms_ = now_ms;
    }

    if (decision.ground_station_alive &&
      (last_heartbeat_publish_ms_ == 0U ||
      now_ms - last_heartbeat_publish_ms_ >=
      static_cast<uint64_t>(heartbeat_publish_period_sec_ * 1000.0)))
    {
      heartbeat_pub_->publish(std_msgs::msg::Empty{});
      last_heartbeat_publish_ms_ = now_ms;
      last_heartbeat_ms_ = now_ms;
    }
  }

  diagnostic_msgs::msg::DiagnosticStatus make_path_status(
    const std::string & name, const std::string & hardware_id, bool open,
    uint64_t received, uint64_t invalid, uint64_t last_receive_ms, int last_errno) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "critical_link/receiver/" + name;
    status.hardware_id = hardware_id;
    const uint64_t now_ms = steady_milliseconds();
    const bool stale = last_receive_ms == 0U ||
      now_ms - last_receive_ms > static_cast<uint64_t>(stale_after_sec_ * 1000.0);
    status.level = !open ? status.WARN : stale ? status.WARN : status.OK;
    status.message = !open ? "path unavailable" : stale ? "path has no recent frames" : "receiving";
    status.values.push_back(key_value("received", std::to_string(received)));
    status.values.push_back(key_value("invalid", std::to_string(invalid)));
    status.values.push_back(key_value("last_errno", std::to_string(last_errno)));
    status.values.push_back(key_value("stale", stale ? "true" : "false"));
    return status;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    for (const auto & path : udp_paths_) {
      array.status.push_back(make_path_status(
          path->spec.name, path->spec.bind_address, path->open,
          path->received, path->invalid, path->last_receive_ms, path->last_errno));
    }
    if (!serial_device_.empty()) {
      array.status.push_back(make_path_status(
          "espnow_serial", serial_device_, serial_open_, serial_received_, 0,
          serial_last_receive_ms_, serial_last_errno_));
    }

    const uint64_t now_ms = steady_milliseconds();
    std::vector<SourceStatus> source_statuses;
    {
      std::lock_guard<std::mutex> lock(gate_mutex_);
      source_statuses = source_arbiter_->statuses(now_ms);
    }
    std::string active_source = "none";
    bool selected_command_current = false;
    for (const auto & source : source_statuses) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "critical_link/receiver/source/" + source.config.name;
      status.hardware_id = std::to_string(source.config.id);
      status.level = source.selected ? status.OK :
        source.command_fresh ? status.WARN : status.STALE;
      status.message = source.selected ? "selected command source" :
        source.command_fresh ? "command source available but not selected" : "command source stale";
      status.values.push_back(key_value("priority", std::to_string(source.config.priority)));
      status.values.push_back(key_value("command_fresh", source.command_fresh ? "true" : "false"));
      status.values.push_back(key_value("heartbeat_fresh", source.heartbeat_fresh ? "true" : "false"));
      status.values.push_back(key_value("control_active", source.control_active ? "true" : "false"));
      status.values.push_back(key_value("last_command_ms", std::to_string(source.last_command_ms)));
      status.values.push_back(key_value("last_heartbeat_ms", std::to_string(source.last_heartbeat_ms)));
      if (source.selected) {
        active_source = source.config.name;
        selected_command_current = true;
      }
      array.status.push_back(std::move(status));
    }

    diagnostic_msgs::msg::DiagnosticStatus aggregate;
    aggregate.name = "critical_link/receiver/aggregate";
    aggregate.hardware_id = "critical_link";
    const bool joy_stale = !selected_command_current;
    aggregate.level = joy_stale ? aggregate.WARN : aggregate.OK;
    aggregate.message = joy_stale ? "no selected fresh Joy source" : "selected Joy source is current";
    aggregate.values.push_back(key_value("accepted_joy", std::to_string(accepted_joy_)));
    aggregate.values.push_back(
      key_value("accepted_heartbeat", std::to_string(accepted_heartbeat_)));
    aggregate.values.push_back(
      key_value("duplicate_or_old", std::to_string(duplicates_or_old_)));
    aggregate.values.push_back(
      key_value("unauthorized_source", std::to_string(unauthorized_source_)));
    aggregate.values.push_back(key_value("invalid_payload", std::to_string(invalid_payload_)));
    aggregate.values.push_back(key_value("active_source", active_source));
    array.status.push_back(std::move(aggregate));
    diagnostics_pub_->publish(array);
  }

  std::string joy_output_topic_;
  std::string heartbeat_output_topic_;
  std::string serial_device_;
  int serial_baud_{921600};
  double stale_after_sec_{3.0};
  double source_publish_period_sec_{0.05};
  double heartbeat_publish_period_sec_{1.0};
  std::atomic<bool> running_{true};
  std::vector<std::unique_ptr<ReceivePath>> udp_paths_;
  std::thread serial_thread_;
  std::atomic<bool> serial_open_{false};
  std::atomic<uint64_t> serial_received_{0};
  std::atomic<uint64_t> serial_last_receive_ms_{0};
  std::atomic<int> serial_last_errno_{0};
  std::mutex gate_mutex_;
  SequenceGate gate_;
  std::unique_ptr<SourceArbiter> source_arbiter_;
  std::atomic<uint64_t> accepted_joy_{0};
  std::atomic<uint64_t> accepted_heartbeat_{0};
  std::atomic<uint64_t> duplicates_or_old_{0};
  std::atomic<uint64_t> unauthorized_source_{0};
  std::atomic<uint64_t> invalid_payload_{0};
  std::atomic<uint64_t> last_joy_ms_{0};
  std::atomic<uint64_t> last_heartbeat_ms_{0};
  std::atomic<uint64_t> last_probe_ms_{0};
  uint64_t last_heartbeat_publish_ms_{0};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace critical_link

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<critical_link::CriticalLinkReceiver>());
  rclcpp::shutdown();
  return 0;
}
