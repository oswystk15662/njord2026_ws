#ifndef DROGGER_WIRED_FLEX_NODE_HPP_
#define DROGGER_WIRED_FLEX_NODE_HPP_

#include <atomic>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace drogger_wired_flex
{

class DroggerWiredFlexNode : public rclcpp::Node
{
public:
  explicit DroggerWiredFlexNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DroggerWiredFlexNode() override;

private:
  struct Params
  {
    std::string transport;
    std::string serial_port;
    int serial_baudrate;
    std::string tcp_host;
    std::string tcp_bind_host;
    int tcp_port;
    std::string udp_bind_host;
    int udp_port;
    std::string frame_id;
    std::string fix_topic;
    int read_timeout_ms;
    double reconnect_sec;
  };

  void declare_and_get_parameters();

  void io_loop();
  bool open_transport_fd();
  bool open_serial_fd();
  bool open_tcp_fd();
  bool open_tcp_server_fd();
  bool open_udp_fd();
  void close_fd();

  void on_bytes(const char * data, std::size_t size);
  void process_line(std::string line);
  void parse_gga(const std::vector<std::string> & tokens);

  static std::vector<std::string> split(const std::string & s, char delimiter);
  static bool validate_nmea_checksum(const std::string & sentence);
  static double convert_nmea_to_latlon(const std::string & value, const std::string & direction);

  static speed_t baudrate_to_speed_t(int baudrate);

  Params params_{};

  std::atomic<bool> keep_running_{true};
  std::thread io_thread_;
  int fd_{-1};
  std::string line_buffer_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
};

}  // namespace drogger_wired_flex

#endif  // DROGGER_WIRED_FLEX_NODE_HPP_
