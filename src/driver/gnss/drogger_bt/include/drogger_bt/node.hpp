#ifndef DROGGER_GNSS_DRIVER_NODE_HPP_
#define DROGGER_GNSS_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <array>
#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <cstdint>

namespace drogger_bt
{

class DroggerDriver : public rclcpp::Node
{
public:
    explicit DroggerDriver(const rclcpp::NodeOptions & options);
    ~DroggerDriver();

private:
    // Parameters
    void init_parameters();
    
    // Connection
    void connect();
    void start_async_read();
    void on_read(const boost::system::error_code& error, std::size_t bytes_transferred);
    void schedule_read_retry();
    void process_received_bytes(const char * data, std::size_t size, bool flush_partial = false);
    
    // Parsing
    void process_data(std::string line);
    void parse_gga(const std::vector<std::string>& tokens);
    
    // Helpers
    std::vector<std::string> split(const std::string & s, char delimiter);
    double convert_nmea_to_latlon(const std::string & value, const std::string & direction);
    bool validate_checksum(const std::string & sentence);

    // Members
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::array<char, 1024> read_chunk_{};
    std::string line_buffer_;
    std::thread io_thread_;
    std::atomic<bool> keep_running_;
    boost::asio::steady_timer retry_timer_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

    struct Params {
        std::string port_name;
        int baudrate;
        std::string frame_id;
        std::string fix_topic;
        bool log_raw_nmea;
    } params_;
};

} // namespace drogger_gnss_driver

#endif // DROGGER_GNSS_DRIVER_NODE_HPP_