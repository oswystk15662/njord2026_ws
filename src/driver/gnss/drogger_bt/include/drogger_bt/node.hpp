#ifndef DROGGER_GNSS_DRIVER_NODE_HPP_
#define DROGGER_GNSS_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

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
    boost::asio::streambuf read_buf_;
    std::thread io_thread_;
    std::atomic<bool> keep_running_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

    struct Params {
        std::string port_name;
        int baudrate;
        std::string frame_id;
    } params_;
};

} // namespace drogger_gnss_driver

#endif // DROGGER_GNSS_DRIVER_NODE_HPP_