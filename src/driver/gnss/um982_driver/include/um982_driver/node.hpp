#ifndef UM982_DRIVER_NODE_HPP_
#define UM982_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <fstream>
#include <atomic>

// 前述のutilsをインクルード
#include "um982_driver/utils.hpp"

namespace um982_driver
{

class UM982Driver : public rclcpp::Node
{
public:
    explicit UM982Driver(const rclcpp::NodeOptions & options);
    ~UM982Driver();

private:
    // --- 初期化 ---
    void init_parameters();
    void init_gnss_connection();
    void init_rtk_connection();

    // --- 通信 (Boost.Asio) ---
    void start_gnss_read();
    void on_gnss_read(const boost::system::error_code& error, std::size_t bytes_transferred);
    void write_to_gnss(const std::vector<uint8_t>& data); // RTKデータ転送用
    void write_to_gnss(const std::string& data);          // コマンド送信

    void connect_rtk_client();
    void start_rtk_read();
    void on_rtk_read(const boost::system::error_code& error, std::size_t bytes_transferred);
    
    // --- データ処理 ---
    void process_gnss_line(std::string line);
    void parse_gga(const std::string& line);
    void parse_uniheading(const std::string& line);

    // --- ROS Callbacks/Timers ---
    void ctrl_callback(const std_msgs::msg::String::SharedPtr msg);
    void rtk_send_gga_callback(); // RTKサーバーへGGAを送信(5秒周期)

    // --- メンバ変数 ---
    
    // Asio Context & Thread
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;

    // GNSS Device
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::unique_ptr<boost::asio::ip::tcp::socket> tcp_socket_;
    boost::asio::streambuf gnss_read_buf_;
    std::mutex gnss_write_mutex_;

    // RTK Client
    std::unique_ptr<boost::asio::ip::tcp::socket> rtk_socket_;
    std::unique_ptr<boost::asio::ip::tcp::resolver> rtk_resolver_;
    boost::asio::streambuf rtk_read_buf_;
    bool is_rtk_connected_;

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_debug_pub_;

    // ROS Subscribers & Timers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctrl_sub_;
    rclcpp::TimerBase::SharedPtr rtk_gga_timer_;

    // State
    std::string last_gpgga_;
    std::ofstream log_file_;
    bool stop_publish_;
    
    // Parameters
    struct {
        std::string gnss_port;
        int gnss_baud;
        std::string tcp_ip;
        int tcp_port;
        std::string uart_or_tcp;
        
        int fix_freq;
        int heading_freq;
        bool rtk_enable;
        std::string heading_frame_id;
        std::string log_file_name;

        // RTK Settings (hardcoded in Python, but good to have structs)
        std::string ntrip_server = "ntrip.ales-corp.co.jp";
        int ntrip_port = 2101;
        std::string mountpoint = "RTCM32MSM7";
        std::string username = "username"; // TODO: set real values
        std::string password = "password";
    } params_;
};

} // namespace um982_driver

#endif // UM982_DRIVER_NODE_HPP_