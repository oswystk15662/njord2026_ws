#include "drogger_bt/node.hpp"
#include <sstream>
#include <iomanip>
#include <chrono>

using namespace std::chrono_literals;

namespace drogger_bt
{

DroggerDriver::DroggerDriver(const rclcpp::NodeOptions & options)
: Node("drogger_driver", options)
, keep_running_(true)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Drogger Driver...");
    
    init_parameters();
    
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);

    // Start IO Thread
    io_thread_ = std::thread([this]() {
        // Retry connection loop
        while (keep_running_ && rclcpp::ok()) {
            try {
                if (!serial_port_ || !serial_port_->is_open()) {
                    connect();
                }
                io_context_.run(); // Blocks until stopped or out of work
                io_context_.restart(); // Prepare for next run if stopped
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Connection Error: %s", e.what());
                if (serial_port_ && serial_port_->is_open()) serial_port_->close();
            }
            
            // Wait before retry
            if (keep_running_) std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });
}

DroggerDriver::~DroggerDriver()
{
    keep_running_ = false;
    io_context_.stop();
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void DroggerDriver::init_parameters()
{
    this->declare_parameter("port_name", "/dev/rfcomm0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("frame_id", "gnss_link");
    this->declare_parameter("bt_mac_address", ""); 
    this->declare_parameter("rfcomm_channel", 1);
    this->declare_parameter("rfcomm_id", 0);

    params_.port_name = this->get_parameter("port_name").as_string();
    params_.baudrate = this->get_parameter("baudrate").as_int();
    params_.frame_id = this->get_parameter("frame_id").as_string();
}

void DroggerDriver::connect()
{
    RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s", params_.port_name.c_str());
    
    serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
    serial_port_->open(params_.port_name);
    
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(params_.baudrate));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    RCLCPP_INFO(this->get_logger(), "Connected to Drogger at %s", params_.port_name.c_str());
    
    start_async_read();
}

void DroggerDriver::start_async_read()
{
    boost::asio::async_read_until(*serial_port_, read_buf_, "\n",
        std::bind(&DroggerDriver::on_read, this, std::placeholders::_1, std::placeholders::_2));
}

void DroggerDriver::on_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error) {
        std::istream is(&read_buf_);
        std::string line;
        std::getline(is, line);
        
        // Trim whitespace
        if (!line.empty() && line.back() == '\r') line.pop_back();

        process_data(line);

        start_async_read();
    } else {
        if (error != boost::asio::error::operation_aborted) {
            RCLCPP_ERROR(this->get_logger(), "Read Error: %s", error.message().c_str());
            serial_port_->close(); // Force reconnect loop in thread
        }
    }
}

void DroggerDriver::process_data(std::string line)
{
    // Simple check for GNGGA or GPGGA
    if (line.find("GGA") != std::string::npos) {
        if (!validate_checksum(line)) {
             RCLCPP_WARN(this->get_logger(), "Checksum failed: %s", line.c_str());
             return;
        }
        
        // Remove checksum part for splitting
        std::string payload = line.substr(0, line.find('*'));
        std::vector<std::string> tokens = split(payload, ',');
        
        parse_gga(tokens);
    }
}

void DroggerDriver::parse_gga(const std::vector<std::string>& tokens)
{
    // $GNGGA,time,lat,N,lon,E,qual,sats,hdop,alt,M,sep,M,age,ref
    // Index 2=lat, 3=N/S, 4=lon, 5=E/W, 6=qual, 9=alt
    
    if (tokens.size() < 10) return;

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = params_.frame_id;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    try {
        msg.latitude = convert_nmea_to_latlon(tokens[2], tokens[3]);
        msg.longitude = convert_nmea_to_latlon(tokens[4], tokens[5]);
        
        double alt = tokens[9].empty() ? 0.0 : std::stod(tokens[9]);
        double sep = (tokens.size() > 11 && !tokens[11].empty()) ? std::stod(tokens[11]) : 0.0;
        msg.altitude = alt - sep; // Ellipsoidal height if needed, or MSL

        int quality = tokens[6].empty() ? 0 : std::stoi(tokens[6]);
        msg.status.status = (quality > 0) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX 
                                          : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        
        // Simple covariance estimation based on quality
        double cov = 10000.0;
        if (quality == 4 || quality == 5) cov = 0.02 * 0.02; // RTK
        else if (quality == 1 || quality == 2) cov = 2.0 * 2.0; // GPS/DGPS
        
        msg.position_covariance[0] = cov;
        msg.position_covariance[4] = cov;
        msg.position_covariance[8] = cov * 2.0; // Altitude error is usually larger
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

        fix_pub_->publish(msg);

    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Parse error: %s", e.what());
    }
}

// --- Utils ---

std::vector<std::string> DroggerDriver::split(const std::string & s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    // Handle trailing comma
    if (!s.empty() && s.back() == delimiter) tokens.push_back("");
    return tokens;
}

double DroggerDriver::convert_nmea_to_latlon(const std::string & value, const std::string & direction)
{
    if (value.empty()) return 0.0;
    
    double raw = std::stod(value);
    int degrees = static_cast<int>(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    
    if (direction == "S" || direction == "W") decimal = -decimal;
    return decimal;
}

bool DroggerDriver::validate_checksum(const std::string & sentence)
{
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos) return false;
    
    // Calculate XOR sum between $ and *
    int sum = 0;
    for (size_t i = 1; i < star_pos; ++i) {
        sum ^= sentence[i];
    }
    
    // Parse hex
    try {
        int provided = std::stoi(sentence.substr(star_pos + 1), nullptr, 16);
        return sum == provided;
    } catch (...) {
        return false;
    }
}

} // namespace drogger_bt

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drogger_bt::DroggerDriver)