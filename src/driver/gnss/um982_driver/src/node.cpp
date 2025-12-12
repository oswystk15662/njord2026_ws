#include "um982_driver/node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace um982_driver
{

UM982Driver::UM982Driver(const rclcpp::NodeOptions & options)
: Node("um982_driver_node", options),
  work_guard_(io_context_.get_executor()),
  is_rtk_connected_(false),
  stop_publish_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing UM982 Driver Node (C++)...");

    init_parameters();

    // Log File
    if (!params_.log_file_name.empty()) {
        // ディレクトリ作成などは省略(C++ではfilesystem推奨だが簡易化のため直接Open)
        log_file_.open(params_.log_file_name, std::ios::app);
        if (log_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Log file opened: %s", params_.log_file_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file.");
        }
    }

    // Publishers
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensor/vehicle_gnss/fix/raw", 10);
    fix_debug_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensor/vehicle_gnss_debug/fix/raw", 10);
    heading_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensor/vehicle_gnss/compass/raw", 10);
    heading_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensor/vehicle_gnss_debug/compass/raw", 10);

    // Subscriber
    ctrl_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sensor/vehicle_gnss/command", 10,
        std::bind(&UM982Driver::ctrl_callback, this, std::placeholders::_1));

    // IO Thread Start
    io_thread_ = std::thread([this]() {
        io_context_.run();
    });

    // Connect
    try {
        init_gnss_connection();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "GNSS Connection Failed: %s", e.what());
    }

    if (params_.rtk_enable) {
        init_rtk_connection();
        // 5秒ごとにGGA送信
        rtk_gga_timer_ = this->create_wall_timer(
            5s, std::bind(&UM982Driver::rtk_send_gga_callback, this));
    }
}

UM982Driver::~UM982Driver()
{
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void UM982Driver::init_parameters()
{
    // Declare parameters with defaults
    this->declare_parameter("GNSS_SerialPort", "/dev/ttyUSB0");
    this->declare_parameter("GNSS_Baudrate", 115200);
    this->declare_parameter("tcp_ip", "192.168.0.126");
    this->declare_parameter("tcp_port", 23);
    this->declare_parameter("uart_or_tcp", "tcp");
    this->declare_parameter("FIX_FREQ", 20);
    this->declare_parameter("HEADING_FREQ", 20);
    this->declare_parameter("GNSS_RTK_Enable", true);
    this->declare_parameter("Heading_FrameID", "odom");
    this->declare_parameter("log_file_name", "um982.log"); // 簡易化のため固定名デフォルト

    // Get parameters
    params_.gnss_port = this->get_parameter("GNSS_SerialPort").as_string();
    params_.gnss_baud = this->get_parameter("GNSS_Baudrate").as_int();
    params_.tcp_ip = this->get_parameter("tcp_ip").as_string();
    params_.tcp_port = this->get_parameter("tcp_port").as_int();
    params_.uart_or_tcp = this->get_parameter("uart_or_tcp").as_string();
    params_.fix_freq = this->get_parameter("FIX_FREQ").as_int();
    params_.heading_freq = this->get_parameter("HEADING_FREQ").as_int();
    params_.rtk_enable = this->get_parameter("GNSS_RTK_Enable").as_bool();
    params_.heading_frame_id = this->get_parameter("Heading_FrameID").as_string();
    params_.log_file_name = this->get_parameter("log_file_name").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Mode: %s", params_.uart_or_tcp.c_str());
}

// --------------------------------------------------------------------------
// GNSS Connection & IO
// --------------------------------------------------------------------------

void UM982Driver::init_gnss_connection()
{
    if (params_.uart_or_tcp == "uart") {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_context_);
        serial_port_->open(params_.gnss_port);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(params_.gnss_baud));
        RCLCPP_INFO(this->get_logger(), "Opened Serial: %s", params_.gnss_port.c_str());
    } else {
        tcp_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
        boost::asio::ip::tcp::endpoint endpoint(
            boost::asio::ip::address::from_string(params_.tcp_ip), params_.tcp_port);
        tcp_socket_->connect(endpoint);
        RCLCPP_INFO(this->get_logger(), "Connected TCP: %s:%d", params_.tcp_ip.c_str(), params_.tcp_port);
    }

    // Configure Device
    double fix_p = 1.0 / params_.fix_freq;
    double head_p = 1.0 / params_.heading_freq;

    // write_to_gnss("OBSVMA " + std::to_string(fix_p) + "\r\n"); // Pythonコメントアウトされていたが有効化する場合
    write_to_gnss("GPGGA " + std::to_string(fix_p) + "\r\n");
    write_to_gnss("UNIHEADINGA " + std::to_string(head_p) + "\r\n");

    start_gnss_read();
}

void UM982Driver::write_to_gnss(const std::string& data)
{
    std::vector<uint8_t> bytes(data.begin(), data.end());
    write_to_gnss(bytes);
}

void UM982Driver::write_to_gnss(const std::vector<uint8_t>& data)
{
    // シリアル/TCPへの書き込みは排他制御推奨
    std::lock_guard<std::mutex> lock(gnss_write_mutex_);
    if (serial_port_ && serial_port_->is_open()) {
        boost::asio::write(*serial_port_, boost::asio::buffer(data));
    } else if (tcp_socket_ && tcp_socket_->is_open()) {
        boost::asio::write(*tcp_socket_, boost::asio::buffer(data));
    }
}

void UM982Driver::start_gnss_read()
{
    // 改行コードまで読み込む (Async)
    auto handler = std::bind(&UM982Driver::on_gnss_read, this, std::placeholders::_1, std::placeholders::_2);

    if (serial_port_) {
        boost::asio::async_read_until(*serial_port_, gnss_read_buf_, "\n", handler);
    } else if (tcp_socket_) {
        boost::asio::async_read_until(*tcp_socket_, gnss_read_buf_, "\n", handler);
    }
}

void UM982Driver::on_gnss_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error) {
        std::istream is(&gnss_read_buf_);
        std::string line;
        std::getline(is, line);
        
        // ログ保存
        if (log_file_.is_open()) {
            log_file_ << line << std::endl;
        }

        // 処理
        process_gnss_line(line);

        // 次の読み込み
        start_gnss_read();
    } else {
        RCLCPP_ERROR(this->get_logger(), "GNSS Read Error: %s", error.message().c_str());
        // 再接続ロジックを入れるならここ
        // 簡易実装として少し待ってリトライなどを検討
    }
}

void UM982Driver::process_gnss_line(std::string line)
{
    // 末尾の \r などを除去
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
    }

    if (line.rfind("$GNGGA", 0) == 0) {
        parse_gga(line);
        last_gpgga_ = line; // RTK用に保存
    } else if (line.rfind("#UNIHEADINGA", 0) == 0) {
        parse_uniheading(line);
    }
}

// --------------------------------------------------------------------------
// Parsing Logic
// --------------------------------------------------------------------------

void UM982Driver::parse_gga(const std::string& line)
{
    if (!utils::validate_checksum(line)) return;

    // * チェックサムを除去して分割
    std::string payload = line.substr(0, line.find('*'));
    auto parts = utils::split(payload, ',');

    if (parts.size() < 15) return;

    // $GNGGA,time,lat,N,lon,E,qual,sats,hdop,alt,M,sep,M,age,ref
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link"; // Python通り
    msg.status.service = 15; // GPS+GLONASS+...

    msg.latitude = utils::convert_nmea_to_latlon(parts[2], parts[3]);
    msg.longitude = utils::convert_nmea_to_latlon(parts[4], parts[5]);

    double alt = parts[9].empty() ? 0.0 : std::stod(parts[9]);
    double sep = parts[11].empty() ? 0.0 : std::stod(parts[11]);
    msg.altitude = alt - sep;

    int quality = parts[6].empty() ? 0 : std::stoi(parts[6]);
    if (quality >= 1) {
        msg.position_covariance[0] = 0.02 * 0.02;
        msg.position_covariance[4] = 0.02 * 0.02;
        msg.position_covariance[8] = 0.02 * 0.02;
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    } else {
        msg.position_covariance[0] = -1; // 不明
    }

    if (!stop_publish_) {
        fix_pub_->publish(msg);
    }
    fix_debug_pub_->publish(msg);
}

void UM982Driver::parse_uniheading(const std::string& line)
{
    // #UNIHEADINGA,header;SOL_COMPUTED,TYPE,len,heading,pitch,...
    // Pythonではsplit(";")してからsplit(",")
    
    auto main_parts = utils::split(line, ';');
    if (main_parts.size() < 2) return;

    // チェックサム検証 (行全体で行う必要がある)
    // ただしUNIHEADINGAのチェックサムはCRC32のケースもあるため、今回はPython同様簡易パース優先
    
    std::string data_part_str = main_parts[1].substr(0, main_parts[1].find('*'));
    auto data = utils::split(data_part_str, ',');

    if (data.size() < 12) return;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = params_.heading_frame_id;

    // data[3] = heading (deg), data[4] = pitch (deg)
    // Python: yaw_rad = radians(90.0 - heading)
    // Python: pitch_rad = radians(-1.0 * pitch)
    
    try {
        double heading_deg = std::stod(data[3]);
        double pitch_deg = std::stod(data[4]);

        double yaw_rad = utils::deg2rad(90.0 - heading_deg);
        double pitch_rad = utils::deg2rad(-1.0 * pitch_deg);

        tf2::Quaternion q;
        q.setRPY(0, pitch_rad, yaw_rad);
        msg.pose.pose.orientation = tf2::toMsg(q);

        // Covariance logic
        std::string sol_status = data[0];
        std::string pos_type = data[1];
        
        double std_dev = 0.0;
        bool is_valid = false;

        if (sol_status == "SOL_COMPUTED") {
            if (pos_type == "NARROW_INT" || pos_type == "INS_RTKFIXED") { // FIXED
                std_dev = 0.5;
                is_valid = true;
            } 
            // INT, FLOAT等はPythonでコメントアウトされていたが、必要なら追加
        }

        if (is_valid) {
            double var = utils::deg2rad(std_dev);
            var = var * var;
            msg.pose.covariance[28] = var; // rotation about Z (Yaw)
            msg.pose.covariance[35] = var; // rotation about Y (Pitch)? 35 is Z in 6x6?
            // index 35 is (5,5) -> theta_z (Yaw).
            // index 28 is (4,4) -> theta_y (Pitch).
            // Python code sets 28 and 35.
        } else {
            msg.pose.covariance[0] = -1; // invalid
        }

        if (!stop_publish_ && is_valid) {
            heading_pub_->publish(msg);
        }
        heading_debug_pub_->publish(msg);

    } catch (...) {
        // 数値変換エラーなど
    }
}

// --------------------------------------------------------------------------
// RTK (NTRIP)
// --------------------------------------------------------------------------

void UM982Driver::init_rtk_connection()
{
    rtk_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
    rtk_resolver_ = std::make_unique<boost::asio::ip::tcp::resolver>(io_context_);

    // 非同期接続
    auto endpoints = rtk_resolver_->resolve(params_.ntrip_server, std::to_string(params_.ntrip_port));
    boost::asio::async_connect(*rtk_socket_, endpoints,
        [this](const boost::system::error_code& error, const boost::asio::ip::tcp::endpoint& /*endpoint*/) {
            if (!error) {
                connect_rtk_client();
            } else {
                RCLCPP_ERROR(this->get_logger(), "RTK Connect Failed: %s", error.message().c_str());
            }
        });
}

void UM982Driver::connect_rtk_client()
{
    std::string auth_str = params_.username + ":" + params_.password;
    std::string auth_base64 = utils::base64_encode(auth_str);

    std::stringstream request;
    request << "GET /" << params_.mountpoint << " HTTP/1.1\r\n";
    request << "Host: " << params_.ntrip_server << ":" << params_.ntrip_port << "\r\n";
    request << "Ntrip-Version: Ntrip/2.0\r\n";
    request << "User-Agent: NTRIP Client/1.0\r\n";
    request << "Authorization: Basic " << auth_base64 << "\r\n";
    request << "Connection: close\r\n";
    request << "\r\n";

    std::string req_str = request.str();
    boost::asio::write(*rtk_socket_, boost::asio::buffer(req_str));

    is_rtk_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to RTK Server. Waiting for data...");

    start_rtk_read();
}

void UM982Driver::start_rtk_read()
{
    // RTKデータはバイナリかもしれないので、read_some で受信する
    auto handler = std::bind(&UM982Driver::on_rtk_read, this, std::placeholders::_1, std::placeholders::_2);
    rtk_socket_->async_read_some(rtk_read_buf_.prepare(1024), handler); 
}

void UM982Driver::on_rtk_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error) {
        rtk_read_buf_.commit(bytes_transferred);
        
        // バッファからデータを取り出してGNSSへ転送
        std::vector<uint8_t> data(bytes_transferred);
        std::istream is(&rtk_read_buf_);
        is.read(reinterpret_cast<char*>(data.data()), bytes_transferred);
        
        // GNSSへ書き込み
        write_to_gnss(data);

        // 次の読み込み
        start_rtk_read();
    } else {
        RCLCPP_ERROR(this->get_logger(), "RTK Read Error: %s", error.message().c_str());
        is_rtk_connected_ = false;
        // リコネクト処理を入れるならここ
    }
}

void UM982Driver::rtk_send_gga_callback()
{
    if (!is_rtk_connected_) {
        // 未接続なら再接続試行
        init_rtk_connection(); // 簡易的な再呼び出し(本来は状態管理が必要)
        return;
    }

    if (!last_gpgga_.empty() && last_gpgga_.rfind("$GNGGA", 0) == 0) {
        // GNGGAをRTKサーバーへ送信して位置を通知
        std::string msg = last_gpgga_ + "\r\n";
        boost::system::error_code ignored_error;
        boost::asio::write(*rtk_socket_, boost::asio::buffer(msg), ignored_error);
    }
}

void UM982Driver::ctrl_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "shutdown") {
        rclcpp::shutdown();
    } else if (msg->data == "stop_publish") {
        stop_publish_ = true;
    } else if (msg->data == "start_publish") {
        stop_publish_ = false;
    }
}

} // namespace um982_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(um982_driver::UM982Driver)