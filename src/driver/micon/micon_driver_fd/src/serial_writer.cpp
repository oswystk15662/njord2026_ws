#include "micon_driver_fd/serial_writer.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <functional>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace micon_driver_fd
{

Packet encode_packet(const std::array<float, 4> & thrust, const Flags & flags)
{
  Packet packet{};
  for (size_t i = 0; i < thrust.size(); ++i) {
    std::memcpy(packet.data() + i * sizeof(float), &thrust[i], sizeof(float));
  }
  if (flags.emergency) {packet.back() |= (1u << 3);}
  if (flags.green) {packet.back() |= (1u << 2);}
  if (flags.yellow) {packet.back() |= (1u << 1);}
  if (flags.red) {packet.back() |= (1u << 0);}
  return packet;
}

std::uint16_t crc16_rx_frame(const uint8_t * data, size_t size)
{
  // RX CRC is CRC-16/Modbus: init 0xFFFF, reflected polynomial 0xA001,
  // calculated over TYPE, LEN and payload bytes. It is stored little-endian.
  std::uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<std::uint16_t>((crc >> 1) ^ 0xA001U);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> encode_bms_rx_frame(const std::array<std::uint16_t, 4> & cells_mv)
{
  std::vector<uint8_t> frame;
  frame.reserve(1U + 1U + 1U + kRxBmsPayloadLen + 2U);
  frame.push_back(kRxStart);
  frame.push_back(kRxTypeBms);
  frame.push_back(kRxBmsPayloadLen);
  for (const auto mv : cells_mv) {
    frame.push_back(static_cast<uint8_t>(mv & 0xFFU));
    frame.push_back(static_cast<uint8_t>((mv >> 8) & 0xFFU));
  }
  const std::uint16_t crc = crc16_rx_frame(frame.data() + 1, frame.size() - 1);
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
  return frame;
}

std::vector<BmsCells> RxParser::push(const std::vector<uint8_t> & data)
{
  return push(data.data(), data.size());
}

std::vector<BmsCells> RxParser::push(const uint8_t * data, size_t size)
{
  std::vector<BmsCells> out;
  buffer_.insert(buffer_.end(), data, data + size);

  constexpr size_t kHeaderSize = 3U;
  constexpr size_t kCrcSize = 2U;
  constexpr uint8_t kMaxPayloadLen = 32U;

  while (true) {
    const auto start_it = std::find(buffer_.begin(), buffer_.end(), kRxStart);
    if (start_it == buffer_.end()) {
      buffer_.clear();
      return out;
    }
    buffer_.erase(buffer_.begin(), start_it);

    if (buffer_.size() < kHeaderSize) {
      return out;
    }

    const uint8_t type = buffer_[1];
    const uint8_t len = buffer_[2];
    if (len > kMaxPayloadLen) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const size_t frame_size = kHeaderSize + len + kCrcSize;
    if (buffer_.size() < frame_size) {
      return out;
    }

    const std::uint16_t expected_crc = static_cast<std::uint16_t>(buffer_[frame_size - 2]) |
      (static_cast<std::uint16_t>(buffer_[frame_size - 1]) << 8);
    const std::uint16_t actual_crc = crc16_rx_frame(buffer_.data() + 1, 2U + len);
    if (expected_crc != actual_crc) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    if (type == kRxTypeBms && len == kRxBmsPayloadLen) {
      BmsCells cells;
      for (size_t i = 0; i < cells.volts.size(); ++i) {
        const size_t offset = kHeaderSize + i * 2U;
        const std::uint16_t mv = static_cast<std::uint16_t>(buffer_[offset]) |
          (static_cast<std::uint16_t>(buffer_[offset + 1]) << 8);
        cells.volts[i] = static_cast<float>(mv) / 1000.0F;
      }
      out.push_back(cells);
    }

    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
}

SerialWriter::SerialWriter(const rclcpp::NodeOptions & options)
: Node("serial_writer", options)
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  command_topic_ = declare_parameter<std::string>("command_topic", "/thruster_command");

  sub_thrust_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    command_topic_, 10, std::bind(&SerialWriter::thrust_cb, this, std::placeholders::_1));
  sub_emg_ = create_subscription<std_msgs::msg::Bool>(
    "/emg", 10, std::bind(&SerialWriter::emg_cb, this, std::placeholders::_1));
  sub_red_ = create_subscription<std_msgs::msg::Bool>(
    "/red", 10, std::bind(&SerialWriter::red_cb, this, std::placeholders::_1));
  sub_yellow_ = create_subscription<std_msgs::msg::Bool>(
    "/yellow", 10, std::bind(&SerialWriter::yellow_cb, this, std::placeholders::_1));
  sub_green_ = create_subscription<std_msgs::msg::Bool>(
    "/green", 10, std::bind(&SerialWriter::green_cb, this, std::placeholders::_1));
  pub_bms_ = create_publisher<std_msgs::msg::Float32MultiArray>("micon/bms_cells", 10);

  fd_ = open_serial(serial_port_, baud_);
  running_ = true;
  rx_thread_ = std::thread(&SerialWriter::rx_loop, this);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&SerialWriter::timer_cb, this));
  RCLCPP_INFO(get_logger(), "serial_writer started, port: %s", serial_port_.c_str());
}

SerialWriter::~SerialWriter()
{
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  close_serial();
}

void SerialWriter::thrust_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < thrust_.size(); ++i) {
    thrust_[i] = i < msg->data.size() ? msg->data[i] : 0.0f;
  }
}

void SerialWriter::emg_cb(const std_msgs::msg::Bool::SharedPtr /*msg*/)
{
  // Emergency stop is triggered by the physical hardware button for now, so
  // the software path never asserts the packet's emergency bit.
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.emergency = false;
}

void SerialWriter::red_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.red = msg->data;
}

void SerialWriter::yellow_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.yellow = msg->data;
}

void SerialWriter::green_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.green = msg->data;
}

void SerialWriter::timer_cb()
{
  int fd = -1;
  {
    std::lock_guard<std::mutex> fd_lock(fd_mutex_);
    fd = fd_;
  }
  if (fd < 0) {return;}
  Packet packet;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    packet = encode_packet(thrust_, flags_);
  }
  const ssize_t result = write(fd, packet.data(), packet.size());
  if (result != static_cast<ssize_t>(packet.size())) {
    RCLCPP_WARN(get_logger(), "serial write failed or incomplete");
    if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      close_serial();
    }
  }
}

int SerialWriter::open_serial(const std::string & device, int baud)
{
  int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "failed to open serial port %s", device.c_str());
    return -1;
  }
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    RCLCPP_ERROR(get_logger(), "tcgetattr failed");
    return -1;
  }
  cfmakeraw(&tty);
  speed_t speed = B115200;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default: speed = B115200; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    RCLCPP_ERROR(get_logger(), "tcsetattr failed");
    return -1;
  }
  return fd;
}

void SerialWriter::rx_loop()
{
  std::array<uint8_t, 128> read_buffer{};
  auto next_reconnect = std::chrono::steady_clock::now();

  while (running_) {
    int fd = -1;
    {
      std::lock_guard<std::mutex> fd_lock(fd_mutex_);
      fd = fd_;
    }

    if (fd < 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_reconnect) {
        const int new_fd = open_serial(serial_port_, baud_);
        if (new_fd >= 0) {
          std::lock_guard<std::mutex> fd_lock(fd_mutex_);
          if (fd_ < 0) {
            fd_ = new_fd;
            RCLCPP_INFO(get_logger(), "serial port reconnected: %s", serial_port_.c_str());
          } else {
            close(new_fd);
          }
        }
        next_reconnect = now + std::chrono::seconds(1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    const ssize_t n = read(fd, read_buffer.data(), read_buffer.size());
    if (n > 0) {
      const auto cells = rx_parser_.push(read_buffer.data(), static_cast<size_t>(n));
      for (const auto & cell_set : cells) {
        publish_bms(cell_set);
      }
    } else if (n == 0) {
      RCLCPP_WARN(get_logger(), "serial read failed; closing port");
      close_serial();
      next_reconnect = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    } else if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } else {
      RCLCPP_WARN(get_logger(), "serial read failed; closing port");
      close_serial();
      next_reconnect = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    }
  }
}

void SerialWriter::close_serial()
{
  std::lock_guard<std::mutex> fd_lock(fd_mutex_);
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void SerialWriter::publish_bms(const BmsCells & cells)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {cells.volts[0], cells.volts[1], cells.volts[2], cells.volts[3]};
  pub_bms_->publish(msg);
}

}  // namespace micon_driver_fd
