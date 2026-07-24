#include "micon_driver_fd/serial_writer.hpp"

#include <cstring>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <termios.h>
#include <unistd.h>

namespace micon_driver_fd
{

namespace
{

void append_uint16_le(Packet & packet, uint16_t value)
{
  packet.push_back(static_cast<uint8_t>(value & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
}

void append_uint32_le(Packet & packet, uint32_t value)
{
  packet.push_back(static_cast<uint8_t>(value & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 16u) & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 24u) & 0xFFu));
}

void append_float32_le(Packet & packet, float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "Protocol requires 32-bit float");
  std::memcpy(&bits, &value, sizeof(bits));
  append_uint32_le(packet, bits);
}

uint16_t crc16_ccitt_false(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8u;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0u ?
        static_cast<uint16_t>((crc << 1u) ^ 0x1021u) :
        static_cast<uint16_t>(crc << 1u);
    }
  }
  return crc;
}

Packet cobs_encode(const Packet & raw)
{
  Packet encoded;
  encoded.reserve(raw.size() + 1);

  size_t code_index = 0;
  uint8_t code = 1;
  encoded.push_back(0);

  for (const uint8_t byte : raw) {
    if (byte == 0) {
      encoded[code_index] = code;
      code_index = encoded.size();
      encoded.push_back(0);
      code = 1;
      continue;
    }

    encoded.push_back(byte);
    ++code;
    if (code == std::numeric_limits<uint8_t>::max()) {
      encoded[code_index] = code;
      code_index = encoded.size();
      encoded.push_back(0);
      code = 1;
    }
  }

  encoded[code_index] = code;
  return encoded;
}

}  // namespace

Packet encode_packet(
  const std::array<float, 4> & thrust,
  const Flags & flags,
  uint16_t sequence)
{
  Packet raw;
  raw.reserve(kRawFrameSize);
  raw.push_back(kProtocolVersion);
  raw.push_back(kThrusterCommandType);
  append_uint16_le(raw, sequence);
  raw.push_back(static_cast<uint8_t>(kPayloadSize));

  for (size_t i = 0; i < thrust.size(); ++i) {
    append_float32_le(raw, thrust[i]);
  }
  uint8_t control_flags = 0;
  if (flags.emergency) {control_flags |= (1u << 3);}
  if (flags.green) {control_flags |= (1u << 2);}
  if (flags.yellow) {control_flags |= (1u << 1);}
  if (flags.red) {control_flags |= (1u << 0);}
  raw.push_back(control_flags);

  const uint16_t crc = crc16_ccitt_false(raw.data(), raw.size());
  append_uint16_le(raw, crc);

  Packet packet = cobs_encode(raw);
  packet.push_back(0);
  return packet;
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

  fd_ = open_serial(serial_port_, baud_);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&SerialWriter::timer_cb, this));
  RCLCPP_INFO(get_logger(), "serial_writer started, port: %s", serial_port_.c_str());
}

SerialWriter::~SerialWriter()
{
  if (fd_ >= 0) {close(fd_);}
}

void SerialWriter::thrust_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < thrust_.size(); ++i) {
    thrust_[i] = i < msg->data.size() ? msg->data[i] : 0.0f;
  }
}

void SerialWriter::emg_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Emergency stop is triggered by the physical hardware button for now, so
  // the software path never asserts the packet's emergency bit.
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.emergency = msg->data;
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
  if (fd_ < 0) {return;}
  Packet packet;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    packet = encode_packet(thrust_, flags_, sequence_++);
  }
  const ssize_t result = write(fd_, packet.data(), packet.size());
  if (result != static_cast<ssize_t>(packet.size())) {
    RCLCPP_WARN(get_logger(), "serial write failed or incomplete");
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
  int file_flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, file_flags & ~O_NONBLOCK);
  return fd;
}

}  // namespace micon_driver_fd
