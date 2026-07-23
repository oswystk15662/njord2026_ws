#include "micon_driver_fd/serial_writer.hpp"

#include <cstring>
#include <fcntl.h>
#include <functional>
#include <termios.h>
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
  if (fd_ < 0) {return;}
  Packet packet;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    packet = encode_packet(thrust_, flags_);
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
