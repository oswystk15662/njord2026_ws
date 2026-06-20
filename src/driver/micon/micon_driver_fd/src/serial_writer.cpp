// serial_writer.cpp
// Subscribes to normalized thruster commands and operator-control flags.
// Packs 4 floats followed by 1 uint8 setting byte (bit3=emg, bit2=green, bit1=yellow, bit0=red)
// and writes to serial port for ESP32.

#include <array>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class SerialWriter : public rclcpp::Node
{
public:
  SerialWriter()
  : Node("serial_writer")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<std::string>("command_topic", "/debug/current_force");
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_ = this->get_parameter("baud").as_int();
    command_topic_ = this->get_parameter("command_topic").as_string();

    sub_thrust_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      command_topic_, 10,
      std::bind(&SerialWriter::thrust_cb, this, std::placeholders::_1));
    sub_emg_ = this->create_subscription<std_msgs::msg::Bool>(
      "/emg", 10,
      std::bind(&SerialWriter::emg_cb, this, std::placeholders::_1));
    sub_red_ = this->create_subscription<std_msgs::msg::Bool>(
      "/red", 10,
      std::bind(&SerialWriter::red_cb, this, std::placeholders::_1));
    sub_yellow_ = this->create_subscription<std_msgs::msg::Bool>(
      "/yellow", 10,
      std::bind(&SerialWriter::yellow_cb, this, std::placeholders::_1));
    sub_green_ = this->create_subscription<std_msgs::msg::Bool>(
      "/green", 10,
      std::bind(&SerialWriter::green_cb, this, std::placeholders::_1));

    fd_ = open_serial(serial_port_, baud_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SerialWriter::timer_cb, this));
    RCLCPP_INFO(this->get_logger(), "serial_writer started, port: %s", serial_port_.c_str());
  }

  ~SerialWriter() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void thrust_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (size_t i = 0; i < 4; ++i) {
      thrust_[i] = (i < msg->data.size()) ? msg->data[i] : 0.0f;
    }
  }

  void emg_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    flags_.emg = msg->data;
  }

  void red_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    flags_.red = msg->data;
  }

  void yellow_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    flags_.yellow = msg->data;
  }

  void green_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    flags_.green = msg->data;
  }

  void timer_cb()
  {
    if (fd_ < 0) {
      return;
    }

    std::vector<uint8_t> buf;
    buf.resize(4 * sizeof(float) + 1);

    {
      std::lock_guard<std::mutex> lk(mutex_);
      uint8_t * p = buf.data();
      for (size_t i = 0; i < 4; ++i) {
        float v = thrust_[i];
        std::memcpy(p + i * sizeof(float), &v, sizeof(float));
      }

      uint8_t setting = 0;
      if (flags_.emg) {
        setting |= (1u << 3);
      }
      if (flags_.green) {
        setting |= (1u << 2);
      }
      if (flags_.yellow) {
        setting |= (1u << 1);
      }
      if (flags_.red) {
        setting |= (1u << 0);
      }
      buf[4 * sizeof(float)] = setting;
    }

    const ssize_t res = write(fd_, buf.data(), buf.size());
    if (res < 0) {
      RCLCPP_WARN(this->get_logger(), "serial write failed");
    }
  }

  int open_serial(const std::string & dev, int baud)
  {
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "failed to open serial port %s", dev.c_str());
      return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      close(fd);
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
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
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      close(fd);
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
      return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    fcntl(fd, F_SETFL, flags);

    return fd;
  }

  struct Flags
  {
    bool emg{false};
    bool red{false};
    bool yellow{false};
    bool green{false};
  } flags_;

  std::array<float, 4> thrust_{{0.0f, 0.0f, 0.0f, 0.0f}};
  std::mutex mutex_;
  int fd_{-1};
  std::string serial_port_;
  int baud_{115200};
  std::string command_topic_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_thrust_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emg_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_red_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_yellow_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_green_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialWriter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
