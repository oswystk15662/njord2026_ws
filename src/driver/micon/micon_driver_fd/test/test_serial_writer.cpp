#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "micon_driver_fd/serial_writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

TEST(SerialPacket, EncodesFloatsAndFlags)
{
  const std::array<float, 4> thrust{{1.0F, -0.5F, 0.25F, 0.0F}};
  micon_driver_fd::Flags flags;
  flags.emergency = true;
  flags.green = true;
  flags.red = true;
  const auto packet = micon_driver_fd::encode_packet(thrust, flags);
  for (size_t i = 0; i < thrust.size(); ++i) {
    float decoded = 0.0F;
    std::memcpy(&decoded, packet.data() + i * sizeof(float), sizeof(float));
    EXPECT_FLOAT_EQ(decoded, thrust[i]);
  }
  EXPECT_EQ(packet.back(), 0x0D);
}

TEST(RxParser, DecodesValidBmsFrame)
{
  micon_driver_fd::RxParser parser;
  const auto frame = micon_driver_fd::encode_bms_rx_frame({3700, 3710, 3720, 3730});
  const auto decoded = parser.push(frame);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 3.7F);
  EXPECT_FLOAT_EQ(decoded[0].volts[1], 3.71F);
  EXPECT_FLOAT_EQ(decoded[0].volts[2], 3.72F);
  EXPECT_FLOAT_EQ(decoded[0].volts[3], 3.73F);
}

TEST(RxParser, HandlesSplitRead)
{
  micon_driver_fd::RxParser parser;
  const auto frame = micon_driver_fd::encode_bms_rx_frame({4100, 4110, 4120, 4130});
  EXPECT_TRUE(parser.push(frame.data(), 4).empty());
  const auto decoded = parser.push(frame.data() + 4, frame.size() - 4);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 4.1F);
  EXPECT_FLOAT_EQ(decoded[0].volts[3], 4.13F);
}

TEST(RxParser, HandlesConcatenatedFrames)
{
  micon_driver_fd::RxParser parser;
  auto stream = micon_driver_fd::encode_bms_rx_frame({3600, 3610, 3620, 3630});
  const auto second = micon_driver_fd::encode_bms_rx_frame({3800, 3810, 3820, 3830});
  stream.insert(stream.end(), second.begin(), second.end());
  const auto decoded = parser.push(stream);
  ASSERT_EQ(decoded.size(), 2U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 3.6F);
  EXPECT_FLOAT_EQ(decoded[1].volts[0], 3.8F);
}

TEST(RxParser, DropsLeadingGarbage)
{
  micon_driver_fd::RxParser parser;
  std::vector<uint8_t> stream{0x00, 0x55, 0x10, 0xAA, 0xAA};
  const auto frame = micon_driver_fd::encode_bms_rx_frame({3900, 3910, 3920, 3930});
  stream.insert(stream.end(), frame.begin(), frame.end());
  const auto decoded = parser.push(stream);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 3.9F);
}

TEST(RxParser, DropsCrcNgAndResynchronizes)
{
  micon_driver_fd::RxParser parser;
  auto bad = micon_driver_fd::encode_bms_rx_frame({3700, 3710, 3720, 3730});
  bad.back() ^= 0x5AU;
  const auto good = micon_driver_fd::encode_bms_rx_frame({4000, 4010, 4020, 4030});
  bad.insert(bad.end(), good.begin(), good.end());
  const auto decoded = parser.push(bad);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 4.0F);
}

TEST(RxParser, DropsLengthNgAndResynchronizes)
{
  micon_driver_fd::RxParser parser;
  std::vector<uint8_t> stream{micon_driver_fd::kRxStart, 0x01, 0xFF};
  const auto good = micon_driver_fd::encode_bms_rx_frame({3500, 3510, 3520, 3530});
  stream.insert(stream.end(), good.begin(), good.end());
  const auto decoded = parser.push(stream);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 3.5F);
}

TEST(RxParser, DropsBmsWrongLengthAndResynchronizes)
{
  micon_driver_fd::RxParser parser;
  std::vector<uint8_t> stream{
    micon_driver_fd::kRxStart,
    micon_driver_fd::kRxTypeBms,
    2,
    0x34,
    0x12,
  };
  const std::uint16_t crc = micon_driver_fd::crc16_rx_frame(stream.data() + 1, 4);
  stream.push_back(static_cast<uint8_t>(crc & 0xFFU));
  stream.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
  const auto good = micon_driver_fd::encode_bms_rx_frame({3550, 3560, 3570, 3580});
  stream.insert(stream.end(), good.begin(), good.end());
  const auto decoded = parser.push(stream);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_FLOAT_EQ(decoded[0].volts[0], 3.55F);
}

TEST(SerialWriterIntegration, WritesRosInputsToPseudoTerminal)
{
  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[128]{};
  ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);
  close(slave_fd);
  fcntl(master_fd, F_SETFL, fcntl(master_fd, F_GETFL, 0) | O_NONBLOCK);

  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", std::string(slave_name))});
  auto writer = std::make_shared<micon_driver_fd::SerialWriter>(options);
  auto publisher_node = std::make_shared<rclcpp::Node>("serial_writer_test_publisher");
  auto thrust_pub = publisher_node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/thruster_command", 10);
  auto emg_pub = publisher_node->create_publisher<std_msgs::msg::Bool>("/emg", 10);
  auto green_pub = publisher_node->create_publisher<std_msgs::msg::Bool>("/green", 10);
  auto red_pub = publisher_node->create_publisher<std_msgs::msg::Bool>("/red", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(writer);
  executor.add_node(publisher_node);
  std_msgs::msg::Float32MultiArray thrust;
  thrust.data = {0.1F, -0.2F, 0.3F, -0.4F};
  thrust_pub->publish(thrust);
  emg_pub->publish(std_msgs::msg::Bool().set__data(true));
  green_pub->publish(std_msgs::msg::Bool().set__data(true));
  red_pub->publish(std_msgs::msg::Bool().set__data(true));
  const auto receive_deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  std::vector<uint8_t> received(1024);
  const ssize_t count = read(master_fd, received.data(), received.size());
  ASSERT_GE(count, static_cast<ssize_t>(micon_driver_fd::kPacketSize));
  const size_t frame_count = static_cast<size_t>(count) / micon_driver_fd::kPacketSize;
  const uint8_t * packet = received.data() + (frame_count - 1) * micon_driver_fd::kPacketSize;
  for (size_t i = 0; i < thrust.data.size(); ++i) {
    float decoded = 0.0F;
    std::memcpy(&decoded, packet + i * sizeof(float), sizeof(float));
    EXPECT_FLOAT_EQ(decoded, thrust.data[i]);
  }
  EXPECT_EQ(packet[micon_driver_fd::kPacketSize - 1], 0x05);

  close(master_fd);
  executor.remove_node(publisher_node);
  executor.remove_node(writer);
  writer.reset();
  publisher_node.reset();
  rclcpp::shutdown();
}
