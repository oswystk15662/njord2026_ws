# Micon Agent ESP32

A ROS2 package providing a micro-ROS agent for communicating with ESP32-based microcontrollers.

## Overview

This package implements a bridge between ROS2 and ESP32 microcontrollers running micro-ROS firmware. It manages serial communication, message parsing, and topic publication/subscription.

## Features

- **Serial Communication**: Direct serial connection to ESP32 devices
- **JSON Message Protocol**: Structured communication over serial
- **IMU Support**: Optional IMU data publishing
- **Sensor Integration**: Generic sensor data handling
- **Multi-Agent**: Support for multiple ESP32 devices simultaneously
- **Configurable Parameters**: YAML-based configuration
- **Status Monitoring**: Connection status publishing

## Installation

Build the package in your ROS2 workspace:

```bash
cd ~/njord2026_ws
colcon build --packages-select micon_agent_esp32
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch micon_agent_esp32 micon_esp32_agent_launch.py
```

### Launch with Custom Port

```bash
ros2 launch micon_agent_esp32 micon_esp32_agent_launch.py port:=/dev/ttyUSB1 baudrate:=230400
```

### Launch with Configuration File

```bash
ros2 launch micon_agent_esp32 micon_esp32_agent_with_config_launch.py
```

### Launch Multiple Agents

```bash
ros2 launch micon_agent_esp32 multi_micon_esp32_launch.py agent1_port:=/dev/ttyUSB0 agent2_port:=/dev/ttyUSB1
```

## Configuration

Configuration files are located in the `config/` directory:

- `micon_esp32_config.yaml`: Default configuration
- `micon_esp32_imu_config.yaml`: Configuration with IMU enabled

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyUSB0` | Serial port for ESP32 connection |
| `baudrate` | int | `115200` | Serial port baudrate |
| `timeout` | float | `1.0` | Serial read timeout in seconds |
| `enable_imu` | bool | `false` | Enable IMU data publishing |
| `enable_sensors` | bool | `true` | Enable general sensor publishing |
| `publish_rate` | int | `10` | Publishing rate in Hz |

## Published Topics

- `/micon/status` (std_msgs/String): Connection status
- `/micon/imu` (sensor_msgs/Imu): IMU data (when enabled)
- `/micon/sensors` (geometry_msgs/Vector3): Generic sensor data
- `/micon/debug` (std_msgs/String): Debug messages

## Subscribed Topics

- `/micon/command` (std_msgs/String): Commands to send to ESP32

## Message Protocol

### Expected JSON Format from ESP32

```json
{
  "type": "imu",
  "accel_x": 0.0,
  "accel_y": 0.0,
  "accel_z": 9.8,
  "gyro_x": 0.0,
  "gyro_y": 0.0,
  "gyro_z": 0.0
}
```

```json
{
  "type": "sensor",
  "x": 0.0,
  "y": 0.0,
  "z": 0.0
}
```

```json
{
  "type": "status",
  "message": "Device ready"
}
```

## Communication with ESP32

### Sending Commands

```bash
ros2 topic pub /micon/command std_msgs/String "data: 'RESET'"
```

### Monitoring Status

```bash
ros2 topic echo /micon/status
```

### Monitoring IMU Data

```bash
ros2 topic echo /micon/imu
```

## Troubleshooting

### Port Not Found

Verify the correct port:
```bash
ls /dev/ttyUSB*
```

### Baudrate Issues

Check the ESP32 firmware configuration and ensure the baudrate matches.

### Serial Permission Denied

Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

## Architecture

```
┌─────────────┐
│   ROS2      │
│   Nodes     │
└──────┬──────┘
       │
   ROS Topics
       │
┌──────▼────────────────────┐
│  MiconESP32Agent Node      │
│                            │
│ - Serial Communication     │
│ - JSON Message Parsing     │
│ - Topic Publishing         │
└──────┬────────────────────┘
       │
   Serial (UART)
       │
┌──────▼──────────┐
│   ESP32         │
│  (micro-ROS)    │
└─────────────────┘
```

## Development

### Node Structure

- `src/micon_agent_esp32/agent.py`: Main node implementation
- `launch/`: Launch file configurations
- `config/`: YAML configuration files

### Dependencies

- rclpy
- std_msgs
- geometry_msgs
- sensor_msgs
- pyserial

## License

Apache License 2.0

## Author

Keio University - osw
