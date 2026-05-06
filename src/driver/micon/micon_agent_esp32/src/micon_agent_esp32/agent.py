#!/usr/bin/env python3
# Copyright 2026 Keio University
# Licensed under the Apache License, Version 2.0

"""
Micro ROS Agent for ESP32 Communication.
This node acts as a bridge between ROS2 and ESP32-based microcontrollers via micro-ROS.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import serial
import json
import threading
from typing import Optional


class MiconESP32Agent(Node):
    """
    Micro ROS Agent for ESP32 communication.
    
    This node manages communication with ESP32 microcontrollers using micro-ROS
    protocols over serial or network connections.
    """

    def __init__(self):
        super().__init__('micon_esp32_agent')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('enable_imu', False)
        self.declare_parameter('enable_sensors', True)
        self.declare_parameter('publish_rate', 10)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.enable_imu = self.get_parameter('enable_imu').value
        self.enable_sensors = self.get_parameter('enable_sensors').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/micon/status', qos)
        self.imu_pub = self.create_publisher(Imu, '/micon/imu', qos)
        self.sensor_pub = self.create_publisher(Vector3, '/micon/sensors', qos)
        self.debug_pub = self.create_publisher(String, '/micon/debug', qos)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            String,
            '/micon/command',
            self.command_callback,
            qos
        )

        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.running = True
        self._connect_serial()

        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Thread for reading serial data
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f'Micon ESP32 Agent initialized on {self.port} @ {self.baudrate} baud'
        )

    def _connect_serial(self):
        """Establish serial connection to ESP32."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'Connected to {self.port}')
            msg = String()
            msg.data = 'CONNECTED'
            self.status_pub.publish(msg)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            msg = String()
            msg.data = 'DISCONNECTED'
            self.status_pub.publish(msg)

    def _read_serial_loop(self):
        """Background thread to read from serial port."""
        while self.running:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line:
                            self._process_serial_data(line)
                except Exception as e:
                    self.get_logger().warn(f'Error reading serial: {e}')
            else:
                # Try to reconnect if disconnected
                self._connect_serial()

    def _process_serial_data(self, data: str):
        """Process incoming serial data from ESP32."""
        try:
            # Try to parse as JSON
            payload = json.loads(data)
            msg_type = payload.get('type', 'unknown')

            if msg_type == 'imu' and self.enable_imu:
                self._handle_imu_data(payload)
            elif msg_type == 'sensor' and self.enable_sensors:
                self._handle_sensor_data(payload)
            elif msg_type == 'status':
                self._handle_status_data(payload)
            else:
                debug_msg = String()
                debug_msg.data = f'Unknown message type: {msg_type}'
                self.debug_pub.publish(debug_msg)

        except json.JSONDecodeError:
            # Log non-JSON messages
            debug_msg = String()
            debug_msg.data = f'Non-JSON data: {data}'
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Error processing serial data: {e}')

    def _handle_imu_data(self, payload: dict):
        """Handle IMU data from ESP32."""
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'esp32_imu'

            # Accelerometer
            imu_msg.linear_acceleration.x = payload.get('accel_x', 0.0)
            imu_msg.linear_acceleration.y = payload.get('accel_y', 0.0)
            imu_msg.linear_acceleration.z = payload.get('accel_z', 0.0)

            # Gyroscope
            imu_msg.angular_velocity.x = payload.get('gyro_x', 0.0)
            imu_msg.angular_velocity.y = payload.get('gyro_y', 0.0)
            imu_msg.angular_velocity.z = payload.get('gyro_z', 0.0)

            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f'Error handling IMU data: {e}')

    def _handle_sensor_data(self, payload: dict):
        """Handle generic sensor data from ESP32."""
        try:
            sensor_msg = Vector3()
            sensor_msg.x = payload.get('x', 0.0)
            sensor_msg.y = payload.get('y', 0.0)
            sensor_msg.z = payload.get('z', 0.0)
            self.sensor_pub.publish(sensor_msg)
        except Exception as e:
            self.get_logger().warn(f'Error handling sensor data: {e}')

    def _handle_status_data(self, payload: dict):
        """Handle status messages from ESP32."""
        try:
            status_msg = String()
            status_msg.data = payload.get('message', 'Status update')
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().warn(f'Error handling status data: {e}')

    def command_callback(self, msg: String):
        """Handle commands from ROS2 topics."""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                command = f'{msg.data}\n'
                self.serial_conn.write(command.encode('utf-8'))
                self.get_logger().debug(f'Sent command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')

    def timer_callback(self):
        """Periodic callback for status updates."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            status_msg = String()
            status_msg.data = 'DISCONNECTED'
            self.status_pub.publish(status_msg)
        else:
            status_msg = String()
            status_msg.data = 'CONNECTED'
            self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    agent = MiconESP32Agent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
