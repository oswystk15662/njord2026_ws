# drogger_wired_flex

FD-based ROS2 GNSS driver for NMEA sources over:
- serial USB (e.g. RZS D01)
- Ethernet TCP (e.g. DG-PRO1RWS)
- Ethernet UDP

The node reads NMEA sentences via a POSIX file descriptor and publishes `sensor_msgs/NavSatFix` from GGA data.

## Published topic

- Configurable by `fix_topic` parameter
- Default: `/sensor/vehicle_gnss/fix/raw`

## Parameters

- `transport`: `serial` | `tcp` | `udp`
- `serial_port`: serial device path (`/dev/ttyUSB0`)
- `serial_baudrate`: serial baudrate (default `115200`)
- `tcp_host`, `tcp_port`: TCP endpoint
- `udp_bind_host`, `udp_port`: UDP bind address and port
- `frame_id`: NavSatFix frame
- `fix_topic`: output topic
- `read_timeout_ms`: poll timeout
- `reconnect_sec`: reconnect interval

## Run

```bash
ros2 launch drogger_wired_flex driver.launch.py
```

Edit `config/params.yaml` for your transport/device.
