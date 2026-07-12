# micon_driver_fd

USB serial で Micon/ESP32 と通信するパッケージです。tty を open するのは
`serial_writer` ノードだけです。

## serial_writer

- subscribes `command_topic` (`std_msgs/msg/Float32MultiArray`), first 4 values in Newton
- subscribes `/emg`, `/red`, `/yellow`, `/green` (`std_msgs/msg/Bool`)
- publishes `micon/bms_cells` (`std_msgs/msg/Float32MultiArray`, 4 cells in V)
- parameters: `serial_port` (default `/dev/ttyUSB0`), `baud` (default `115200`),
  and `command_topic` (default `/thruster_command`)

## TX raw v1

Every 50 ms, the node writes:

`4 x float32 native little-endian Newton + 1 flag byte`

The flag byte is `bit3=emg`, `bit2=green`, `bit1=yellow`, `bit0=red`.
`/thruster_command` is not normalized and is not rescaled by this node.

## RX BMS frame

The receive stream is framed as:

`START(0xAA) | TYPE(1B) | LEN(1B) | payload(LEN) | CRC16(2B little-endian)`

`TYPE=0x01` is BMS. Its payload is `uint16[4]` little-endian in mV. The
node publishes the decoded values as V to `micon/bms_cells`.

CRC is CRC-16/Modbus: initial value `0xFFFF`, reflected polynomial `0xA001`,
calculated over `TYPE`, `LEN`, and payload bytes. The parser accepts partial
reads and concatenated frames, drops bad CRC/length frames, and resynchronizes
at the next `0xAA`.
