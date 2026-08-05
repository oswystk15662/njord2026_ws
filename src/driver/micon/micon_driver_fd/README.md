# micon_driver_fd

Transport package for communication with the Micon controller. The current
implementation provides a serial file-descriptor transport; a UDP transport can
be added alongside it without changing the manual-control package.

## serial_writer

- subscribes `command_topic` (`std_msgs/msg/Float32MultiArray`), first 4 thrust commands in N
- subscribes `/soft_emg`, `/red`, `/yellow`, `/green` (`std_msgs/msg/Bool`)
- publishes `/micon/relay_active` (`std_msgs/msg/Bool`) from the firmware's relay-state byte
  and `/safety/emergency_stop` (`std_msgs/msg/UInt8`): `RUNNING=0`, `SOFT_EMG=1`,
  `HARD_EMG=2`
- parameters: `serial_port` (default `/dev/ttyUSB0`), `baud` (default `115200`),
  `command_topic` (default `/thruster_command` from `thruster_driver`), and
  `bms_topic` (default `/bms`)

Every 50 ms, `serial_writer` sends a `THRUSTER_COMMAND` frame compatible with
`Docs/PROTOCOL.md`:

```text
COBS(raw_frame) || 0x00
```

The raw frame is 24 bytes: 5 byte header, 17 byte payload, and little-endian
CRC-16/CCITT-FALSE. The payload contains four little-endian `float32` thrust
values followed by `control_flags`: `bit3=emg`, `bit2=green`, `bit1=yellow`,
`bit0=red`.

The relay-state byte is telemetry only: GPIO15 follows the relay and can become
active during a software stop, so it is not an independent physical-E-stop signal.
`SOFT_EMG` is emitted whenever `/soft_emg` is true; otherwise an active relay is
reported as `HARD_EMG` for operational status.

## BMS receive

`Docs/njord_BMS_v0/master` sends one CSV row per second over the same serial
port. `serial_writer` extracts `cell1_V` through `cell4_V` and publishes them
as a four-element `std_msgs/msg/Float32MultiArray` on `bms_topic`. CSV headers,
stale rows containing `nan`, and malformed rows are ignored.
