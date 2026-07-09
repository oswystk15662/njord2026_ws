# micon_driver_fd

Transport package for communication with the Micon controller. The current
implementation provides a serial file-descriptor transport; a UDP transport can
be added alongside it without changing the manual-control package.

## serial_writer

- subscribes `command_topic` (`std_msgs/msg/Float32MultiArray`), first 4 normalized commands
- subscribes `/emg`, `/red`, `/yellow`, `/green` (`std_msgs/msg/Bool`)
- parameters: `serial_port` (default `/dev/ttyUSB0`), `baud` (default `115200`),
  and `command_topic` (default `/thruster_command` from `thruster_driver`)

Each 50 ms packet contains four native little-endian `float32` values followed
by a flag byte: `bit3=emg`, `bit2=green`, `bit1=yellow`, `bit0=red`. Emergency
stop enforcement is the Micon firmware's responsibility and requires hardware
validation.
