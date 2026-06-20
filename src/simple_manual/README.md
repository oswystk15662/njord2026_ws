# simple_manual

Manual velocity-command input for the maintained X4 thruster control pipeline.

## Interfaces

- `joy` (`sensor_msgs/msg/Joy`) is read by both manual nodes.
- `joy_converter_node` publishes `cmd_vel`, `/emg`, `/red`, `/yellow`, and `/green`.
- `/emg` uses an active-low joystick contract: it is `false` only while button 0 is held.
- `thruster_driver` performs velocity feedback control and X4 allocation.
- `micon_driver_fd/serial_writer` packs the four normalized thruster commands and one flag byte.

The flag byte is `bit3=emg`, `bit2=green`, `bit1=yellow`, `bit0=red`. The serial
writer transports the emergency flag but does not suppress commands locally; the
ESP32 firmware must enforce the stop. Validate that behavior on restrained
hardware before operation.

```bash
ros2 launch simple_manual manual_control.launch.py serial_port:=/dev/ttyUSB0
```
