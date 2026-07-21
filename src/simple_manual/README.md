# simple_manual

Real-vessel manual bringup and velocity-command input for the maintained X4
thruster control pipeline. Navigation is not started.

The launch starts:

- MID360S, UM982, Advanced Navigation Spatial, and wired Drogger drivers
- the complete localization launch
- the LiDAR preprocessing, segmentation/clustering, and tracking pipeline
- `/joy` from the shore PC -> `joy_converter` -> `thruster_driver` ->
  `micon_driver_fd`

Camera/YOLO detection is not started because it is not part of this sensor
configuration. The LiDAR detection pipeline can be disabled for troubleshooting
with `enable_detection:=false`.

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

The `axis.*`, `button.*`, and `scale.*` parameters belong to the
`/joy_converter` node and can be changed with ROS 2 parameter commands.

## Running the joystick on the shore PC

`manual_control.launch.py` does not start `joy_node`. Start it on the shore PC
in the same ROS domain so its `/joy` topic reaches the vessel PC.

## Pairing a DualShock4 controller over Bluetooth

The shore PC's `joy` node reads from `/dev/input/js0`, which requires the
DualShock4 to be paired and connected over Bluetooth first.

MAC address on record: `28:C1:3C:3F:D4:80`

1. Put the controller into pairing mode: hold **PS + SHARE** simultaneously
   for ~5 seconds until the light bar flashes white rapidly.
2. On the host (Jetson), run `bluetoothctl` and execute:
   ```
   agent on
   default-agent
   pairable on
   scan on
   ```
   Wait until `Device 28:C1:3C:3F:D4:80 Wireless Controller` appears, then
   `scan off`.
3. Pair, trust, and connect:
   ```
   pair 28:C1:3C:3F:D4:80
   trust 28:C1:3C:3F:D4:80
   connect 28:C1:3C:3F:D4:80
   ```
   `pair` only needs to succeed once; on subsequent uses (after re-enabling
   pairing mode on the controller) `connect` alone is enough. If `connect`
   fails with `br-connection-create-socket`, the controller is not currently
   in pairing mode — repeat step 1.
4. Verify the joystick device node exists:
   ```bash
   ls /dev/input/js0
   ```
5. Start `joy_node` on the shore PC:
   ```bash
   ros2 run joy joy_node
   ```

To disconnect: `bluetoothctl disconnect 28:C1:3C:3F:D4:80`. The pairing
(trust) persists across reboots, so a bare `connect` (after waking the
controller with the PS button) is usually sufficient after the first setup.
