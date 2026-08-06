# alert_lamp

`alert_lamp` centralizes vessel status evaluation and controls the alert lamp through
`/alert_lamp/command`. The manager never drives hardware directly. The driver turns
the command into on/off outputs and enters red blink when manager commands time out.

## Nodes

- `alert_lamp_manager_node`: subscribes to mode, E-stop, readiness, localization,
  GNSS, diagnostics, and configurable heartbeat topics. It publishes
  `alert_lamp/msg/AlertLampCommand` on `/alert_lamp/command` and manager diagnostics.
- `alert_lamp_driver_node`: performs blinking and publishes `std_msgs/msg/Bool` to
  `/red`, `/yellow`, and `/green`. Those are the existing inputs of
  `micon_driver_fd/serial_writer`, which owns the USB Serial device.

The repository uses hierarchical health-derived heartbeats for driver and
localization status. Other control-layer heartbeats are emitted by their owning
nodes. The integration contract is:

| Input | Type | Default topic |
| --- | --- | --- |
| Operating mode | `std_msgs/msg/String` (`manual` or `auto`) | `/system/operating_mode` |
| Emergency stop | `std_msgs/msg/UInt8` (`RUNNING=0`, `SOFT_EMG=1`, `HARD_EMG=2`) | `/safety/emergency_stop` |
| Autonomy ready | `std_msgs/msg/Bool` | `/autonomy/ready` |
| Localization | `nav_msgs/msg/Odometry` | `/odometry/filtered/global` |
| GNSS / RTK quality | `sensor_msgs/msg/NavSatFix` | `/sensor/vehicle_gnss/fix/raw` |
| Heartbeats | configurable serialized type | `/heartbeat/*` |

Heartbeat subscriptions use ROS 2 generic subscriptions. Driver and localization
heartbeats are `std_msgs/msg/Empty` outputs from `diagnostic_monitors` aggregators;
only receipt time is evaluated here. An absent required input is intentionally a
critical (red blink) state.

GNSS RTK quality is initially inferred from a valid `NavSatFix` with covariance below
`localization.rtk_covariance_threshold`, then retained for `rtk_grace_period_sec`.

## Lamp patterns

| State | Output |
| --- | --- |
| `INITIALIZING` / `CRITICAL_FAULT` | red blink, 0.05 s |
| `MANUAL_NORMAL` | yellow solid |
| `AUTO_NORMAL` | green blink, 0.1 s |
| `AUTONOMY_NOT_READY` | green and yellow blink together, 0.1 s |
| `GROUND_COMMUNICATION_LOST` in AUTO | green and red solid |
| `GROUND_COMMUNICATION_LOST` in MANUAL | yellow and red solid |

Critical faults have priority. In AUTO, loss of high-level, autonomy, or localization
heartbeat is critical; loss of ground station is shown only while localization remains
stable. The driver also red-blinks at a 0.05 s period if no manager command arrives within one
second.
`AlertLampCommand.color` is a bit flag, so a command can turn on multiple lamps.

## Configuration and diagnostics

All topic names, heartbeat types/timeouts, localization thresholds, blink periods, and
driver output topics are in `config/alert_lamp.yaml`. Both nodes publish standard ROS 2
diagnostics on `/diagnostics` named `Alert Lamp Manager` and `Alert Lamp Driver`.

The initial `output_type: topic` is deliberate: GPIO/CAN/Serial ownership is not
implemented in this package because the existing Micon serial writer already owns the
physical serial connection. A dedicated hardware backend can be added later without
changing manager behavior.

## Run and test

```bash
ros2 launch alert_lamp alert_lamp.launch.py
ros2 topic echo /alert_lamp/command

colcon test --packages-select alert_lamp
colcon test-result --verbose
```
