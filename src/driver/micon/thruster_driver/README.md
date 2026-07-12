# thruster_driver

`cmd_vel` と feedback odometry から船体 wrench を計算し、URDF 上の
4スラスタ配置に配分して `/thruster_command` へ publish します。

- 入力:
  - `cmd_vel` (`geometry_msgs/msg/Twist`)
  - feedback odometry (`nav_msgs/msg/Odometry`)
  - または `thruster_command` (`std_msgs/msg/Int16MultiArray`) を直接受ける
- 出力:
  - `/thruster_command` (`std_msgs/msg/Float32MultiArray`, 4要素, Newton)

`/thruster_command` は常に Newton です。実機では `micon_driver_fd/serial_writer`
がこの値を再スケールせず、そのまま 4×`float32` として ESP32 へ送ります。

## ポイント

- CAN や per-thruster ROS topic への出力は持ちません。
- URDFはスラスタの固定poseだけを持ち、推力方向・反転・ゲインはconfigで切替えます。
- `control.dob.enable` でP+DOBとP-onlyを切替できます。
- 起動時、cmd timeout、feedback timeout時は 0N を publish します。
