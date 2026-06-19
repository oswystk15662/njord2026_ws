# thruster_driver

ESP32向けスラスタ指令ブリッジです。通常の自律航行経路では、このノードが
`cmd_vel` から4スラスタ分の指令までを一貫して生成します。

- 入力:
  - `cmd_vel` (`geometry_msgs/msg/Twist`)
  - feedback odometry (`nav_msgs/msg/Odometry`)
  - または `thruster_command` (`std_msgs/msg/Int16MultiArray`) を直接受ける
- 出力:
  - simulation: `/thruster_command` (`std_msgs/msg/Int16MultiArray`, 4要素)
  - mROS(USB): 各ESC向け `std_msgs/msg/UInt16`
  - CAN: 各ESC向け `can_msgs/msg/Frame` (2byte payload, little-endian)

## ポイント

- GPIO(libgpiod)制御は含みません。
- URDFはスラスタの固定poseだけを持ち、推力方向・反転・ゲインはconfigで切替えます。
- `control.dob.enable` でP+DOBとP-onlyを切替できます。
- `transport_mode` パラメータで `sim` / `mros_usb` / `can` / `both` を切替できます。
- ESP32 1台にESC 1台の前提で、1トピックに1つの `UInt16` を送ります。
