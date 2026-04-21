# thruster_driver

ESP32向けスラスタ指令ブリッジです。

- 入力:
  - `cmd_vel` (`geometry_msgs/msg/Twist`) から内部ミキシング
  - または `thruster_command` (`std_msgs/msg/Int16MultiArray`) を直接受ける
- 出力:
  - mROS(USB): 左右それぞれ `std_msgs/msg/UInt16`
  - CAN: 左右それぞれ `can_msgs/msg/Frame` (2byte payload, little-endian)

## ポイント

- GPIO(libgpiod)制御は含みません。
- `transport_mode` パラメータで `mros_usb` / `can` / `both` を切替できます。
- ESP32 1台にESC 1台の前提で、1トピックに1つの `UInt16` を送ります。
