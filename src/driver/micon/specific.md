# micon 配下 現行仕様

## 目的

sim で使っている `/thruster_command` と実機 ESP32 の USB serial を接続する。

## thruster

- `thruster_driver` は `cmd_vel` と feedback odometry から4スラスタの Newton 指令を生成する。
- 出力は `/thruster_command` (`std_msgs/msg/Float32MultiArray`, 4要素, Newton) のみ。
- 実機送信は `micon_driver_fd/serial_writer` が担当し、値は再スケールしない。

## serial

- tty を open するのは `micon_driver_fd/serial_writer` だけ。
- TX: `4 x float32 native little-endian Newton + flag byte`, 50 ms period。
- RX: framed BMS telemetry を復号し、`micon/bms_cells` (`Float32MultiArray[4]`, V) へ publish。

## bms

- `bms` は `micon/bms_cells` を購読し、`bms/cell_voltages` と diagnostics を publish する。
- `bms` は serial device を open しない。
