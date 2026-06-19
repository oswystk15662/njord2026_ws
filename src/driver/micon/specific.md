# micon配下 変更予定

## 目的
- `src/driver/micon/` に `thruster_driver` パッケージを追加する。
- 既存のRasPi GPIO直叩き実装は使わず、制御計算部分をベースとしてESP32向けI/Fに置き換える。
- `bms` パッケージを4セル仕様に合わせる。
- `thruster_driver` と `bms` の両方で ESP32 との接続方式を CAN / mROS(USB) どちらにも対応可能にする。
- `kinematics` は互換用に残すが、通常のtask sim/自律航行経路では使わない。

## 要件整理
1. thruster:
- 1台のESP32に1つのESCを接続する前提。
- ESP32へ送る指令値は `std_msgs/msg/UInt16` を1つ。
- 制御計算は `thruster_driver` が `cmd_vel` とfeedback odometryからP+DOBで全体wrenchを計算し、URDF上のスラスタposeとconfig上の推力方向を使って4スラスタへ配分する。
- URDFはスラスタの固定poseだけを持つ。推力方向、反転、ゲイン、deadzone、出力先topic/CAN IDは `thruster_driver` configで管理する。
- ESP32へ送る指令値は `std_msgs/msg/UInt16` を1つ。
- CAN の場合は `can_msgs/msg/Frame` に 2 byte little-endian で詰める。
- mROS の場合は thruster ごとの topic にそのまま流す。

2. bms:
- 4セル分の電圧を `std_msgs/msg/Float32MultiArray` で扱う。
- 配列長は4固定（`[cell1, cell2, cell3, cell4]`）。

3. transport:
- `transport_mode` パラメータで `mros_usb` / `can` / `both` を選択可能にする。
- mROS: ROSトピックで `UInt16` や `Float32MultiArray` をそのまま送受信。
- CAN: `can_msgs/msg/Frame` 経由で送受信。

## 実装方針
### A. thruster_driver 追加
- 追加先: `src/driver/micon/thruster_driver`
- 構成:
  - `ThrusterDriverNode`:
    - subscribe: `geometry_msgs/msg/Twist` (`cmd_vel`)
    - subscribe: `nav_msgs/msg/Odometry` (`/odometry/filtered/local`)
    - subscribe: `std_msgs/msg/Int16MultiArray` (`thruster_command`, bypass用)
    - publish:
      - sim用: `std_msgs/msg/Int16MultiArray` (`/thruster_command`, 4要素)
      - mROS用: `std_msgs/msg/UInt16` を各ESC向けトピックへ
      - CAN用: `can_msgs/msg/Frame` を送信
    - 変換:
      - `cmd_vel -> P+DOB -> wrench -> per-thruster duty`
      - `signed duty -> uint16 PWM(us)`
      - スラスタ順序はconfigの `thrusters.ids` に対応させる
  - GPIO制御クラス（`software_pwm`）は追加しない。

### B. bms 改修
- `src/driver/micon/bms` を実装パッケージ化。
- `BmsBridgeNode`:
  - mROS受信: `std_msgs/msg/Float32MultiArray` (4要素)
  - CAN受信: `can_msgs/msg/Frame` 受信して4セル電圧へdecode
  - 出力: 標準化トピック `bms/cell_voltages` (`Float32MultiArray`, 4要素)
- CANデコード仕様:
  - 8byteを `uint16 x 4` little-endian (単位mV)
  - Vへ変換して配列化 (`mV / 1000.0f`)

### C. 設定・起動
- それぞれのpkgに `config/config.yaml` と `launch/*.launch.py` を追加。
- パラメータで topic名や CAN ID を変更可能にする。

### D. 検証
- `colcon build --packages-select bms thruster_driver robot task2_sim task3_sim` でビルド確認。
- コンパイルエラーが出た場合は依存とincludeを調整する。

## 変更対象
- 新規: `src/driver/micon/thruster_driver/*`
- 更新: `src/driver/micon/bms/*`
- 新規: `src/driver/micon/specific.md`

書き終わり
