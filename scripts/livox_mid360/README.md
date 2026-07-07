Livox Mid360 動作確認手順

## 前提
- Mid360 をイーサネット経由で `enP8p1s0` に接続する
- `src/driver/lidar/livox_ros_driver2/config/MID360_config.json` の `host_net_info` は `192.168.1.5` 固定なので、ホスト側もこのIPを持つ必要がある

## 手順
1. Mid360 を `enP8p1s0` に接続する
2. `./set_static_ip.sh` を実行（sudoパスワードを求められます）
3. ビルド & 起動
   ```bash
   colcon build --packages-up-to livox_ros_driver2 --symlink-install
   source install/setup.bash
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ```
4. 別ターミナルで topic を確認
   ```bash
   ros2 topic list
   ros2 topic hz /livox/lidar
   ros2 topic hz /livox/imu
   ```
5. 確認が終わったら `./set_dhcp.sh` で DHCP に戻す（sudoパスワードを求められます）

## 既知の結果 (2026-07-07)
- ケーブル未接続 + 静的IP未設定の状態で起動を試したところ、SDKが `192.168.1.5` へのUDPソケットbindに失敗し
  (`bind failed` / `Failed to init livox lidar sdk.`)、`/livox/lidar` 等のtopicは生成されなかった
  (`ros2 topic list` は `/parameter_events`, `/rosout` のみ)。
- sudoはパスワード入力が必須のため、`set_static_ip.sh` の実行と実機接続は人手で行う必要がある
