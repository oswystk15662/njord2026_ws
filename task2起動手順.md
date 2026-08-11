# Task 2 起動手順

## 0. 役割と注意

Task 2 は 3 台で分担して起動する。

| 端末 | 担当 |
| --- | --- |
| Jetson | MID360S LiDAR、ZED 2i、Task 2 の認識・他船追跡・MPPI・安全点群 |
| 船体 miniPC | UM982 GNSS、自己位置推定、Nav2、Control Manager、Mission Manager、スラスタ境界 |
| Ground PC | ジョイスティック、非常停止・通信監視、Foxglove、映像受信、ミッション操作 |

各端末は同じネットワーク、`ROS_DOMAIN_ID`、`NJORD_RMW` を使う。実艇を接続した通常起動では、AUTO にする前に非常停止、通信、自己位置、GNSS、Nav2 の状態を必ず確認する。`check task2` が失敗した場合は開始しない。

以下のコマンド中の `/home/hashilab/Desktop/njord2026_ws` は各端末のワークスペースのパスに読み替える。

## 1. GNSS 接続後の通常起動

### 1.1 Jetson（知覚・追跡・MPPI）

Jetson で Livox と ZED を接続してから実行する。`enable_task2_autonomy:=true` が Task 2 の LiDAR 認識、他船追跡、MPPI、`/task2/safety_points` を起動する。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=jetson NJORD_ROLE=jetson
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot jetson_bringup.launch.py enable_task2_autonomy:=true
```

### 1.2 船体 miniPC（GNSS・航法・制御）

miniPC に UM982 GNSS、スラスタ用 Micon、BMS を接続してから実行する。UM982 の既定ポートは `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` である。機体によって異なる場合は、先に `ls -l /dev/serial/by-id/` で確認し、`um982_port:=...` を指定する。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot minipc_bringup.launch.py active_nav2_profile:=task2
```

この launch は既定で UM982、自己位置推定、Task 2 用 Nav2、Control Manager、Mission Manager、スラスタ境界を起動する。GNSS を接続していることは次で確認する。

```bash
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
ros2 topic echo /sensor/vehicle_gnss/compass/raw --once
```

`NavSatFix` が受信できない、または座標が不正な間は AUTO にしない。RTK 補正を使わない確認に限り、`enable_um982_rtk:=false` を追加できる。

### 1.3 Ground PC（操縦・監視）

Ground PC で次を実行する。Foxglove Bridge は既定で起動する。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=groundpc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot ground_pc.launch.py
```

### 1.4 開始前確認とミッション開始

Jetson、miniPC、Ground PC が起動してから、Ground PC で dry-run を行う。`check` は推進・ミッションを開始しない。

```bash
ros2 run mission_manager njord-task check task2
ros2 run mission_manager njord-task start task2 --auto
ros2 run mission_manager njord-task status
```

停止または手動復帰は以下を使う。

```bash
ros2 run mission_manager njord-task stop
ros2 run mission_manager njord-task manual
```

## 2. 屋内での起動確認（ダミー GNSS を含むシミュレーション）

屋内では **実機の miniPC/Jetson bringup をダミー GNSS と組み合わせない**。代わりに `task2_sim` を使う。この launch はダミー GNSS (`/gps/fix`)、ダミー IMU、自己位置、相手船、Task 2 用 Nav2、MPPI を起動する。一方で LiDAR、ZED、UM982、Micon、スラスタドライバは起動しないため、室内で安全に起動連鎖を確認できる。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch task2_sim task2_sim.launch.py
```

別ターミナルでダミー GNSS、自己位置、計画経路を確認する。

```bash
source /opt/ros/humble/setup.bash
source /home/hashilab/Desktop/njord2026_ws/install/setup.bash
ros2 topic echo /gps/fix --once
ros2 topic echo /odometry/filtered/global --once
ros2 topic echo /planned_path_pruned --once
```

シミュレーション用 GNSS を Foxglove の GNSS 地図パネルでも表示する場合は、さらに別ターミナルでトピックを中継する。これはシミュレーション中だけ実行し、実UM982起動中には実行しない（同一トピックへの二重 publish を避けるため）。

```bash
source /opt/ros/humble/setup.bash
source /home/hashilab/Desktop/njord2026_ws/install/setup.bash
ros2 run topic_tools relay /gps/fix /sensor/vehicle_gnss/fix/raw
```

この確認は仮想船体の動作確認であり、実GNSS・Zenoh 通信・実センサー・推進系の動作確認ではない。GNSS を接続した実機確認へ進むときは、この節の `task2_sim` と `topic_tools relay` を停止してから、[1. GNSS 接続後の通常起動](#1-gnss-接続後の通常起動) を Jetson → miniPC → Ground PC の順で実行する。

## 3. Foxglove で Waypoint だけを見る

船体・GNSSなしで Task 2 の waypoint だけを地図で確認する専用 launch がある。Ground PC（または Foxglove を使うPC）で実行する。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=groundpc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot ground_waypoint_map_only.launch.py task_type:=task2
```

これは `/ground_waypoint_markers` を publish して Foxglove Bridge を起動する表示専用モードである。GNSS、Nav2、Mission Manager、critical link、映像、推進ノードは起動せず、自律目標や速度指令も publish しない。

Foxglove で Bridge に接続し、GNSS Map Telemetry のレイアウト（または同パネル）を開く。Waypoint のトピックを `/ground_waypoint_markers` に設定すると、Task 2 の waypoint を表示できる。

通常起動で UM982 が接続されている場合は、同じパネルの GNSS 入力を `/sensor/vehicle_gnss/fix/raw` に設定する。このトピックは実機位置を表示するため、waypoint と機体の現在地を同時に確認できる。屋内シミュレーションでは前節の relay を起動すれば、同じ入力にダミー GNSS を表示できる。

最小確認コマンドは以下である。

```bash
ros2 topic echo /ground_waypoint_markers --once
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
```
