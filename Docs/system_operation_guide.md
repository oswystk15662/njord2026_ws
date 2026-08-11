# NJORD システム運用ガイド

この文書は、Jetson、船体 miniPC、Ground PC を分けて起動し、Ground PC から
ミッションを確認・開始するための実運用手順である。各端末は同じ
`ROS_DOMAIN_ID` と `NJORD_RMW` を使う。Task 4 の経路は暫定 Task 1 測量値を
流用しているため、実海域での自律運転前には承認済みの座標へ置き換えること。

## 1. 共通の準備

各端末でワークスペースを更新後、役割に応じた環境を読み込んでビルドする。
`njord_env.sh` の表示で GPU の有無と役割が意図どおりであることを必ず確認する。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
source scripts/njord_env.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

GPUのない Ground PC では、環境を明示する。ビルド対象を絞る手順は
[`two_machine_split.md`](two_machine_split.md) の「Ground PC（GPUなし）のビルド」を使う。

```bash
export NJORD_PROFILE=minipc
export NJORD_ROLE=groundpc
```

## 2. 通常の起動順

### Jetson（Task 2知覚・追跡・MPPI）

ZED / Livox / GPU知覚を持つ端末だけで実行する。Task 2では
`enable_task2_autonomy:=true` を必ず付ける。これがLiDAR認識、他船追跡、MPPIと
`/task2/safety_points` を起動し、Zenoh bridge経由でminiPCへ渡す。

```bash
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=jetson NJORD_ROLE=jetson
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot jetson_bringup.launch.py enable_task2_autonomy:=true
```

### 船体 miniPC（Task 2航法・制御・Mission Manager）

Task 2を実行する日は `task2` プロファイルで起動する。miniPCはTask 2用
FollowPath、velocity smoother、Collision Monitor、safety-cloud gate、Control Manager、
Mission Managerとスラスタ境界を担当する。推進系を含むため、非常停止、通信、自己位置、
Nav2、collision monitor の状態を現場で確認してから AUTO を要求する。

```bash
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot minipc_bringup.launch.py active_nav2_profile:=task2
```

### Ground PC（操縦・監視）

Ground PCはGPU処理を行わず、ジョイスティック、ground heartbeat、critical link、
Zenoh bridge、Foxglove、映像受信、NTRIP caster、経路表示を担当する。

```bash
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=groundpc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot ground_pc.launch.py
```

FoxgloveをGround PCのbridgeへ接続し、`/mission/status`、`/ground_waypoint_markers`、
`/actual_path_marker`、必要な映像トピックを表示する。

## 3. Task 2 の確認・開始・停止

Task 2は以下の順で行う。`check` は dry-run であり、ミッションや推進を開始しない。

1. Jetsonを `enable_task2_autonomy:=true` 付きで起動し、LiDAR・追跡・MPPIが稼働することを確認する。
2. miniPCを `active_nav2_profile:=task2` 付きで起動し、制御・Nav2・Mission Managerを稼働させる。
3. Ground PCのジョイスティック、非常停止、critical-link heartbeat、映像とFoxglove表示を確認する。
4. Ground PCから `check task2` を実行し、成功した場合だけAUTOを要求する。

```bash
ros2 run mission_manager njord-task check task2
ros2 run mission_manager njord-task start task2 --auto
ros2 run mission_manager njord-task status
```

停止または手動復帰は次を使う。

```bash
ros2 run mission_manager njord-task stop
ros2 run mission_manager njord-task manual
```

`check task2` が失敗した場合は開始しない。表示された profile、Jetson由来の経路・
安全点群readiness、通信、安全インターロックを解消してから再確認する。

## 4. GNSSなし・船体なしで waypoint 地図だけを見る

`ground_waypoint_map_only.launch.py` は表示専用である。GNSS、Nav2、Mission Manager、
critical link、映像受信、推進ノードを一切起動しない。YAMLに保存済みの緯度経度を
WGS84 MarkerArrayとして再送するため、ダミーGNSSは不要である。

```bash
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=groundpc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot ground_waypoint_map_only.launch.py task_type:=task4
```

Taskを替える場合は `task_type:=task1`、`task2`、`task3_1`、`task3_2` を指定する。
Foxgloveで `/ground_waypoint_markers` を地図表示に追加する。このモードは表示専用で、
いかなる自律目標・速度指令・実機GNSSトピックも publish しない。

## 5. 起動後の最小確認

通常運用では次を確認する。

```bash
ros2 topic echo /mission/status --once
ros2 run mission_manager njord-task status
```

地図専用モードでは次だけでよい。

```bash
ros2 topic echo /ground_waypoint_markers --once
```
