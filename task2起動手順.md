# Task 2 起動手順

## 0. 役割と注意

Task 2 は 3 台で分担して起動する。

| 端末 | 担当 |
| --- | --- |
| Jetson | MID360S LiDAR、ZED 2i、Task 2 の認識・他船追跡・MPPI・安全点群 |
| 船体 miniPC | UM982 GNSS、自己位置推定、Nav2、Control Manager、Mission Manager、スラスタ境界 |
| Ground PC | ジョイスティック、非常停止・通信監視、Foxglove、映像受信、ミッション操作 |

各端末は同じネットワーク、`ROS_DOMAIN_ID`、`NJORD_RMW` を使う。実艇を接続した通常起動では、AUTO にする前に非常停止、通信、自己位置、GNSS、Nav2 の状態を必ず確認する。`check task2` が失敗した場合は開始しない。

miniPC と Jetson は、ホームディレクトリから `cd ./njord2026_ws` でワークスペースへ移動できるものとする。Ground PC のパスは、その端末のワークスペースに合わせて読み替える。

## 新しいターミナルを開いたとき

新しいターミナルにはROSやワークスペースの設定は引き継がれない。**ビルドは不要**だが、`ros2` コマンド、launch、topic確認、ダミーGNSSのどれを実行する場合も、最初にその端末の役割に対応する次の設定を行う。

3台で同じ `ROS_DOMAIN_ID` と `NJORD_RMW` を使う。以下では `ROS_DOMAIN_ID=0` を例にしているため、運用で決めた値が別の場合は3台とも同じ値に置き換える。

Jetson（Jazzy）:

```bash
cd ./njord2026_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 NJORD_RMW=fastrtps
export NJORD_PROFILE=jetson NJORD_ROLE=jetson
source scripts/njord_env.sh
source install/setup.bash
```

miniPC（Humble）:

```bash
cd ./njord2026_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 NJORD_RMW=fastrtps
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
source install/setup.bash
```

Ground PC（Humble）:

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 NJORD_RMW=fastrtps
export NJORD_PROFILE=minipc NJORD_ROLE=groundpc
source scripts/njord_env.sh
source install/setup.bash
```

この設定が終わった同じターミナルで、以降に示す `ros2 launch`、`ros2 topic echo`、`njord-task` のいずれかを実行する。別のターミナルを開いた場合も、目的に合う上のブロックを再実行する。

## ビルド方法

`scripts/njord_env.sh` は端末のプロファイルを設定し、GPUがない端末ではGPU専用パッケージをビルド対象から外す。**各端末でビルド前に必ず source する。** 実行後の表示で、Jetson は `NJORD_PROFILE=jetson`、miniPC/Ground PC は `NJORD_PROFILE=minipc` と `NJORD_ENABLE_GPU_SENSORS=<unset>` であることを確認する。

### Jetson（GPUあり）

JetsonではCUDA・ZED SDK・Livox・GLIM・Task 2知覚を含めてビルドする。

```bash
cd ./njord2026_ws
source /opt/ros/jazzy/setup.bash
export NJORD_PROFILE=jetson NJORD_ROLE=jetson
source scripts/njord_env.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

JetsonのROSディストリビューションがJazzy以外の場合だけ、1行目の `jazzy` をその端末のディストリビューション名に置き換える。

### miniPC（GPUなし・実艇側）

miniPCはGPUセンサ処理をビルドしない。`njord_env.sh` がLivox SDK・Livox ROS driver・PCL segmentationを除外し、CPUで動くminiPC側のGNSS、Nav2、制御、Mission Manager、ダミーGNSSをビルドする。

```bash
cd ./njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Ground PC（GPUなし）

Ground PCはGPUセンサ処理をビルドしない。操縦、critical link、Foxglove、映像受信、waypoint表示に必要な範囲だけビルドする。

```bash
cd /home/hashilab/Desktop/njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=groundpc
source scripts/njord_env.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to robot ntripcaster critical_link simple_manual \
  tf_frame_arrow_publisher waypoint_publisher zed2i_driver \
  control_manager mission_manager
source install/setup.bash
```

Ground PCにはJetson用のCUDA/TensorRT/ZED SDKは不要である。`zed2i_driver` はSDKなしではCPU/stub版としてビルドされ、Ground PCでは映像受信launchだけを使用する。

## 1. GNSS 接続後の通常起動

### 1.1 Jetson（知覚・追跡・MPPI）

Jetson で Livox と ZED を接続してから実行する。`enable_task2_autonomy:=true` が Task 2 の LiDAR 認識、他船追跡、MPPI、`/task2/safety_points` を起動する。`enable_glim:=true` により、Livox/IMU からの自己位置 `/odom` も起動する。

```bash
cd ./njord2026_ws
source /opt/ros/jazzy/setup.bash
export NJORD_PROFILE=jetson NJORD_ROLE=jetson
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot jetson_bringup.launch.py enable_task2_autonomy:=true enable_glim:=true
```

### 1.2 船体 miniPC（GNSS・航法・制御）

miniPC に UM982 GNSS、スラスタ用 Micon、BMS を接続してから実行する。UM982 の既定ポートは `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` である。機体によって異なる場合は、先に `ls -l /dev/serial/by-id/` で確認し、`um982_port:=...` を指定する。

```bash
cd ./njord2026_ws
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

## 2. 屋内での起動確認（実機構成 + ダミー GNSS）

屋内確認では、GNSS 以外を通常起動と同じにする。つまり Jetson の Livox、ZED、GLIM、Task 2 認識・追跡・MPPI、miniPC の Nav2、Control Manager、Mission Manager、Micon／スラスタ境界、BMS、背面カメラ、Ground PC の操縦・監視をすべて起動する。**置き換えるのは UM982 だけ**である。

安全のため、係留またはプロペラを外した状態で実施し、AUTO は要求しない。ダミーGNSSは起動確認用であり、屋内での自律航行を許可するものではない。

### 2.1 Jetson と Ground PC

Jetson と Ground PC は [通常起動](#1-gnss-接続後の通常起動) とまったく同じコマンドで起動する。Jetson の GLIM が `/odom` を出力し、この値をダミーGNSSの元データとして使う。

### 2.2 miniPC（UM982 だけ無効化）

miniPC では実UM982を起動しない以外、通常と同じ Task 2 bringup を起動する。`enable_um982:=false` 以外の既定値は変更しないため、スラスタを含む実機ノード群が起動する。

```bash
cd ./njord2026_ws
source /opt/ros/humble/setup.bash
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
source install/setup.bash
ros2 launch robot minipc_bringup.launch.py active_nav2_profile:=task2 enable_um982:=false
```

### 2.3 miniPC でダミー GNSS を起動

別ターミナルで、ダミーGNSSをUM982と同じ `/sensor/vehicle_gnss/fix/raw` へ出す。方位 `/sensor/vehicle_gnss/compass/raw` も同時に出力される。Jetson のGLIMから `/odom` が届くまでメッセージは出ないため、先に `/odom` を確認する。

```bash
cd ./njord2026_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 NJORD_RMW=fastrtps
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
source install/setup.bash
ros2 topic echo /odom --once
```

次の `gps_origin_lat` と `gps_origin_lon` は、屋内確認で地図に表示したい基準位置へ変更する。既定値はGPS 5の約15 m北（GPS 5→6距離の約1/3）である。Task 2 のWPと重ねて確認する場合は、Task 2 水域に近い座標を指定する。

```bash
cd ./njord2026_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 NJORD_RMW=fastrtps
export NJORD_PROFILE=minipc NJORD_ROLE=minipc
source scripts/njord_env.sh
source install/setup.bash
ros2 run sensor_sim_with_noise gnss_noise_simulator --ros-args \
  -r /gps/fix:=/sensor/vehicle_gnss/fix/raw \
  -p gps_origin_lat:=63.4409375 \
  -p gps_origin_lon:=10.4233194444 \
  -p gps_origin_alt:=0.0
```

`sensor_sim_with_noise` が未ビルドの場合は、このコマンドの前にminiPCで一度だけ実行する。

```bash
cd ./njord2026_ws
colcon build --packages-select sensor_sim_with_noise
source install/setup.bash
```

起動後は、実GNSS時と同じトピック名で確認できる。

```bash
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
ros2 topic echo /sensor/vehicle_gnss/compass/raw --once
ros2 topic echo /odometry/filtered/global --once
```

実GNSSに戻す場合はダミーGNSSノードを `Ctrl-C` で停止し、miniPC bringupも停止してから、[1. GNSS 接続後の通常起動](#1-gnss-接続後の通常起動) のminiPCコマンド（`enable_um982:=false` を付けないもの）で再起動する。実UM982とダミーGNSSを同時に起動してはならない。

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

通常起動で UM982 が接続されている場合は、同じパネルの GNSS 入力を `/sensor/vehicle_gnss/fix/raw` に設定する。このトピックは実機位置を表示するため、waypoint と機体の現在地を同時に確認できる。屋内確認でもダミーGNSSは同じトピックを使うため、追加設定なしで現在地表示を確認できる。

最小確認コマンドは以下である。

```bash
ros2 topic echo /ground_waypoint_markers --once
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
```
