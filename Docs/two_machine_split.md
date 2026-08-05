# Jetson / miniPC 2台構成

## なぜ分割したか

Jetson Orin Nano Super 1台で全ノードを動かしていた構成は、実測で GPU 95-99%、RAM 5.9/7.3 GiB まで飽和し、各ノードの更新周期が落ちていた（`README.md` の 2026-07-17 実機テスト記録を参照）。

そこで **ZED 2i と Livox MID360S を使う処理だけを Jetson に残し、それ以外を miniPC へ移した**。

同一ソースツリーが両方の端末でビルドできるよう、CUDA / TensorRT / ZED SDK / ROS distro の有無をビルド時に検出して依存とリンク先を切り替える仕組み（`njord_platform`）を入れてある。

## ノード配置

### Jetson（`jetson_bringup.launch.py`）

| ノード | パッケージ | 理由 |
|---|---|---|
| `livox_ros::DriverNode` | `livox_ros_driver2` | `/livox/lidar` は約 42 Mbps。GLIM / pcl_det と intra-process で同居させ、線に出さない |
| `glim::GlimROS` | `glim_ros` | `VGICP_GPU`。出力は `/odom` と TF のみ |
| `pcl_det::PclBuoyDetectionNode` | `pcl_det` | CPU only だが `/livox/lidar` を食うので同居必須 |
| `zed2i_driver::SdkNode` | `zed2i_driver` | ZED SDK + CUDA + TensorRT。`/livox/lidar` も購読する |

上 3 つは `lidar.launch.py` の `livox_perception_container`（`component_container_mt` + `use_intra_process_comms: True`）に載る。

### miniPC（`minipc_bringup.launch.py`）

`robot_state_publisher` / static TF / `um982_driver` / `um982_feedback_filter` / `drogger_wired_flex` / `witmotion_imu_driver` / `ekf_local` / `ekf_global` / `navsat_transform_node` / `joy_node` / `joy_converter` / `twist_mux` / `thruster_driver` / `micon_driver_fd serial_writer` / `bms` / `alert_lamp` / `buoy_obstacle_publisher` / `diagnostic_monitors` / `foxglove_bridge`

**USB シリアル機器は全て miniPC 側に接続する**（ESP32 micon = スラスタとランプ、UM982 GNSS、Drogger、WIT IMU、joy パッド）。

Nav2 は `minipc_bringup` では既定 `false`。`task1/2/3.launch.py` が params ファイルを選んで起動する。

## トピックの流れ

### 線に出るもの（すべて低帯域）

`/odom`、`/tf`、`/tf_static`、`/livox/imu`（約 0.5 Mbps）、`/buoy_detections`、`/buoy_detections_3d`、`/virtual_obstacles`、`/buoy_roi`、`/diagnostics`

### 絶対に線に出してはいけないもの

| トピック | 型 | 帯域 |
|---|---|---|
| `/livox/lidar` | PointCloud2 (26 B/点) | 約 42 Mbps |
| `/zed2i/left/image_rect` | Image bgra8 1280x720 @15 | 約 442 Mbps |
| `/zed2i/right/image_rect` | 同上 | 約 442 Mbps |
| `/zed2i/depth/image` | 32FC1 1280x720 @15 | 約 442 Mbps |
| `/zed2i/points` | PointCloud2 | 数 Gbps（既定で publish 無効） |

miniPC 側で映像を見たい場合は DDS ではなく既存の GPU-JPEG → RTP/UDP 経路を使う:
`zed2i_driver/src/ground_video_streamer.cu`（送信、Jetson）と `zed2i_driver/launch/ground_video_receiver.launch.py`（受信、miniPC）。
`jetson_bringup.launch.py` の `enable_ground_video:=true ground_video_host:=<miniPC の IP>` で有効化する。
起動順序(受信側を先に起動)・再ビルド必須・ポート二重起動禁止・実測レートといった手順の詳細は
最上位 `README.md` と `src/driver/camera/zed2i_driver/README.md` の
「陸上映像伝送(ground video)の正しい起動手順」を参照。

### 時刻同期は必須

`pcl_det` は `/livox/lidar` と `/buoy_roi` をヘッダ時刻で相関させ、ROI が 0.5 秒より古いクラウドを捨てる。両機の時刻がずれると検出が出なくなる。

```bash
sudo apt install chrony
# miniPC を NTP サーバ、Jetson をクライアントにする（chrony.conf を編集）
chronyc tracking     # 両機で確認。差が 1 ms 以下であること
```

## ビルド

### プラットフォーム検出

`njord_platform` パッケージが `njord_detect_platform()` を提供し、ビルド時に以下を検出する:

`ROS_DISTRO` / CUDA Toolkit + nvcc / TensorRT (`NvInfer.h` + `libnvinfer`) / ZED SDK (`zed-config.cmake` の CUDA メジャーが一致する場合のみ) / `glim_ros` / `livox_ros_driver2`

検出結果は `share/njord_platform/platform.yaml` と `platform.env` に書き出される。launch ファイルやテストはこれを読めばよく、自前で再検出する必要はない。

```bash
cat install/njord_platform/share/njord_platform/platform.yaml
```

### 手順（両機共通）

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source scripts/njord_env.sh      # プロファイル判定と環境変数の export
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`scripts/build.sh` は上記をまとめたラッパ。

`scripts/njord_env.sh` がやること:

1. `NJORD_PROFILE` を `jetson` / `minipc` に決定（CUDA と ZED SDK の両方があれば `jetson`）。既に環境変数で設定済みなら**尊重して上書きしない**
2. `NJORD_ENABLE_GPU_SENSORS` を export（jetson なら `1`、minipc なら **unset**）
3. `NJORD_ROLE`、`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` を export
4. minipc プロファイル時、`src/driver/lidar/Livox-SDK2` と `livox_ros_driver2` に `COLCON_IGNORE` を置く（jetson なら削除する）

### 依存の切り替え方

`package.xml` の format 3 `condition` 属性を使っている:

```xml
<exec_depend condition="$NJORD_ENABLE_GPU_SENSORS == 1">glim_ros</exec_depend>
<exec_depend condition="$NJORD_ENABLE_GPU_SENSORS == 1">livox_ros_driver2</exec_depend>
```

環境変数が**未設定のとき依存しない**向きにしてあるので、`njord_env.sh` を source し忘れても miniPC で `rosdep install` が壊れない。

### CUDA / ZED SDK が無い環境での挙動

`zed2i_driver` は CMake が自動で `src/sdk_node_zed.cpp` の代わりに `src/sdk_node_stub.cpp` をビルドする。実行ファイル名 `zed2i_sdk_node` と component 登録は変わらないので、パッケージ全体は常にビルドできる。**miniPC では `zed2i_sdk_node` を起動しないこと**（stub なので何もしない）。

ZED SDK の探索は、`zed-config.cmake` に記録された CUDA メジャーバージョンが検出済み CUDA Toolkit と一致した場合にのみ `find_package(zed)` を呼ぶ。SDK の config が内部で `find_package(CUDA <major> REQUIRED)` するため、不一致のまま呼ぶと CPU フォールバックすら configure できなくなるため。

### ヘッダ名の distro 差異

- `njord_distro_header()` — foxy は `.h`、humble/jazzy は `.hpp`（`tf2_geometry_msgs` 用）
- `njord_find_header()` — 実際にヘッダが存在するかを probe する。`cv_bridge` は Humble のパッチリリース途中で `.h` → `.hpp` に変わったため、distro 名からは決められない

## 起動

### 2台構成（通常）

Jetson:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash && source scripts/njord_env.sh && source install/setup.bash
ros2 launch robot jetson_bringup.launch.py
```

miniPC:
```bash
source /opt/ros/humble/setup.bash && source scripts/njord_env.sh && source install/setup.bash
ros2 launch robot task1.launch.py          # role:=minipc が既定
```

`task2.launch.py` / `task3.launch.py` も同様。

### systemd による基盤 bringup の常駐化

`jetson_bringup` と `minipc_bringup` は、タスクに依存しない基盤として
systemd 常駐にできる。workspace をビルドした後、それぞれの端末で実行する。

```bash
# Jetson（Jazzy の例）
scripts/install_bringup_service.sh --role jetson --ros-distro jazzy --domain-id 0

# miniPC（Humble の例）
scripts/install_bringup_service.sh --role minipc --ros-distro humble --domain-id 0
```

このスクリプトは通常ユーザーで ROS プロセスを実行する
`njord-jetson-bringup.service` または `njord-minipc-bringup.service` を登録し、
有線ネットワークと時刻同期の後に起動する。service 内でも
`scripts/njord_env.sh` を source するため、`ROS_DOMAIN_ID` と `NJORD_RMW` が両機で
一致するよう、導入時に同じ値を指定すること。

miniPC service は `enable_nav2:=false` 固定である。Nav2、waypoint publisher、
autonomy supervisor は task manager が start/stop を管理するタスク層であり、
常駐 service に含めない。

確認・停止・無効化:

```bash
sudo systemctl status njord-jetson-bringup.service   # Jetson
sudo systemctl status njord-minipc-bringup.service   # miniPC
journalctl -u njord-minipc-bringup.service -f
sudo systemctl disable --now njord-minipc-bringup.service
```

接続デバイスの権限は service の実行ユーザーに引き継がれる。miniPC では少なくとも
`dialout`（serial）、必要に応じて `video` / `render` グループへそのユーザーを追加し、
再ログインしてから導入する。最初の起動前に、時刻同期・DDS 通信・各
`/dev/serial/by-id/...` パスを手動で確認すること。

### 1台構成（回帰確認用）

Jetson 1台で分割前と同じ構成を動かす:

```bash
ros2 launch robot task1.launch.py role:=standalone
```

`standalone_bringup.launch.py` が `jetson_bringup` と `minipc_bringup` の両方を include する。分割前の `manual_control.launch.py` にあった段階起動（LiDAR 18 秒、カメラ 20 秒）も再現する。

`simple_manual/launch/manual_control.launch.py` は従来経路としてそのまま残してある。

### GLIM の CPU バックエンド

GLIM を CUDA 無しの端末で動かす場合:

```bash
ros2 launch robot jetson_bringup.launch.py glim_backend:=cpu
```

`share/robot/config/glim_config_cpu/` を使う。`config_odometry_cpu.json` / `config_sub_mapping_cpu.json` / `config_global_mapping_cpu.json` を選択し、ヘッドレス運用のため `libstandard_viewer.so`（OpenGL GUI）を外してある。

ただし `glim` / `gtsam_points` 自体が CUDA 無しでビルドされている必要がある。

## ネットワーク

### 有線 Ethernet

Jetson と miniPC は有線で直結（またはスイッチ経由）する。Livox MID360S は Jetson 側の別 NIC に繋ぐ（host `192.168.1.5` / LiDAR `192.168.1.114`、`src/robot/config/livox/MID360S_config.json`）。

### FastRTPS（既定）

```bash
export NJORD_RMW=fastrtps    # 省略可（既定）
source scripts/njord_env.sh
```

`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` と `FASTRTPS_DEFAULT_PROFILES_FILE=config/dds/fastdds_profile.xml` が設定される。

**Jetson が Jazzy、miniPC が Humble という distro 混在では FastRTPS が必須**（`Docs/instruction/about_thruster.md` 参照）。

`config/dds/fastdds_profile.xml` の `interfaceWhiteList` は既定でコメントアウトしてある。Wi-Fi 経由のディスカバリを排除したい場合は、有線 NIC の IP を書いて有効化する。

両機で `ROS_DOMAIN_ID` を揃えること。

### Zenoh（オプトイン）

```bash
export NJORD_RMW=zenoh
source scripts/njord_env.sh
```

`RMW_IMPLEMENTATION=rmw_zenoh_cpp`、`ZENOH_ROUTER_CONFIG_URI` / `ZENOH_SESSION_CONFIG_URI` が `config/zenoh/` を指す。

miniPC で router を**1つだけ**起動する:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Jetson 側のセッションは `client` モードで router に接続するので、マルチキャストに依存しない。

**注意点**:

- `rmw_zenoh` は Jazzy 以降が本流。Humble 版はソースビルドが必要
- 両機の `rmw_zenoh` リビジョン / zenoh プロトコルバージョンが揃っていないと接続できない
- **実機投入前に必ず FastRTPS で通してから切り替え検証すること**

## 検証

### miniPC でのビルド回帰

```bash
source /opt/ros/humble/setup.bash && source scripts/njord_env.sh
rosdep install --from-paths src --ignore-src -y   # glim_ros / livox_ros_driver2 を要求しないこと
colcon build --symlink-install
colcon test --packages-select robot && colcon test-result --verbose
```

`src/robot/test/test_launch_roles.py` が分割の回帰テスト。`minipc_bringup.launch.py` が GPU 側パッケージ（`glim_ros` / `livox_ros_driver2` / `zed2i_driver` / `glim_config`）を参照しないこと、`jetson_bringup.launch.py` が miniPC 側パッケージ（`robot_localization` / `thruster_driver` / `micon_driver_fd` / `joy_node`）を参照しないことを assert する。

`src/robot/test/test_platform_manifest.py` は `platform.yaml` の妥当性を検証する。

CI（`.github/workflows/ros2-ci.yaml`）の `core-simulation-build` ジョブは `ros:humble-ros-base-jammy` = CUDA 無しなので、miniPC プロファイルの回帰テストとしてそのまま機能する。

### 実機での確認

miniPC 側で:

```bash
ros2 topic hz /odom /tf /buoy_detections_3d    # 期待レートで見えること
ros2 topic hz /livox/lidar                     # 見えないこと（帯域漏れの検出）
chronyc tracking                               # 時刻差 1 ms 以下
```

Jetson 側で `tegrastats` を流し、GPU 使用率と RAM が分割前（GPU 95-99% / RAM 5.9 GiB）より下がっていることを確認する。
