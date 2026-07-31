this pkg is for launch and visualization

## ロール別 bringup(Jetson / miniPC 2台構成)

構成の全体像・ネットワーク設定・ビルド手順は `Docs/two_machine_split.md` を参照。

| launch | 動かす端末 | 内容 |
|---|---|---|
| `jetson_bringup.launch.py` | Jetson | MID360S + GLIM + pcl_det + ZED 2i。TF publisher / EKF / スラスタ / micon / joy は**含まない** |
| `minipc_bringup.launch.py` | miniPC | 上記以外すべて。USB シリアル機器(micon, UM982, Drogger, IMU, joy)は miniPC に接続する |
| `standalone_bringup.launch.py` | Jetson 1台 | 上記2つを両方 include。分割前の構成を再現する回帰用 |

```
# Jetson
ros2 launch robot jetson_bringup.launch.py

# miniPC
ros2 launch robot task1.launch.py           # role:=minipc が既定
```

`task1/2/3.launch.py` は `role` 引数(`minipc` / `standalone`、既定 `minipc`)でどちらの bringup を使うか選ぶ。

`glim_backend`(`gpu` / `cpu`、既定 `gpu`)で GLIM の設定ディレクトリを切り替えられる。`cpu` は `config/glim_config_cpu/` を使い、OpenGL ビューアを外したヘッドレス構成になる。

## back_cam(背面USBカメラ)

Adesso CyberTrack H7 を使用。

```
ros2 launch robot back_cam.launch.py
```

発行トピック(実測):

- `/back_cam/image_raw`(~26-30Hz)
- `/back_cam/camera_info`

フォーマットは MJPG 640x480 @30fps。

`video_device` は既定で by-id 安定パスを指定している。launch 起動時に `os.path.realpath` で実際の `/dev/videoN` に解決してから `usb_cam` に渡している(`usb_cam` が by-id シンボリックリンクを直接開けないため)。

上書きする場合:

```
ros2 launch robot back_cam.launch.py video_device:=/dev/videoN
```

## real_bringup(実機一括起動)

```
ros2 launch robot real_bringup.launch.py
```

`enable_*` 引数(`enable_imu` のみ既定 `false`、それ以外は `true`):

- `enable_mid360`
- `enable_zed2i`
- `enable_back_cam`
- `enable_um982`
- `enable_drogger_rzs`
- `enable_imu`
- `enable_localization`
- `enable_thruster`
- `enable_nav2`(既定 `true`)

その他の引数:

- `lidar_model`(既定 `mid360s`、`mid360` または `mid360s`)
- `serial_port`(既定 `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c82421728a9aef118808b29061ce3355-if00-port0`)
- `baud`(既定 `115200`)
- `um982_protocol`(既定 `uart`)
- `enable_glim`(既定 `true`)
- `glim_backend`(既定 `gpu`、`gpu` または `cpu`)
- `enable_pcl_buoy_detection`(既定 `false`)
- `enable_gpu_perception`(既定 `false`)
- `gpu_perception_engine_path`(既定は空)
- `camera_resolution`(既定 `HD720`) / `camera_framerate`(既定 `15`)
- `enable_ground_video`(既定 `false`) / `ground_video_host` / `ground_video_port`(既定 `5600`)
- `imu_port`(既定 `/dev/ttyUSB0`) / `imu_baud`(既定 `9600`)
- `um982_port`(既定 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`)
- `enable_um982_rtk`(既定 `false`)
- `drogger_rzs_port`(既定 `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_ACCQg146B12-if00-port0`)
- `drogger_rzs_baud`(既定 `115200`)
- `drogger_rzs_fix_topic`(既定 `/gnss/fix`)
- `device`(back_cam の by-id パス)

使用例(back_cam のみ起動):

```
ros2 launch robot real_bringup.launch.py enable_mid360:=false enable_zed2i:=false enable_um982:=false enable_imu:=false enable_localization:=false enable_thruster:=false
```

注意: `zed2i` を含める場合、SDKモードの GPU 前提が満たされている必要がある(詳細は `src/driver/camera/zed2i_driver/README.md` を参照)。
