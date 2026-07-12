this pkg is for launch and visualization

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

`enable_*` 引数(既定はすべて `true`、`enable_nav2` のみ `false`):

- `enable_mid360`
- `enable_zed2i`
- `enable_back_cam`
- `enable_um982`
- `enable_imu`
- `enable_localization`
- `enable_thruster`
- `enable_nav2`(既定 `false`)

その他の引数:

- `serial_port`(既定 `/dev/ttyUSB1`)
- `baud`(既定 `115200`)
- `device`(back_cam の by-id パス)

使用例(back_cam のみ起動):

```
ros2 launch robot real_bringup.launch.py enable_mid360:=false enable_zed2i:=false enable_um982:=false enable_imu:=false enable_localization:=false enable_thruster:=false
```

注意: `zed2i` を含める場合、SDKモードの GPU 前提が満たされている必要がある(詳細は `src/driver/camera/zed2i_driver/README.md` を参照)。