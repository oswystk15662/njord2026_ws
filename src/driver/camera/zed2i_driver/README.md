# zed2i_driver

ZED 2i カメラ用ドライバ。以下の2モードを提供する。

- `sdk`(既定): ZED SDK + CUDA を使用する本線モード。
- `cpu`: OpenCV ステレオマッチングによるフォールバックモード。

## 起動方法

SDK モード(既定):

```
ros2 launch zed2i_driver zed2i.launch.py
```

CPU モード:

```
ros2 launch zed2i_driver zed2i.launch.py mode:=cpu
```

パラメータファイルを差し替える場合:

```
ros2 launch zed2i_driver zed2i.launch.py params_file:=/path/to.yaml
```

既定のパラメータファイルは `config/zed2i_jetson_orin_nano.yaml`。

SDK/CPUノードはいずれもROS 2 componentとして登録され、上記launchでは
`component_container_mt`内に`use_intra_process_comms=true`でロードされる。
大容量のImage/PointCloud2は`UniquePtr`でpublishするため、同じcontainerへC++の
購読componentを追加した場合はintra-process通信を利用できる。Pythonノードなど
container外の購読者との通信は従来どおりDDS経由になる。

## 発行トピック(SDKモードで確認済み)

- `/zed2i/left/image_rect`
- `/zed2i/right/image_rect`
- `/zed2i/left/camera_info`
- `/zed2i/right/camera_info`
- `/zed2i/depth/image`
- `/zed2i/points`

## config について

`left_device` / `right_device` は **CPUモード専用**のパラメータで、SDKモードでは無視される。

安定運用のため、`/dev/videoN` の番号ずれを避けて by-id パスを使用する。

- 左: `/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0`
- 右: `/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index1`

## 前提条件(重要)

SDKモードは**動作する GPU/CUDA が必須**。現在このホストは iGPU がドライバに未登録で `NO GPU DETECTED` により SDK ノードが起動不可(ホスト側ブロッカー)。

詳細と復旧手順は `.ai/escalations/2026-07-12-zed-gpu-cuda-blocker.md` を参照。

## CPUモードの既知の制約

ZED 2i はステレオ対を単一の UVC デバイスとして公開するため、現状の左右別デバイス前提の `cpu_stereo_node` では right デバイスの open に失敗する(要 node 改修、別タスク)。

そのため、**SDKモードが本線**であり、CPUモードは GPU 復旧までの暫定フォールバックとして位置づける。
