# Jetson での YOLO11s ノード運用手順

`yolo11_node` (標準 Ultralytics YOLO11s 用の新ノード) を Jetson で動かすための
環境構築とノード起動の手順。既存の `yolo_node` (YOLOv10 fork ベース) とは
独立した並行実装で、既存ノードの環境は変更しない。

## 環境保護方針

| コンポーネント | 置き場所 | 理由 |
|---|---|---|
| ROS 2 / cv_bridge | システム Python | apt 管理の ROS と ABI を揃えるため venv に入れない |
| ultralytics (YOLO) | 専用 venv (`--system-site-packages`) | システム Python を汚さない |
| torch | NVIDIA 提供 wheel (JetPack 対応品) | **PyPI の torch で pip 上書き禁止** (CUDA が使えなくなる)。requirements にも入れていない |
| numpy | 1.26 系固定 | **numpy 2 系は禁止**。cv_bridge / ROS Python が壊れる |
| ultralytics バージョン | 8.3.x | YOLO11 対応系列に固定 |

## venv 作成手順 (Jetson 上)

```bash
# 1. システム側の ROS 2 を先に source しておく
source /opt/ros/humble/setup.bash

# 2. 専用 venv を --system-site-packages で作成
#    (cv_bridge などのシステムパッケージを venv から見えるようにする)
python3 -m venv ~/venvs/yolo11 --system-site-packages
source ~/venvs/yolo11/bin/activate

# 3. Jetson 用 torch (NVIDIA wheel) を先にインストールする
#    JetPack のバージョンに対応した wheel を NVIDIA のドキュメントから選ぶこと:
#    https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/
#    例 (バージョンは実機の JetPack に合わせて要確認):
#    pip install --no-cache <nvidia-torch-wheel-url>
#    ★注意★ この後どの手順でも `pip install torch` で PyPI 版を入れないこと。

# 4. YOLO 関連をインストール (torch は含まれない)
pip install -r src/detection/yolo/requirements_jetson_yolo11.txt

# 5. numpy が 1.26 系のままであることを必ず確認
python3 -c "import numpy; print(numpy.__version__)"   # 1.26.x であること
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

ultralytics の依存で numpy や opencv が更新されそうになった場合は
`pip install "numpy>=1.26,<2"` で戻すこと。

## 重みファイルの配置

`yolo11s.pt`（学習済み重み）はリポジトリに **含まれていない**。

```bash
# 例: パッケージ config に配置してビルド (share にコピーされる)
cp /path/to/yolo11s.pt src/detection/yolo/config/yolo11s.pt
colcon build --packages-select yolo
```

または任意の絶対パスに置き、`model_path` パラメータ / launch 引数で指定する。

## TensorRT engine の作成 (任意、高速化)

```bash
source ~/venvs/yolo11/bin/activate
python3 scripts/yolo/export_yolo11s_tensorrt.py \
    --weights /path/to/yolo11s.pt --imgsz 640 --output-dir ~/yolo11_engines
```

- Jetson 上でのみ実行する (engine はデバイス固有。Mac では実行禁止)
- FP16 がデフォルト。INT8 は `--int8 --calib-data <dataset.yaml>` が必須
- 出力先に `export_env.json` (torch/ultralytics/tensorrt/CUDA/JetPack のバージョン記録) が生成される

## ノード起動

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venvs/yolo11/bin/activate    # launch に venv ガードがあるので必須

# PyTorch backend
ros2 launch yolo yolo11s.launch.py model_path:=/path/to/yolo11s.pt device:=cuda:0

# TensorRT backend (.engine を渡すだけで自動切替)
ros2 launch yolo yolo11s.launch.py \
    model_path:=/path/to/yolo11s.engine backend:=tensorrt
```

パラメータの詳細は `config/yolo11s_params.yaml` のコメントを参照。

## ベンチマーク

```bash
source ~/venvs/yolo11/bin/activate
python3 scripts/task2/benchmark_yolo11s.py --weights /path/to/yolo11s.pt --iters 200
python3 scripts/task2/benchmark_yolo11s.py --weights /path/to/yolo11s.engine --iters 200
```

`sudo tegrastats --interval 1000` を別ターミナルで回して GPU 使用率・電力も記録する。
