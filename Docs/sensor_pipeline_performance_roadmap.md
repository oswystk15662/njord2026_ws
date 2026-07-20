# センサパイプライン性能改善ロードマップ

ZED 2iとLivox MID-360SをJetson上で同時使用する際の、CPU/GPU負荷、メモリコピー、
ROS 2通信量を削減するための優先項目をまとめる。

## 現状

- MID-360Sは`/livox/lidar`を約10 Hzで配信できている。
- ZED 2iは15 Hz設定に対し、フル負荷時の実測が画像約1.6 Hz、depth約2.6 Hz、
  点群約1.5 Hzまで低下した。
- Python YOLOとGLIMはC++ component container外で動作するため、これらとの通信は
  DDS経由のままである。

## 優先項目

### 1. ZEDのlazy publishing

状態: 実装完了・実機評価待ち

購読者がいないright image、depth、PointCloud2は、publishだけでなくZED SDKの
`retrieveImage`、`retrieveMeasure`とROSメッセージ変換自体を実行しない。
`get_subscription_count()`と個別のpublish有効化パラメータを併用する。

SDK/CPUモードとも、Image、CameraInfo、depth、PointCloud2のsubscriber数を周期ごとに
確認する。必要な出力がない場合はframe取得を省略し、depth/points未購読時はdepth取得・
ステレオ計算・点群変換を行わない。SDKモードでは左右画像も個別にretrieveする。

完了条件:

- left imageだけを購読した場合、right/depth/pointsの取得・変換時間が発生しない。
- 購読開始・終了に追従して必要な出力だけが生成される。

### 2. 設定値をZED SDKモードへ反映

状態: 未実装

現在のSDKモードはHD720固定で、点群生成もstride=1固定である。YAMLの
`image_width`、`image_height`、`pointcloud_stride`をSDK初期化と点群生成へ反映する。
必要に応じてdepth modeも実行時パラメータ化する。

完了条件:

- `pointcloud_stride:=2`で生成点数が概ね1/4になる。
- 選択した解像度、FPS、depth modeが起動ログと実出力へ反映される。

### 3. YOLOを最新フレーム優先にする

状態: 実装完了・実機評価待ち

画像QoSを`KeepLast(1)`にし、推論中に届いた古いフレームを蓄積しない。
画像callbackと推論処理を分離し、常に最新画像だけを推論する。debug imageは無効化
パラメータまたは購読者数に応じて生成を省略する。

購読QoSをBest Effort、`KeepLast(1)`とし、ROS callbackはsingle-slot bufferの最新画像を
置換するだけにした。専用workerが推論中でもcallbackは次の最新画像を受け取れる。
debug image未購読時は矩形・文字・仮想壁線の描画と画像publishを省略する。

完了条件:

- 推論速度より入力FPSが高い場合も遅延が累積しない。
- debug image未購読時に描画・変換処理が発生しない。

### 4. ローカルコピーの削減

状態: 部分完了

実装済み:

- ZED SDKバッファを非所有`cv::Mat` viewとして扱い、`clone()`を廃止した。
- ImageとPointCloud2を`UniquePtr`で確保してmove publishする。
- ZED点群は一時点群vectorを介さず、最終`PointCloud2::data`へ直接構築する。
- Livox点群も一時vectorと最終一括memcpyを廃止し、最終バッファへ直接構築する。

残作業:

- 実機でメモリ帯域、CPU使用率、publish周期を変更前後比較する。
- 必要ならメッセージ用バッファの再利用・pool化を検討する。

### 5. component化とintra-process通信

状態: 部分完了

実装済み:

- ZED SDK/CPUノードをROS 2 componentとして登録した。
- PCLブイ検出をcomponentとして登録した。
- Livox driverとPCLブイ検出を同じ`component_container_mt`へロードできる。
- 対象componentで`use_intra_process_comms=true`を使用する。

起動例:

```bash
ros2 launch robot lidar.launch.py \
  lidar_model:=mid360s enable_buoy_detection:=true
```

残る境界:

- Python YOLOは同じC++ component containerへロードできない。
- GLIMが独立processで動作する構成では、LivoxからGLIMまではDDS経由になる。
- ZED componentと同一containerで動作するC++画像購読componentがない間は、
  ZED画像のintra-process効果は得られない。

### 6. ZEDから推論までGPU上で接続

状態: 未実装・長期項目

Python、ROS Image、cv_bridgeを経由せず、ZED SDKのGPUバッファをTensorRTや
NVIDIA Isaac ROS NITROS等へ渡す構成を検討する。実装規模は大きいが、画像の
CPU readback、色変換、DDS転送をまとめて削減できる可能性がある。

完了条件:

- カメラ取得から推論入力まで不要なCPUコピーがないことを計測で確認する。
- 現行YOLOモデルと同等の検出精度を維持する。
- フルスタック時のGPU使用率、RAM、入力FPS、推論遅延を記録する。

## 評価指標

各項目の変更前後で、最低限以下を同じnvpmodel・同じ入力条件で記録する。

- `ros2 topic hz`によるleft image、depth、ZED points、Livox pointsの実効周期
- カメラtimestampから検出結果publishまでのend-to-end遅延
- `tegrastats`のCPU、GPU、RAM、EMC使用率
- component内／外subscriber数と使用している通信経路
- 10分以上の連続運転でのdrop、segfault、メモリ増加
