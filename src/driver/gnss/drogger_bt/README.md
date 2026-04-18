# drogger_bt

Bluetooth SPP (rfcomm) で Drogger GNSS から NMEA を受信し、`sensor_msgs/msg/NavSatFix` を publish する ROS 2 ノードです。

## Current Status

- ノード起動、rfcomm 接続、再試行ロジックは動作確認済み。
- 受信データがある場合のデコード経路（NMEA 正規化、checksum 検証、GGA 解析）は実装済み。
- 2026-04-17 時点の実機テストでは `/dev/rfcomm0` から受信バイトが来ず、`EOF` が継続するケースを確認。

## Topics

- publish: `gnss/fix` (`sensor_msgs/msg/NavSatFix`)

`config/config.yaml` で `fix_topic` を指定すれば publish 先を変更できます。

## Launch

1. Bluetooth 接続作成（特権）

```bash
source install/setup.bash
ros2 launch drogger_bt connect.launch.py
```

2. ノード起動（通常権限）

```bash
source install/setup.bash
ros2 launch drogger_bt launch.py
```

## Parameters

`config/config.yaml` 内の主要パラメータ:

- `bt_mac_address`: Drogger の Bluetooth MAC
- `rfcomm_id`: 生成する rfcomm デバイス番号（`/dev/rfcommX`）
- `rfcomm_channel`: SPP channel
- `port_name`: 読み込み対象デバイス（通常 `/dev/rfcomm0`）
- `baudrate`: serial 設定値（SPP では実質的に意味が薄い）
- `frame_id`: NavSatFix frame id
- `fix_topic`: publish topic
- `log_raw_nmea`: 受信 NMEA 行ログを出すか

## What Was Improved

- launch を役割分離
  - `connect.launch.py`: 接続専用
  - `launch.py`: node 起動専用
- `connect.sh` の bind 判定強化
  - `Address already in use` の誤成功を防止
  - 再試行と既存 bind 確認を追加
- node の受信処理強化
  - `async_read_some` によるチャンク受信
  - CR/LF 両対応の行切り出し
  - EOF 直前バッファの flush 解析
  - NMEA 正規化（先頭ノイズ除去）
  - checksum 検証の厳密化（2桁 hex）
  - publish ログ追加

## Debug Commands

### topic 確認

```bash
source install/setup.bash
ros2 topic echo /gnss/fix
```

### デバイス生データ確認

```bash
timeout 8s cat /dev/rfcomm0 | hexdump -C | head -n 40
```

### ノードログ観察

`log_raw_nmea: true` のとき、受信時に `RX NMEA: ...` が出ます。

## Troubleshooting

### 1) `Permission denied`

- `connect.launch.py` を先に実行
- `osw` ユーザーを `dialout` グループへ追加し、再ログイン

```bash
sudo usermod -a -G dialout osw
```

### 2) `Address already in use`

- 既存 rfcomm の残骸がある状態
- 現在の `connect.sh` は release + retry を行う

### 3) `Read EOF detected` が繰り返され、topic が出ない

多くの場合、入力データ自体が来ていません。まず GNSS 側設定を確認してください。

- Drogger アプリで NMEA 出力を有効化
- 可能ならまず `GGA` のみ有効化（切り分けが容易）
- Bluetooth SPP channel が `rfcomm_channel` と一致しているか確認
- 屋外で衛星捕捉できる環境で再試験

## Recommended Next Test Procedure

1. Drogger アプリで NMEA 出力を有効化（後で GGA only にする予定なら最初から GGA only 推奨）
2. `ros2 launch drogger_bt connect.launch.py`
3. `ros2 launch drogger_bt launch.py`
4. `ros2 topic echo /gnss/fix`
5. 受信が始まったら `log_raw_nmea` を `false` に戻して本運用
