# GNSS Map Telemetry の使い方

この拡張は、地図の右上に現在の緯度・経度・対地速度（SOG）を重ねて表示します。
競技中に必要な位置と速度を、Mapパネルから視線を外さず確認するためのパネルです。

## 1. 事前準備

実機の起動構成で、次のトピックがpublishされていることを確認します。

| 表示項目 | トピック | 型 |
| --- | --- | --- |
| 緯度・経度 | `/sensor/vehicle_gnss/fix/raw` | `sensor_msgs/msg/NavSatFix` |
| 対地速度 | `/gui/ground_speed_mps` | `std_msgs/msg/Float32` |

`/gui/ground_speed_mps` は、このワークスペースの
`ground_speed_publisher` が `/odometry/feedback` の東西・南北速度から算出します。

```bash
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
ros2 topic echo /gui/ground_speed_mps --once
```

## 2. Foxglove拡張を導入する

Foxglove Desktopを開き、次のファイルを画面へドラッグ＆ドロップします。

```text
tools/foxglove_extensions/gnss_map_telemetry/gnss-map-telemetry-0.1.1.foxe
```

Foxgloveの Settings > Extensions に **GNSS Map Telemetry** が表示され、有効になっていることを確認してください。

## 3. レイアウトを読み込む

Foxgloveで Layout メニューから Import を選び、ワークスペース直下の
`foxglove_setting.json` を読み込みます。

読み込み後、左側下段の **GNSS Course (Map)** は次を表示します。

- OpenStreetMap地図と現在位置マーカー
- 右上の `VESSEL TELEMETRY` 凡例
  - `LAT`: 緯度
  - `LON`: 経度
  - `SOG`: 対地速度（m/s）

地図領域は通常どおりマウスホイールで拡大縮小、ドラッグで移動できます。

## トラブルシューティング

- `--` と表示される: 対応トピックが未publish、またはトピック名が異なります。
- 地図が表示されない: OpenStreetMapのタイルを使うため、表示PCからインターネットへ接続できるか確認してください。
- パネルが見つからない: `.foxe` を先に導入してからレイアウトをimportし、Foxgloveを再起動してください。
