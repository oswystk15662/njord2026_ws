# GNSS Map Telemetry の使い方

この拡張は、地図上に船の位置・`base_link` +X方向を示す矢印を表示し、
右上に現在の緯度・経度・heading・対地速度（SOG）を重ねて表示します。
競技中に必要な位置と速度を、Mapパネルから視線を外さず確認するためのパネルです。

## 1. 事前準備

実機の起動構成で、次のトピックがpublishされていることを確認します。

| 表示項目 | トピック | 型 |
| --- | --- | --- |
| 緯度・経度 | `/sensor/vehicle_gnss/fix/raw` | `sensor_msgs/msg/NavSatFix` |
| 対地速度 | `/gui/ground_speed_mps` | `std_msgs/msg/Float32` |
| 船体の向き | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` |

`/gui/ground_speed_mps` は、このワークスペースの
`ground_speed_publisher` が `/odometry/feedback` の東西・南北速度から算出します。

```bash
ros2 topic echo /sensor/vehicle_gnss/fix/raw --once
ros2 topic echo /gui/ground_speed_mps --once
ros2 run tf2_ros tf2_echo map base_link
```

陸上PCでは通常のbringupにWP表示が既定で含まれます。
Mission ManagerでAUTOタスクを開始すると、Zenoh経由の`/mission/status.task_id`に
従って該当taskのWPだけを表示します。これは表示専用であり、Nav2や推進を操作しません。

```bash
ros2 launch robot ground_pc.launch.py
```

## 2. Foxglove拡張を導入する

Foxglove Desktopを開き、次のファイルを画面へドラッグ＆ドロップします。

```text
src/gui/foxglove_extensions/gnss_map_telemetry/gnss-map-telemetry-0.3.2.foxe
```

Foxgloveの Settings > Extensions に **GNSS Map Telemetry** が表示され、有効になっていることを確認してください。

## 3. レイアウトを読み込む

Foxgloveで Layout メニューから Import を選び、ワークスペース直下の
`foxglove_setting.json` を読み込みます。

読み込み後、左側下段の **GNSS Course (Map)** は次を表示します。

- OpenStreetMap地図と、`base_link` +X方向を示す現在位置の船体アイコン（`SELF` ラベル付き）
- 右上の `VESSEL TELEMETRY` 凡例
  - `LAT`: 緯度
  - `LON`: 経度
  - `HDG`: `base_link` +Xの地理方位（北0度、時計回り）
  - `SOG`: 対地速度（m/s）

最初の有効なGNSS位置を受信したときに地図を中央合わせします。WP確認用launchでGNSSを
起動していない場合は、`/ground_waypoint_markers` のWP群に中央合わせします。その後は船が
移動しても地図中心とzoomを変更しません。マウスホイールで拡大縮小、ドラッグで自由に移動できます。

## トラブルシューティング

- `--` と表示される: 対応トピックが未publish、またはトピック名が異なります。
- 位置が円で表示され、`HDG --` になる: `map` から `base_link` までのTF chainを確認してください。
- 地図が表示されない: OpenStreetMapのタイルを使うため、表示PCからインターネットへ接続できるか確認してください。
- パネルが見つからない: `.foxe` を先に導入してからレイアウトをimportし、Foxgloveを再起動してください。
