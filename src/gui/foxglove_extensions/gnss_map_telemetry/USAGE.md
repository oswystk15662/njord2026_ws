# GNSS Map Telemetry の使い方

この拡張は、地図上に船の位置・`base_link` +X方向を示す矢印を表示し、
右上に現在の緯度・経度・heading・対地速度（SOG）を重ねて表示します。
競技中に必要な位置と速度を、Mapパネルから視線を外さず確認するためのパネルです。

## 1. 事前準備

実機の起動構成で、`foxglove_logger` が次のトピックをpublishしていることを確認します。

| 表示項目 | トピック | 型 |
| --- | --- | --- |
| 緯度・経度・対地速度・船体方位 | `/foxglove_log` | `rcl_interfaces/msg/Log` |

`foxglove_logger` はGNSS、速度、方位を収集して、`NAV LAT/LON`、`SOG`、`HDG`として
`/foxglove_log` に集約します。

```bash
ros2 topic echo /foxglove_log --once
```

## 2. Foxglove拡張を導入する

Foxglove Desktopを開き、次のファイルを画面へドラッグ＆ドロップします。

```text
src/gui/foxglove_extensions/gnss_map_telemetry/gnss-map-telemetry-0.2.3.foxe
```

Foxgloveの Settings > Extensions に **GNSS Map Telemetry** が表示され、有効になっていることを確認してください。

## 3. レイアウトを読み込む

Foxgloveで Layout メニューから Import を選び、ワークスペース直下の
`foxglove_setting.json` を読み込みます。

読み込み後、左側下段の **GNSS Course (Map)** は次を表示します。

- OpenStreetMap地図と、`/foxglove_log` の `HDG` が示す現在位置の矢印
- 右上の `VESSEL TELEMETRY` 凡例
  - `LAT`: 緯度
  - `LON`: 経度
  - `HDG`: `base_link` +Xの地理方位（北0度、時計回り）
  - `SOG`: 対地速度（m/s）

最初の有効なGNSS位置を受信したときだけ地図を中央合わせします。その後は船が移動しても
地図中心とzoomを変更しません。マウスホイールで拡大縮小、ドラッグで自由に移動できます。

## トラブルシューティング

- `--` と表示される: `/foxglove_log` が未publish、または `NAV LAT/LON`、`SOG`、`HDG` を含むログが未出力です。
- 位置が円で表示され、`HDG --` になる: `/foxglove_log` の `HDG` 値を確認してください。
- 地図が表示されない: OpenStreetMapのタイルを使うため、表示PCからインターネットへ接続できるか確認してください。
- パネルが見つからない: `.foxe` を先に導入してからレイアウトをimportし、Foxgloveを再起動してください。
