# Task 1 waypoint coordinate modes

`task1_waypoints.yaml` の `task1_config.coordinate_mode` で、Mission Manager が
Nav2へ送る `map` 座標の作り方を選ぶ。GPS値は `/fromLL`
（`navsat_transform_node`）で実行時の `map` 座標へ変換されるため、UM982をどこで
起動しても、現在のmap datumとのずれを吸収する。

## 基準GPSからのローカルXY

GPS1などの基準点を `origin` に書き、各WPは基準点から東を `x` 正、北を `y` 正とする
メートル値で書く。Task 1の既定モードである。

```yaml
coordinate_mode: origin_relative_xy
origin: {latitude: 34.5456666667, longitude: 135.5070555556, altitude: 0.0}
waypoints:
  - {id: 1, competition_id: "1", x: 0.0, y: 0.0, yaw: 0.0}
  - {id: 2, competition_id: "1.1", x: 12.0, y: -3.0, yaw: 0.0}
```

## 全WPをGPSで書く

`x` と `y` を省き、全WPに `latitude` と `longitude` を書く。`yaw` はradのまま必要。

```yaml
coordinate_mode: geodetic
waypoints:
  - {id: 1, competition_id: "1", latitude: 34.5456666667, longitude: 135.5070555556, yaw: 0.0}
  - {id: 2, competition_id: "1.1", latitude: 34.5457000000, longitude: 135.5071000000, yaw: 0.0}
```

`njord-task start task1 --auto` は、GPSモードでは `/fromLL` が利用可能になるまで
開始を拒否する。GPS3/GPS4の `competition_id` は残すこと。これによりGPS3通過判定と
GPS3→GPS4方位による仮想壁が同じ投影済み座標を使う。
