# GNSS Map Telemetry test publisher

`GNSS Map Telemetry` パネルを確認するため、移動するGNSS位置、対地速度、
`map -> base_link` TFを配信するROS 2パッケージです。
既定では東京湾付近（35.609596, 139.683751）から東（方位90度）へ2 m/sで移動します。

```bash
colcon build --packages-select gnss_map_telemetry_test_publisher
source install/setup.bash
ros2 launch gnss_map_telemetry_test_publisher gnss_map_telemetry_test.launch.py
```

パネルが購読する既定トピックを配信します。

- `/sensor/vehicle_gnss/fix/raw` (`sensor_msgs/msg/NavSatFix`)
- `/gui/ground_speed_mps` (`std_msgs/msg/Float32`)
- `/tf` (`map -> base_link`)

各値は起動時に上書きできます。`heading_degrees` は北を0度、東を90度とする時計回りの方位で、
移動方向と`base_link` +Xの向きの両方に使用されます。

```bash
ros2 run gnss_map_telemetry_test_publisher gnss_map_telemetry_test_publisher --ros-args \
  -p latitude:=35.680959 -p longitude:=139.767306 \
  -p heading_degrees:=225.0 -p speed_mps:=1.5 -p publish_rate_hz:=5.0
```

実機GNSS、速度配信ノード、`map -> base_link` TF publisherと同時に起動すると競合するため、
このテストノードは単独のテスト構成で起動してください。

## ノルウェー Task ウェイポイントの地図表示

`norway_waypoint_publisher` は、
[`Docs/norway_task_waypoint_proposal.md`](../../../../Docs/norway_task_waypoint_proposal.md)
の検討用ウェイポイントを、実機GNSSとは別の `NavSatFix` トピックとして継続配信します。
Task 1 の観測点、Task 2 の監視点、Task 3 の左右接岸案を同時に地図へ重ねられます。

```bash
colcon build --packages-select gnss_map_telemetry_test_publisher
source install/setup.bash
ros2 launch gnss_map_telemetry_test_publisher norway_waypoints.launch.py
```

既定のトピックは次の通りです（すべて `sensor_msgs/msg/NavSatFix`、1 Hz）。

- `/visualization/norway_waypoints/start`
- `/visualization/norway_waypoints/task1/{entry,marker_observe,decision,exit,goal}`
- `/visualization/norway_waypoints/task2/{risk_check_1,risk_check_2,risk_check_3,goal}`
- `/visualization/norway_waypoints/task3/{gate,left_approach,left_berth,right_approach,right_berth,finish}`

Foxglove の **Map** パネルで上記のトピックを追加すると、各点を緯度・経度の位置として
表示できる。点は transient-local QoS かつ周期再送のため、Foxglove を後から接続しても
表示される。これは検討用の可視化であり、Nav2 の目標や実機のGNSS入力は変更しない。
