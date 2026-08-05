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
