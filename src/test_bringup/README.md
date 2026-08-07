# test_bringup

Njordの実機試験とオフライン再生にだけ使用するbringupを、運用用の
`robot`パッケージから分離したパッケージです。通常の実機bringupから
これらのlaunchをincludeしないでください。

## Launch files

| Launch | Purpose |
|---|---|
| `nav2_mintest.launch.py` | Nav2を最小構成で起動する。`ROS_DISTRO`に応じてHumble/Jazzy設定を選ぶ |
| `soft_emg_alert_lamp_test.launch.py` | joy、soft-EMG、Micon、alert lampの実機試験 |
| `um982_ekf_bag_playback.launch.py` | UM982とglobal EKFをrosbagの時刻で再生検証する |

Examples:

```bash
ros2 launch test_bringup nav2_mintest.launch.py
ros2 launch test_bringup soft_emg_alert_lamp_test.launch.py serial_port:=/dev/ttyUSB0
ros2 launch test_bringup um982_ekf_bag_playback.launch.py
```

soft-EMG試験では推進電源を切るか、船体を確実に拘束してください。

パッケージ単体の確認:

```bash
colcon build --packages-up-to test_bringup
colcon test --packages-select test_bringup
colcon test-result --verbose
```
