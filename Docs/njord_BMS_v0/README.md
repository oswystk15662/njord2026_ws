# 4S Li-Po Cell Voltage Telemetry (XIAO ESP32C3)

4台のセル側 XIAO ESP32C3 が、それぞれローカルGND基準でセル電圧を測定し、ESP-NOWで1台のマスターへ送信します。マスターは受信値をUSBシリアルへCSV形式で出力します。

> [!WARNING]
> 直列セルの各段はGND電位が異なります。セル側XIAO同士、またはセル側XIAOとPCをUSB/GNDで同時に接続しないでください。書き込み・デバッグ時は対象基板をバッテリーから切り離すか、適切な絶縁を使用してください。セル電圧は必ず抵抗分圧し、XIAOのADC許容電圧を超えないようにしてください。

## フォルダー

- `cell_node_1` ～ `cell_node_4`: 各セル用（違いは `CELL_ID` のみ）
- `master`: ESP-NOW受信・USBシリアル出力用
- `common`: 共通の通信パケット定義

## 初期設定

各 `cell_node_N/cell_node_N.ino` の先頭にある次の値を実際の基板に合わせます。

- `ADC_PIN`: 分圧回路につながるGPIO。初期値は `D0`（XIAOのA0/D0端子）
- `R_TOP_OHM`: セル＋側からADCまでの抵抗
- `R_BOTTOM_OHM`: ADCからローカルGNDまでの抵抗
- `CALIBRATION_GAIN`: テスターとの比率補正
- `CALIBRATION_OFFSET_V`: オフセット補正
- `ESPNOW_CHANNEL`: 全5台で同じWi-Fiチャネル

初期値は上側/下側とも100 kΩ（1/2分圧）です。実基板の抵抗値が異なる場合は必ず変更してください。

## シリアル出力

マスターを115200 bpsで開くと、1秒ごとに次のCSVを出力します。

```text
ms,cell1_V,cell2_V,cell3_V,cell4_V,total_V,temperature_C,age1_ms,age2_ms,age3_ms,age4_ms,status
1234,4.1010,4.0870,4.0930,4.0990,16.3800,25.0000,41,38,44,40,OK
```

2秒以上受信していないセルは `nan`、`status` は `STALE` になります。ADCが上限付近まで飽和した場合は、そのセルを `nan` として `ADC_FAULT` を出力します。`total_V` は有効な4セルが揃った場合のみ出力します。

`temperature_C` は4セル基板から受け取った温度の平均です。温度センサー選定前は、各セル基板が `CELL_TEMPERATURE_C` の定数（既定 `25.0` °C）を送ります。実測センサーを追加する際は、`cell_node_impl.h` のこの定数値をセンサー値へ置き換えます。

## ビルドと書き込み

Arduino IDE/CLIに Espressif の `esp32` ボードパッケージを導入し、ボードを **XIAO_ESP32C3** にします。各フォルダーを個別のスケッチとしてビルド・書き込みしてください。

CLI例（ポート名は環境に合わせて変更）:

```powershell
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 cell_node_1
arduino-cli upload -p COM3 --fqbn esp32:esp32:XIAO_ESP32C3 cell_node_1
```

セル側のUSBシリアルには、起動時のMACアドレス、測定値、送信成否が表示されます。量産前に各基板を安定化電源とテスターで校正してください。

`CALIBRATION_GAIN` の目安は `テスター実測値 ÷ シリアル表示値` です。2点以上で確認し、一定量の差が残る場合だけ `CALIBRATION_OFFSET_V` も調整してください。

ESP-NOWパケットはブロードキャストかつ暗号化なしです。このファームウェアは監視専用とし、充放電の遮断など安全機能は独立した保護回路で実施してください。
