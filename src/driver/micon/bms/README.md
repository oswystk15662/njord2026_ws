# bms

4セルBMS情報をESP32から受信し、上位向けに統一トピックへ流すノードです。

- mROS(USB): `std_msgs/msg/Float32MultiArray` (4要素, 単位V)
- CAN: `can_msgs/msg/Frame` 8byte payload (`uint16[mV] x 4`, little-endian)
- 出力: `std_msgs/msg/Float32MultiArray` (4要素, 単位V)

`transport_mode` で `mros_usb` / `can` / `both` を切り替え可能です。
