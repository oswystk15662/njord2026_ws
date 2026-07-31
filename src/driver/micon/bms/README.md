# bms

4セルBMS情報を上位向けに統一トピックへ流すノードです。

- 入力: `micon/bms_cells` (`std_msgs/msg/Float32MultiArray`, 4要素, V)
- 出力: `bms/cell_voltages` (`std_msgs/msg/Float32MultiArray`, 4要素, V)
- diagnostics: `/diagnostics` に最低セル電圧の OK/WARN/ERROR を publish

USB serial の tty は `micon_driver_fd/serial_writer` だけが open します。この
パッケージは復号済み topic を購読するだけで、serial device には触れません。
