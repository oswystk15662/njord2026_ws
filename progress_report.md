TODO
* [ ] 1~5 knot checker : memo
* [ ] GUI
* [ ] 

センサやtopic通信が途絶えていないかをチェックするためのdiagnosticを発行したいです。
/home/osw/njord2026_ws/src/diagnostic
の下に、
* USB to CANの接続状況（socket CANで通信し、Heatbeatを出します）。具体的な仕様を決めてないので、一旦pkgだけ作ってください
* LiDARの接続状況mid360をつなぎます。EtherがUPかどうか、もしくはpointcloudが出続けているかどうかなどから判断します。
* miniPC 2 PC間通信（GUIの更新が止まるので一瞬でわかるとは思いますが、一応）
* IMUの接続、dev/ttyUSB~でつながるのを見てもいいし、topicを見てもいいです。
*  CAMEraの接続、dev/camera0?でつながるのを見てもいいし、topicを見てもいいです。 
* GNSSの接続、bluetoothか有線のどちらかです。それぞれ別で作ってください。