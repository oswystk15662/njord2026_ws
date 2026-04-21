このpkgはnjord2026のtask1のシミュレーションをするpkgです。
task1の詳細はこのpkg内のDocs内を見てください。

このpkgは、
* task1_simのための以下のlaunchを含みます
  * dutyed_tf_pub_with_disturbance
  * nav2
  * configに緯度経度で指定されたGPSpoint4つとwaypoint1.1~1.10・3.1~3.3をnav2に渡す
  * 下記自分のnode
* yoloの代替として、yoloの推論開始serviceが来たら、即座にランダムなEWNSいずれかを返すnodeを実装。
* start（初期位置を与える必要があれば初期位置も）とgoal（こちらはGPSpoint4の半径~mに入ったら）を出せるようにする。
などtask1_simのorchestratorを担当します。