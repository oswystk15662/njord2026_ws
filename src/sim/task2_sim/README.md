このpkgはnjord2026のtask2のシミュレーションをするpkgです。
task2の詳細はこのpkg内のDocs内を見てください。

このpkgは、
* task2_simのための以下のlaunchを含みます
  * dutyed_tf_pub_with_disturbance
  * nav2
  * configに緯度経度で指定されたGPSpoint2つ
  * marker vessel the otter of Njordの投影範囲
  * 下記自分のnode
* yoloの代替として、時々刻々と微妙に位置が変わる赤と緑のブイのtfをnav2に渡す。
* start（初期位置を与える必要があれば初期位置も）とgoal（こちらはGPSpoint4の半径~mに入ったら）を出せるようにする。
などtask2_simのorchestratorを担当します。