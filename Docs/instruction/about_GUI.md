手順
1. 陸上PCで```ros2 launch foxglove_bridge foxglove_bridge_launch.xml```, ```ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5600```(zed 2i left cam), ```ros2 launch zed2i_driver ground_video_receiver.launch.py port:=5601```(back cam)
2. 陸上PCでFoxgloveを開き、layoutは```~~~```を読み込みlocalhostに接続

