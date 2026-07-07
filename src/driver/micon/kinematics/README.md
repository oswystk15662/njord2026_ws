役割
1. 旧構成の `cmd_vel` から `thruster_command` への変換ノードです。
2. 現在の自律航行・task sim では `thruster_driver` がURDF poseとconfigを使って配分まで担当します。
3. このpackageは互換確認用に残していますが、通常のlaunch経路では起動しません。
