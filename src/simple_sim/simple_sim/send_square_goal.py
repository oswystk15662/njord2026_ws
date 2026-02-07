import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()

    # Nav2を操作するAPIクライアント
    navigator = BasicNavigator()

    # 初期化待ち
    # (simple_sim_node と nav2_mintest が起動している必要があります)
    navigator.waitUntilNav2Active()

    # --- 四角い経路のウェイポイントを作成 ---
    # 座標系は odom (GPS走行を想定した設定の場合)
    frame_id = 'odom' 
    
    goal_poses = []
    
    # 4つの角の座標 (x, y)
    coordinates = [
        (5.0, 0.0),
        (5.0, 5.0),
        (0.0, 5.0),
        (0.0, 0.0)
    ]

    for pt in coordinates:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.w = 1.0 # 向きはとりあえず気にしない(1.0)
        goal_poses.append(pose)

    # --- Nav2に指令を送信 (Navigate Through Poses) ---
    print("Nav2に行き先リストを送信します...")
    navigator.goThroughPoses(goal_poses)

    # --- 実行状況の監視 ---
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'残り距離: {feedback.distance_remaining:.2f} m', end='\r')

    # 結果確認
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\nゴールしました！")
    elif result == TaskResult.CANCELED:
        print("\nキャンセルされました")
    elif result == TaskResult.FAILED:
        print("\n失敗しました")

    exit(0)

if __name__ == '__main__':
    main()