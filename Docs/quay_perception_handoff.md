# 岸壁認識 引き継ぎ資料 (feature/quay-perception)

- 作成日: 2026-07-20
- 目的: 岸壁認識をTask 2から切り離すにあたり、実装済みの岸壁認識一式を本ブランチに保存し、別Taskへ引き継ぐ。
- 状態: Mac静的確認・synthetic test通過のみ。**Jetson・実機では未検証。**

## 1. 岸壁認識の目的

LiDAR点群から岸壁(鉛直な連続面)を線分として検出し、(a) Nav2 costmapへの障害物マーキング、(b) MPPIへの実験的距離ペナルティ、の2経路で安全制約に利用できるようにする。

## 2. 保存ブランチ名

`feature/quay-perception`(`task2-experiment`の岸壁削除前tip `feff261` から作成。岸壁関連の全実装+本資料を含む)

## 3. 主要ファイル

| パス | 内容 |
|---|---|
| `src/detection/task2_perception/task2_perception/quay_wall_detector_node.py` | 岸壁検出ROSノード |
| `src/detection/task2_perception/task2_perception/wall_fit.py` | 純numpy計算層(候補抽出・RANSAC line・占有格子化) |
| `src/detection/task2_perception/test/test_wall_fit.py` | synthetic テスト |
| `src/detection/task2_perception/config/task2_perception_params.yaml` | `quay_wall_detector:` ブロック |
| `src/navigation/path_generator/mppi/asv_trajectory_planner/quay_cost.py` | MPPI実験的岸壁コスト(torch) |
| `src/robot/config/nav2_params_task2.yaml` | costmap `quay` observation source |
| `src/robot/launch/task2_real.launch.py` | `enable_quay_detection` 引数と起動配線 |
| `scripts/task2/record_task2_bag.sh` | `/quay_wall/*` 記録トピック |

## 4. ノード名

- ノード名/実行子: `quay_wall_detector_node`(パッケージ `task2_perception`)

## 5. Subscribe topic

- `/pcl/nonground`(`sensor_msgs/PointCloud2`, base_link)— pcl_segmentationサブモジュール `ground_remover_node` の出力(上流で水面・地面除去済み)

## 6. Publish topic

- `/quay_wall/markers`(`visualization_msgs/MarkerArray`, LINE_LIST, base_link)— 確定壁線分
- `/quay_wall/points`(`sensor_msgs/PointCloud2`, base_link)— 確定壁インライア点(Nav2 obstacle layer向け)
- `/quay_wall/costmap`(`nav_msgs/OccupancyGrid`, map)— `quay_safety_margin_m` で膨張した占有格子

## 7. 使用TF

- `map → base_link`(costmap生成時の座標変換に使用。実機ではGLIMが所有)
- 入力点群は base_link 前提(上流 `task2_cloud_filter` がTFで `livox_frame → base_link` 変換済み)

## 8. 水面除去との関係

- 岸壁検出は**水面除去より下流**(`/pcl/nonground`)で動く。水面除去(`cloud_ops.water_removal`)は法線ガード `normal_z ≥ water_plane_normal_z_min` と高さガード `|平面高さ − waterline_z_m| ≤ water_plane_max_height_error_m` を持ち、**鉛直な岸壁面を構造的に除去できない**設計。この保証はTask 2側に残るテスト(`鉛直面が水面として消えない`)で回帰防止されている。
- したがって別Taskへ移設しても、同じ前処理チェーンの後段に接続すれば追加の保護は不要。

## 9. 岸壁認識アルゴリズム

1. xyセル毎の鉛直方向広がり(z extent)で壁候補点を抽出(`extract_wall_candidates`)
2. xy平面へ投影し2D RANSACで線分フィッティング(`fit_lines_ansac`系, 逐次最大インライア線分を `max_lines` 本まで)
3. `wall_min_points` / `wall_min_length_m` / `wall_max_distance_m` でゲート
4. 時系列確認: 距離+角度の許容内で `wall_temporal_confirmations` 回再出現した線分のみ確定、`wall_timeout_sec` 未観測で破棄
5. 確定線分をmarkers/points/costmapとしてpublish(costmapは線分を `quay_safety_margin_m` で膨張して格子化)

## 10. パラメータ一覧(`quay_wall_detector.ros__parameters`)

| 名前 | 型 | 単位 | 既定値 | 説明 |
|---|---|---|---|---|
| input_topic | str | - | /pcl/nonground | 入力点群 |
| map_frame / base_frame | str | - | map / base_link | フレーム |
| wall_min_points | int | pts | 30 | 線分の最小インライア数(**要実測**: 点密度依存) |
| wall_min_length_m | float | m | 2.0 | 最小線分長 |
| wall_max_distance_m | float | m | 40.0 | 最大検出距離 |
| wall_line_distance_threshold_m | float | m | 0.15 | RANSACインライア距離 |
| wall_normal_z_max | float | - | 0.3 | 鉛直面判定(法線zの上限) |
| wall_temporal_confirmations | int | frames | 3 | 確定に必要な再出現回数 |
| wall_timeout_sec | float | s | 2.0 | 未観測破棄時間 |
| quay_safety_margin_m | float | m | 5.0 | costmap膨張マージン |
| publish_wall_markers / publish_wall_costmap | bool | - | true | 出力有効化 |
| costmap_resolution_m | float | m | 0.5 | 格子解像度 |
| costmap_size_m | float | m | 100.0 | 格子一辺(自船中心) |

MPPI側(`asv_trajectory_planner`): `enable_mppi_quay_cost`(既定 false)、`mppi.quay_cost_weight`(50.0)、`mppi.quay_safety_margin_m`(3.0)、`quay_markers_topic`。

## 11. テスト一覧(`test_wall_fit.py`ほか)

- 既知線分のRANSAC復元(端点・長さ・原点距離の許容内一致)
- 短すぎる/遠すぎる線分の棄却
- 占有格子化(線分+マージンのラスタライズ)
- (`test_cloud_ops.py`側)鉛直壁が水面除去で生存すること — **この1件はTask 2にも残置**(水面除去の回帰防止として)

## 12. 別Taskへ統合する際の手順

1. 本ブランチの §3 のファイル群を対象Taskブランチへ取り込む(`git cherry-pick` または `git checkout feature/quay-perception -- <paths>`)
2. `task2_perception/setup.py` に entry point `quay_wall_detector_node` を復元
3. 対象Taskのlaunchで `quay_wall_detector_node` を起動(入力は水面除去済みの `/pcl/nonground` 相当に接続)
4. Nav2に接続する場合: costmap obstacle layerへ `quay` source(topic `/quay_wall/points`, PointCloud2, marking true / clearing false)を追加
5. MPPIに接続する場合: `quay_cost.py` を復元し `enable_mppi_quay_cost` を有効化
6. **必読**: §15の「Nav2経路は現行RPP設定では制御に効かない」問題を対象Taskでも必ず確認すること

## 13. Jetsonで未確認の項目

- colcon build・実行時動作(全ノード)
- 実Livox点群での壁候補抽出の点密度・CPU負荷・処理周期
- costmap生成のTF可用性(GLIM起動順との関係)
- `wall_min_points` / RANSACしきい値の実点群での妥当性

## 14. 実機で調整する項目

- `wall_min_points`(点密度依存・最重要)、`wall_line_distance_threshold_m`、`wall_normal_z_max`
- `quay_safety_margin_m`(安全距離: 運用判断)
- `wall_temporal_confirmations` / `wall_timeout_sec`(波・揺動での安定性)
- 接続先の安全経路の選択(下記 §15)

## 15. 現在Task 2へ接続されている箇所(分離時に削除される配線)

| 接続箇所 | 内容 |
|---|---|
| `task2_real.launch.py` | `enable_quay_detection` 引数 → `task2_perception.launch.py` へ転送 |
| `task2_perception.launch.py` | `quay_wall_detector_node` 起動(`enable_quay_detection` ゲート) |
| `nav2_params_task2.yaml` | local/global costmap `observation_sources: pointcloud quay` + `quay` ブロック |
| `planner_node.py` | `enable_mppi_quay_cost` パラメータ、`/quay_wall/markers` 購読、`quay_cost.py` 呼び出し(既定オフ) |
| `mppi_params.yaml` | quay関連3パラメータ |
| `record_task2_bag.sh` | `/quay_wall/{markers,points,costmap}` |
| 文書 | `Docs/task2_*` 5件の岸壁記述 |

**重要な既知問題(引き継ぎ必須)**: 現行のTask 2 Nav2設定ではFollowPath(RegulatedPurePursuit)が `use_collision_detection: false` のため、costmapへのquayマーキングは**制御に伝搬しない**。岸壁回避を実効化するには、(a) RPP衝突検出の有効化(水上挙動未検証)、または (b) `enable_mppi_quay_cost: true`、のどちらかを人間が選択して有効化する必要がある(`Docs/task2_full_integration_report.md` §20-7/§20b 参照)。
