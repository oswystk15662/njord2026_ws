# Task 2 統合最終レポート (task2-experiment)

- 作業日: 2026-07-19
- 作業環境: Mac(ROS 2 環境なし。**全項目静的確認のみ、実行時未検証**)
- 関連文書: `task2_experiment_architecture.md`(構成)、`task2_parameter_reference.md`(パラメータ)、`task2_file_map.md`(ファイル)、`task2_jetson_validation.md`(検証手順)、`task2_mppi_integration_report.md`(前段 MPPI 統合)

---

## 1. 実施内容

test07089 実機ハードウェアスタック上に Task 2(避航)一式を統合した:

1. mppi ブランチの MPPI プランナを取込・パラメータ化(デフォルト挙動不変)
2. YOLO11s 検出ノード(標準 Ultralytics・.pt/.engine 両対応)を既存ノードに並置追加
3. LiDAR 認識パイプライン(前段フィルタ+水面除去 → サブモジュール追跡再利用 → MPPI グルー → 岸壁検出)を新規実装
4. 実機統合 launch `task2_real.launch.py`(dry-run 三重ゲート)を新設
5. Jetson 検証用スクリプト・ドキュメント一式を整備

ハードウェア寄りのスタック(driver / real_bringup / nav2.launch.py / nav2_params.yaml / .gitmodules)は **test07089 とバイト単位で同一**(空 diff を確認済み)。

## 2. ブランチ

| ブランチ | 起点 | tip |
|---|---|---|
| `task2-experiment` | `origin/test07089`(tip `834f433`) | `1647689` |

作業ブランチ `feature/task2-yolo11s`, `feature/task2-lidar-perception`, `feature/task2-real-integration` はすべてローカルで task2-experiment にマージ済み。**何も push していない**。

## 3. commit 一覧(`git log --oneline --first-parent 834f433..HEAD`)

```
1647689 Fix yolo_backend default and trailing whitespace after integration
c25c08f Merge feature/task2-real-integration: real Task 2 launch, MPPI params, Jetson scripts
aa541b3 Merge feature/task2-lidar-perception: cloud filtering, tracking glue, quay walls
bc94b40 Merge feature/task2-yolo11s consolidation: spec alignment for YOLO11s tooling
9e5c617 Merge feature/task2-yolo11s: YOLO11s detector and Jetson export path
bb1bd3f Add Task 2 MPPI integration report
f7fd02a Declare missing package dependencies for Task 2 integration
877fe77 Integrate Task 2 MPPI into test07089 hardware stack
17ed890 Merge origin/mppi into task2-experiment
```

(マージコミット内の作業コミット: `fcceffb`/`c15020e`(YOLO11s)、`fd1135b`/`9aed38a`(spec 整合・MPPI パラメータ化)、`1d6c3c3`(実機 launch)、`136b13d`(URDF roll=π)、`9ff8f39`/`bbcc31b`(task2_perception・Nav2 quay ソース)、`16c39e5`(Jetson スクリプト)。`17ed890` 経由で mppi ブランチ履歴も入る。)

## 4. 変更ファイル

`git diff --name-status 834f433 HEAD`: **74 ファイル(+10576 / −44 行)**。全ファイルの役割・変更理由・実機/シミュ区分は `Docs/task2_file_map.md` 参照。概要:

- 新規パッケージ: `task2_perception`(20 ファイル)、`asv_trajectory_planner`(23 ファイル、mppi 由来+新規 4 ファイル)
- yolo パッケージ: 4 追加 + 2 変更(main.py 無変更)
- robot: launch 2 追加、テスト 1 追加、URDF 2 変更、nav2_params_task2.yaml 変更、CMakeLists 変更
- sim: launch/adapter 等(MPPI 統合時のまま)
- scripts 6 本、Docs 2 本、.gitignore

## 5. サブモジュール変更 = なし

| サブモジュール | ポインタ | 状態 |
|---|---|---|
| `src/detection/pcl_segmentation` | `db84af9` | **変更ゼロ**。ローカルコミットなし。push すべきものなし |
| その他 4 件(wit_node_ros2, ros2-driver, Livox-SDK2, livox_ros_driver2) | test07089 のまま | 変更ゼロ |

サブモジュール再利用は launch 引数(`lidar_topic`, `ego_odom_topic`)の差し替えのみで実現した。

## 6. 再利用した既存コード

| 再利用元 | 内容 |
|---|---|
| test07089 実機スタック | real_bringup / GLIM / EKF / thruster_driver / serial_writer / E-stop / Nav2 基盤(全て無変更) |
| pcl_segmentation サブモジュール | classical_pipeline(preprocessing → ground removal → clustering → EKF+Hungarian 追跡)を無改造で再利用 |
| mppi ブランチ | MPPI コア(mppi_torch / crm_torch / trajectory_generator)、planner/pruner/follow_path_client、シミュ構成 |
| 既存 yolo main.py | BuoyRoi の意味論(画素中心→方位・固定レンジ)を踏襲(ファイル自体は無変更) |
| camera-yolo-lidar-fusion ブランチ | 推論レート制限・ROI クロップ・HSV 色推定のロジックを yolo11_node に移植 |
| opponent_twist_from_tf_node | ローパス+スパイク棄却の平滑化スタイルを `smoothing.TwistSmoother` として移植 |

## 7. 新規実装

| 実装 | ファイル | 要点 |
|---|---|---|
| task2_cloud_filter | `cloud_filter_node.py` + `cloud_ops.py` | TF 変換・レンジ・自船クロップ・高さバンド・水面除去。純 numpy 部はテスト済み |
| opponent_selector | `opponent_selector_node.py` + `tracking_glue.py` + `smoothing.py` | ゲーティング(confirmed/距離/寸法/点数/鮮度)、policy(nearest / min_tcpa / track_id)、body→base 回転、ego 補償、map 変換、平滑化 |
| quay_wall_detector | `quay_wall_detector_node.py` + `wall_fit.py` | 鉛直候補 → 2D RANSAC 直線 → 法線ゲート → 時間確認 → points/markers/costmap |
| yolo11_node | `yolo/yolo11_node.py` | 標準 Ultralytics のみ(vendored fork なし)。重み欠如は fatal + クリーン終了 |
| MPPI パラメータ化 | `planner_node.py`, `mppi_params.yaml` | `mppi.*` 23 個+実験フック。デフォルト=旧ハードコード値 |
| quay cost(実験的) | `quay_cost.py` | 線分距離二乗ペナルティ。`enable_mppi_quay_cost`(既定 false)時のみ |
| task2_waypoint_pose_publisher | 同名 .py | 共有 task2_waypoints.yaml の start/goal を `/waypoint1_pose` `/waypoint2_pose` 2 Hz 配信 |
| 実機 launch | `task2_real.launch.py`, `planner_real.launch.py` | 三重ゲート dry-run、シミュノード排除 |
| 検証基盤 | `scripts/task2/*`, `scripts/yolo/*`, `test_task2_real_launch_static.py`, pytest 29 件 | ROS 不要で回る静的・ベンチ・記録ツール |

## 8. YOLO 軽量化

- YOLO11s(小型モデル)+ 推論レート制限(`inference_hz` 5 Hz、超過フレームは**破棄**しキュー無し)+ デバッグ描画のレート制限/省略で計算負荷を制御。
- ROI クロップ(`use_roi_crop`)で空・自船体を推論対象から除外可能。
- TensorRT パス: `scripts/yolo/export_yolo11s_tensorrt.py`(**Jetson 上専用**、FP16 既定)で .engine 化し、`use_tensorrt:=true` または `.engine` パス指定で切替。INT8 はキャリブレーションデータ必須(なければ拒否)。
- 環境戦略(`Docs/yolo11s_jetson_setup.md`): システム ROS + `--system-site-packages` の専用 venv、NVIDIA torch wheel を上書きしない、numpy<2、`requirements_jetson_yolo11.txt`(ultralytics==8.3.237)。
- **重みはリポジトリ非同梱**。既定の launch 引数 `yolo_model_path` は旧 `best.pt` を指すため、YOLO11s 重みで上書きが必要。

## 9. LiDAR 反転補正

- Mid-360 は上下逆さま搭載。補正は **URDF `lidar_joint` の roll=π ただ 1 箇所**(`robot.urdf.xacro`。展開済み `robot.urdf_modified.urdf` にも反映)。
- 全ノードは TF `livox_frame→base_link` を引くだけで、点群を独自に再回転しない。
- cloud_filter の `lidar_inverted` / `lidar_*_deg` は URDF 無し環境用の緊急手段(既定 no-op、有効時は起動警告)。
- **取付角・位置(z=0.8 m 等)はすべて公称値。実測必須**(GLIM・認識の双方に影響)。

## 10. 水面除去

- z バンド(waterline ± [−0.3, +0.15] m)+ ガード付き RANSAC 平面の 2 段。
- RANSAC は「法線ほぼ鉛直(normal_z ≥ 0.9)」かつ「高さが waterline ±0.3 m」の平面のみ除去 → 岸壁・デッキを誤って食わない設計。合成テストで壁/デッキ生存を確認済み。
- `waterline_z_m`(既定 0.0)は喫水依存で**実測必須**。

## 11. 他船追跡

- 追跡本体はサブモジュール(EKF + Hungarian)を無改造で再利用し、入力を `/task2/points_filtered`、ego odom を `/odometry/filtered/local` に差し替え。
- サブモジュール tracker の twist は**物体 body frame の相対速度**(`ship_tracker_node.cpp` の twist 組み立て。tracker 内部の ego 補償は no-op スタブ)。この解釈に基づき、グルー(`opponent_selector`)側で body→base 回転 + ego 速度加算 + map 変換を行う。**tracker 本体は触っていない**。
- 出力 `/other_ship/twist` は map 系の絶対対地速度(planner は `other_twist_is_relative: False`)。
- トラック喪失時は出力沈黙 → MPPI 直進(`require_other_ship: False`)という安全縮退。
- **要実データ確認**: twist frame 規約の解釈は実データで未確認(§20)。

## 12. 岸壁認識

- `/pcl/nonground` から鉛直壁面を直線として抽出し、3 フレームの時間確認を経て出力。
- 一次安全経路は **Nav2 obstacle layer**(`/quay_wall/points` を第 2 ソース `quay` として local/global 両 costmap に追加。marking のみ・clearing なし → シミュ等で沈黙しても無影響)。
- `/quay_wall/costmap`(margin 5 m 膨張)は可視化・将来用。MPPI 内 quay cost は実験的(既定オフ)。

## 13. MPPI 接続

- 実機配線: `/odom`(GLIM)→ own、`/other_ship/twist` + TF `map→opponent_vessel`(認識)→ other、`/waypoint1_pose` `/waypoint2_pose`(task2_waypoint_pose_publisher)→ 基準線。出力 `/planned_path_pruned` → follow_path_client → Nav2 FollowPath。
- シミュ検証済みの配線(planner 直接 `/planned_path_pruned`、pruner バイパス)を実機でも維持。
- 全ハイパーパラメータを `mppi.*` として ROS パラメータ化。**デフォルト=旧ハードコード値**(horizon 225、dt 0.1、samples 5000、λ 12、σ [35,0]、target 1.02889 m/s(2 kn)、div 150、col 10/50、gate 3、buoy 120、speed 0.1、norm 0.2、CRM 3.2/1.6/6.4/1.6 LOA、loa 2.0、gate 半幅 4.0)。不変性はテスト 25/25 で確認。
- アルゴリズム本体(KT モデル・CRM・コスト構造)は無変更。

## 14. Nav2 接続

- Task 2 専用スタック(`navigation_launch_task2.py` + `nav2_params_task2.yaml`)を `enable_nav2` でゲートして起動。real_bringup の Nav2 は常に無効化(`enable_nav2:=false` 固定)し、実機汎用 `nav2.launch.py` / `nav2_params.yaml` は無変更。
- controller は RegulatedPurePursuit(desired 1.10 m/s)。velocity_smoother 出力を `/cmd_vel_thruster` にリマップ(max_velocity [1.20, 0, 0.18])。
- costmap への変更は quay ソース追加のみ。

## 15. 実機 launch

`task2_real.launch.py`: 既存 launch の合成のみ(コピペなし)。引数 15 個(enable_* 9、YOLO 系 4、安全系 2)。**安全既定**: `dry_run=true` / `send_thruster_commands=false` → thruster_driver / serial_writer は起動せず、`/cmd_vel_thruster` までを観測可能。スラスタ起動は `enable_thrusters ∧ ¬dry_run ∧ send_thruster_commands` の三重ゲート成立時のみ。シミュ専用 6 ノードの非混入を静的テストで保証。

## 16. テスト結果(Mac)

| テスト | 結果 |
|---|---|
| task2_perception pytest(cloud_ops 15 / tracking_glue 9 / wall_fit 5) | **29/29 PASS** |
| robot 静的 launch テスト(`test_task2_real_launch_static.py`) | **8/8 PASS** |
| MPPI デフォルト不変性検証 | **25/25 一致**(パラメータ化前後で挙動同一) |
| `benchmark_mppi.py`(Mac CPU, torch 2.7.0, 実構成 225×5000) | **≈ 9.25 Hz**(1 solve ≈ 108 ms) |
| `task2_static_check.sh`(py_compile / YAML / XML) | FAIL 0 |

## 17. Mac で未確認の項目

- `colcon build`(C++ 含む)と `ros2 launch` の実行時解決(ROS 2 が Mac に無い)
- 全ノードの実行時挙動(topic 接続はコード上の静的整合のみ確認)
- CUDA での MPPI / YOLO 実行、TensorRT エクスポート
- サブモジュールとのメッセージ ABI(ship_perception_msgs のビルド)
- **本レポートで「動作する」と主張している箇所は存在しない。すべて「静的確認済み」である**

## 18. Jetson で確認する項目

`Docs/task2_jetson_validation.md` の 18 段階に集約。特に:

1. colcon build 全通過(ship_perception_msgs → task2_perception の順序依存)
2. URDF LiDAR 取付角の実測 → 点群天地確認(段階 3)
3. ship_tracker twist frame 規約の実データ確認(段階 5)
4. MPPI CUDA 1 solve 時間(2 Hz 比で余裕があるか)
5. フルスタック同時稼働時の CPU/GPU/RAM(tegrastats。**Jetson に nvidia-smi は無い**)
6. 三重ゲート実機確認(段階 14 の表)と E-stop(段階 18)

## 19. 人が調整する項目(単位・公称値付き)

| 項目 | 場所 | 公称値 | 区分 |
|---|---|---|---|
| LiDAR 取付角 roll/pitch/yaw・位置 x/y/z | robot.urdf.xacro | π / 0 / 0 rad、(0, 0, 0.8) m | **実測必須** |
| `waterline_z_m` | task2_perception_params.yaml | 0.0 m | **実測必須** |
| `self_crop_{min,max}_{x,y,z}` | 同上 | ±1.2 / ±0.8 / −0.5〜1.5 m | **実測必須** |
| `wall_min_points` | 同上 | 30 点 | **実測必須**(点密度依存) |
| YOLO11s 重み | yolo_model_path / model_path | 非同梱 | **配置必須** |
| task2_waypoints.yaml の x/y | waypoint_publisher/config | プール C→A 対角 ≈46 m | **現地再生成** |
| 水面除去帯・RANSAC しきい値 | task2_perception_params.yaml | −0.3/+0.15/0.05/0.9/0.3 m | 実機調整 |
| opponent ゲート(距離 60 m、長さ 0.5–30 m、点数 5、鮮度 2 s)・α 0.3・スパイク 5 m/s | 同上 | 左記 | 実機調整 |
| 岸壁(長さ 2 m、距離 40 m、RANSAC 0.15 m、確認 3 回、margin 5 m) | 同上 | 左記 | 実機調整 |
| MPPI 全 `mppi.*`(target_speed 1.02889 m/s、horizon 225×0.1 s、samples 5000、λ 12、CRM 3.2/1.6/6.4/1.6 LOA ほか) | mppi_params.yaml | 左記 | 実機調整(変更を YAML に記録) |
| YOLO [TUNE](conf 0.25、iou 0.45、imgsz 640、inference_hz 5、class_names、buoy_class_names、camera_fov_deg 90、fixed_range_m 5) | yolo11s_params.yaml | 左記 | 実映像調整 |
| Nav2(desired 1.10 m/s、max_velocity [1.20, 0, 0.18]、lookahead 3.0 m) | nav2_params_task2.yaml | 左記 | 実艇確認 |

詳細は `Docs/task2_parameter_reference.md` §1。

## 20. 既知の問題・制約

1. **実行時未検証**: 全コードが Mac 静的確認のみ(ROS 2 / Jetson 環境なし)。
2. **YOLO11s 重み不在**: リポジトリ非同梱。配置まで YOLO 系は起動不可(fatal 終了)。
3. **URDF 取付角は公称値**: roll=π は仮置き。実測まで GLIM・認識を信用しない。
4. **公称パラメータ群**: waterline / self_crop / wall / クラスタ関連は実測前提の placeholder。
5. **ship_tracker twist frame 規約**: 「物体 body frame・相対速度」というコード解釈は実データ未確認。誤りなら opponent_selector の回転・補償の修正が必要。
6. **ego 補償はグルー側**: tracker 内部の no-op スタブは触っていない(サブモジュール無改造方針)。将来サブモジュール側で補償が実装されたら**二重補償に注意**。
7. **MPPI quay cost は実験的**(既定オフ)。一次安全経路は Nav2 costmap。
8. **`use_int8` は予約・未使用**: 宣言のみで何も消費しない。
9. **カメラ選定未解決**: ZED2i CPU フォールバック vs USB カメラは上流課題のまま。
10. **実機の Nav2 `pointcloud` ソースは空**: `/pointcloud` はシミュ専用。実機 costmap の障害物は現状 quay ソースのみ(他船は MPPI の CRM が担当)。
11. **yolo11s.launch.py の venv ガード**: venv 未活性だと launch 記述生成時に例外 → `enable_yolo:=true` の統合 launch 全体が落ちる。venv なしでは `enable_yolo:=false` にする。
12. 細部: `task2_static_check.sh` に旧パス `src/perception/task2_perception` の探索行が残る(後続の `find` で実パスを拾うため実害なし)。`planner_node.py` の起動ログに同一 2 行の重複あり(表示のみ)。`record_task2_bag.sh` の既定リストに `/quay_wall/costmap` `/pcl/nonground` が無い。

## 21. 今後の改善

- YOLO 検出と LiDAR 追跡の fusion(現状 YOLO 出力は制御ループ未接続)
- 複数他船対応(opponent_selector の ranked list は既に複数返せる設計)
- tracker 側 ego 補償の本実装(サブモジュール側課題)+ グルー側補償の撤去
- MPPI quay cost の実データ検証 → 有効化判断
- `/pointcloud` 相当の実機障害物ソース(フィルタ済み点群の costmap 直結)検討
- record_task2_bag.sh の既定リストへ costmap/nonground 追加
- INT8 パス(`use_int8`)の実配線とキャリブレーションデータ整備
- ROS 2 ディストリビューションの最終確定(preflight は ROS_DISTRO を検査するのみ)

## 22. push 前の注意事項

- **現時点で GitHub には何も push していない**(task2-experiment もフィーチャーブランチもすべてローカル)。
- push 前に**ユーザーのレビューと明示的な承認が必要**。
- リポジトリ規約(AGENTS.md / CLAUDE.md)により、GitHub への write は owner が `IBO-ASV` または `oswystk15662` のリポジトリに限る。push 先 remote の実 owner を URL/API で確認してから操作すること(remote 名だけで判断しない)。
- サブモジュールには push すべき変更が存在しない(ポインタ db84af9・ローカルコミットなし)。
- push 対象は task2-experiment のみで足りる(フィーチャーブランチはマージ済み)。マージ済み履歴に mppi ブランチ由来のコミットが含まれる点に留意。
