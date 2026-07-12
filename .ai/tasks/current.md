# current task — sim↔real ブリッジ実装 (branch: test07089)

最終更新: 2026-07-12 / 記録者: メインClaude(司令塔)

## 背景

test07089 では急ピッチで「実センサ・実スラスタを使えるようにする」変更を進めている。
sim で成立しているハイレイヤ(cmd_vel / task / nav2 / localization)と現実(ESP32・センサ)を
つなぐ部分の実装が抜けていたため、その接続層を整備する。

前提(ユーザ指示):
- ESP32 とは **USB serial のみ**で通信する。CAN は使わない。
- `can_msgs` pkg / uros(micro-ROS)は廃止してよい。
- mid360s / zed2i / red hill web cam を使えるようにする。
- スラスタの id check など実運用を想定した実装を進める。

## 現状把握(コード調査結果)

- USB serial ブリッジは既存: `micon_driver_fd/serial_writer`
  - sub `/thruster_command` (`std_msgs/Float32MultiArray`, 先頭4要素) + `/emg /red /yellow /green` (`Bool`)
  - 50ms 周期で `4×float32 (native LE) + flag byte` を書き込み。flag: bit3=emg,bit2=green,bit1=yellow,bit0=red
  - README は「normalized [-1,1] を送る」と記載。
- `thruster_driver`: cmd_vel + odom → P(+DOB) → wrench → 4スラスタ配分。
  - sim 出力は `/thruster_command` に **Newton値** (`command * force_per_duty`, 最大40N) を publish → **serial_writer の期待(normalized)と単位不整合**。
  - transport_mode に `can` / `mros_usb` / `both` などが残存(CAN=`can_msgs/Frame`, mros=per-thruster `UInt16`)。
- スラスタ id check: `simple_manual/thruster_id_test.cpp` が3秒ごとに各スラスタへ順番に thrust を出す簡易版が既存。
- CAN 依存箇所: `can_msgs`(pkg本体), `thruster_driver`(package.xml/CMake/node.cpp), `bms`(package.xml/CMake/node.cpp)。
- uros/micro-ROS: 実コードは未検出(`thruster_driver` の命名 "mros" のみ)。
- センサ launch:
  - `zed2i_driver/launch/zed2i.launch.py`, `zed2i_cpu.launch.py`
  - `robot/launch/back_cam.launch.py`(`usb_cam`, /dev/video0)= red hill web cam 候補
  - `um982_driver/launch/um982.launch.py`
  - livox/mid360 driver 本体は存在するが unified launch から未起動
  - `robot/launch/localization.launch.py` は glim/ekf/navsat を起動するが **実 lidar/gnss/imu ドライバは起動していない**
  - `robot/launch/all.launch.py` は localization+navigation のみ

## 要求(Requirements)

### R1. スラスタ実駆動パス(USB serial)の一貫性確保 [必須]
- ハイレイヤ cmd_vel → `thruster_driver` → `/thruster_command` → `serial_writer` → ESP32 が
  **単位・スケールが破綻せず**通ること。
- serial_writer と thruster_driver の間で送る値の意味(normalized か Newton か)を統一する。
- ESP32 は 1台につき 1 ESC、UInt16 指令 1 個という前提(specific.md)と矛盾しないこと。

### R2. CAN / uros の廃止 [必須]
- `thruster_driver` から CAN transport (`can_msgs/Frame`)と mros(micro-ROS想定)分岐を除去。
- `bms` から CAN 依存を除去(BMS 値は USB serial 経由に一本化 or 明示的に別線と定義)。
- `can_msgs` pkg を削除。ビルド対象・依存から外す。
- 除去後も sim 経路(`/thruster_command` Float32MultiArray)は維持。

### R3. スラスタ id check(実運用向け) [必須]
- どの ESP32/ESC が FR/FL/RR/RL のどれに対応するかを実機で確認できる手段。
- 既存 `thruster_id_test` を実運用要件(安全停止・対象選択・出力単位が実パスと一致)に合わせて整備。

### R4. センサ bringup [必須]
- mid360(livox lidar), zed2i, red hill web cam(usb_cam back_cam)を
  実機 bringup launch から起動できること。
- GNSS(um982) / IMU / localization(glim,ekf) と統合した「実機起動」launch を用意 or 整備。

### R5. 実運用を想定した安全・診断 [推奨]
- watchdog / feedback timeout / emg 停止が実パスで機能すること。
- センサ・スラスタの死活が diagnostics で見えること(既存 diagnostic_monitors 活用)。

## 受け入れ条件(Acceptance Criteria)

- AC1: `colcon build` が対象パッケージ(thruster_driver, micon_driver_fd, bms, robot, simple_manual, 各driver)で成功する。
- AC2: `can_msgs` への参照がビルドグラフから消え、grep で実コード参照が残らない(README等の記述除く)。
- AC3: 既存ユニットテスト(`thruster_driver` allocation, `micon_driver_fd` serial_writer)が pass。
       単位統一に関する回帰を守るテストを追加。
- AC4: 実機 bringup launch が **ドライラン(ros2 launch --show-args / node graph)**で
       正しいノード構成・topic 接続になっている(実 HW なしで検証可能な範囲)。
- AC5: cmd_vel → serial パケットの数値変換が期待通り(単位・neutral・span・クランプ)であることを
       テストまたは明示的な計算で確認。
- AC6: id check が実パス(serial_writer 経由)と同じ単位・安全挙動で1スラスタずつ駆動する。
- AC7: 変更が sim 経路(task*_sim / manual_control)を壊さない。

## 確定事項(2026-07-12 ユーザ判断)

- D1(完了ライン): **コード/launch/config 完成 + colcon build + テスト + ドライラン検証**まで。実HW通電検証は別工程(艇上)。
- D2(単位規約): **Newton のまま送る**。`/thruster_command`(Float32MultiArray)は Newton 値。serial_writer は 4×float32(native LE, Newton)+flag byte をそのまま送信。ESP32 側で thrust→PWM 換算。
  → thruster_driver の sim 出力(Newton)と serial_writer の生 float 送信が既に一致。README の "normalized" 記述が誤りなので Newton に訂正。id_test/manual も Newton 一貫に。
- D3(pkg整理): **can_msgs 削除 / kinematics 削除 / bms は serial 一本化して残す**。thruster_driver は CAN・mros 分岐を除去し sim(Float32MultiArray)経路のみ。
- D4(webcam): red hill web cam = **back_cam**(usb_cam, /dev/video0)。実機 bringup に統合。

### D3 に伴う新規設計論点(→ Sol レビュー対象)
- bms を serial 一本化 = ESP32→ホストの **BMS 受信経路が現状存在しない**(micon_driver_fd は送信専用)。
  ESP32 が同一 USB serial 上で BMS(4セル電圧)を返す前提で、受信・デコード・publish の設計が必要。
  想定アーキ: **単一 Micon/ESP32** が USB serial 1本で 4スラスタ(+flag)を受け、BMS 等を返す
  (specific.md の "ESP32 1台=1ESC + per-thruster mros topic" 案は D2/D3 で破棄)。

## 進め方(CLAUDE.md ルーティング)

1. 本要求整理(完了) → ユーザにオープン論点を確認。
2. 確定後、必要なら Codex Sol に設計レビュー依頼(単位規約・パケット互換・pkg削除影響)。
3. 実装担当を luna/terra/sonnet から作業規模に応じ1つ選択(現時点想定=terra: 複数pkg横断の中規模)。
4. 実装 → 別モデルでレビュー(.ai/reviews/)。
5. テスト失敗は同一修正を最大2回、以降 Sol→(昇格条件で)Fable(分析・計画のみ)。
6. 各段階を .ai/tasks/current.md, .ai/reviews/, .ai/escalations/ に記録。

## 確定設計(Sol レビュー + 司令塔裁定, 詳細は .ai/reviews/2026-07-12-sol-design-review.md)

- 単一 serial 所有ノード = `micon_driver_fd`(双方向化, executable 名 `serial_writer` 維持)。他ノードは tty を open しない。
- TX: 現行 raw v1(`4×float32 LE + flag byte`)維持。ESP32 ファーム変更は本タスク**スコープ外**。
- RX: BMS を framed+CRC でホスト側実装 + loopback ユニットテスト(実 BMS 通電はHW工程)。
- bms pkg は残す: 所有ノードの復号済み topic を購読し `bms/cell_voltages` 標準化 + diagnostics。tty は open しない。
- thruster_driver: CAN/mros/transport_mode 除去、非sim指定は起動エラー、`/thruster_command`=Newton。pkg は残す。
- can_msgs / kinematics: 削除。thruster_config.yaml/各README/task1_sim.launch.py 等の参照残を掃除。
- real_bringup.launch.py 新設: mid360/zed2i/back_cam/um982/imu/localization/thruster+serial を enable_* 引数で統合、use_sim_time=false。
- 安全: 起動時0N/cmd timeout0N/feedback timeout0N/serial disconnect0N。実機 bringup は stop_on_feedback_timeout=true 固定。

## 実装担当の選択

- 選択: **codex-terra**。理由: micon 複数pkg横断 + launch新設 + 削除掃除 + テスト付きリファクタ = 通常〜中規模、既存構造準拠。
- レビュー: 実装後に別モデル(Sol もしくは sonnet-coder)で実施。実装者≠レビュアー維持。

## ログ

- 2026-07-12: コード調査・要求整理・.ai 構築(メインClaude)。オープン論点をユーザ確認→D1〜D4 確定。
- 2026-07-12: Codex 一時利用不可(gpt-5.6-sol 要CLI更新)→ model=gpt-5.5 回避で復帰(.ai/escalations/)。
- 2026-07-12: Codex Sol 設計レビュー実施→設計確定(.ai/reviews/)。実装担当=terra を選択。
- 2026-07-12: Codex 書込 sandbox 不可(bwrap)→ ユーザ承認で danger-full-access 実行。Terra 実装完了。
  環境は ROS2 Jazzy と判明(Humble 未導入)。Jazzy で build/test/dry-run 成功。
- 2026-07-12: 実装レビュー(Opus, 実装者と別モデル)= **合格**。独立に build + 44 tests/0 fail を再確認。
  残課題: SW E-stop 未接続(ユーザ確認事項)/ 実 HW 一括起動は艇上工程。→ .ai/reviews/2026-07-12-review-terra-impl.md
- ステータス: **実装+レビュー完了**。テスト失敗ゼロのため Sol/Fable 昇格なし。
