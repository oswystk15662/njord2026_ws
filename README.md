# 環境について
## 環境構築
まずはros2 humbleが入っているubuntu22.04を起動し、以下のコマンドでこのワークスペースをクローンしてください。

```shell
# githubのsshを通していない場合
git clone https://github.com/oswystk15662/njord2026_ws.git --recursive

# githubのsshを通している場合
git clone git@github.com:oswystk15662/njord2026_ws.git --recursive
```

正常にクローンできたか確認するために、vscodeでこのワークスペースディレクトリを開き、./src/driver/imu/内のwit_node_ros2の中身があるか確認してください。

ない場合は以下のコマンドを叩いてください

```shell
# ワークスペースディレクトリにいる状態で以下を打つ
cd ./src/driver/imu/wit_node_ros2 && git submodule update --init --remote

# ワークスペースディレクトリに戻る
cd ../../../../
```

クローンできたら、colcon buildしてください

```shell
cd njord2026_ws && colcon build --symlink-install
```

おそらく以下のaptで入れるパッケージがないというエラーが出るので、入れましょう。
でなくてもlaunchするときにエラーが出ると思うので入れましょう。

```shell
sudo apt install ros-humble-ecl* ros-humble-robot-localization
```

ビルドが成功したら環境構築はうまく行っていると思います。

## ディレクトリ構成について

colcon buildしたあとは以下のようなディレクトリ構造になっていると思います。

```shell
$ tree -d
.
├── bag_files/  # rosbagを保存する場所。なおrosbag自体をgitで管理するわけではなく、入れるところっていうだけ
├── build/      # colcon buildで生成されたもの。vscodeの左側ではsrcと違い、薄い灰色文字になっているはず
├── install/    # colcon buildで生成されたもの。vscodeの左側ではsrcと違い、薄い灰色文字になっているはず
├── log/        # colcon buildで生成されたもの。vscodeの左側ではsrcと違い、薄い灰色文字になっているはず
└── src
    ├── bt      # behavior tree周りの諸々（action nodeなど）を入れる。
    ├── ditection   # 物体認識周り
    │   ├── pcl_det # Point CLoud DETection
    │   └── yolo    # You Look Only Once
    │
    ├── driver
    │   ├── camera
    │   │   ├── usb_camera_driver   # logicool c270用
    │   │   └── zed-ros2-wrapper/ # zed 2i用
    │   │       ├── zed_ros2
    │   │       └── zed_wrapper/
    │   ├── gnss
    │   │   ├── drogger_bt  # drogger gnss moduleとblue toothで接続する用
    │   │   ├── drogger_rws # drogger gnss moduleの中身がublox ZED-F9Pのやつ
    │   │   ├── drogger_rzs # drogger gnss moduleの中身がseptentorio mozaicのやつ
    │   │   └── um982_driver # unicore um982用
    │   └── imu
    │       ├── sonota # その他（）
    │       └── wit_node_ros2 # witmotion mpu9250用
    │           ├── doc
    │           ├── wit_driver
    │           ├── wit_msgs
    │           └── wit_node
    ├── path_follower # 経路追従アルゴリズム・ノードを入れるところ。
    ├── path_generator # 経路生成アルゴリズム・ノードを入れるところ。避航も含む
    └── robot
        ├── config # ここには最終的にlaunchするときに使う設定ファイルとかを入れる。単体試験は各パッケージlaunch書いてください。
        ├── launch # ここには最終的にlaunchするときに使うlaunchファイルを入れる。単体試験用launchを呼び出す形で書くと良いとされている
        ├── meshes # 表示用STLファイルを入れるところ
        └── urdf # tf設定・表示用のurdfを入れるところ
```

colcon buildによって生まれた、build・installl・logディレクトリは.gitignoreによって、gitが変更を追跡しないようにしているため、薄い灰色になっています。
中見ればわかりますが、非常に大量のファイルが生まれ、変更差分が分かりづらくなるため、無視するのが一般的です。
bag_filesの中身も同様に追跡しないようになっています。

## visualizerについて
fox gloveを推奨します。
https://foxglove.dev/download

ros2ネットワークにそのままアクセスできるようにするため、
```shell
sudo apt install ros-humble-foxglove-bridge
```
をしてください。

## YOLOについて
以下の記事のvenvでなんとかする方法をもとにhumbleですが、仮想環境にして端末の環境をpipから守るようにしています。
https://zenn.dev/kimushun1101/articles/ros2-jazzy-pip
この記事にも書かれていますが、むやみにrosdepすると破壊されるので注意が必要です（大澤はまだやってないはずです、たぶん、、、）

でactivateした後に
```shell
pip install -r requirements.txt
```
だけで大丈夫です。

↓いらないけどログ
```shell
pip install ultralystics
```
すると1GByte以上のダウンロードが始まります。
**注意**
* numpy 2.0以上のものが入る可能性があるので、これが終わったら以下のコマンドを打って下さい
* rosのpythonライブラリはnumpy 1.0系列を前提としているそうです。
* もし、YOLOとrosで相性が悪かったらがんばるしかないので、どうしましょ感はあります

```shell
pip install "numpy<2.0" "opencv-python<=4.10.0.84"
python3 -c "import numpy; import cv2; import cv_bridge; import ultralytics; print(f'Numpy: {numpy.__version__}, OpenCV: {cv2.__version__}, OK')"
```

もし ultralysticsのinstallで
```shell
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
generate-parameter-library-py 0.5.0 requires typeguard, which is not installed.
```
というerrorが出たら```pip install typeguard```をうってください。
↑いらないけどログ

そういえばultralysticsの激ヤバ利用規約ってどうなったんですかね


#　座標系について
機体前方をxとし、鉛直地面から空向きをz正とするようにyをとります。
基本的にすべてENUで考えるために、NEDで出てきがちなimuはだいたい回しています。
**センサの座標はdatasheetを信用せず、dataを見てやってください**


# aptで入れるパッケージ
for wit motion
* ros-${ROS_DISTRO}-sophus
* ros-${ROS_DISTRO}-tf2-geometry-msgs
* ros-${ROS_DISTRO}-ecl*

for usb camera
* opencv(ubuntu22.04にあったものを入れてください)
* ros-humble-cv-bridge
* ros-humble-image-pipeline
* gstreamer系の諸々(mjpg経由は動くけどgstreamer経由はうまく動かない可能性が高いが、依存の中に入れてしまってるので入れてください。すみません)

for zed 2i
* cuda 12.1(もしくはdockerでやる)
* zed sdk

for localization
* ros-humble-robot-localization

for navigation
* ros-humble-navigation2
* ros-humble-nav2-bringup

about topic network



大澤個人的ToDo

* waypoint生成
    * [ ] ダイクストラ理解・python実装・pyqtでのGUI
    * [ ] A*理解・cpp実装・rqtとかで表示
    * [ ] theta*理解・ros2 cpp実装・o3de表示
* 補間
    * [ ] スプライン理解・python実装・pyqt表示
    * [ ] beta spline理解・python実装・pyqt表示
    * [ ] NURBS理解・cpp実装・いい感じ表示
    * [ ] クロソイド理解・ros2 cpp実装・o3de表示
* 

Nav2（ROS 2 Navigation Stack）を中心とした実装方針、非常に良い選択です。Njordの自律船（ASV）タスクにおいて、拡張性と堅牢性を確保できます。

Task 1（ウェイポイント追従 + 方位標識による回避）をNav2で実現するための、具体的な実装ステップを4段階で解説します。

---

###ステップ1：座標系と位置推定の確立 (Localization)Nav2を動かすには、「ロボットが地図（地球）上のどこにいるか」を正確に定義する必要があります。

1. **TFツリーの構築:**
ロボットの形状定義（URDF）と座標変換（TF）を作成します。
* `base_link`: 船の中心
* `gnss_link`, `imu_link`, `lidar_link`: 各センサの取り付け位置


2. **GPS + IMU フュージョン (`robot_localization`):**
船はオドメトリ（エンコーダ）がないため、GPSとIMUを融合して `map` 座標系と `odom` 座標系を作ります。
* `ekf_node`: IMUの加速度・角速度から、短期間の滑らかな位置変化（`odom` -> `base_link`）を計算。
* `navsat_transform_node`: GPSの緯度経度を `map` 座標系のXY平面に変換（`map` -> `odom`）。



**実装すること:**

* `robot_localization` パッケージのインストールとパラメータ設定（`ekf.yaml`）。

---

###ステップ2：Nav2 の基本設定 (Configuration)まずは「障害物がない海面を、指定したGPS座標まで直進する」部分（Task 1の前半）を作ります。

1. **Costmap（コストマップ）の設定:**
海上の障害物を管理するマップです。
* **Global Costmap:** 全体経路計画用。GPS座標と地図のサイズ（十分大きく、例えば 2000m x 2000m）を設定。
* **Local Costmap:** 直近の回避用。Rolling Window（ロボット中心に追従する設定）をTrueにします。
* **Layer:** `ObstacleLayer` を入れ、LiDAR (`LaserScan` or `PointCloud2`) を入力ソースに指定します。これで「ブイ」が障害物として認識されます。


2. **Controller (Local Planner) の選択:**
船の挙動に合ったコントローラを選びます。
* **推奨:** **`Regulated Pure Pursuit`**
* 理由: レガシーコードでも Pure Pursuit が使われており、挙動が直感的です。中〜高速での経路追従に優れています。


* **次点:** **`MPPI Controller`**
* 理由: GPU（Jetson等）があるならこちらが最強です。動的な障害物回避能力が非常に高いです。





**実装すること:**

* `nav2_params.yaml` の作成（Planner, Controller, Costmapの設定）。
* `gps_waypoint_follower` (Nav2のサンプル) を実行し、船がGPS点へ動くか確認。

---

###ステップ3：方位標識（Cardinal Marks）の対応ロジックここがTask 1後半の肝です。「北方位標識が見えたら、北側を通る（＝南側に行ってはいけない）」というルールをNav2に守らせます。

**アプローチ：「仮想壁（Virtual Wall）」の生成**

Nav2の標準機能だけでは「北を通れ」という指示は理解できません。そこで、**「南側に嘘の障害物（壁）を作る」** ことで、Plannerに強制的に北側のルートを引かせます。

**実装の流れ:**

1. **認識ノード (`yolo_to_marker_node`):**
* YOLOでブイを検出。
* LiDARと突き合わせて、ブイの正確な3次元位置 (x, y) を特定。
* ブイの種類（例：North Cardinal）を判別。


2. **仮想障害物生成:**
* もし **North Cardinal** が (x, y) にあったら、その **南側** 半径 R メートルの範囲に、点群（PointCloud2）を生成して配信します。
* トピック名例: `/virtual_obstacles`


3. **Nav2連携:**
* Costmapの `ObstacleLayer` に `/virtual_obstacles` トピックを追加します。
* これでNav2は「標識の南側は壁だらけだ」と認識し、自動的に北側を通るルートを再計算（Replanning）します。



**Python擬似コード:**

```python
def callback(self, detection):
    if detection.label == "North_Mark":
        # 北標識の南側に仮想の壁を作る
        fake_points = []
        buoy_x, buoy_y = detection.position
        
        # ブイの南側(yマイナス方向)に半円状に点を配置
        for theta in range(180, 360): 
            rad = math.radians(theta)
            px = buoy_x + radius * math.cos(rad)
            py = buoy_y + radius * math.sin(rad)
            fake_points.append([px, py, 0.0])
        
        self.publish_point_cloud(fake_points) # Costmapがこれを読み込む

```

---

###ステップ4：Behavior Tree (BT) の構築最後に、これらを統合します。Task 1用のシンプルなBT XMLを作成します。

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <NavigateThroughPoses goals="{gps_waypoints}" />
    </Sequence>
  </BehaviorTree>
</root>

```

* **動的な回避:**
ステップ3の「仮想壁」の仕組みがあれば、BT側で特別な分岐（if North then...）を書く必要はありません。Nav2の `MapsThroughPoses` が実行されている裏で、Costmapが更新されれば、自動的に経路が修正されるからです。

---

###まとめ：開発ロードマップ1. **足回り:** `robot_localization` で GPS/IMU を統合し、TF (`map` -> `base_link`) を確立する。
2. **基本航行:** Nav2 を `Regulated Pure Pursuit` で設定し、障害物のない海でGPS走行できるようにする。
3. **認識統合:** YOLOとLiDARを統合し、ブイの位置特定ノードを作る。
4. **ルール適用:** 「仮想壁生成ノード」を作り、方位標識の特定方向をNav2上で通行止めにする。

まずは **ステップ1と2**（GPSでの自律航行）を最優先で完了させることをお勧めします。これだけでTask 1の半分はクリアできます。
