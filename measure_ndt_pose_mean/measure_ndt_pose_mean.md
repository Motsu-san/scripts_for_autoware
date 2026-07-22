# NDT 固定初期位置・1点群での姿勢平均（`measure_ndt_pose_mean`）

> 現在の `measure_ndt_pose_mean/measure_ndt_pose_mean.sh` は、Autoware localization を起動して
> `/localization/pose_estimator/pose_with_covariance` を待つ旧方式ではなく、
> `~/autoware/src/tools/localization/ndt_direct_measure` の direct NDT align を
> `N_RUNS` 回実行し、平均 pose・ばらつき・平均 pose からの最大偏差を出力します。
> direct 方式の詳細は [`measure_ndt_direct.md`](measure_ndt_direct.md) を参照してください。

`measure_pose_mean.sh` が行う複数回の bag **全再生**では、再生ごとの初期化や pose_initializer の経路の影響で **NDT への入力初期位置がばらつき**、`EKF 出力の平均` を単純に「そのスキャンの NDT 真値」とみなせない場合があります。

本ツールは次を満たすために、**bag 再生なし**で Autoware（localization を中心に）を起動し、**手動で用意した初期位置 YAML** と **指定時刻に最も近い LiDAR 点群の 1 フレーム**を繰り返し投入して、`/localization/pose_estimator/pose_with_covariance`（NDT 側）を **多数回評価し平均**します。

---

## 追加・関連ファイル

| パス | 役割 |
|------|------|
| [`measure_ndt_pose_mean.sh`](measure_ndt_pose_mean.sh) | メインエントリ（direct NDT 実行〜集計） |
| [`measure_ndt_direct.sh`](measure_ndt_direct.sh) | `ndt_direct_measure_node` 起動ラッパ |
| [`aggregate_ndt_direct_result.py`](aggregate_ndt_direct_result.py) | N 回結果の平均・ばらつき集計 |
| [`aggregate_pose_mean_from_bags.py`](aggregate_pose_mean_from_bags.py) | 平均 pose YAML / 偏差統計ユーティリティ |
| [`../sh/launch_localization_for_ndt_measure.sh`](../sh/launch_localization_for_ndt_measure.sh) | （旧方式）bag **再生せず** localization スタックのみ起動 |
| [`../py/measure_ndt_pose_mean.py`](../py/measure_ndt_pose_mean.py) | （旧方式）Autoware 経路での試行ループ |

既存ツールとの位置づけ:

- [`../sh/measure_pose_mean.sh`](../sh/measure_pose_mean.sh): 複数回 **bag 再生**＋EKF pose などの記録後、bag 間で平均
- 本ツール: **単一センサーフレーム×固定初期 pose** で NDT 出力のばらつきを評価・平均

---

## 既定の前提

| 項目 | 既定・推奨 |
|------|-----------|
| **試行回数** | **100**（第 4 引数で変更可） |
| **ndt_start_pose.yaml** | **ソース rosbag と同じディレクトリ**<br>例: `.../final_merged/final_merged_0.db3` と同じ階層の `ndt_start_pose.yaml`<br>（ディレクトリ形式の bag はそのディレクトリ直下）<br>bag 再生用の `initial_pose.yaml` とは別ファイル |
| **NDT の pose 出力** | `/localization/pose_estimator/pose_with_covariance`（環境変数で変更可） |
| **点群トピック** | `/sensing/lidar/concatenated/pointcloud`（環境変数で変更可） |

`ndt_start_pose.yaml` に `skip_initial_localization: true` を書いておいても、`set_initial_pose.py` の挙動はコマンドの `--skip-initial-localization` と整合します。

---

## 使い方

Autoware ワークスペースのルートから（ビルド済み `install` があるディレクトリ）:

```bash
cd /path/to/autoware_ws
# rosbag と同じディレクトリに ndt_start_pose.yaml を置く

AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_pose_mean.sh \
  /path/to/map \
  /path/to/rosbag.db3 \
  1772096549.105
```

試行回数を変える例（50 回）:

```bash
AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_pose_mean.sh \
  MAP BAG 1772096549.105 50
```

vehicle を sample に固定する例（direct 方式では未使用・互換のため受け取るのみ）:

```bash
./measure_ndt_pose_mean.sh --force-sample-vehicle MAP BAG TARGET
```

---

## 出力

出力先の既定は **rosbag と同じ親ディレクトリ** 配下:

`mean_ndt_pose_<TARGET>_n<N>_<日時>/`

- `ndt_pose_mean_<TARGET>_n<N>.json` — 集計結果・各試行のメタデータ
- `mean_ndt_pose.yaml` — `set_initial_pose.py` 互換の `pose` ブロックに加え、集計メタデータ

ディレクトリやファイル名は環境変数で上書き可能（後述）。

---

## 処理の流れ（概要）

1. `launch_localization_for_ndt_measure.sh` で **localization 系を起動**（通常の `launch_autoware.sh` のような **bag 全再生はしない**）。
2. 指定 **UNIX 時刻**で **`/clock` を固定**して publish（シミュレーション時間と整合）。
3. 元 bag から **`/tf_static` を loop 再生**（点群の `frame_id` チェーンに必要）。
4. 元 bag から `POINTCLOUD_TOPIC` 上の **指定時刻に最も近い 1 フレーム**を読み出し。
5. 各試行で:
   - EKF / NDT の `trigger_node` で **deactivate**
   - `set_initial_pose.py --skip-initial-localization` で **pose_initializer を経由せず** EKF 初期化＋NDT 有効化（既存スクリプトの意図どおり）
   - map loader へ `ndt_start_pose.yaml` 周辺を問い合わせ、試行時に返るタイル ID・範囲をログと JSON に記録
   - **EKF を再度 deactivate** して、同じ初期 pose トピック上の EKF 出力が NDT の seed pose バッファを消さないようにする
   - 同じ点群を短いバーストで **publish**。各 publish の直前に、bag 近傍の車速・角速度から生成した **疑似 EKF pose を 2 点** publish（NDT の補間バッファ用。下記「EKF seed pose」参照）
   - NDT の `pose_with_covariance` を **1 サンプル**取得
6. 全試行の pose を平均（位置の算術平均、姿勢は四元数平均・既存 `aggregate_pose_mean_from_bags` と同系の統計）。

---

## EKF seed pose（`pose_buffer_.size() < 2` 対策）

NDT scan matcher は `input_initial_pose_topic`（tier4 既定では `/localization/pose_twist_fusion_filter/biased_pose_with_covariance`）を **2 点以上**で補間します。固定 `/clock` と `set_initial_pose` のみだと履歴が 1 点になり、`pose_buffer_.size() < 2` でタイムアウトすることがあります。

本ツールは **評価用の疑似 EKF 履歴**として、次を行います（自然な EKF 出力ではありません。出力 JSON の `ekf_seed` に明記されます）。

1. bag から **target 時刻近傍**の `VelocityReport`（`/vehicle/status/velocity_status`）と、必要なら IMU（`/sensing/imu/tamagawa/imu_raw`）を読む。
2. `ndt_start_pose.yaml` を **target 時刻の中心姿勢**とみなす。
3. 角速度は **`VelocityReport.heading_rate` を優先**。該当メッセージが無いときだけ **IMU の `angular_velocity.z`** を使う。
4. 車体座標の `longitudinal_velocity` / `lateral_velocity` を yaw で map へ投影し、`target ± EKF_SEED_DT_SEC` の 2 点を生成する。

```text
t0 = target - dt
T  = target（ndt_start_pose の中心）
t1 = target + dt
pose(t0) = ndt_start_pose - twist * dt
pose(t1) = ndt_start_pose + twist * dt
```

5. NDT activate 後・点群 publish **前**に、上記 2 点を `NDT_EKF_POSE_TOPIC` へ publish する。

`dt` が小さすぎると補間差分がほぼゼロ、大きすぎると seed の移動量が大きくなります。既定は **0.05 s** です。

### EKF 出力との干渉回避

tier4 の既定では、NDT の `input_initial_pose_topic` は EKF の `/localization/pose_twist_fusion_filter/biased_pose_with_covariance` と同じです。`SmartPoseBuffer` は時刻が逆順の pose を受けるとバッファを clear するため、`target - dt` / `target + dt` の seed を入れた直後に EKF が `target` 付近の pose を出すと、seed が 1 点に潰れて再び `pose_buffer_.size() < 2` になることがあります。

そのため seed 有効時は、既定で **initial pose 適用後に EKF を deactivate** し、NDT の初期 pose バッファを seed pose だけで満たします。この挙動は `EKF_PAUSE_FOR_SEED=0` で無効化できます。

さらに、NDT の subscriber discovery と callback 処理のタイミングで seed が欠けることを避けるため、既定では subscriber を最大 5 秒待ち、seed の 2 点セットを 3 回 publish してから点群を投入します。NDT は補間成功後に古い seed を `pop_old()` するため、点群をバースト publish する場合は **各点群 publish の直前**に seed を入れ直します。最後に publish される順序も `target - dt` → `target + dt` なので、NDT 側の時刻順バッファを保ちます。

---

## 環境変数（主要）

| 変数 | 説明 |
|------|------|
| `AUTOWARE_WS` | Autoware ビルドルート（既定: 実行時の `pwd`） |
| `NDT_START_POSE_YAML` | NDT 試行の開始 pose YAML（省略時は rosbag 親ディレクトリの `ndt_start_pose.yaml`） |
| `NDT_POSE_TOPIC` | 購読する pose（既定: `/localization/pose_estimator/pose_with_covariance`） |
| `POINTCLOUD_TOPIC` | bag から読む点群トピック（既定: `/sensing/lidar/concatenated/pointcloud`） |
| `MEAN_NDT_POSE_OUTPUT_DIR` | 出力ディレクトリ |
| `MEAN_NDT_POSE_YAML` | 出力 YAML パス |
| `NDT_AGG_JSON` | 出力 JSON パス |
| `GNSS_RECEIVER` | `ublox` / `septentrio`（未指定時は bag の `ros2 bag info` から推定） |
| `EXTRA_LAUNCH_ARGS` | launch コマンド末尾に追加する引数 |
| `SKIP_LAUNCH` | `1` で Autoware 起動をスキップ（手動起動済み用） |
| `TRIAL_TIMEOUT_SEC` | 1 試行あたり NDT pose 待ちのタイムアウト（既定 15） |
| `SETTLE_SEC` | 初期 pose 設定後の待ち（既定 1） |
| `CLOUD_PUBLISH_COUNT` / `CLOUD_PUBLISH_HZ` | 点群の publish 回数・レート |
| `EKF_SEED_POSE` | `1`（既定）で疑似 EKF pose 2 点を投入。`0` で無効 |
| `EKF_SEED_DT_SEC` | target 前後の時間差 [s]（既定 `0.05`） |
| `VELOCITY_TOPIC` | bag から読む車速トピック（既定 `/vehicle/status/velocity_status`） |
| `IMU_TOPIC` | 角速度 fallback 用 IMU（既定 `/sensing/imu/tamagawa/imu_raw`） |
| `NDT_EKF_POSE_TOPIC` | NDT 初期 pose 入力（既定 `/localization/pose_twist_fusion_filter/biased_pose_with_covariance`） |
| `EKF_PAUSE_FOR_SEED` | `1`（既定）で seed 投入前に EKF を停止し、NDT バッファへの干渉を防ぐ |
| `EKF_SEED_PUBLISH_REPEAT` | seed 2 点セットの publish 回数（既定 `3`） |
| `EKF_SEED_SUBSCRIBER_TIMEOUT_SEC` | seed publish 前に subscriber discovery を待つ秒数（既定 `5.0`） |
| `EKF_SEED_POST_PUBLISH_WAIT_SEC` | seed publish 後、点群投入までに待つ秒数（既定 `0.3`） |
| `MAP_TILE_PROBE` | `1`（既定）で試行中に map loader へ問い合わせ、タイル ID をログ・JSON に記録 |
| `MAP_TILE_PROBE_RADIUS` | `ndt_start_pose` 周辺のタイル問い合わせ半径 [m]（既定 `100.0`） |
| `MAP_TILE_PROBE_TIMEOUT_SEC` | map tile probe の service 待ち・呼び出し timeout [s]（既定 `10.0`） |

`launch_localization_for_ndt_measure.sh` 側では `CLOCK_HZ`（既定 10）なども利用可能です。

---

## スモークテスト（`N_RUNS=1`）

`pose_buffer_.size() < 2` が解消され、NDT pose が 1 回取れることを確認する手順です。

```bash
cd /path/to/autoware_ws
# rosbag 同階層に ndt_start_pose.yaml を配置

N_RUNS=1 AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_pose_mean.sh \
  /path/to/map \
  /path/to/rosbag.db3 \
  1722303384.244407296 \
  1
```

確認ポイント:

- launch ログに **`pose_buffer_.size() < 2` が出ない**こと
- 測定ログに `Deactivating EKF before publishing NDT seed poses` が出ること
- 測定ログに `Map tile probe after_initial_pose: ... tile(s)` が出ること
- 測定ログに `Found ... subscriber(s)` と `Publishing 2 EKF seed pose(s) ... (3 set(s))` が出ること
- 出力 JSON の `status` が `ok` で、`per_run` に 1 件、`/localization/pose_estimator/pose_with_covariance` 相当の pose が記録されること
- `ekf_seed` に使用した速度・角速度ソース・`seed_stamp_sec` が入り、`map_tile_probe` に `cell_ids` と `bbox` が入っていること

seed を切って旧挙動と比較する場合:

```bash
EKF_SEED_POSE=0 N_RUNS=1 ... ./measure_ndt_pose_mean.sh MAP BAG TARGET 1
```

---

## 注意・制限

- **NDT の「初期推定」**は、稼働後は毎スキャン **EKF の予測**などが絡むことが多く、YAML の数値がそのまま毎回 NDT の内部初期値になるわけではありません。試行ごとに EKF/NDT を deactivate して同じ経路から立ち上げ直す現在の構成は、その影響を揃える意図です。
- **`/tf_static`** が bag に無い場合、TF が足りず NDT が動かない可能性があります。
- 点群は **concatenated** を想定。sensing が立ち上がる前提（既定 `LAUNCH_SENSING=true`）。別トピックにする場合は `POINTCLOUD_TOPIC` と、sensing/remap が整合するようにしてください。
- 本ツールの平均は **「同一静的スキャンに対する NDT 応答」の統計**向きであり、連続時間の EKF 軌跡全体の GT ではありません。
- **クリーンアップ**で `kill_autoware.sh` が走るため、同一マシン上の別 ROS 2 プロセスにも影響し得ます。並行作業に注意してください。

---

## 参考: EKF pose 側の複数回放送平均

複数回放送での EKF・点群アライン付き平均は [`../sh/measure_pose_mean.sh`](../sh/measure_pose_mean.sh) と [`aggregate_pose_mean_from_bags.py`](aggregate_pose_mean_from_bags.py) を参照してください。
