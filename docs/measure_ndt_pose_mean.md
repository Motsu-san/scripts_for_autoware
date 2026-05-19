# NDT 固定初期位置・1点群での姿勢平均（`measure_ndt_pose_mean`）

`measure_pose_mean.sh` が行う複数回の bag **全再生**では、再生ごとの初期化や pose_initializer の経路の影響で **NDT への入力初期位置がばらつき**、`EKF 出力の平均` を単純に「そのスキャンの NDT 真値」とみなせない場合があります。

本ツールは次を満たすために、**bag 再生なし**で Autoware（localization を中心に）を起動し、**手動で用意した初期位置 YAML** と **指定時刻に最も近い LiDAR 点群の 1 フレーム**を繰り返し投入して、`/localization/pose_estimator/pose_with_covariance`（NDT 側）を **多数回評価し平均**します。

---

## 追加・関連ファイル

| パス | 役割 |
|------|------|
| [`sh/measure_ndt_pose_mean.sh`](../sh/measure_ndt_pose_mean.sh) | メインエントリ（起動〜試行〜集計） |
| [`sh/launch_localization_for_ndt_measure.sh`](../sh/launch_localization_for_ndt_measure.sh) | bag **再生せず** localization スタックのみ起動し、`/clock`・`/tf_static` を補助する |
| [`py/measure_ndt_pose_mean.py`](../py/measure_ndt_pose_mean.py) | bag から点群 1 フレーム抽出、試行ループ、JSON/YAML 出力 |

既存ツールとの位置づけ:

- [`measure_pose_mean.sh`](../sh/measure_pose_mean.sh): 複数回 **bag 再生**＋EKF pose などの記録後、bag 間で平均
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

AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/sh/measure_ndt_pose_mean.sh \
  /path/to/map \
  /path/to/rosbag.db3 \
  1772096549.105
```

試行回数を変える例（50 回）:

```bash
AUTOWARE_WS=$PWD ./measure_ndt_pose_mean.sh MAP BAG 1772096549.105 50
```

vehicle を sample に固定する例:

```bash
./measure_ndt_pose_mean.sh --force-sample-vehicle MAP BAG TARGET
```

**既に Autoware を手動で起動している場合**は、起動スクリプトは使わず測定のみ:

```bash
SKIP_LAUNCH=1 AUTOWARE_WS=$PWD ./measure_ndt_pose_mean.sh MAP BAG TARGET
```

（`SKIP_LAUNCH=1` 時はスクリプト側で `/clock` を publish します。`/clock` が既に正しい時刻で出ているなら、そのまま試行のみです。）

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
   - 同じ点群を短いバーストで **publish**
   - NDT の `pose_with_covariance` を **1 サンプル**取得
6. 全試行の pose を平均（位置の算術平均、姿勢は四元数平均・既存 `aggregate_pose_mean_from_bags` と同系の統計）。

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

`launch_localization_for_ndt_measure.sh` 側では `CLOCK_HZ`（既定 10）なども利用可能です。

---

## 注意・制限

- **NDT の「初期推定」**は、稼働後は毎スキャン **EKF の予測**などが絡むことが多く、YAML の数値がそのまま毎回 NDT の内部初期値になるわけではありません。試行ごとに EKF/NDT を deactivate して同じ経路から立ち上げ直す現在の構成は、その影響を揃える意図です。
- **`/tf_static`** が bag に無い場合、TF が足りず NDT が動かない可能性があります。
- 点群は **concatenated** を想定。sensing が立ち上がる前提（既定 `LAUNCH_SENSING=true`）。別トピックにする場合は `POINTCLOUD_TOPIC` と、sensing/remap が整合するようにしてください。
- 本ツールの平均は **「同一静的スキャンに対する NDT 応答」の統計**向きであり、連続時間の EKF 軌跡全体の GT ではありません。
- **クリーンアップ**で `kill_autoware.sh` が走るため、同一マシン上の別 ROS 2 プロセスにも影響し得ます。並行作業に注意してください。

---

## 参考: EKF pose 側の複数回放送平均

複数回放送での EKF・点群アライン付き平均は [`measure_pose_mean.sh`](../sh/measure_pose_mean.sh) と [`aggregate_pose_mean_from_bags.py`](../py/aggregate_pose_mean_from_bags.py) を参照してください。
