# NDT 固定初期位置・1点群での姿勢平均（`measure_ndt_pose_mean`）

`measure_ndt_pose_mean.sh` は `ndt_direct_measure` で direct NDT align を `N_RUNS` 回実行し、
平均 pose・ばらつき・平均 pose からの最大偏差を出力します。

Autoware localization ノードの起動は不要です。direct 方式の詳細は
[`measure_ndt_direct.md`](measure_ndt_direct.md) を参照してください。

`measure_pose_mean.sh` が行う複数回の bag **全再生**では、再生ごとの初期化や
pose_initializer の経路の影響で **NDT への入力初期位置がばらつき**、
`EKF 出力の平均` を単純に「そのスキャンの NDT 真値」とみなせない場合があります。

本ツールは **単一センサーフレーム × 固定初期 pose** で NDT アルゴリズム応答の
ばらつきを評価・平均するためのものです。

---

## 関連ファイル

| パス | 役割 |
|------|------|
| [`measure_ndt_pose_mean.sh`](measure_ndt_pose_mean.sh) | メインエントリ（direct NDT 実行〜集計） |
| [`measure_ndt_direct.sh`](measure_ndt_direct.sh) | `ndt_direct_measure_node` 起動ラッパ |
| [`aggregate_ndt_direct_result.py`](aggregate_ndt_direct_result.py) | N 回結果の平均・ばらつき集計 |
| [`aggregate_pose_mean_from_bags.py`](aggregate_pose_mean_from_bags.py) | 平均 pose YAML / 偏差統計ユーティリティ |
| `~/autoware/src/tools/localization/ndt_direct_measure/` | C++ 実行体 |

既存ツールとの位置づけ:

- [`../sh/measure_pose_mean.sh`](../sh/measure_pose_mean.sh): 複数回 **bag 再生**＋EKF pose などの記録後、bag 間で平均
- 本ツール: **単一センサーフレーム×固定初期 pose** で NDT 出力のばらつきを評価・平均
- [`measure_ndt_direct.sh`](measure_ndt_direct.sh): 本ツールが内部で呼ぶ raw align 実行（既定 `N_RUNS=1`）

---

## 既定の前提

| 項目 | 既定・推奨 |
|------|-----------|
| **試行回数** | **3**（`-n` / `--n-runs` で変更可） |
| **TARGET_UNIX_SEC** | 第 3 引数。省略時は `ndt_start_pose.yaml` の `header.stamp`（`sec.nanosec`）を使用 |
| **ndt_start_pose.yaml** | **ソース rosbag と同じディレクトリ**<br>例: `.../final_merged/final_merged_0.db3` と同じ階層の `ndt_start_pose.yaml`<br>（ディレクトリ形式の bag はそのディレクトリ直下）<br>bag 再生用の `initial_pose.yaml` とは別ファイル |
| **点群トピック** | `/sensing/lidar/concatenated/pointcloud`（環境変数で変更可） |

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

`TARGET_UNIX_SEC`（第 3 引数）は省略可能。省略時は `ndt_start_pose.yaml` の
`header.stamp` を使います:

```bash
AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_pose_mean.sh \
  /path/to/map \
  /path/to/rosbag.db3
```

試行回数を変える例（50 回）:

```bash
AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_pose_mean.sh \
  -n 50 MAP BAG 1772096549.105
```

---

## 出力

出力先の既定は **rosbag と同じ親ディレクトリ** 配下:

`mean_ndt_pose_direct_<TARGET>_n<N>_<日時>/`

- `ndt_direct_runs_<TARGET>_n<N>.json` / `.csv` — direct 実行体の raw 結果
- `ndt_pose_mean_<TARGET>_n<N>.json` — 集計結果
- `mean_ndt_pose.yaml` — 平均 pose YAML（集計メタデータ付き）

ディレクトリやファイル名は環境変数で上書き可能（後述）。

---

## 処理の流れ（概要）

1. `measure_ndt_direct.sh` で `ndt_direct_measure_node` を起動し、指定時刻近傍の点群と
   `ndt_start_pose.yaml` に対して `N_RUNS` 回 `align()` する。
2. `aggregate_ndt_direct_result.py` で平均 pose・ばらつき・最大偏差を集計し、
   JSON / `mean_ndt_pose.yaml` を書き出す。

---

## 環境変数（主要）

| 変数 | 説明 |
|------|------|
| `AUTOWARE_WS` | Autoware ビルドルート（既定: 実行時の `pwd`） |
| `NDT_START_POSE_YAML` | NDT 試行の開始 pose YAML（省略時は rosbag 親ディレクトリの `ndt_start_pose.yaml`） |
| `POINTCLOUD_TOPIC` | bag から読む点群トピック（既定: `/sensing/lidar/concatenated/pointcloud`） |
| `MEAN_NDT_POSE_OUTPUT_DIR` | 出力ディレクトリ |
| `MEAN_NDT_POSE_YAML` | 出力 YAML パス |
| `NDT_AGG_JSON` | 集計 JSON パス |
| `NDT_DIRECT_RAW_JSON` / `NDT_DIRECT_RAW_CSV` | direct 実行体の raw 出力 |
| `NDT_PARAM_YAML` | NDT パラメータ YAML |
| `MAP_LOAD_MODE` | `all` または `metadata_radius`（既定） |
| `MAP_RADIUS_M` | metadata モードの半径 [m]（既定 `150`） |
| `MAP_METADATA_YAML` | `pointcloud_map_metadata.yaml` |
| `NEIGHBOR_SCANS` | 最近傍の前後に align するスキャン数（既定 `2`） |
| `BUILD_IF_MISSING` | `1`（既定）で未ビルド時に colcon build |

---

## スモークテスト（`-n 1`）

```bash
cd /path/to/autoware_ws
# rosbag 同階層に ndt_start_pose.yaml を配置

AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_pose_mean.sh \
  -n 1 \
  /path/to/map \
  /path/to/rosbag.db3 \
  1722303384.244407296
```

確認ポイント:

- raw JSON / CSV が出力されること
- 集計 JSON と `mean_ndt_pose.yaml` が出力されること
- JSON に平均 pose とばらつき指標が入っていること

---

## 注意・制限

- direct 方式は車載パイプライン（crop_box、dynamic map、EKF 予測初期値）を通しません。
- 本ツールの平均は **「同一静的スキャンに対する NDT 応答」の統計**向きであり、連続時間の EKF 軌跡全体の GT ではありません。
- 点群は **concatenated** を想定。別トピックにする場合は `POINTCLOUD_TOPIC` を指定してください。

---

## 参考: EKF pose 側の複数回放送平均

複数回放送での EKF・点群アライン付き平均は [`../sh/measure_pose_mean.sh`](../sh/measure_pose_mean.sh) と [`aggregate_pose_mean_from_bags.py`](aggregate_pose_mean_from_bags.py) を参照してください。
