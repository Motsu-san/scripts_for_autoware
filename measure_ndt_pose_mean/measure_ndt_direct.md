# Direct NDT 測定（`measure_ndt_direct`）

`convergence_evaluator` と同様、Autoware の NDT scan matcher **ノードを起動せず**、bag から読んだ点群と `ndt_start_pose.yaml` を入力に `MultiGridNormalDistributionsTransform::align()` を直接実行します。

rosbag の再記録は不要です（既存 bag + 既存 `ndt_start_pose.yaml` + PCD map）。

---

## 関連ファイル

| パス | 役割 |
|------|------|
| `/home/motsu/autoware/src/tools/localization/ndt_direct_measure/` | C++ 実行体（`ndt_direct_measure_node`） |
| [`measure_ndt_direct.sh`](measure_ndt_direct.sh) | エントリスクリプト |
| [`measure_ndt_pose_mean.sh`](measure_ndt_pose_mean.sh) | N 回実行 + 平均・ばらつき集計ラッパ |

---

## `measure_ndt_pose_mean` との違い

| 項目 | `measure_ndt_direct` | `measure_ndt_pose_mean` |
|------|----------------------|-------------------------|
| 役割 | raw align 実行 | N 回実行 + 平均・ばらつき集計 |
| 既定 `N_RUNS` | 1 | 100 |
| 出力 | raw JSON/CSV | raw + 集計 JSON + `mean_ndt_pose.yaml` |
| 実装 | 本スクリプトが node を直接起動 | 本スクリプトを呼び、続けて集計 |

**注意:** direct 方式は車載パイプライン（crop_box、dynamic map、EKF 予測初期値）を通しません。NDT アルゴリズムの挙動確認・切り分け用です。

---

## ビルド（初回のみ）

パッケージは Autoware tools 配下の `src/tools/localization/ndt_direct_measure` にあります。

```bash
cd /path/to/autoware_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ndt_direct_measure --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`measure_ndt_direct.sh` は `BUILD_IF_MISSING=1`（既定）のとき、実行体が無ければ自動ビルドします。

---

## 使い方

```bash
cd /path/to/autoware_ws
# rosbag と同じ階層に ndt_start_pose.yaml を置く

AUTOWARE_WS=$PWD /path/to/scripts_for_autoware/measure_ndt_pose_mean/measure_ndt_direct.sh \
  /path/to/map/pointcloud_map.pcd \
  /path/to/rosbag.db3 \
  1722303384.244407296
```

出力（既定）: `rosbag` 親ディレクトリ配下の `ndt_direct_<TARGET>_<日時>/`

- `ndt_direct_<TARGET>.json`
- `ndt_direct_<TARGET>.csv`

---

## 環境変数

| 変数 | 説明 |
|------|------|
| `AUTOWARE_WS` | Autoware ビルドルート |
| `NDT_START_POSE_YAML` | 初期 pose YAML（省略時は bag 同階層） |
| `POINTCLOUD_TOPIC` | bag から読む点群（既定: `/sensing/lidar/concatenated/pointcloud`） |
| `NDT_PARAM_YAML` | NDT パラメータ YAML（省略時は `ndt_direct_measure` 付属、`max_iterations=100`） |
| `N_RUNS` | 同一入力での align 回数（既定 `1`） |
| `MAP_LOAD_MODE` | `all` または `metadata_radius`（既定） |
| `MAP_RADIUS_M` | metadata モードの半径 [m]（既定 `150`） |
| `MAP_METADATA_YAML` | 省略時: `<MAP_PATH>/pointcloud_map_metadata.yaml`（無ければ `<MAP_PATH>` の親） |
| `NDT_DIRECT_OUTPUT_DIR` | 出力ディレクトリ |
| `BUILD_IF_MISSING` | `1` で未ビルド時に colcon build |

---

## 出力の読み方

JSON / CSV の各 run に次が含まれます。

| フィールド | 意味 |
|------------|------|
| `has_converged` | NDT ライブラリが `trans_epsilon` 未満で早期終了したか（`iteration < max_iterations`。上限到達時は false） |
| `passes_score_threshold` | Autoware NDT param の score 閾値を満たすか |
| `score_nvtl` | Nearest Voxel Transformation Likelihood |
| `score_tp` | Transformation Probability |
| `scan_matching_pose` | align 後の pose（map 座標） |
| `iteration` | 反復回数 |

**指定時刻点群の収束結果**は、`pointcloud_header.stamp` が target に最も近い 1 フレームに対する上記フィールドで判断します。

---

## 検証の目安

1. `passes_score_threshold: true` かつ `has_converged: true` → NDT ライブラリとしては良好
2. 失敗する場合 → 初期 pose / 点群 frame / map 範囲 / param を確認
3. 平均・ばらつきまで欲しい場合は [`measure_ndt_pose_mean.sh`](measure_ndt_pose_mean.sh) を使う

---

## 制限

- 点群は `PointXYZ` に変換（強度・ring は未使用）。`convergence_evaluator` と同じ。
- 点群 `frame_id` が `base_link` の場合、車載と同様「初期 pose = map 上の車両姿勢」として align します（crop_box 等の前処理は未実施）。
- `metadata_radius` モードでは `MAP_PATH` またはその親にある `pointcloud_map_metadata.yaml` が必要です。
