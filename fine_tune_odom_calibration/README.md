# fine_tune_odom_calibration

NDT で求めた真値姿勢（`mean_ndt_pose.yaml`）を基準に、**オドメトリのみ走行**の rosbag からデッドレコニングの誤差を測定し、`speed_scale_factor`（車速）と `angular_velocity_offset_z`（ヨーレートバイアス）の補正値を推定・適用するツール群。

「rosbag 再生＆録画 → 補正値計算 → param 更新」を自動で反復し、位置誤差が閾値以下に収束するまで較正を追い込む。

## 仕組み

`initial_pose` の時刻を積分始点、`mean_ndt_pose` の `target_unix_sec` を終点として、その区間の走行を評価する。

- **speed_scale_factor** — 終点の NDT 車体座標での**縦位置誤差 ÷ 走行距離**から推定。
- **angular_velocity_offset_z** — 開始点方位角差 ÷ 区間時間（`start_bearing_geometry`）から推定。曲線走行でも過補正しにくい方針。

車輪積分・heading_rate 積分・endpoint_yaw は診断表示のみで、補正値には使わない。

## 構成

| ファイル | 役割 |
|---|---|
| [`fine_tune_odom_calibration.sh`](fine_tune_odom_calibration.sh) | **メインの反復ループ**。launch → calibrate → apply を最大 N 回繰り返す |
| [`calibrate_odom_from_bag.sh`](calibrate_odom_from_bag.sh) / [`calibrate_odom_from_bag.py`](calibrate_odom_from_bag.py) | rosbag から補正値を計算し `*_odom_calibration.yaml` を出力 |
| [`apply_odom_calibration.py`](apply_odom_calibration.py) | 計算結果を `vehicle_velocity_converter.param.yaml` / `imu_corrector.param.yaml` に反映 |

依存: 同リポジトリの [`launch_replay_localization/`](../launch_replay_localization/)（`launch_autoware.sh`）、[`measure_ndt_pose_mean/`](../measure_ndt_pose_mean/)、[`compare_mean_pose/`](../compare_mean_pose/)。

## 前提

- ROS 2 / Autoware をビルドして source 済みであること（rosbag の `velocity_status` デシリアライズに必要）。

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/autoware/install/setup.bash
  ```
- Python: `numpy`, `scipy`, `PyYAML`。
- 較正対象区間の真値 `mean_ndt_pose.yaml`（[`measure_ndt_pose_mean/`](../measure_ndt_pose_mean/) で作成）と、積分始点の `initial_pose.yaml`。

## 使い方（反復ループ）

Autoware のビルド済みワークスペースをカレントディレクトリにして実行する。

```bash
cd ~/autoware
~/scripts_for_autoware/fine_tune_odom_calibration/fine_tune_odom_calibration.sh \
  <MAP_PATH> <SOURCE_ROSBAG> \
  --mean-ndt-pose-yaml <mean_ndt_pose.yaml>
```

1 反復あたりの流れ:

1. `launch_autoware.sh` で `SOURCE_ROSBAG` を再生（`POSE_SOURCE_ID=99` = オドメトリのみ）し、出力を `record_replay_*` として録画。
2. 録画 bag から補正値を計算（`iter_NN_calibration.yaml`）。
3. 縦位置誤差・横位置誤差が閾値以下なら**収束**として終了（exit 0）。
4. 未収束なら param YAML を更新し、install 側へ同期・値を検証してから次の反復へ。

再生区間は `initial_pose` 時刻から `target_unix_sec` を整数秒に切り上げた時刻まで（較正区間の終端を確実に含めるため）。

### 主な引数・オプション

| 引数 / オプション | 説明 |
|---|---|
| `MAP_PATH`（位置引数） | 地図ディレクトリ（必須） |
| `SOURCE_ROSBAG`（位置引数） | 再生元の rosbag（必須） |
| `--mean-ndt-pose-yaml <yaml>` | 真値の平均 NDT 姿勢 YAML（必須） |
| `--max-iterations N` | 最大反復回数（既定: 10） |
| `--rate R` | 再生速度（既定: 1.0） |
| `--convergence-lat-threshold M` | 横位置誤差の収束閾値 [m]（既定: 0.10） |
| `--convergence-lon-threshold M` | 縦位置誤差の収束閾値 [m]（既定: 0.10） |
| `--initial-pose-yaml PATH` | 積分始点（省略時は `dirname(SOURCE_ROSBAG)/initial_pose.yaml`） |
| `--pose-topic TOPIC` | 較正に使う pose トピック（既定: EKF の `biased_pose_with_covariance`） |
| `--vehicle-velocity-param-yaml` / `--imu-corrector-param-yaml` | 更新対象の param YAML（省略時は WS 内 `sample_sensor_kit` から自動解決） |
| `--force-sample-vehicle` / `--no-force-sample-vehicle` | sample_vehicle の付与（既定は WS 名で自動判定。`autoware` の時のみ付与） |
| `--resume` / `--reset` | 状態ファイルから再開 / 削除して最初から |
| `--abort-on-divergence` | 位置誤差が 2 連続で悪化したら停止 |
| `--dry-run` | launch / apply をスキップ（`SKIP_LAUNCH=1` 時は calibrate のみ） |

### 環境変数

| 変数 | 説明 |
|---|---|
| `AUTOWARE_WS` | Autoware ワークスペース（既定: カレントディレクトリ） |
| `GNSS_RECEIVER` | sensing の GNSS プリセット（`ublox` / `septentrio`）。既定 `septentrio` なので `ublox` の場合のみ指定 |
| `POSE_TOPIC` | 較正 pose トピック（`--pose-topic` と同義） |
| `EXTRA_LAUNCH_ARGS` | `launch_autoware.sh` に渡す追加引数（`-t` / `-T` はこちらが優先） |
| `SKIP_LAUNCH=1` | launch をスキップし既存の録画 bag で calibrate のみ実行 |
| `RECORD_BAG=...` | 使用する録画 bag を明示指定 |

### 状態・出力

`dirname(SOURCE_ROSBAG)/fine_tune_state/` 以下に生成される。

- `state.tsv` — 各反復のメタ情報と結果（lat / lon / yaw / sf / bias_z / converged）。`--resume` はこれを読んで続きから再開する。
- `iter_NN_calibration.yaml` — 各反復の較正結果。

## 単発での補正値計算

反復ループを使わず、1 本の rosbag から補正値だけ計算したい場合:

```bash
./calibrate_odom_from_bag.sh <mean_ndt_pose.yaml> <odom_rosbag> \
  --initial-pose-yaml <initial_pose.yaml> \
  --vehicle-velocity-param-yaml <vehicle_velocity_converter.param.yaml> \
  --imu-corrector-param-yaml <imu_corrector.param.yaml> \
  --yaml-out calibration.yaml
```

主なオプション（`calibrate_odom_from_bag.py`）:

| オプション | 説明 |
|---|---|
| `--initial-pose-yaml PATH` | 積分始点の pose YAML（必須） |
| `--start-unix-sec` / `--target-unix-sec` | 積分区間の開始 / 終了 UNIX 時刻（省略時は YAML から解決） |
| `--pose-topic` / `--velocity-topic` / `--imu-topic` | 読み取りトピック |
| `--wheel-rosbag PATH` | 車速・角速度を別 bag から読む（録画 bag に velocity が無い場合） |
| `--individual-params-root PATH` | rosbag パスから vehicle を推定し param YAML を自動解決 |
| `--correction-mode` | `both` / `yaw_only` / `speed_only` |
| `--yaw-bias-method` | `start_bearing_geometry`（既定） / `endpoint_yaw`（診断用） / `auto` |
| `--convergence-lat-threshold` / `--convergence-lon-threshold` | 収束判定の閾値 [m] |
| `--check-convergence` | 収束時 exit 0、未収束 exit 4 |
| `--yaml-out` / `--json-out` | 結果の出力先 |

## 補正値の適用のみ

計算済みの `*_odom_calibration.yaml` を param YAML に反映する:

```bash
python3 apply_odom_calibration.py <calibration.yaml> \
  --vehicle-velocity-param-yaml <vehicle_velocity_converter.param.yaml> \
  --imu-corrector-param-yaml <imu_corrector.param.yaml> \
  [--correction-mode both|yaw_only|speed_only] [--no-backup]
```

初回適用時に param YAML のバックアップ（`*.bak_fine_tune_<timestamp>`）を作成する（`--no-backup` で無効化）。
