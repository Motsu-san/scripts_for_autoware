# `run_deviation_estimator_with_bag.sh` による車速・角速度パラメータ調整手順

`sh/run_deviation_estimator_with_bag.sh` は、`deviation_estimator` を起動し、指定した rosbag を再生して、車速係数・車速標準偏差・IMU 角速度オフセット・角速度標準偏差を推定するためのラッパースクリプトです。

推定結果は rosbag ディレクトリ配下の `param_estimation_result_<YYYYMMDD_HHMMSS>/` に出力されます。

---

## 調整対象

この手順で得られる主な出力は次の 3 ファイルです。

- `vehicle_velocity_converter.param.yaml`: 車速系の推定結果
  - `speed_scale_factor`: 車速スケール係数
  - `velocity_stddev_xx`: 車速標準偏差
- `imu_corrector.param.yaml`: IMU 角速度系の推定結果
  - `angular_velocity_offset_x`
  - `angular_velocity_offset_y`
  - `angular_velocity_offset_z`
  - `angular_velocity_stddev_xx`
  - `angular_velocity_stddev_yy`
  - `angular_velocity_stddev_zz`
- `output.txt`: 推定値の収束・妥当性チェック結果

`deviation_estimator_launch.log` には、launch 中の警告やエラーが保存されます。

---

## 入力 rosbag の条件

rosbag には、少なくとも次のトピックが必要です。

- IMU: 既定 `/sensing/imu/tamagawa/imu_raw`
- 車輪速: 既定 `/vehicle/status/velocity_status`
- pose: 既定 `/localization/pose_estimator/pose_with_covariance`
- TF: `/tf` または `/tf_static`
- clock: `/clock` があればそれを使用。無い場合はスクリプトが `ros2 bag play --clock 100` を付けます

pose は NDT などの localization 結果を基準値として使います。NDT の TP / NVTL warning が多い区間や、明らかに localization が破綻している bag は避けてください。

推奨データは次のような走行です。

- 数分程度、できれば 500 m 前後の走行を含む
- 直進区間を十分に含む
- 実運用で使う速度域を含む
- 急加減速だけでなく、定速に近い区間も含む
- IMU やタイヤ交換後など、現在の車両状態に近い時期のデータ

---

## パス表記

| 表記 | 意味 |
|------|------|
| `[AUTOWARE_WS]` | Autoware の colcon ワークスペースルート（`install/local_setup.bash` があるディレクトリ） |
| `[SCRIPTS_FOR_AUTOWARE]` | 本リポジトリ（`scripts_for_autoware`）のルート |

---

## 事前準備

Autoware / deviation_estimator がビルド済みで、`install/` が存在するワークスペースを使います。

```bash
cd [AUTOWARE_WS]
test -f install/local_setup.bash
```

必要なら deviation estimator を含めてビルドします。

```bash
cd [AUTOWARE_WS]
source /opt/ros/humble/setup.bash
colcon build --packages-up-to deviation_estimator --cmake-args -DCMAKE_BUILD_TYPE=Release
```

rosbag のトピックを先に確認します。

```bash
ros2 bag info /path/to/rosbag
```

既定トピックと異なる場合は、後述の `EXTRA_LAUNCH_ARGS` と事前チェック用環境変数を合わせて指定します。

---

## 基本実行

Autoware ワークスペースのルートから実行します。

```bash
cd [AUTOWARE_WS]

[SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

別ディレクトリから実行する場合は `PILOT_AUTO_WS` に Autoware ワークスペースを指定します。

```bash
PILOT_AUTO_WS=[AUTOWARE_WS] \
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

実行が終わると、次のようなディレクトリが作られます。

```text
/path/to/rosbag/param_estimation_result_20260519_113000/
```

---

## トピック名が既定と異なる場合

`run_deviation_estimator_with_bag.sh` の `POSE_TOPIC` と `WHEEL_TOPIC` は、rosbag 事前チェック用です。`deviation_estimator.launch.xml` へ実際に渡す remap は `EXTRA_LAUNCH_ARGS` で指定します。

例: 車輪速・pose・IMU トピックを変える場合。

```bash
cd [AUTOWARE_WS]

IMU_TOPIC=/sensing/imu/imu_raw \
POSE_TOPIC=/localization/pose_estimator/pose_with_covariance \
WHEEL_TOPIC=/vehicle/status/velocity_status \
EXTRA_LAUNCH_ARGS="in_pose_with_cov_name:=/localization/pose_estimator/pose_with_covariance in_wheel_odometry:=/vehicle/status/velocity_status" \
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

IMU はスクリプト側で `in_imu` に渡しているため、`IMU_TOPIC` の指定だけで launch 側にも反映されます。

---

## 推定条件を変更する場合

既定の推定条件は `deviation_estimator.param.yaml` です。主な条件は次の通りです。

- `time_window`: 推定に使う時間窓。既定 `4.0` 秒
- `vx_threshold`: moving 判定の車速しきい値。既定 `1.5` m/s
- `wz_threshold`: straight 判定の角速度しきい値。既定 `0.01` rad/s
- `accel_threshold`: constant velocity 判定の加速度しきい値。既定 `0.3` m/s^2
- `gyro_estimation.only_use_straight`: gyro 推定に直進区間だけ使う
- `velocity_estimation.only_use_straight`: 車速推定に直進区間だけ使う
- `velocity_estimation.only_use_moving`: 車速推定に走行中区間だけ使う
- `velocity_estimation.only_use_constant_velocity`: 車速推定に定速区間だけ使う

条件を変える場合は、元の YAML をコピーして編集し、`param_path` で渡します。

```bash
cp [AUTOWARE_WS]/install/deviation_estimator/share/deviation_estimator/config/deviation_estimator.param.yaml \
  /tmp/deviation_estimator.param.yaml

# /tmp/deviation_estimator.param.yaml を編集

EXTRA_LAUNCH_ARGS="param_path:=/tmp/deviation_estimator.param.yaml" \
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

まずは既定条件で実行し、`output.txt` の `Not enough data provided yet` や `[NG]` が多い場合にだけ条件を緩めるのが安全です。

---

## 結果確認

出力ディレクトリを確認します。

```bash
ls /path/to/rosbag/param_estimation_result_*/
```

`output.txt` の例です。

```text
# Validation results
# value: [min, max]
[OK] coef_vx: [0.99538, 0.99593]
[OK] stddev_vx: [0.17192, 0.19161]
[OK] angular_velocity_offset_x: [-0.00742, -0.00727]
[OK] angular_velocity_offset_y: [-0.00119, -0.00115]
[OK] angular_velocity_offset_z: [0.00635, 0.00641]
[OK] angular_velocity_stddev_xx: [0.04151, 0.04258]
[OK] angular_velocity_stddev_yy: [0.04151, 0.04258]
[OK] angular_velocity_stddev_zz: [0.04151, 0.04258]
```

判断の目安:

- 全項目 `[OK]`: その出力を候補として採用できます
- 一部 `[NG]`: データ量、走行条件、NDT の破綻、しきい値設定を確認します
- `Not enough data provided yet`: 使用条件を満たす区間が不足しています
- launch log に `No IMU data`: IMU トピック名、bag 内容、再生タイミングを確認します
- launch log に `No wheel odometry`: 車輪速トピック名、`in_wheel_odometry` remap を確認します
- launch log に `Please publish TF base_link to ...`: `/tf_static` または IMU link と `base_link` の TF を確認します

---

## パラメータ反映

推定結果をそのまま本番設定に上書きする前に、出力値を既存設定と比較します。

車速系:

```bash
less /path/to/rosbag/param_estimation_result_YYYYMMDD_HHMMSS/vehicle_velocity_converter.param.yaml
```

IMU 角速度系:

```bash
less /path/to/rosbag/param_estimation_result_YYYYMMDD_HHMMSS/imu_corrector.param.yaml
```

反映先は使用している Autoware launch / vehicle config に依存します。基本的には、`autoware_vehicle_velocity_converter` が読む `vehicle_velocity_converter.param.yaml` と、`autoware_imu_corrector` が読む `imu_corrector.param.yaml` に、推定された値を反映します。

反映後は localization replay などで次を確認します。

- localization が安定している
- `localization_error_monitor` や関連 diagnostics が悪化していない
- EKF / NDT の姿勢が走行全体で不自然にずれない
- 停止時・低速時に IMU 角速度補正の影響で yaw が流れない

---

## 再実行・比較の進め方

1 回の結果だけで決めず、可能なら複数 bag で実行します。

```bash
for bag in /path/to/bags/*; do
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh "$bag"
done
```

比較時は、各出力ディレクトリの次の値を見ます。

- `vehicle_velocity_converter.param.yaml` の `speed_scale_factor`
- `vehicle_velocity_converter.param.yaml` の `velocity_stddev_xx`
- `imu_corrector.param.yaml` の `angular_velocity_offset_*`
- `imu_corrector.param.yaml` の `angular_velocity_stddev_*`
- `output.txt` の `[OK]` / `[NG]`

bag ごとに値が大きくばらつく場合は、車両状態の違い、NDT 品質、走行条件、トピックの取り違えを疑います。

---

## よくあるトラブル

### `rosbag に deviation_estimator 用トピックが無い`

localization だけを再記録した bag では、IMU、車輪速、TF が欠けていることがあります。元の sensing / vehicle / TF を含む bag を使うか、別 bag を同時再生して不足トピックを補います。

欠落をエラー扱いにしたい場合:

```bash
STRICT_BAG_TOPICS=1 \
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

### `/clock` が二重になる

bag に `/clock` がある場合、スクリプトは `ros2 bag play --clock` を付けません。bag に `/clock` が無い場合だけ `--clock ${PLAY_CLOCK_HZ}` を付けます。必要なら次のように変更できます。

```bash
PLAY_CLOCK_HZ=40 \
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

### `No wheel odometry` が出続ける

`WHEEL_TOPIC` だけを変えても、launch の remap は変わりません。必ず `EXTRA_LAUNCH_ARGS` に `in_wheel_odometry:=...` を入れます。

```bash
WHEEL_TOPIC=/my/vehicle/status \
EXTRA_LAUNCH_ARGS="in_wheel_odometry:=/my/vehicle/status" \
  [SCRIPTS_FOR_AUTOWARE]/sh/run_deviation_estimator_with_bag.sh \
  /path/to/rosbag
```

### `[NG]` または `Not enough data provided yet` が残る

まず bag の走行条件を確認します。直進・走行中・定速区間が少ない場合、既定条件では採用される trajectory が不足します。

条件を緩める例:

```yaml
/**:
  ros__parameters:
    gyro_estimation:
      only_use_straight: true
      only_use_moving: false
      only_use_constant_velocity: false
    velocity_estimation:
      only_use_straight: true
      only_use_moving: true
      only_use_constant_velocity: false
```

条件を緩めた結果は、必ず別 bag や replay で検証してから採用してください。
