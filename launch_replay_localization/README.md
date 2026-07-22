# launch_replay_localization

録画済み rosbag を再生して Autoware の Localization（NDT / EKF）をリプレイ・評価するためのスクリプト群。

Autoware を起動し、rosbag を再生しながら初期位置を設定し、必要に応じて出力トピックを録画・RViz を録画する、といった一連の作業をまとめて実行する。

## 前提

- カレントディレクトリを Autoware のビルド済みディレクトリ（`install/setup.bash` が存在する場所。例: `$HOME/autoware`）にした状態で実行すること。
- `vehicle_hash_map.yaml` が必要（後述）。
- RViz 録画には `xdotool` と `ffmpeg`、GNSS を初期位置に使う場合は該当トピックの録画が必要。

## 基本的な使い方

```bash
cd ~/autoware   # Autoware ビルド済みディレクトリ
~/scripts_for_autoware/launch_replay_localization/launch_autoware.sh <MAP_PATH> <ROSBAG_PATH>
```

例:

```bash
./launch_autoware.sh "$HOME/autoware_map/sample-map" "$HOME/rosbag_replay/rosbag_0.db3"
```

### 主な引数

| 位置引数 | 説明 |
|---|---|
| `MAP_PATH` | 地図ディレクトリ（必須） |
| `ROSBAG_PATH` | 再生する rosbag（必須） |
| `POSE_SOURCE_ID` | `0`=ndt（デフォルト） / `1`=ndt+lidar-marker / `99`=odometry only |
| `SAVE_LAUNCH_LOG` | `true` で ros2 launch ログを保存 |
| `TOPIC_TYPE` | 録画するトピックセット（省略時は録画しない） |

### 主なオプション

| オプション | 説明 |
|---|---|
| `-t TIME` | 再生開始時刻（UNIX 時刻 または JST `'2026-02-26 12:34:56'`）。省略時は `initial_pose.yaml` の時刻を使用 |
| `-T, --end-time TIME` | 再生終了時刻 |
| `--rate RATE` | 再生速度（デフォルト 0.2） |
| `--compare-bag PATH` | 比較用の録画済み rosbag |
| `--compare-topics ...` | 比較用 rosbag から再生するトピック |
| `--force-sample-vehicle` | bag パス検出を無視して sample_vehicle を使用 |
| `--gnss-receiver NAME` | GNSS プリセット（`ublox` / `septentrio`） |
| `--record-rviz` | RViz 画面を録画 |

引数の詳細は `./launch_autoware.sh` を引数なしで実行するとヘルプが表示される。

## `TOPIC_TYPE`（録画トピックセット）

`record_rosbag_localization_replay.sh` が定義するトピックのプリセット。用途に応じて選択する。

- `default` — Localization リプレイ入力（concatenated pointcloud / imu / velocity / gnss / tf_static）
- `concatenated_only` — concatenated pointcloud のみ中心
- `calibration` — オドメトリ調整（`POSE_SOURCE_ID=99`）用
- `lidar-marker_replay` / `full-sensing_replay` — lidar-marker / フルセンシング構成のリプレイ入力
- `output` / `output_pose_mean` / `output_lidar-marker` — Localization 出力（評価用）
- `convergence` — 収束・変動評価用
- `occlusion` — オクルージョン付加用
- `output_localization_evaluation_scrpts` — [autoware_localization_evaluation_scripts](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/autoware_localization_evaluation_scripts) 向け

## ファイル構成

| ファイル / ディレクトリ | 役割 |
|---|---|
| `launch_autoware.sh` | メインスクリプト。引数解析 → 車両検出 → 起動 → 再生 → 録画 → 終了を統括 |
| `record_rosbag_localization_replay.sh` | `TOPIC_TYPE` に応じたトピックを `ros2 bag record` |
| `capture_rviz_display.sh` | RViz 画面を `ffmpeg` で mp4 録画 |
| `kill_autoware.sh` | Autoware プロセスの停止（`../sh/kill_autoware.sh` に委譲） |
| `vehicle_configs.sh` | `vehicle_hash_map.yaml` を読み込み、rosbag パスから車両を検出 |
| `vehicle_hash_map.yaml` | 車両 ID フラグメント → vehicle_model / vehicle_id / sensor_model のマッピング |
| `vehicle_hash_map_sample.yaml` | 上記のサンプル。コピーして自分の車両を追加する |
| `lib/` | `launch_autoware.sh` が source する処理単位ごとのモジュール群 |
| `py/` | 初期位置設定・GNSS 初期位置・複数 bag 再生などの Python ヘルパー |

### `py/` ヘルパー

- `set_initial_pose.py` — ROS 2 トピック経由で初期位置を設定（引数 or YAML）
- `gnss_to_initial_pose.py` — GNSS の最初の pose を初期位置として publish
- `prime_bag_playback.py` — `-t` 指定時に一時停止 → sim time 合わせ → 初期位置 → resume
- `play_multiple_rosbags.py` — 複数 rosbag の同時再生（`--compare-bag` 用）
- `parse_vehicle_configs.py` — `vehicle_hash_map.yaml` を bash 向けに変換（`vehicle_configs.sh` が使用）

## 車両設定

`vehicle_hash_map.yaml` で rosbag パスに含まれる ID フラグメントから車両モデル等を自動判定する。初回は sample をコピーして自分の車両を追記する:

```bash
cp vehicle_hash_map_sample.yaml vehicle_hash_map.yaml
```

## Sample rosbag 再生時の地図チラつきについて

use_sim_time + rosbag の `--clock` で再生すると、RViz が「Detected jump back in time」を検出するたびに表示をリセットし、地図などが激しくチラつくことがある。

**原因**: tf2 が「現在より過去のタイムスタンプ」のメッセージを受信すると時間逆行と判断し、RViz はその都度リセットする。`--clock` の高頻度 publish やバッグ内メッセージの届く順序により、再生中に何千回も検出されることがある。

**スクリプト側の対策**（`launch_autoware.sh`）:
- sample rosbag 時は再生を**先に開始**し、/clock を流してから launch。
- `--clock 40`（40Hz）、`--read-ahead-queue-size 5000` でメッセージ順の安定化を図る。
- **sample rosbag 時は RViz を launch に含めない**（チラつき防止）。起動時に表示されるコマンドを別ターミナルで実行すると RViz を表示できる。従来どおり launch に含めたい場合は `SAMPLE_ROSBAG_LAUNCH_RVIZ=true` を付けて実行。

**RViz を別ターミナルで起動する例**（表示されたコマンドをコピーして実行）:
```bash
source <autoware_install>/install/setup.bash && ros2 run rviz2 rviz2 -d <autoware_install>/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz --ros-args -p use_sim_time:=true
```

**従来どおり launch に RViz を含める（チラつきは出る）**:
```bash
SAMPLE_ROSBAG_LAUNCH_RVIZ=true ./launch_autoware.sh <MAP_PATH> <ROSBAG_PATH> ...
```
