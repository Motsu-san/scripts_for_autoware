# scripts_for_autoware

Autoware の開発・評価（主に Localization 周り）で使う自作スクリプト集。

rosbag を再生した Localization のリプレイ・評価、NDT 姿勢のばらつき測定、オドメトリ較正、rosbag / 点群の各種変換など、開発中に繰り返し使うツールをまとめている。

> 各ツールの細かい使い方は、それぞれのディレクトリ内の README / ドキュメント（下表のリンク）を参照。ここではリポジトリ全体の概要のみを示す。

## ディレクトリ構成

| パス | 概要 |
|------|------|
| [`launch_replay_localization/`](launch_replay_localization/README.md) | 録画済み rosbag を再生して Localization（NDT / EKF）をリプレイ・評価する `launch_autoware.sh` 一式 |
| [`launch_unified_localization/`](launch_unified_localization/) | `unified_localization`（NDT+EKF を 1 ノード化）専用のランチスクリプト |
| [`measure_ndt_pose_mean/`](measure_ndt_pose_mean/) | direct NDT を複数回実行し、姿勢の平均・ばらつきを測定するツール |
| [`compare_mean_pose/`](compare_mean_pose/) | 測定した平均姿勢 YAML 同士を比較 |
| [`fine_tune_odom_calibration/`](fine_tune_odom_calibration/) | rosbag からオドメトリ（車速・角速度）較正値を推定・適用 |
| [`cpp/ndt_direct_measure/`](cpp/ndt_direct_measure/) | direct NDT align を直接実行する C++ パッケージ（上記測定ツールが利用） |
| [`py/`](py/) | rosbag / 点群 / 診断まわりの Python ユーティリティ群 |
| [`sh/`](sh/) | rosbag マージ・再生・DLR 実行・ビルドなどの補助シェルスクリプト群 |
| [`docs/`](docs/) | 手順ドキュメント |

## 主なツール

### Localization リプレイ・評価

| ツール | 説明 |
|--------|------|
| [`launch_replay_localization/`](launch_replay_localization/README.md) | rosbag 再生 → Autoware 起動 → 初期位置設定 → 出力録画 / RViz 録画までを統括 |
| [`sh/launch_unified_localization.sh`](sh/launch_unified_localization.sh) | `unified_localization` 専用ランチ |
| [`docs/run_deviation_estimator_with_bag.md`](docs/run_deviation_estimator_with_bag.md) | rosbag 再生による車速・角速度パラメータ推定手順 |
| [`sh/batch_replay.sh`](sh/batch_replay.sh) / [`sh/batch_compare_localization.sh`](sh/batch_compare_localization.sh) | 複数構成をまとめて再生・比較 |
| [`sh/DLR_command*.sh`](sh/) / [`sh/download_dlr_results.sh`](sh/download_dlr_results.sh) | `driving_log_replayer` の実行・結果ダウンロード |

### NDT 姿勢測定・比較

| ツール | 説明 |
|--------|------|
| [`measure_ndt_pose_mean/measure_ndt_pose_mean.md`](measure_ndt_pose_mean/measure_ndt_pose_mean.md) | direct NDT を N 回実行し平均 pose・ばらつきを出力 |
| [`measure_ndt_pose_mean/measure_ndt_direct.md`](measure_ndt_pose_mean/measure_ndt_direct.md) | `convergence_evaluator` 方式の direct NDT align（1 時刻・1 点群） |
| [`compare_mean_pose/`](compare_mean_pose/) | 平均姿勢 YAML 同士の差分比較 |

### オドメトリ較正

| ツール | 説明 |
|--------|------|
| [`fine_tune_odom_calibration/`](fine_tune_odom_calibration/) | rosbag から車速係数・角速度オフセット等を推定し設定へ適用 |

### rosbag ユーティリティ（`py/`, `sh/`）

| ツール | 説明 |
|--------|------|
| [`py/filter_rosbag_exclude_topics.py`](py/filter_rosbag_exclude_topics.py) | 指定トピックを除外した rosbag2 の複製を作成 |
| [`py/merge_rosbag_topic.py`](py/merge_rosbag_topic.py) | 別 rosbag から特定トピックをリマップして追加 |
| [`py/inject_tf_static_from_bag.py`](py/inject_tf_static_from_bag.py) | 別 bag から `/tf_static` を取り出し先頭時刻に合わせて注入 |
| [`py/rewrite_bag_timestamps.py`](py/rewrite_bag_timestamps.py) | `header.stamp` を bag のタイムスタンプで上書き |
| [`py/convert_pointcloud_type.py`](py/convert_pointcloud_type_README.md) | PointCloud2 レイアウト変換（Autoware pointcloud_preprocessor 互換） |
| [`py/extract_cropped_pointcloud.py`](py/extract_cropped_pointcloud_README.md) | 指定ボックス領域で点群をクロップして抽出 |
| [`sh/merge_rosbag.sh`](sh/merge_rosbag.sh) / [`sh/merge_many_rosbag.sh`](sh/merge_many_rosbag.sh) | rosbag のマージ |
| [`sh/merge_pcd_maps.sh`](sh/merge_pcd_maps.sh) | PCD 地図のマージ |
| [`sh/play_mcap_with_rviz.sh`](sh/play_mcap_with_rviz.sh) | MCAP を再生して RViz で可視化 |

### 診断・チェック（`py/`, `sh/`）

| ツール | 説明 |
|--------|------|
| [`py/check_ekf_diagnostics_frequency.py`](py/check_ekf_diagnostics_frequency.py) | EKF localizer の diagnostics publish 頻度をチェック |
| [`py/check_ekf_diagnostics_stamp_delta.py`](py/check_ekf_diagnostics_stamp_delta.py) | `/diagnostics` の stream ごとの stamp 間隔を表示 |
| [`py/compare_localization_diff.py`](py/compare_localization_diff.py) | Localization 結果の差分比較 |
| [`py/timestamp_checker.py`](py/timestamp_checker.py) / [`sh/timestamp_checker.sh`](sh/timestamp_checker.sh) | トピックのタイムスタンプ確認 |
| [`sh/check_direct_auto_conflict.sh`](sh/check_direct_auto_conflict.sh) | DIRECT / AUTO モードの同時実行を検出 |
| [`sh/search_ndt_crash_detailed.sh`](sh/search_ndt_crash_detailed.sh) | `ndt_scan_matcher` クラッシュの詳細検索 |

### その他

| ツール | 説明 |
|--------|------|
| [`py/extract_vehicle_configs.py`](py/extract_vehicle_configs.py) | launch ログから車両モデル / ID / センサモデルの組を抽出 |
| [`py/make_pointcloud_map_metadata.py`](py/make_pointcloud_map_metadata.py) | 点群地図の metadata 生成 |
| [`py/pose_publisher.py`](py/pose_publisher.py) | YAML の固定 pose を可視化用に publish |
| [`sh/build_commands.sh`](sh/build_commands.sh) | リポジトリ管理・ビルド用コマンド集 |
| [`sh/iv_to_auto.sh`](sh/iv_to_auto.sh) | AutowareIV → Autoware への変換補助 |
