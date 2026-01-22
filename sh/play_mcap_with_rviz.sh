#!/bin/bash

# MCAP形式のROSバッグファイルを再生してrvizで可視化するスクリプト

# rviz設定ファイルのパス
RVIZ_CONFIG_FILE="$HOME/RViz_conf/mcap_visualization.rviz"

# デフォルトのバッグファイルパス
DEFAULT_BAG_FILE="$HOME/rosbag_replay/rosbag_0.db3"

# 引数からバッグファイルパスを取得、またはデフォルトを使用
BAG_FILE="${1:-$DEFAULT_BAG_FILE}"

# ワークスペースのパス
WORKSPACE_PATH=$(pwd) # run at autoware workspace directory

# ROS 2環境をセットアップ
if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    source "$WORKSPACE_PATH/install/setup.bash"
else
    echo "警告: install/setup.bash が見つかりません。"
    echo "colcon buildを実行してビルドしてください。"
    exit 1
fi

# バッグファイルの存在確認
if [ ! -f "$BAG_FILE" ]; then
    echo "エラー: バッグファイルが見つかりません: $BAG_FILE"
    echo ""
    echo "使用方法: $0 [バッグファイルのパス]"
    exit 1
fi

echo "=========================================="
echo "MCAPバッグファイルをrvizで可視化"
echo "=========================================="
echo "バッグファイル: $BAG_FILE"
echo ""

# 既存の設定ファイルを使用
if [ -f "$RVIZ_CONFIG_FILE" ]; then
    echo "既存のrviz設定ファイルを使用します: $RVIZ_CONFIG_FILE"
else
    echo "エラー: rviz設定ファイルが見つかりません: $RVIZ_CONFIG_FILE"
    echo "設定ファイルを作成してください。"
    exit 1
fi

# バッグファイルの情報を表示
echo ""
echo "=========================================="
echo "バッグファイル情報"
echo "=========================================="
if command -v ros2 &> /dev/null; then
    echo "ros2 bag info を実行中..."
    ros2 bag info "$BAG_FILE" 2>&1 | head -50

    echo ""
    echo "=========================================="
    echo "点群地図関連のトピックを確認中..."
    echo "=========================================="
    ros2 bag info "$BAG_FILE" 2>&1 | grep -i "pointcloud\|map" || echo "点群地図関連のトピックが見つかりません"
else
    echo "ros2コマンドが見つかりません"
fi

echo ""
echo "=========================================="
echo "バッグファイルを再生します"
echo "=========================================="
echo ""

# rviz2を自動起動
echo "rviz2を起動中..."
rviz2 -d "$RVIZ_CONFIG_FILE" &
RVIZ_PID=$!
sleep 3
echo "rviz2が起動しました (PID: $RVIZ_PID)"
echo ""

echo "バッグファイルを再生中..."
echo "Ctrl+Cで停止します"
echo ""

# バッグファイルを再生
ros2 bag play "$BAG_FILE" --clock

# rviz2を終了
echo ""
echo "rviz2を終了します..."
kill $RVIZ_PID 2>/dev/null || true

echo ""
echo "再生が完了しました。"
