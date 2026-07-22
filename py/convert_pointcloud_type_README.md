# PointCloud2型変換スクリプトの使い方

## 依存パッケージ
- ROS2 Humble以降
- Pythonパッケージ: numpy, rosbag2_py, rclpy, sensor_msgs

インストール例:
```
source /opt/ros/humble/setup.bash
pip install numpy
# rosbag2_py, rclpy, sensor_msgsはROS2パッケージに含まれます
```

## 使い方
```
python3 tools/convert_pointcloud_type.py <入力rosbagディレクトリ> <出力rosbagディレクトリ> <変換対象トピック名>
```
例:
```
python3 tools/convert_pointcloud_type.py my_old_bag my_new_bag /sensing/lidar/front_lower/rectified/pointcloud_ex
```

- 指定したトピックのPointCloud2メッセージのみ型変換されます
- 他のトピックはそのままコピーされます
- 変換後のrosbagをAutoware Universeでリプレイしてください

## 注意
- 入力rosbagはrosbag2形式（sqlite3）である必要があります
- 変換後のトピック名・タイムスタンプは元のままです
- 必要に応じてAutoware Universeのlaunchファイルで新しいrosbagを参照してください
