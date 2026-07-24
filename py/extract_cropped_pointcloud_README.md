# 指定領域でクロップした点群の抽出

rosbag から `/sensing/lidar/front_lower/...` の点群を**指定ボックス領域でクロップ**して抽出する方法です。

## 重要: pandar_packets と PointCloud2 の違い

- **`/sensing/lidar/front_lower/pandar_packets`** は **生の LiDAR パケット**(PointCloud2 ではない)です。
- **crop_box_filter**(autoware_pointcloud_preprocessor)は **PointCloud2 の入出力**です。

そのため「pandar_packets をクロップ」するには、**いったんパケット→点群に変換したトピック**を使う必要があります。

---

## 方法1: オフラインでクロップ(推奨・本スクリプト)

rosbag に **PointCloud2 トピック**が含まれている場合に使えます。

- 例: `/sensing/lidar/front_lower/rectified/pointcloud_ex` が bag に記録されている場合
- Autoware の **crop_box_filter** と同じボックス指定(min_x, max_x, min_y, max_y, min_z, max_z)に対応

### 使い方

```bash
# 指定領域内の点のみ残して新しい rosbag に出力(他トピックは含めない)
python3 py/extract_cropped_pointcloud.py <input_bag> <output_bag> /sensing/lidar/front_lower/rectified/pointcloud_ex \
  --min-x -50 --max-x 50 --min-y -30 --max-y 30 --min-z -2 --max-z 2

# 他トピックもそのままコピーする場合
python3 py/extract_cropped_pointcloud.py <input_bag> <output_bag> /sensing/lidar/front_lower/rectified/pointcloud_ex \
  --min-x -50 --max-x 50 --min-y -30 --max-y 30 --min-z -2 --max-z 2 --copy-other-topics
```

- **negative**: 付けると「ボックス内を除去」(車体除去など)。付けないと「ボックス内のみ残す」(指定領域でクロップ)。

---

## 方法2: Autoware の crop_box_filter を利用する(リアルタイム／再生時)

**pandar_packets しか含まない rosbag** の場合は、Autoware で再生しながら点群に変換し、その点群を crop_box_filter に通して記録する方法があります。

1. **Autoware を起動**(sensing 含む。front_lower の LiDAR ドライバ＋rectified pointcloud まで有効にしておく)。
2. **crop_box_filter** の launch を追加し、入力を `/sensing/lidar/front_lower/rectified/pointcloud_ex`、出力を例: `/sensing/lidar/front_lower/cropped/pointcloud` に。
3. パラメータで **min_x, max_x, min_y, max_y, min_z, max_z** を指定(単位: m)。**negative: false** で「指定領域内のみ残す」。
4. **rosbag 再生**して、crop_box_filter の**出力トピック**を `ros2 bag record` で記録。

これで「pandar_packets → 点群 → 指定領域でクロップ」が一度に行えます。オフラインで同じ結果が欲しい場合は、このように記録した bag を方法1の入力にしても構いません。

---

## まとめ

| 状況 | 推奨 |
|------|------|
| bag に PointCloud2 トピックがある | **方法1**(本スクリプト)でオフラインクロップ |
| bag に pandar_packets しかない | **方法2**(Autoware 再生 + crop_box_filter で出力を record) |
