# tile_ground_segmentation

一个基于ROS1 Noetic的点云地面分割包，采用平面栅格（tile）方式进行地面估计。

## 主要特性
- 基于栅格的点云地面估计，适用于3D点云
- 所有关键参数均可通过launch文件配置
- 发布话题：
  - `/ground_points`：分割出的地面点云
  - `/obstacle_points`：分割出的障碍物点云（高于地面阈值）
  - `/ground_est_marker_array`：地面估计可视化（MarkerArray薄片）
- 对缺失/异常数据有鲁棒性，支持坡度和邻域补全约束

## 主要参数（详见launch/tile_ground_segmentation.launch）
- `input_topic`：输入点云话题
- `leaf_size`：体素降采样尺寸
- `roi_x_min`/`roi_x_max`/`roi_y_min`/`roi_y_max`/`roi_z_min`/`roi_z_max`：ROI范围
- `tile_size`：栅格单元尺寸（米）
- `z_bin_size`：地面检测z轴bin尺寸
- `z_bin_min_points`：地面bin最小点数
- `max_slope`：地面最大坡度（比例）
- `publish_ground_est_cloud`：是否发布地面估计可视化
- `obstacle_height_thresh`：障碍物判据（高于地面多少米）

## 使用方法
1. `catkin_make` 编译包
2. `source devel/setup.bash`
3. 启动：
   ```bash
   roslaunch tile_ground_segmentation tile_ground_segmentation.launch
   ```
4. RViz可视化：
   - 添加 `/ground_points`、`/obstacle_points`（PointCloud2）
   - 添加 `/ground_est_marker_array`（MarkerArray）

## 说明
- 算法对地面缺失/遮挡有较好补全能力
- 主要逻辑见 `src/tile_ground_segmentation_node.cpp`
- 建议根据实际传感器和场景调整参数以获得最佳效果

## 第三方库说明
本工程直接源码引用了CSF库（https://github.com/jianboqi/CSF，Apache-2.0 license），请参见CSF库主页获取许可协议详情。
