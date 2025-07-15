# tile_ground_segmentation

A ROS1 Noetic package for tile-based ground segmentation of point clouds.

## Features
- Tile/grid-based ground estimation for 3D point clouds
- All key parameters are configurable via launch file
- Publishes:
  - `/ground_points`: segmented ground points
  - `/obstacle_points`: segmented obstacle points (height above ground > threshold)
  - `/ground_est_marker_array`: visualization of estimated ground as thin tiles (MarkerArray)
- Robust to missing data and outliers, with slope and neighbor constraints

## Main Parameters (see launch/tile_ground_segmentation.launch)
- `input_topic`: Input point cloud topic
- `leaf_size`: VoxelGrid downsample size
- `roi_x_min`, `roi_x_max`, `roi_y_min`, `roi_y_max`, `roi_z_min`, `roi_z_max`: ROI limits
- `tile_size`: Grid cell size (meters)
- `z_bin_size`: Z bin size for ground detection
- `z_bin_min_points`: Minimum points in a bin to be considered ground
- `max_slope`: Maximum allowed ground slope (ratio)
- `publish_ground_est_cloud`: Whether to publish ground estimation visualization
- `obstacle_height_thresh`: Height above ground to be considered obstacle (meters)

## Usage
1. Build the package with `catkin_make`.
2. Source your workspace: `source devel/setup.bash`
3. Launch:
   ```bash
   roslaunch tile_ground_segmentation tile_ground_segmentation.launch
   ```
4. Visualize in RViz:
   - Add `/ground_points`, `/obstacle_points` as PointCloud2
   - Add `/ground_est_marker_array` as MarkerArray

## Notes
- The algorithm is robust to missing or occluded ground by using neighbor interpolation.
- All logic is implemented in `src/tile_ground_segmentation_node.cpp`.
- For best results, tune parameters for your sensor and environment.