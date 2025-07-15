// visualization_msgs
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

// STL
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>


class GroundSegmentationNode {
public:
    GroundSegmentationNode() : last_adjust_time_(ros::Time::now()) {
        ROS_INFO("Initializing GroundSegmentationNode...");
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        // 参数化所有数字
        pnh.param<float>("leaf_size", leaf_size_, 0.05f);
        pnh.param<std::string>("input_topic", input_topic_, "/camera/depth/color/points");
        pnh.param<float>("roi_x_min", roi_x_min_, 0.0f);
        pnh.param<float>("roi_x_max", roi_x_max_, 6.0f);
        pnh.param<float>("roi_y_min", roi_y_min_, -5.0f);
        pnh.param<float>("roi_y_max", roi_y_max_, 5.0f);
        pnh.param<float>("roi_z_min", roi_z_min_, -1.0f);
        pnh.param<float>("roi_z_max", roi_z_max_, 2.0f);
        pnh.param<float>("tile_size", tile_size_, 0.5f);
        pnh.param<float>("z_bin_size", z_bin_size_, 0.05f);
        pnh.param<int>("z_bin_min_points", z_bin_min_points_, 3);
        pnh.param<float>("max_slope", max_slope_, 0.15f); // 15%

        pnh.param<bool>("publish_ground_est_cloud", publish_ground_est_cloud_, true);
        pnh.param<float>("obstacle_height_thresh", obstacle_height_thresh_, 0.2f);

        cloud_sub_ = nh.subscribe(input_topic_, 1, &GroundSegmentationNode::cloudCallback, this);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1);
        ground_est_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/ground_est_marker_array", 1);
        ROS_INFO("Finished initializing GroundSegmentationNode.");
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        auto t1 = std::chrono::high_resolution_clock::now();
        sensor_msgs::PointCloud2 cloud_base;
        try {
            listener_.waitForTransform("base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
            pcl_ros::transformPointCloud("base_link", *msg, cloud_base, listener_);
        } catch (tf::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_base, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x"); pass.setFilterLimits(roi_x_min_, roi_x_max_); pass.filter(*cloud_roi);
        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("y"); pass.setFilterLimits(roi_y_min_, roi_y_max_); pass.filter(*cloud_roi);
        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("z"); pass.setFilterLimits(roi_z_min_, roi_z_max_); pass.filter(*cloud_roi);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_roi);
        voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        voxel.filter(*cloud_ds);

        // 1. 平面栅格化
        int grid_x = std::ceil((roi_x_max_ - roi_x_min_) / tile_size_);
        int grid_y = std::ceil((roi_y_max_ - roi_y_min_) / tile_size_);
        struct TileCell {
            std::vector<float> z_values;
            float ground_z = std::numeric_limits<float>::quiet_NaN();
            bool valid = false;
            bool guessed_by_slope = false; // 是否通过坡度约束或均值补全猜测的地面高度
        };
        std::vector<std::vector<TileCell>> grid(grid_x, std::vector<TileCell>(grid_y));

        // 2. 遍历点云分配到栅格
        for (const auto& pt : cloud_ds->points) {
            int ix = int((pt.x - roi_x_min_) / tile_size_);
            int iy = int((pt.y - roi_y_min_) / tile_size_);
            if (ix >= 0 && ix < grid_x && iy >= 0 && iy < grid_y) {
                grid[ix][iy].z_values.push_back(pt.z);
            }
        }

        // 2.1/2.2/2.3 统计z分布，找地面bin
        for (int ix = 0; ix < grid_x; ++ix) {
            for (int iy = 0; iy < grid_y; ++iy) {
                auto& cell = grid[ix][iy];
                if (cell.z_values.empty()) continue;
                float z_min = *std::min_element(cell.z_values.begin(), cell.z_values.end());
                float z_max = *std::max_element(cell.z_values.begin(), cell.z_values.end());
                int n_bins = std::max(1, int(std::ceil((z_max - z_min) / z_bin_size_)));
                std::vector<int> bin_counts(n_bins, 0);
                for (float z : cell.z_values) {
                    int bin = std::min(int((z - z_min) / z_bin_size_), n_bins - 1);
                    bin_counts[bin]++;
                }
                int ground_bin = -1;
                for (int b = 0; b < n_bins; ++b) {
                    if (bin_counts[b] >= z_bin_min_points_) {
                        ground_bin = b;
                        break;
                    }
                }
                if (ground_bin >= 0) {
                    cell.ground_z = z_min + ground_bin * z_bin_size_;
                    cell.valid = true;
                }
            }
        }

        // 3. 地面坡度约束修复离群
        for (int ix = 0; ix < grid_x; ++ix) {
            for (int iy = 0; iy < grid_y; ++iy) {
                auto& cell = grid[ix][iy];
                if (!cell.valid) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (std::abs(dx) + std::abs(dy) != 1) continue;
                        int nx = ix + dx, ny = iy + dy;
                        if (nx >= 0 && nx < grid_x && ny >= 0 && ny < grid_y) {
                            auto& ncell = grid[nx][ny];
                            if (!ncell.valid) continue;
                            float dz = std::abs(cell.ground_z - ncell.ground_z);
                            float dist = tile_size_;
                            if (dz / dist > max_slope_) {
                                cell.ground_z = ncell.ground_z;
                            }
                        }
                    }
                }
            }
        }

        // 4. 缺失补全（均值法）
        for (int ix = 0; ix < grid_x; ++ix) {
            for (int iy = 0; iy < grid_y; ++iy) {
                auto& cell = grid[ix][iy];
                if (cell.valid) continue;
                float sum = 0; int cnt = 0;
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        int nx = ix + dx, ny = iy + dy;
                        if (nx >= 0 && nx < grid_x && ny >= 0 && ny < grid_y) {
                            auto& ncell = grid[nx][ny];
                            if (ncell.valid && !ncell.guessed_by_slope) { sum += ncell.ground_z; cnt++; }
                        }
                    }
                }
                if (cnt > 0) {
                    cell.ground_z = sum / cnt;
                    cell.valid = true;
                    cell.guessed_by_slope = true;
                }
            }
        }

        // 5. 构造地面估计可视化MarkerArray
        if (publish_ground_est_cloud_) {
            visualization_msgs::MarkerArray marker_array;
            int id = 0;
            for (int ix = 0; ix < grid_x; ++ix) {
                for (int iy = 0; iy < grid_y; ++iy) {
                    auto& cell = grid[ix][iy];
                    if (!cell.valid) continue;
                    float x = roi_x_min_ + (ix + 0.5f) * tile_size_;
                    float y = roi_y_min_ + (iy + 0.5f) * tile_size_;
                    visualization_msgs::Marker marker;
                    marker.header = cloud_base.header;
                    marker.ns = "ground_tiles";
                    marker.id = id++;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = x;
                    marker.pose.position.y = y;
                    marker.pose.position.z = cell.ground_z;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = tile_size_;
                    marker.scale.y = tile_size_;
                    marker.scale.z = 0.03; // 薄片厚度
                    marker.color.r = 0.2f;
                    marker.color.g = 0.8f;
                    marker.color.b = 0.2f;
                    marker.color.a = 0.5f;
                    marker.lifetime = ros::Duration(0.2);
                    marker_array.markers.push_back(marker);
                }
            }
            ground_est_marker_pub_.publish(marker_array);
        }

        // 发布分割后的地面/障碍物点云
        pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
        for (const auto& pt : cloud_ds->points) {
            int ix = int((pt.x - roi_x_min_) / tile_size_);
            int iy = int((pt.y - roi_y_min_) / tile_size_);
            if (ix >= 0 && ix < grid_x && iy >= 0 && iy < grid_y) {
                const auto& cell = grid[ix][iy];
                if (cell.valid) {
                    if (pt.z - cell.ground_z > obstacle_height_thresh_) {
                        obstacle_cloud.push_back(pt);
                    } else if (std::abs(pt.z - cell.ground_z) < 0.1) {
                        ground_cloud.push_back(pt);
                    }
                }
            }
        }
        sensor_msgs::PointCloud2 ground_msg, obstacle_msg;
        pcl::toROSMsg(ground_cloud, ground_msg);
        pcl::toROSMsg(obstacle_cloud, obstacle_msg);
        ground_msg.header = cloud_base.header;
        obstacle_msg.header = cloud_base.header;
        ground_pub_.publish(ground_msg);
        obstacle_pub_.publish(obstacle_msg);

        auto t2 = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        ROS_INFO("Frame processed in %.2f ms", elapsed_ms);

    }

private:
    ros::Subscriber cloud_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher ground_est_marker_pub_;
    tf::TransformListener listener_;
    float leaf_size_;
    float roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_;
    float tile_size_;
    float z_bin_size_;
    int z_bin_min_points_;
    float max_slope_;
    bool publish_ground_est_cloud_;
    float obstacle_height_thresh_;
    ros::Time last_adjust_time_;
    std::string input_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_segmentation_node");
    GroundSegmentationNode node;
    ros::spin();
    return 0;
}