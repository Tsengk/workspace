#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace lidar_rp_cali {

/**
 * @brief 地面点检测器
 *
 * 坐标系：FLU（前左上），X=前，Y=左，Z=上
 * 地面点在 LiDAR 帧下 Z 值为负（传感器在地面上方）
 *
 * 处理流程：
 *   1. 高度过滤（Z 轴范围）
 *   2. 距离过滤（XY 平面距离）
 *   3. 体素网格降采样
 *   4. 统计离群点去除
 *   5. 欧几里得聚类，取最大簇
 */
class GroundDetector {
public:
    struct Params {
        // 高度过滤（传感器坐标系 Z 值，地面在负方向）
        float min_z;               // 默认 -3.0m
        float max_z;               // 默认 -0.3m

        // 水平距离过滤
        float min_distance;        // 默认 2.0m（排除车身下方）
        float max_distance;        // 默认 30.0m

        // 体素降采样
        float voxel_size;          // 默认 0.2m

        // 统计离群点去除
        int sor_mean_k;            // 默认 20
        float sor_std_thresh;      // 默认 1.5

        // 欧几里得聚类
        float cluster_tolerance;   // 默认 0.5m
        int min_cluster_points;    // 默认 200
        int max_cluster_points;    // 默认 50000

        Params()
            : min_z(-3.0f), max_z(-0.3f),
              min_distance(2.0f), max_distance(30.0f),
              voxel_size(0.2f),
              sor_mean_k(20), sor_std_thresh(1.5f),
              cluster_tolerance(0.5f),
              min_cluster_points(200), max_cluster_points(50000) {}
    };

    explicit GroundDetector(const Params& params = Params());

    /**
     * @brief 从输入点云中提取地面点
     * @param input  原始 LiDAR 点云（LiDAR 帧，FLU）
     * @param output 提取的地面点云
     * @return true 提取成功
     */
    bool detectGround(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& output);

    void setParams(const Params& params) { params_ = params; }
    const Params& getParams() const { return params_; }

private:
    void filterByHeight(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    void filterByDistance(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    void applyVoxelGrid(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    void removeOutliers(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    bool extractLargestCluster(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    Params params_;
};

} // namespace lidar_rp_cali
