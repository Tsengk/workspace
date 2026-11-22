#pragma once

#include "lidar_yaw1/types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_yaw1 {

/**
 * @brief 锥形桶检测器
 *
 * 坐标系：FLU（前左上），X=前，Y=左，Z=上
 * 输入点云已经过 Roll/Pitch 标定，地面近似水平。
 *
 * 检测流程：
 *   1. ROI 过滤（X/Y 前向范围 + Z 高度范围）
 *      - Z 范围：[-(z_offset - height_min), -(z_offset - height_max)]
 *        等价于：高于地面 [height_min, height_max]
 *   2. 体素网格降采样
 *   3. 欧几里得聚类分离各锥桶
 *   4. 按点数和 XY 半径过滤锥桶形状
 */
class ConeDetector {
public:
    struct Params {
        // 感兴趣区域（VCS 坐标系，m）
        float x_min;                     // 最近检测距离，默认 2.0m
        float x_max;                     // 最远检测距离，默认 40.0m
        float y_min;                     // 横向左边界，默认 -10.0m
        float y_max;                     // 横向右边界，默认  10.0m

        // 高度范围（相对地面，m）
        float height_above_ground_min;   // 默认 0.08m（过滤地面残差）
        float height_above_ground_max;   // 默认 0.80m（锥桶顶部）

        // 地面 VCS z 值 = -z_offset
        float z_offset;                  // 后轴到地面高度，默认 0.37m

        // 体素降采样
        float voxel_size;                // 默认 0.05m

        // 欧几里得聚类
        float cluster_tolerance;         // 聚类距离阈值，默认 0.30m
        int   min_cluster_points;        // 最少点数，默认 5
        int   max_cluster_points;        // 最多点数，默认 500

        // 锥桶形状过滤
        float max_cluster_radius_xy;     // 最大 XY 半径，默认 0.50m

        // 有效锥桶最少数量
        int   min_cones;                 // 默认 2

        Params()
            : x_min(2.0f), x_max(40.0f),
              y_min(-10.0f), y_max(10.0f),
              height_above_ground_min(0.08f),
              height_above_ground_max(0.80f),
              z_offset(0.37f),
              voxel_size(0.05f),
              cluster_tolerance(0.30f),
              min_cluster_points(5),
              max_cluster_points(500),
              max_cluster_radius_xy(0.50f),
              min_cones(2) {}
    };

    explicit ConeDetector(const Params& params = Params());

    /**
     * @brief 从输入点云中检测锥形桶
     *
     * @param cloud      原始 VCS 点云（FLU 坐标系）
     * @param cones      输出：检测到的锥桶列表
     * @param cone_cloud 输出：锥桶点云（用于可视化）
     * @return true 成功检测到 >= min_cones 个锥桶
     */
    bool detectCones(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        std::vector<ConeCluster>& cones,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cone_cloud);

    void setParams(const Params& p) { params_ = p; }
    const Params& getParams() const { return params_; }

private:
    Params params_;

    // ROI 过滤：XY 范围 + 锥桶高度 Z 范围
    void filterROI(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    // 体素网格降采样
    void downsample(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

    // 聚类并按锥桶形状过滤
    bool clusterAndFilter(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
        std::vector<ConeCluster>& cones,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cone_cloud);
};

} // namespace lidar_yaw1
