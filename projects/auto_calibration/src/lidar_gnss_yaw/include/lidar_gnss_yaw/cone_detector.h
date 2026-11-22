#pragma once

/**
 * @file cone_detector.h
 * @brief 从预变换点云（VCS）中检测锥形桶，返回 VCS 坐标系下的中心 (x, y)
 *
 * 输入点云已由驱动变换到 VCS（车体坐标系，FLU，原点=后轴中心）。
 * 地面在 VCS 中位于 z ≈ -z_offset（后轴中心到地面高度取负）。
 *
 * 流程：ROI 过滤（z + range）→ 体素降采样 → 欧氏聚类 → 形状过滤
 */

#include "lidar_gnss_yaw/types.h"

namespace lidar_gnss_yaw {

class ConeDetector {
public:
    explicit ConeDetector(const CalibParams& p);

    /**
     * @brief 检测锥桶
     * @param cloud   VCS 点云（已由驱动完成安装变换）
     * @param centers 输出：各锥桶中心 (x,y) in VCS
     * @return 检测到的锥桶数
     */
    int detect(const PointCloudPtr& cloud,
               std::vector<Eigen::Vector2d>& centers) const;

    PointCloudPtr lastConeCloud() const { return last_cone_cloud_; }

private:
    CalibParams params_;
    mutable PointCloudPtr last_cone_cloud_;

    void filterROI(const PointCloudPtr& in, PointCloudPtr& out) const;
    void downsample(const PointCloudPtr& in, PointCloudPtr& out) const;
    bool checkCone(const PointCloudPtr& cloud,
                   const std::vector<int>& indices,
                   Eigen::Vector2d& center) const;
};

}  // namespace lidar_gnss_yaw
