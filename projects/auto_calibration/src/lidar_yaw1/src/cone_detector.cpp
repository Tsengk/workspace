#include "lidar_yaw1/cone_detector.h"

#include <ros/ros.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

#include <algorithm>
#include <cmath>

namespace lidar_yaw1 {

ConeDetector::ConeDetector(const Params& params)
    : params_(params) {}

// ---------------------------------------------------------------------------
bool ConeDetector::detectCones(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    std::vector<ConeCluster>& cones,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cone_cloud)
{
    cones.clear();
    cone_cloud->clear();

    if (cloud->empty()) {
        ROS_WARN_THROTTLE(5.0, "[ConeDetector] input cloud is empty");
        return false;
    }

    // 1. ROI 过滤（x / y 范围 + 锥桶高度 z 范围）
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi(new pcl::PointCloud<pcl::PointXYZI>());
    filterROI(cloud, roi);
    if (roi->empty()) {
        ROS_DEBUG("[ConeDetector] no points in ROI after filtering");
        return false;
    }

    // 2. 体素降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>());
    downsample(roi, ds);
    if (ds->empty()) {
        ROS_DEBUG("[ConeDetector] no points after downsampling");
        return false;
    }

    // 3. 聚类 + 形状过滤
    return clusterAndFilter(ds, cones, cone_cloud);
}

// ---------------------------------------------------------------------------
void ConeDetector::filterROI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    // 地面 VCS z 值 ≈ -z_offset
    // 锥桶 z 范围 = [-z_offset + height_min, -z_offset + height_max]
    const float z_min = -params_.z_offset + params_.height_above_ground_min;
    const float z_max = -params_.z_offset + params_.height_above_ground_max;

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
    tmp->reserve(in->size());

    for (const auto& pt : in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        if (pt.x < params_.x_min || pt.x > params_.x_max)
            continue;
        if (pt.y < params_.y_min || pt.y > params_.y_max)
            continue;
        if (pt.z < z_min || pt.z > z_max)
            continue;
        tmp->push_back(pt);
    }

    tmp->width    = static_cast<uint32_t>(tmp->size());
    tmp->height   = 1;
    tmp->is_dense = true;
    *out = *tmp;
}

// ---------------------------------------------------------------------------
void ConeDetector::downsample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
    vg.filter(*out);
}

// ---------------------------------------------------------------------------
bool ConeDetector::clusterAndFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    std::vector<ConeCluster>& cones,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cone_cloud)
{
    // KdTree 构建
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(in);

    // 欧几里得聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(params_.cluster_tolerance);
    ec.setMinClusterSize(params_.min_cluster_points);
    ec.setMaxClusterSize(params_.max_cluster_points);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        ROS_DEBUG("[ConeDetector] no clusters found");
        return false;
    }

    cone_cloud->clear();

    for (const auto& indices : cluster_indices) {
        // 计算质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*in, indices.indices, centroid);

        // 计算 XY 最大半径（形状过滤）
        float max_r2 = 0.0f;
        for (int idx : indices.indices) {
            const auto& pt = in->points[idx];
            float dx = pt.x - centroid[0];
            float dy = pt.y - centroid[1];
            float r2 = dx * dx + dy * dy;
            if (r2 > max_r2) max_r2 = r2;
        }
        float radius_xy = std::sqrt(max_r2);

        // 过滤：XY 半径过大的聚类不是锥桶
        if (radius_xy > params_.max_cluster_radius_xy) {
            ROS_DEBUG("[ConeDetector] cluster rejected: radius_xy=%.2f > max=%.2f",
                      radius_xy, params_.max_cluster_radius_xy);
            continue;
        }

        // 记录锥桶
        ConeCluster cone;
        cone.center      = centroid.head<3>();
        cone.radius_xy   = radius_xy;
        cone.point_count = static_cast<int>(indices.indices.size());
        cone.valid       = true;
        cones.push_back(cone);

        // 合并到可视化点云
        for (int idx : indices.indices) {
            cone_cloud->push_back(in->points[idx]);
        }
    }

    cone_cloud->width    = static_cast<uint32_t>(cone_cloud->size());
    cone_cloud->height   = 1;
    cone_cloud->is_dense = true;

    if (static_cast<int>(cones.size()) < params_.min_cones) {
        ROS_DEBUG("[ConeDetector] insufficient cones: %zu < min=%d",
                  cones.size(), params_.min_cones);
        return false;
    }

    ROS_DEBUG("[ConeDetector] detected %zu cones", cones.size());
    return true;
}

} // namespace lidar_yaw1
