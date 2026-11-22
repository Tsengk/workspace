#include "lidar_gnss_yaw/cone_detector.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

#include <ros/ros.h>
#include <algorithm>
#include <cmath>

namespace lidar_gnss_yaw {

ConeDetector::ConeDetector(const CalibParams& p)
    : params_(p)
    , last_cone_cloud_(PointCloudPtr(new PointCloud()))
{}

// --------------------------------------------------------------------------
int ConeDetector::detect(const PointCloudPtr& cloud,
                         std::vector<Eigen::Vector2d>& centers) const
{
    centers.clear();
    last_cone_cloud_->clear();
    last_cone_cloud_->header = cloud->header;

    if (cloud->empty()) return 0;
    // 1. ROI 过滤：z 范围（去地面）+ 距离范围
    PointCloudPtr roi(new PointCloud());
    filterROI(cloud, roi);
    if (roi->empty()) return 0;

    // 2. 体素降采样
    PointCloudPtr ds(new PointCloud());
    downsample(roi, ds);
    if (ds->empty()) return 0;

    // 3. 欧氏聚类
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(ds);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(params_.cluster_tolerance);
    ec.setMinClusterSize(params_.min_cluster_points);
    ec.setMaxClusterSize(params_.max_cluster_points);
    ec.setSearchMethod(tree);
    ec.setInputCloud(ds);
    ec.extract(cluster_indices);

    // 4. 形状过滤：高度 + 半径符合锥桶
    for (const auto& idx_set : cluster_indices) {
        Eigen::Vector2d center;
        if (checkCone(ds, idx_set.indices, center)) {
            centers.push_back(center);
            for (int idx : idx_set.indices)
                last_cone_cloud_->push_back(ds->points[idx]);
        }
    }

    last_cone_cloud_->width    = static_cast<uint32_t>(last_cone_cloud_->size());
    last_cone_cloud_->height   = 1;
    last_cone_cloud_->is_dense = true;

    return static_cast<int>(centers.size());
}

// --------------------------------------------------------------------------
void ConeDetector::filterROI(const PointCloudPtr& in, PointCloudPtr& out) const
{
    // 输入点云已在 VCS（车体坐标系）中：
    //   地面 z ≈ -z_offset（后轴中心到地面高度取负）
    //   例：z_offset=0.37m → 地面 z ≈ -0.37m
    // 锥桶 z 范围：[地面 + ground_margin, 地面 + cone_height_max]
    const float z_ground = -static_cast<float>(params_.z_offset);
    const float z_min    = z_ground + static_cast<float>(params_.ground_margin);
    const float z_max    = z_ground + static_cast<float>(params_.cone_height_max);

    const float r_min2 = static_cast<float>(params_.cone_min_range * params_.cone_min_range);
    const float r_max2 = static_cast<float>(params_.cone_max_range * params_.cone_max_range);

    out->reserve(in->size());
    out->header = in->header;

    for (const auto& pt : in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        if (pt.z < z_min || pt.z > z_max)
            continue;
        float r2 = pt.x * pt.x + pt.y * pt.y;
        if (r2 < r_min2 || r2 > r_max2)
            continue;
        out->push_back(pt);
    }
    out->width    = static_cast<uint32_t>(out->size());
    out->height   = 1;
    out->is_dense = true;
}

// --------------------------------------------------------------------------
void ConeDetector::downsample(const PointCloudPtr& in, PointCloudPtr& out) const
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(in);
    const float vs = static_cast<float>(params_.voxel_size);
    vg.setLeafSize(vs, vs, vs);
    vg.filter(*out);
}

// --------------------------------------------------------------------------
bool ConeDetector::checkCone(const PointCloudPtr& cloud,
                              const std::vector<int>& indices,
                              Eigen::Vector2d& center) const
{
    if (indices.empty()) return false;

    float x_sum = 0, y_sum = 0;
    float z_min = 1e6f, z_max = -1e6f;
    float x_min = 1e6f, x_max = -1e6f;
    float y_min = 1e6f, y_max = -1e6f;

    for (int idx : indices) {
        const auto& pt = cloud->points[idx];
        x_sum += pt.x;  y_sum += pt.y;
        z_min = std::min(z_min, pt.z);  z_max = std::max(z_max, pt.z);
        x_min = std::min(x_min, pt.x);  x_max = std::max(x_max, pt.x);
        y_min = std::min(y_min, pt.y);  y_max = std::max(y_max, pt.y);
    }

    float n      = static_cast<float>(indices.size());
    float cx     = x_sum / n;
    float cy     = y_sum / n;
    float height = z_max - z_min;
    float rx     = (x_max - x_min) * 0.5f;
    float ry     = (y_max - y_min) * 0.5f;
    float radius = std::max(rx, ry);

    if (height < static_cast<float>(params_.cone_height_min)) return false;
    if (height > static_cast<float>(params_.cone_height_max)) return false;
    if (radius > static_cast<float>(params_.cone_radius_max)) return false;

    center = Eigen::Vector2d(cx, cy);
    return true;
}

}  // namespace lidar_gnss_yaw
