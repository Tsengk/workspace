#include "lidar_rp_cali/ground_detector.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <ros/ros.h>

namespace lidar_rp_cali {

GroundDetector::GroundDetector(const Params& params) : params_(params) {}

bool GroundDetector::detectGround(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& output)
{
    if (!input || input->empty()) {
        ROS_WARN("[GroundDetector] input cloud is empty");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp4(new pcl::PointCloud<pcl::PointXYZI>);

    // 1. 高度过滤（FLU：Z 为上，地面在传感器下方为负）
    filterByHeight(input, tmp1);
    if (tmp1->empty()) {
        ROS_WARN("[GroundDetector] empty after height filter");
        return false;
    }

    // 2. distance filter
    filterByDistance(tmp1, tmp2);
    if (tmp2->empty()) {
        ROS_WARN("[GroundDetector] empty after distance filter");
        return false;
    }

    // 3. voxel grid downsampling
    applyVoxelGrid(tmp2, tmp3);
    if (tmp3->empty()) {
        ROS_WARN("[GroundDetector] empty after voxel grid");
        return false;
    }

    // 4. statistical outlier removal
    removeOutliers(tmp3, tmp4);
    if (tmp4->size() < static_cast<size_t>(params_.min_cluster_points)) {
        ROS_WARN("[GroundDetector] too few pts after SOR: %zu", tmp4->size());
        return false;
    }

    // 5. euclidean clustering, take largest
    if (!extractLargestCluster(tmp4, output)) {
        ROS_WARN("[GroundDetector] no valid ground cluster found");
        return false;
    }

    ROS_DEBUG("[GroundDetector] ground pts: %zu (raw: %zu)",
              output->size(), input->size());
    return true;
}

// ---------------------------------------------------------------------------
void GroundDetector::filterByHeight(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(params_.min_z, params_.max_z);
    pass.filter(*out);
}

// ---------------------------------------------------------------------------
void GroundDetector::filterByDistance(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    out->clear();
    out->header = in->header;
    for (const auto& pt : *in) {
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (dist >= params_.min_distance && dist <= params_.max_distance) {
            out->push_back(pt);
        }
    }
}

// ---------------------------------------------------------------------------
void GroundDetector::applyVoxelGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
    vg.filter(*out);
}

// ---------------------------------------------------------------------------
void GroundDetector::removeOutliers(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(in);
    sor.setMeanK(params_.sor_mean_k);
    sor.setStddevMulThresh(params_.sor_std_thresh);
    sor.filter(*out);
}

// ---------------------------------------------------------------------------
bool GroundDetector::extractLargestCluster(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(params_.cluster_tolerance);
    ec.setMinClusterSize(params_.min_cluster_points);
    ec.setMaxClusterSize(params_.max_cluster_points);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        return false;
    }

    // 选取最大簇
    const pcl::PointIndices& largest = cluster_indices.front();
    out->clear();
    out->header = in->header;
    for (int idx : largest.indices) {
        out->push_back((*in)[idx]);
    }
    return true;
}

} // namespace lidar_rp_cali
