#include "lidar_rp_cali/plane_fitter.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <cmath>

namespace lidar_rp_cali {

PlaneFitter::PlaneFitter(const Params& params) : params_(params) {}

bool PlaneFitter::fitPlane(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    PlaneModel& model)
{
    if (!cloud || cloud->size() < 3) {
        ROS_WARN("[PlaneFitter] insufficient points (%zu)", cloud ? cloud->size() : 0);
        return false;
    }

    bool ok = false;
    switch (params_.method) {
        case FitMethod::RANSAC:
            ok = fitRANSAC(cloud, model);
            break;
        case FitMethod::LEAST_SQUARES:
            ok = fitLeastSquares(cloud, model);
            break;
        case FitMethod::PCA:
            ok = fitPCA(cloud, model);
            break;
    }

    if (ok) {
        normalizeUpward(model);
        model.rms_error = computeRMSError(cloud, model.coefficients);
        model.valid = true;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// RANSAC：使用 PCL SACSegmentation
// ---------------------------------------------------------------------------
bool PlaneFitter::fitRANSAC(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    PlaneModel& model)
{
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params_.ransac_distance_threshold);
    seg.setMaxIterations(params_.ransac_max_iterations);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("[PlaneFitter] RANSAC: no inliers found");
        return false;
    }

    float inlier_ratio = static_cast<float>(inliers->indices.size()) /
                         static_cast<float>(cloud->size());
    if (inlier_ratio < params_.min_inlier_ratio) {
        ROS_WARN("[PlaneFitter] RANSAC: low inlier ratio: %.2f", inlier_ratio);
        return false;
    }

    model.coefficients = Eigen::Vector4f(
        coefficients->values[0], coefficients->values[1],
        coefficients->values[2], coefficients->values[3]);
    model.normal = model.coefficients.head<3>();
    model.inliers_count = static_cast<int>(inliers->indices.size());
    return true;
}

// ---------------------------------------------------------------------------
// 最小二乘：协方差矩阵特征分解（最小特征值对应法向量）
// ---------------------------------------------------------------------------
bool PlaneFitter::fitLeastSquares(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    PlaneModel& model)
{
    const size_t N = cloud->size();

    // 计算质心
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& pt : *cloud) {
        centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<float>(N);

    // 构建协方差矩阵
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& pt : *cloud) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(N);

    // 特征分解（最小特征值对应法向量）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        ROS_WARN("[PlaneFitter] LeastSq: eigen decomposition failed");
        return false;
    }

    // 特征值升序排列，index 0 对应最小特征值（法向量方向）
    Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
    model.normal = normal;
    float d = -normal.dot(centroid);
    model.coefficients = Eigen::Vector4f(normal.x(), normal.y(), normal.z(), d);
    model.curvature = solver.eigenvalues()(0) /
                      (solver.eigenvalues()(0) + solver.eigenvalues()(1) +
                       solver.eigenvalues()(2) + 1e-6f);
    model.inliers_count = static_cast<int>(N);
    return true;
}

// ---------------------------------------------------------------------------
// PCA：与最小二乘本质相同，使用 Eigen 求解
// ---------------------------------------------------------------------------
bool PlaneFitter::fitPCA(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    PlaneModel& model)
{
    // PCA 与最小二乘法向量求解等价
    return fitLeastSquares(cloud, model);
}

// ---------------------------------------------------------------------------
float PlaneFitter::computeRMSError(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const Eigen::Vector4f& coefficients)
{
    const Eigen::Vector3f n = coefficients.head<3>();
    const float d = coefficients[3];
    const float norm = n.norm();
    if (norm < 1e-6f) return 1.0f;

    double sum_sq = 0.0;
    for (const auto& pt : *cloud) {
        float dist = (n.dot(Eigen::Vector3f(pt.x, pt.y, pt.z)) + d) / norm;
        sum_sq += dist * dist;
    }
    return static_cast<float>(std::sqrt(sum_sq / cloud->size()));
}

// ---------------------------------------------------------------------------
void PlaneFitter::normalizeUpward(PlaneModel& model)
{
    // FLU 坐标系下，地面法向量 Z 分量应为正（朝上）
    if (model.normal.z() < 0) {
        model.normal = -model.normal;
        model.coefficients.head<3>() = model.normal;
        model.coefficients[3] = -model.coefficients[3];
    }
    model.normal.normalize();
    model.coefficients.head<3>() = model.normal;
}

} // namespace lidar_rp_cali
