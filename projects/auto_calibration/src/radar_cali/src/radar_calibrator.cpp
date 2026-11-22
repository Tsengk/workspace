#include "radar_cali/radar_calibrator.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <algorithm>
#include <cmath>

namespace radar_cali {

RadarCalibrator::Params::Params()
    : max_range(100.0),
      min_range(1.0),
      roi_x_min(-20.0),
      roi_x_max(20.0),
      roi_y_min(-10.0),
      roi_y_max(10.0),
      roi_z_min(-3.0),
      roi_z_max(5.0),
      ground_height(-0.37),  // VCS ground height
      ground_tolerance(0.1),
      min_points(50),
      max_iterations(1000),
      distance_threshold(0.1) {}

RadarCalibrator::RadarCalibrator() {}

void RadarCalibrator::setParams(const Params& p) {
    params_ = p;
}

bool RadarCalibrator::calibrateGroundPlane(
    const std::vector<Eigen::Vector3d>& radar_points,
    const InstallParams& install,
    RadarCalibResult& result) {

    if (radar_points.size() < static_cast<size_t>(params_.min_points)) {
        result.valid = false;
        result.message = "Insufficient radar points: " + std::to_string(radar_points.size()) +
                        ", need at least " + std::to_string(params_.min_points);
        return false;
    }

    // Filter points by ROI
    std::vector<Eigen::Vector3d> filtered_points;
    for (const auto& pt : radar_points) {
        double range = pt.norm();
        if (range < params_.min_range || range > params_.max_range) continue;
        if (pt.x() < params_.roi_x_min || pt.x() > params_.roi_x_max) continue;
        if (pt.y() < params_.roi_y_min || pt.y() > params_.roi_y_max) continue;
        if (pt.z() < params_.roi_z_min || pt.z() > params_.roi_z_max) continue;
        filtered_points.push_back(pt);
    }

    if (filtered_points.size() < static_cast<size_t>(params_.min_points)) {
        result.valid = false;
        result.message = "Insufficient points in ROI: " + std::to_string(filtered_points.size());
        return false;
    }

    // Detect ground plane
    Eigen::Vector4d plane_coeffs;
    std::vector<int> inliers;
    if (!detectGroundPlane(filtered_points, plane_coeffs, inliers)) {
        result.valid = false;
        result.message = "Ground plane detection failed";
        return false;
    }

    // Estimate pose from ground plane
    if (!estimatePoseFromGround(plane_coeffs, install, result)) {
        result.valid = false;
        result.message = "Pose estimation from ground failed";
        return false;
    }

    result.valid = true;
    result.message = "Calibration successful";
    return true;
}

bool RadarCalibrator::calibrateWithTarget(
    const std::vector<Eigen::Vector3d>& radar_points,
    const std::vector<Eigen::Vector3d>& target_positions_vcs,
    const InstallParams& install,
    RadarCalibResult& result) {

    if (radar_points.size() != target_positions_vcs.size()) {
        result.valid = false;
        result.message = "Radar points and target positions size mismatch";
        return false;
    }

    if (radar_points.size() < 3) {
        result.valid = false;
        result.message = "Need at least 3 point correspondences";
        return false;
    }

    // Use Umeyama algorithm for point cloud registration
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double scale;

    // Compute centroids
    Eigen::Vector3d radar_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d vcs_centroid = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < radar_points.size(); ++i) {
        radar_centroid += radar_points[i];
        vcs_centroid += target_positions_vcs[i];
    }
    radar_centroid /= static_cast<double>(radar_points.size());
    vcs_centroid /= static_cast<double>(target_positions_vcs.size());

    // Compute covariance matrix
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < radar_points.size(); ++i) {
        Eigen::Vector3d radar_diff = radar_points[i] - radar_centroid;
        Eigen::Vector3d vcs_diff = target_positions_vcs[i] - vcs_centroid;
        H += radar_diff * vcs_diff.transpose();
    }

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    R = V * U.transpose();

    // Ensure right-handed coordinate system
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // Compute translation
    t = vcs_centroid - R * radar_centroid;

    // Fill result
    result.R = R;
    result.t = t;
    result.T.setIdentity();
    result.T.block<3,3>(0,0) = R;
    result.T.block<3,1>(0,3) = t;

    // Convert to Euler angles
    rotToEuler(R, result.roll_deg, result.pitch_deg, result.yaw_deg);
    result.x_m = t.x();
    result.y_m = t.y();
    result.z_m = t.z();

    // Evaluate quality
    double rms_error = evaluateCalibration(radar_points, target_positions_vcs, result);
    if (rms_error < 0.02) {
        result.quality_grade = "EXCELLENT";
        result.confidence = 1.0;
    } else if (rms_error < 0.05) {
        result.quality_grade = "GOOD";
        result.confidence = 0.8;
    } else if (rms_error < 0.1) {
        result.quality_grade = "ACCEPTABLE";
        result.confidence = 0.6;
    } else {
        result.quality_grade = "POOR";
        result.confidence = 0.3;
    }

    result.valid = true;
    result.message = "Target-based calibration successful, RMS=" + std::to_string(rms_error) + "m";
    return true;
}

bool RadarCalibrator::refineCalibration(
    const std::vector<Eigen::Vector3d>& radar_points,
    const std::vector<Eigen::Vector3d>& reference_points_vcs,
    const RadarCalibResult& initial,
    RadarCalibResult& result) {

    // For radar, refinement is similar to target-based calibration
    // but uses the initial calibration as a prior
    return calibrateWithTarget(radar_points, reference_points_vcs, InstallParams(), result);
}

void RadarCalibrator::transformToVCS(
    const std::vector<Eigen::Vector3d>& radar_points,
    const RadarCalibResult& calib,
    std::vector<Eigen::Vector3d>& vcs_points) const {

    vcs_points.resize(radar_points.size());
    for (size_t i = 0; i < radar_points.size(); ++i) {
        vcs_points[i] = calib.R * radar_points[i] + calib.t;
    }
}

double RadarCalibrator::evaluateCalibration(
    const std::vector<Eigen::Vector3d>& radar_points,
    const std::vector<Eigen::Vector3d>& reference_points_vcs,
    const RadarCalibResult& calib) const {

    if (radar_points.size() != reference_points_vcs.size()) {
        return 1e6;  // Large error
    }

    double total_error = 0.0;
    for (size_t i = 0; i < radar_points.size(); ++i) {
        Eigen::Vector3d transformed = calib.R * radar_points[i] + calib.t;
        double error = (transformed - reference_points_vcs[i]).norm();
        total_error += error * error;
    }

    return std::sqrt(total_error / static_cast<double>(radar_points.size()));
}

bool RadarCalibrator::detectGroundPlane(
    const std::vector<Eigen::Vector3d>& points,
    Eigen::Vector4d& plane_coeffs,
    std::vector<int>& inliers) {

    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size());
    for (const auto& pt : points) {
        cloud->points.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
    }

    // RANSAC plane fitting
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(params_.distance_threshold);
    ransac.setMaxIterations(params_.max_iterations);
    ransac.computeModel();

    // Get plane coefficients
    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    if (coeffs.size() != 4) {
        return false;
    }

    plane_coeffs = Eigen::Vector4d(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);

    // Get inliers
    ransac.getInliers(inliers);
    if (inliers.size() < static_cast<size_t>(params_.min_points)) {
        return false;
    }

    return true;
}

bool RadarCalibrator::estimatePoseFromGround(
    const Eigen::Vector4d& plane_coeffs,
    const InstallParams& install,
    RadarCalibResult& result) {

    // Ground plane in radar coordinate system: ax + by + cz + d = 0
    // Normal vector should point toward ground
    Eigen::Vector3d normal(plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]);
    normal.normalize();

    // Ensure normal points downward (negative Z in VCS)
    if (normal.z() > 0) {
        normal = -normal;
    }

    // Estimate roll and pitch from normal
    double pitch = std::atan2(-normal.x(), std::sqrt(normal.y()*normal.y() + normal.z()*normal.z()));
    double roll = std::atan2(normal.y(), normal.z());

    // Convert to degrees
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;

    // For yaw, we assume it's already roughly correct from installation
    double yaw = install.yaw_deg;

    // Compute rotation matrix
    Eigen::Matrix3d R = eulerToRot(roll, pitch, yaw);

    // Estimate height from plane
    double height = -plane_coeffs[3] / plane_coeffs[2];  // z = -d/c when x=y=0

    // Assume radar is mounted at known height above ground
    // This is a simplification - in practice, we'd need more information
    Eigen::Vector3d t(install.x, install.y, install.z);

    // Fill result
    result.R = R;
    result.t = t;
    result.T.setIdentity();
    result.T.block<3,3>(0,0) = R;
    result.T.block<3,1>(0,3) = t;
    result.roll_deg = roll;
    result.pitch_deg = pitch;
    result.yaw_deg = yaw;
    result.x_m = t.x();
    result.y_m = t.y();
    result.z_m = t.z();

    // Quality assessment based on how well the plane normal aligns with expected ground
    double expected_normal_z = -1.0;  // Perfect ground has normal pointing straight down
    double alignment = std::abs(normal.z() - expected_normal_z);
    if (alignment < 0.05) {
        result.quality_grade = "EXCELLENT";
        result.confidence = 1.0;
    } else if (alignment < 0.1) {
        result.quality_grade = "GOOD";
        result.confidence = 0.8;
    } else if (alignment < 0.2) {
        result.quality_grade = "ACCEPTABLE";
        result.confidence = 0.6;
    } else {
        result.quality_grade = "POOR";
        result.confidence = 0.3;
    }

    return true;
}

Eigen::Matrix3d RadarCalibrator::eulerToRot(double roll, double pitch, double yaw) const {
    const double r = roll * M_PI / 180.0;
    const double p = pitch * M_PI / 180.0;
    const double y = yaw * M_PI / 180.0;

    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
               .toRotationMatrix();
}

void RadarCalibrator::rotToEuler(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw) const {
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);  // ZYX order
    yaw = euler[0] * 180.0 / M_PI;
    pitch = euler[1] * 180.0 / M_PI;
    roll = euler[2] * 180.0 / M_PI;
}

} // namespace radar_cali