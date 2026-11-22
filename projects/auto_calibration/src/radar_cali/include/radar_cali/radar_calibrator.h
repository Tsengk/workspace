#pragma once
#include <vector>
#include <Eigen/Dense>
#include "radar_cali/types.h"

namespace radar_cali {

class RadarCalibrator {
public:
    struct Params {
        double max_range;         ///< Maximum detection range (m)
        double min_range;         ///< Minimum detection range (m)
        double roi_x_min;         ///< ROI X minimum (m)
        double roi_x_max;         ///< ROI X maximum (m)
        double roi_y_min;         ///< ROI Y minimum (m)
        double roi_y_max;         ///< ROI Y maximum (m)
        double roi_z_min;         ///< ROI Z minimum (m)
        double roi_z_max;         ///< ROI Z maximum (m)
        double ground_height;     ///< Expected ground height in VCS (m)
        double ground_tolerance;  ///< Ground detection tolerance (m)
        int min_points;           ///< Minimum points for calibration
        double max_iterations;    ///< Maximum RANSAC iterations
        double distance_threshold; ///< RANSAC distance threshold (m)

        Params();
    };

    RadarCalibrator();
    void setParams(const Params& p);

    /// Calibrate radar using ground plane detection
    /// @param radar_points Radar point cloud in radar coordinate system
    /// @param install Initial installation parameters
    /// @param result Output calibration result
    /// @return true if calibration successful
    bool calibrateGroundPlane(
        const std::vector<Eigen::Vector3d>& radar_points,
        const InstallParams& install,
        RadarCalibResult& result);

    /// Calibrate radar using known target (cones or calibration board)
    /// @param radar_points Radar point cloud
    /// @param target_positions Known target positions in VCS
    /// @param install Initial installation parameters
    /// @param result Output calibration result
    /// @return true if calibration successful
    bool calibrateWithTarget(
        const std::vector<Eigen::Vector3d>& radar_points,
        const std::vector<Eigen::Vector3d>& target_positions_vcs,
        const InstallParams& install,
        RadarCalibResult& result);

    /// Refine calibration using optimization
    /// @param radar_points Radar point cloud
    /// @param reference_points Corresponding points in VCS
    /// @param initial Initial calibration
    /// @param result Output refined calibration
    /// @return true if refinement successful
    bool refineCalibration(
        const std::vector<Eigen::Vector3d>& radar_points,
        const std::vector<Eigen::Vector3d>& reference_points_vcs,
        const RadarCalibResult& initial,
        RadarCalibResult& result);

    /// Transform radar points to VCS using calibration
    /// @param radar_points Input points in radar coordinate system
    /// @param calib Calibration parameters
    /// @param vcs_points Output points in VCS
    void transformToVCS(
        const std::vector<Eigen::Vector3d>& radar_points,
        const RadarCalibResult& calib,
        std::vector<Eigen::Vector3d>& vcs_points) const;

    /// Evaluate calibration quality
    /// @param radar_points Radar points
    /// @param reference_points Reference points in VCS
    /// @param calib Calibration to evaluate
    /// @return RMS error in meters
    double evaluateCalibration(
        const std::vector<Eigen::Vector3d>& radar_points,
        const std::vector<Eigen::Vector3d>& reference_points_vcs,
        const RadarCalibResult& calib) const;

private:
    /// Detect ground plane from radar points
    /// @param points Input radar points
    /// @param plane_coeffs Output plane coefficients [a,b,c,d] for ax+by+cz+d=0
    /// @param inliers Output inlier indices
    /// @return true if plane detected
    bool detectGroundPlane(
        const std::vector<Eigen::Vector3d>& points,
        Eigen::Vector4d& plane_coeffs,
        std::vector<int>& inliers);

    /// Estimate radar pose from ground plane
    /// @param plane_coeffs Plane coefficients in radar coordinate system
    /// @param install Initial installation
    /// @param result Output calibration
    /// @return true if pose estimation successful
    bool estimatePoseFromGround(
        const Eigen::Vector4d& plane_coeffs,
        const InstallParams& install,
        RadarCalibResult& result);

    /// Convert Euler angles to rotation matrix (ZYX order)
    Eigen::Matrix3d eulerToRot(double roll, double pitch, double yaw) const;

    /// Convert rotation matrix to Euler angles (ZYX order)
    void rotToEuler(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw) const;

    Params params_;
};

} // namespace radar_cali