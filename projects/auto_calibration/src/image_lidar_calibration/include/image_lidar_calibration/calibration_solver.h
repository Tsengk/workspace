#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

#include "image_lidar_calibration/types.h"

namespace image_lidar_calibration {

class CalibrationSolver {
public:
    struct Params {
        int min_scenes = 1;
        bool use_prior_pairing = true;
        double prior_x_m = 0.0;
        double prior_y_m = 0.0;
        double prior_z_m = 0.0;
        double prior_roll_deg = 0.0;
        double prior_pitch_deg = 0.0;
        double prior_yaw_deg = 0.0;
        double max_pairing_rmse_m = 1.0;
        double max_translation_delta_from_prior_m = 0.05;
        double max_rotation_delta_from_prior_deg = 0.0;
        double target_delta_width_circles_m = 0.50;
        double target_delta_height_circles_m = 0.40;
        
        // Ceres优化参数
        bool use_ceres_optimization = true;
        int optimization_max_iterations = 200;
        
        // xyz边界约束参数（仅在 use_ceres_optimization=true 时生效）
        double xyz_delta_limit_m = 0.03;
        bool separate_axis_constraint = false;
        
        // 优化权重参数
        double lidar_point_prior_weight = 1.0;
        double lidar_geometry_weight = 10.0;
        double pose_rotation_prior_weight = 0.0;
        
        bool use_ransac_rejection = true;
        double ransac_reproj_threshold_px = 6.0;
        int ransac_min_inliers = 4;
        int ransac_min_scene_inliers = 3;
    };

    CalibrationSolver();
    void setParams(const Params& p);
    void setCamera(const CameraIntrinsics& cam);

    bool solve(const std::vector<SceneData>& scenes,
               CalibResult& result);

    static double computeRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt);

    static void sortPatternCenters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& out,
                                   const std::string& axis_mode = "camera");

private:
    Params params_;
    CameraIntrinsics cam_;
    bool camera_ready_ = false;

    static void extractEulerZYX(const Eigen::Matrix3d& R,
                                double& roll,
                                double& pitch,
                                double& yaw);
};

}  // namespace image_lidar_calibration
