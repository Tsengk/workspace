#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "camera_model/mei.h"

namespace bev_surround_view {

struct FisheyeAutoCameraConfig {
    std::string role;
    std::string camera_name;
    std::string image_topic;
    cv::Mat K;
    cv::Mat new_K;
    cv::Mat D;
    double epsilon = 0.0;
    int width = 0;
    int height = 0;
    int new_width = 0;
    int new_height = 0;
    std::vector<double> t;
    std::vector<double> r_euler;
    double vcs_to_ground_height = 0.0;
    std::array<int, 3> rotation_order = {{2, 1, 0}};
};

struct FisheyePoseConfig {
    cv::Mat T_c_w = cv::Mat::eye(4, 4, CV_64F);
    std::vector<double> t_values;
    std::vector<double> r_values;
    double reproj_rmse_px = -1.0;
    bool valid = false;
};

struct FisheyeDetectionResult {
    bool valid = false;
    std::vector<cv::Point2f> undistorted_points;
    std::vector<cv::Point2f> distorted_points;
    cv::Mat debug_image;
};

struct FisheyeBevRenderOutput {
    bool valid = false;
    std::map<std::string, cv::Mat> bev_images;
    cv::Mat fused_bev_image;
};

class FisheyeAutoCalibrator {
public:
    explicit FisheyeAutoCalibrator(const std::array<FisheyeAutoCameraConfig, 4>& cameras);

    struct Options {
        double detector_score = 0.6;
        double solve_ground_z = 0.0;
        double ground_offset_z_m = 0.37;
        double back_sort_threshold = 0.15;
        bool enable_joint_optimization = true;
        double bev_scale = 50.0;
        int bev_width = 2000;
        int bev_height = 2000;
        double bev_range_front = 0.0;
        double bev_range_rear = 0.0;
        double bev_range_left = 0.0;
        double bev_range_right = 0.0;
        bool enable_roi_crop = true;
        bool bev_range_front_auto = true;
        bool bev_range_rear_auto = true;
        bool bev_range_left_auto = true;
        bool bev_range_right_auto = true;
        double origin_offset_x = 0.0;
        double origin_offset_y = 0.0;
        bool save_debug_images = true;
        bool save_bev_images = true;
        std::string debug_dir;
        std::string output_front_yaml_path;
        std::string output_rear_yaml_path;
        std::string output_left_yaml_path;
        std::string output_right_yaml_path;
        std::string chessboard_yaml_path;
        int debug_boundary_scale = 4;
    };

    struct Output {
        bool valid = false;
        std::string message;
        double shared_reproj_rmse_px = -1.0;
        std::map<std::string, FisheyePoseConfig> poses;
        std::map<std::string, std::vector<cv::Point3f> > object_points_3d;
        std::map<std::string, cv::Mat> corner_debug_images;
        std::map<std::string, cv::Mat> bev_images;
        cv::Mat fused_bev_image;
    };

    bool detect(const FisheyeAutoCameraConfig& camera,
                const cv::Mat& image,
                const Options& options,
                FisheyeDetectionResult& detection,
                std::string& error) const;

    bool calibrate(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                   const std::array<FisheyeDetectionResult, 4>& detections,
                   const Options& options,
                   Output& output);

    bool renderBevImages(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                         const Output& calibration_output,
                         Options& options,
                         const std::array<cv::Mat, 4>& images,
                         bool refresh_ipm_info,
                         FisheyeBevRenderOutput& render_output,
                         std::string& error);

    bool refreshIpmCameras(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                           const Output& calibration_output,
                           Options& options,
                           std::string& error);

    Eigen::Vector3d getCameraEulerRadians(const std::string& camera_name, const cv::Mat& R_c_w);
    FisheyePoseConfig resolvePoseConfig(const std::string& camera_name,
                                        const cv::Mat& T_c_w,
                                        const cv::Mat& T_init,
                                        double vehicle_center_x_m,
                                        double ground_offset_z_m,
                                        double reproj_rmse_px);

    bool savePoseYaml(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                      const Output& output,
                      const Options& options,
                      std::string& error) const;

    bool getIpmMaps(const std::string& camera_name, cv::Mat& map_x, cv::Mat& map_y);

    struct IpmRoiInfo {
        int x = 0;
        int y = 0;
        int width = 0;
        int height = 0;
    };
    bool getIpmRoi(const std::string& camera_name, IpmRoiInfo& roi_info);

private:
    std::map<std::string, camera::Mei> ipm_cameras_;
};

}  // namespace bev_surround_view
