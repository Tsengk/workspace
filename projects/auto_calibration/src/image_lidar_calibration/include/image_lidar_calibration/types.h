#pragma once
#define PCL_NO_PRECOMPILE

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace image_lidar_calibration {

struct PointXYZIR {
    PCL_ADD_POINT4D;
    float         intensity = 0.f;
    std::uint16_t ring      = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct TargetParams {
    double marker_size            = 0.20;
    double delta_width_qr_center  = 0.55;
    double delta_height_qr_center = 0.35;
    double delta_width_circles    = 0.50;
    double delta_height_circles   = 0.40;
    double circle_radius          = 0.12;
    int    min_detected_markers   = 3;
};

struct BoardPose {
    double x = 0, y = 0, z = 0;
    double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;

    double rotationDiffDeg(const BoardPose& other) const {
        auto eulerToRot = [](double r, double p, double y) {
            double cr = cos(r * M_PI / 180.0), sr = sin(r * M_PI / 180.0);
            double cp = cos(p * M_PI / 180.0), sp = sin(p * M_PI / 180.0);
            double cy = cos(y * M_PI / 180.0), sy = sin(y * M_PI / 180.0);
            Eigen::Matrix3d R;
            R(0,0) = cy*cp;  R(0,1) = cy*sp*sr - sy*cr; R(0,2) = cy*sp*cr + sy*sr;
            R(1,0) = sy*cp;  R(1,1) = sy*sp*sr + cy*cr; R(1,2) = sy*sp*cr - cy*sr;
            R(2,0) = -sp;    R(2,1) = cp*sr;            R(2,2) = cp*cr;
            return R;
        };
        Eigen::Matrix3d R1 = eulerToRot(roll_deg, pitch_deg, yaw_deg);
        Eigen::Matrix3d R2 = eulerToRot(other.roll_deg, other.pitch_deg, other.yaw_deg);
        Eigen::Matrix3d R_err = R1.transpose() * R2;
        double cos_val = std::max(-1.0, std::min(1.0, (R_err.trace() - 1.0) * 0.5));
        return std::acos(cos_val) * 180.0 / M_PI;
    }

    double positionDiffM(const BoardPose& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

struct SceneData {
    std::vector<Eigen::Vector3d> camera_centers;
    std::vector<cv::Point2f> camera_circle_pixels;
    std::vector<Eigen::Vector3d> lidar_centers;
    BoardPose board_pose;
    bool valid = false;
};

struct CalibResult {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    double x_m = 0, y_m = 0, z_m = 0;
    double rmse_m     = 0;
    double raw_rmse_m = 0;
    double reproj_rmse_px = 0;
    int    num_scenes = 0;
    int    accepted_scenes = 0;
    int    history_filtered_scenes = 0;
    int    total_keyframes = 0;
    int    solver_input_scenes = 0;
    int    solver_dropped_scenes = 0;
    int    optimization_scenes = 0;
    std::vector<int> optimization_scene_indices;
    bool   valid      = false;
    std::string message;
    std::string quality_grade = "POOR";
    double confidence = 0.0;
    std::string stage_debug_report;
};

enum CameraModel { PINHOLE, MEI };

struct CameraIntrinsics {
    CameraModel camera_model = PINHOLE;
    double xi = 0.0;
    double fx = 1000, fy = 1000, cx = 640, cy = 360;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0;
    int    width = 1280, height = 720;

    cv::Mat getCameraMatrix() const {
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        K.at<double>(0, 0) = fx;  K.at<double>(0, 2) = cx;
        K.at<double>(1, 1) = fy;  K.at<double>(1, 2) = cy;
        return K;
    }
    cv::Mat getDistCoeffs() const {
        return (cv::Mat_<double>(1, 4) << k1, k2, p1, p2);
    }
    cv::Mat getPnPCameraMatrix() const {
        if (camera_model == MEI) {
            return (cv::Mat_<double>(3, 3) <<
                1000.0, 0.0, static_cast<double>(width) * 0.5,
                0.0, 1000.0, static_cast<double>(height) * 0.5,
                0.0, 0.0, 1.0);
        }
        return getCameraMatrix();
    }
    cv::Mat getPnPDistCoeffs() const {
        if (camera_model == MEI) {
            return cv::Mat::zeros(1, 4, CV_64F);
        }
        return getDistCoeffs();
    }

    void buildUndistortMaps(cv::Mat& map1, cv::Mat& map2) const {
        if (camera_model == MEI) {
            map1.create(height, width, CV_32FC1);
            map2.create(height, width, CV_32FC1);
            for (int v = 0; v < height; ++v) {
                for (int u = 0; u < width; ++u) {
                    const double x = (u - cx) / fx;
                    const double y = (v - cy) / fy;
                    const double r = std::sqrt(x*x + y*y + 1.0);
                    const double xs = x/r, ys = y/r, zs = 1.0/r;
                    const double denom = zs + xi;
                    const double mx = xs / denom, my = ys / denom;
                    const double r2 = mx*mx + my*my;
                    const double r4 = r2*r2;
                    const double radial = 1.0 + k1*r2 + k2*r4;
                    const double mxd = mx*radial + 2.0*p1*mx*my + p2*(r2+2.0*mx*mx);
                    const double myd = my*radial + p1*(r2+2.0*my*my) + 2.0*p2*mx*my;
                    map1.at<float>(v, u) = static_cast<float>(fx*mxd + cx);
                    map2.at<float>(v, u) = static_cast<float>(fy*myd + cy);
                }
            }
        } else {
            cv::initUndistortRectifyMap(
                getCameraMatrix(), getDistCoeffs(),
                cv::Mat(), getCameraMatrix(),
                cv::Size(width, height), CV_32FC1, map1, map2);
        }
    }

    bool loadFromFile(const std::string& path) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        std::string model_str = "PINHOLE";
        fs["camera_model"] >> model_str;
        if (model_str.empty()) {
            fs["model_type"] >> model_str;
        }
        camera_model = (model_str == "MEI") ? MEI : PINHOLE;

        fs["image_width"] >> width;
        fs["image_height"] >> height;

        cv::FileNode mirror_node = fs["mirror_parameters"];
        if (!mirror_node.empty()) {
            mirror_node["xi"] >> xi;
        } else if (camera_model == MEI) {
            fs["xi"] >> xi;
        }

        cv::FileNode projection_node = fs["projection_parameters"];
        if (!projection_node.empty()) {
            projection_node["gamma1"] >> fx;
            projection_node["gamma2"] >> fy;
            projection_node["u0"] >> cx;
            projection_node["v0"] >> cy;
        } else {
            fs["fx"] >> fx;
            fs["fy"] >> fy;
            fs["cx"] >> cx;
            fs["cy"] >> cy;
        }

        cv::FileNode distortion_node = fs["distortion_parameters"];
        if (!distortion_node.empty()) {
            distortion_node["k1"] >> k1;
            distortion_node["k2"] >> k2;
            distortion_node["p1"] >> p1;
            distortion_node["p2"] >> p2;
        } else {
            fs["k1"] >> k1;
            fs["k2"] >> k2;
            fs["p1"] >> p1;
            fs["p2"] >> p2;
        }
        return true;
    }
};

struct InstallParams {
    std::string camera_name = "camera";
    double x = 0, y = 0, z = 0;
    double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
};

struct CalibOutput {
    InstallParams    installation;
    CameraIntrinsics intrinsics;
    CalibResult      result;
    std::string      output_dir;
    std::string      yaml_path;
};

} // namespace image_lidar_calibration

POINT_CLOUD_REGISTER_POINT_STRUCT(image_lidar_calibration::PointXYZIR,
    (float,          x,         x)
    (float,          y,         y)
    (float,          z,         z)
    (float,          intensity, intensity)
    (std::uint16_t,  ring,      ring)
)
