#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <vector>
#include "image_lidar_calibration/types.h"

namespace image_lidar_calibration {

// 基于ArUco标记的标定板圆孔中心检测器
// 参照 FAST-Calib demo (qr_detect.hpp) 实现
// 检测4个ArUco标记 → estimatePoseBoard → 计算4个圆孔中心在相机坐标系中的3D坐标
// 同时输出由当前 board pose 反投影得到的 4 个伪圆心像素，用于过渡版 PnP 标定
class FastLivo2Detector {
public:
    FastLivo2Detector();

    void setTarget(const TargetParams& target);
    void setCamera(const CameraIntrinsics& cam);

    bool detectWithPose(const cv::Mat& image,
                        std::vector<Eigen::Vector3d>& circle_centers_cam,
                        std::vector<cv::Point2f>& circle_pixels,
                        BoardPose& board_pose,
                        cv::Mat& debug_image);

private:
    TargetParams target_;
    CameraIntrinsics cam_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    cv::Mat mei_map1_, mei_map2_;
    cv::Mat mei_virtual_K_;
    bool mei_maps_valid_ = false;

    void buildMEIUndistortMaps();

    bool verifyGeometry(const std::vector<Eigen::Vector3d>& centers) const;
};

} // namespace image_lidar_calibration
