#pragma once

#include <vector>
#include <opencv2/core.hpp>

#include "bev_surround_view/fisheye_auto_calibrator.h"

namespace bev_surround_view {

// Pose matrix construction
cv::Mat buildPoseMatrix(const cv::Mat& rvec, const cv::Mat& tvec);
cv::Mat buildPoseMatrixWithFixedCenter(const cv::Mat& rvec, const cv::Mat& tvec,
                                       const cv::Vec3d& fixed_center_w);
cv::Mat buildPoseMatrixFromJointCameraPose(const struct JointCameraPoseState& pose_state);
cv::Mat buildRotationMatrixFromOpenCvRpy(double roll_rad, double pitch_rad, double yaw_rad);
bool rotationMatrixToOpenCvRpy(const cv::Mat& R, double& roll_rad, double& pitch_rad, double& yaw_rad);

// Pose solving
bool solvePose(const std::vector<cv::Point3f>& object_points,
               const std::vector<cv::Point2f>& image_points,
               const FisheyeAutoCameraConfig& cam,
               cv::Mat& T_c_w,
               double& reproj_rmse_px);
bool solveUndistortedPinholePoseForSorting(const std::vector<cv::Point3f>& object_points,
                                           const std::vector<cv::Point2f>& image_points,
                                           const FisheyeAutoCameraConfig& cam,
                                           cv::Mat& T_c_w,
                                           double& reproj_rmse_px);

// Camera center / vehicle
cv::Vec3d cameraCenterFromPose(const cv::Mat& T_c_w);
double computeVehicleCenterX(const cv::Mat& front_T_c_w, const cv::Mat& back_T_c_w);

// Back projection
std::vector<cv::Point3f> backProjectToGround(const std::vector<cv::Point2f>& points,
                                             const cv::Mat& K,
                                             const cv::Mat& T_c_w,
                                             double z_world);
bool backProjectDistortedToGround(const std::vector<cv::Point2f>& image_points,
                                  const FisheyeAutoCameraConfig& cam,
                                  const cv::Mat& T_c_w,
                                  double ground_z,
                                  std::vector<cv::Point3f>& ground_points);

// Grid/point sorting
std::vector<cv::Point3f> sortPointsGrid(const std::vector<cv::Point3f>& points,
                                        int primary_axis,
                                        double threshold,
                                        bool reverse_primary,
                                        bool reverse_secondary);

// Pose extraction
bool extractJointCameraPoseState(const cv::Mat& T_c_w,
                                 struct JointCameraPoseState& pose_state);
void poseMatrixToRt(const cv::Mat& T_c_w, cv::Mat& rvec, cv::Mat& tvec);

// Middle board / plane
bool solveMiddleBoardPoseForPlane(const std::vector<cv::Point2f>& ordered_image_points,
                                  int rows, int cols,
                                  const FisheyeAutoCameraConfig& cam,
                                  cv::Mat& homography);
std::vector<cv::Point3f> imagePointsToPlane(const std::vector<cv::Point2f>& image_points,
                                            const cv::Mat& homography);
std::vector<size_t> recoverPlaneGridTopologyIndices(const std::vector<cv::Point3f>& plane_points,
                                                    int rows, int cols);

// Forward declarations for types used by this module
struct JointCameraPoseState {
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
};

}  // namespace bev_surround_view
