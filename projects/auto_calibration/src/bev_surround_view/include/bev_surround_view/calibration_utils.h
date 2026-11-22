#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "bev_surround_view/fisheye_auto_calibrator.h"

namespace YAML { class Node; }

namespace bev_surround_view {

// --- Math / conversion utilities ---
double distortionCoeff(const cv::Mat& D, int idx);
double deg(double rad);
double round2(double value);
double clamp(double value, double low, double high);
double normalizeAngleRad(double angle);

Eigen::Vector3d toEigenVector3d(const std::vector<double>& values, double y_offset);
Eigen::Vector3d toEigenEulerRadians(const std::vector<double>& values_deg);

bool isSameDoubleValue(double lhs, double rhs);
bool isSameVector3d(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs);
bool isSameMatValue(const cv::Mat& lhs, const cv::Mat& rhs);

camera::Mei initMeiCamera(const FisheyeAutoCameraConfig& cam);

// --- Projection helpers ---
bool projectCameraPointToRawPixel(const FisheyeAutoCameraConfig& cam,
                                  const cv::Vec3d& point_camera,
                                  cv::Point2f& pixel);

bool projectWorldPointToRawPixel(const FisheyeAutoCameraConfig& cam,
                                 const cv::Mat& T_c_w,
                                 const cv::Point3f& point_world,
                                 cv::Point2f& pixel);

void rawProjectionResidualOrPenalty(const FisheyeAutoCameraConfig& cam,
                                    const cv::Mat& T_c_w,
                                    const cv::Point3f& object_point,
                                    const cv::Point2f& observed_pixel,
                                    double& dx,
                                    double& dy);

// --- IPM / scan utilities ---
bool isValidMapPixel(const cv::Mat& map_x, const cv::Mat& map_y, int x, int y);

int scanFirstValidUp(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y);
int scanFirstValidDown(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y);
int scanFirstValidLeft(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y);
int scanFirstValidRight(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y);

// --- File utilities ---
bool ensureDirectory(const std::string& path);
bool ensureParentDirectory(const std::string& file_path);
bool saveImageIfNeeded(const std::string& path, const cv::Mat& image);
bool loadYamlFromExistingFile(const std::string& path,
                              const std::string& node_name,
                              YAML::Node& node,
                              std::string& error);

// --- Eigen <-> cv::Mat ---
Eigen::Matrix3d cvMatToEigenMatrix3d(const cv::Mat& matrix);
cv::Mat eigenMatrix3dToCvMat(const Eigen::Matrix3d& matrix);

// --- String ---
std::string joinMessage(const std::vector<std::string>& parts);

}  // namespace bev_surround_view
