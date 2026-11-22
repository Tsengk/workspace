#include "bev_surround_view/calibration_utils.h"

#include <cerrno>
#include <cmath>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace bev_surround_view {

double distortionCoeff(const cv::Mat& D, int idx)
{
    if (D.empty()) {
        return 0.0;
    }
    if (D.rows == 1 && idx < D.cols) {
        return D.at<double>(0, idx);
    }
    if (D.cols == 1 && idx < D.rows) {
        return D.at<double>(idx, 0);
    }
    return 0.0;
}

bool projectCameraPointToRawPixel(const FisheyeAutoCameraConfig& cam,
                                  const cv::Vec3d& point_camera,
                                  cv::Point2f& pixel)
{
    if (cam.K.empty()) {
        return false;
    }

    const double x = point_camera[0];
    const double y = point_camera[1];
    const double z = point_camera[2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 1e-9) {
        return false;
    }

    const double norm = cv::norm(point_camera);
    if (!std::isfinite(norm)) {
        return false;
    }
    const double denom = z + cam.epsilon * norm;
    if (!std::isfinite(denom) || std::abs(denom) <= 1e-9) {
        return false;
    }

    const double xu = x / denom;
    const double yu = y / denom;
    if (!std::isfinite(xu) || !std::isfinite(yu)) {
        return false;
    }

    const double k1 = distortionCoeff(cam.D, 0);
    const double k2 = distortionCoeff(cam.D, 1);
    const double p1 = distortionCoeff(cam.D, 2);
    const double p2 = distortionCoeff(cam.D, 3);
    const double r2 = xu * xu + yu * yu;
    const double r4 = r2 * r2;
    const double radial = 1.0 + k1 * r2 + k2 * r4;
    const double xd = xu * radial + 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * xu * xu);
    const double yd = yu * radial + p1 * (r2 + 2.0 * yu * yu) + 2.0 * p2 * xu * yu;
    if (!std::isfinite(xd) || !std::isfinite(yd)) {
        return false;
    }

    const double fx = cam.K.at<double>(0, 0);
    const double fy = cam.K.at<double>(1, 1);
    const double cx = cam.K.at<double>(0, 2);
    const double cy = cam.K.at<double>(1, 2);
    if (std::abs(fx) <= 1e-9 || std::abs(fy) <= 1e-9) {
        return false;
    }

    pixel.x = static_cast<float>(fx * xd + cx);
    pixel.y = static_cast<float>(fy * yd + cy);
    return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

bool projectWorldPointToRawPixel(const FisheyeAutoCameraConfig& cam,
                                 const cv::Mat& T_c_w,
                                 const cv::Point3f& point_world,
                                 cv::Point2f& pixel)
{
    if (T_c_w.empty() || T_c_w.rows != 4 || T_c_w.cols != 4) {
        return false;
    }

    const cv::Mat R = T_c_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t = T_c_w(cv::Rect(3, 0, 1, 3));
    const cv::Mat point_w = (cv::Mat_<double>(3, 1) << point_world.x,
                                                      point_world.y,
                                                      point_world.z);
    const cv::Mat point_c = R * point_w + t;
    return projectCameraPointToRawPixel(
        cam,
        cv::Vec3d(point_c.at<double>(0, 0),
                  point_c.at<double>(1, 0),
                  point_c.at<double>(2, 0)),
        pixel);
}

static const double kInvalidRawProjectionResidualPx = 10000.0;

void rawProjectionResidualOrPenalty(const FisheyeAutoCameraConfig& cam,
                                    const cv::Mat& T_c_w,
                                    const cv::Point3f& object_point,
                                    const cv::Point2f& observed_pixel,
                                    double& dx,
                                    double& dy)
{
    cv::Point2f projected;
    if (projectWorldPointToRawPixel(cam, T_c_w, object_point, projected)) {
        dx = static_cast<double>(projected.x) - static_cast<double>(observed_pixel.x);
        dy = static_cast<double>(projected.y) - static_cast<double>(observed_pixel.y);
        if (std::isfinite(dx) && std::isfinite(dy)) {
            return;
        }
    }

    dx = kInvalidRawProjectionResidualPx;
    dy = kInvalidRawProjectionResidualPx;
}

Eigen::Vector3d toEigenVector3d(const std::vector<double>& values, double y_offset)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    if (values.size() >= 1) {
        result.x() = values[0];
    }
    if (values.size() >= 2) {
        result.y() = values[1] + y_offset;
    } else {
        result.y() = y_offset;
    }
    if (values.size() >= 3) {
        result.z() = values[2];
    }
    return result;
}

Eigen::Vector3d toEigenEulerRadians(const std::vector<double>& values_deg)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    if (values_deg.size() >= 1) {
        result.x() = values_deg[0] * M_PI / 180.0;
    }
    if (values_deg.size() >= 2) {
        result.y() = values_deg[1] * M_PI / 180.0;
    }
    if (values_deg.size() >= 3) {
        result.z() = values_deg[2] * M_PI / 180.0;
    }
    return result;
}

camera::Mei initMeiCamera(const FisheyeAutoCameraConfig& cam)
{
    camera::Mei camera_model(cam.rotation_order);
    if (cam.K.empty() || cam.width <= 0 || cam.height <= 0 || cam.t.size() < 3 || cam.r_euler.size() < 3) {
        return camera_model;
    }

    camera_model.setIntrinsics(cam.epsilon,
                               cam.K,
                               cam.D,
                               cam.width,
                               cam.height);
    camera_model.setGroundHeight(cam.vcs_to_ground_height);
    camera_model.setExtrinsics(toEigenVector3d(cam.t, cam.vcs_to_ground_height),
                               toEigenEulerRadians(cam.r_euler));
    return camera_model;
}

bool isSameDoubleValue(double lhs, double rhs)
{
    return std::abs(lhs - rhs) <= 1e-12;
}

bool isSameVector3d(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
    return (lhs - rhs).cwiseAbs().maxCoeff() <= 1e-12;
}

bool isSameMatValue(const cv::Mat& lhs, const cv::Mat& rhs)
{
    if (lhs.empty() && rhs.empty()) {
        return true;
    }
    if (lhs.empty() || rhs.empty()) {
        return false;
    }
    if (lhs.rows != rhs.rows || lhs.cols != rhs.cols || lhs.type() != rhs.type()) {
        return false;
    }
    return cv::norm(lhs, rhs, cv::NORM_INF) <= 1e-12;
}

double deg(double rad)
{
    return rad * 180.0 / M_PI;
}

double round2(double value)
{
    return std::round(value * 100.0) / 100.0;
}

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

bool isValidMapPixel(const cv::Mat& map_x, const cv::Mat& map_y, int x, int y)
{
    return x >= 0 && x < map_x.cols && y >= 0 && y < map_x.rows &&
           map_x.at<float>(y, x) >= 0.0f && map_y.at<float>(y, x) >= 0.0f;
}

int scanFirstValidUp(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y)
{
    for (int y = center_y; y >= 0; --y) {
        if (isValidMapPixel(map_x, map_y, center_x, y)) {
            return y;
        }
    }
    return 0;
}

int scanFirstValidDown(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y)
{
    for (int y = center_y; y < map_x.rows; ++y) {
        if (isValidMapPixel(map_x, map_y, center_x, y)) {
            return y;
        }
    }
    return map_x.rows;
}

int scanFirstValidLeft(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y)
{
    for (int x = center_x; x >= 0; --x) {
        if (isValidMapPixel(map_x, map_y, x, center_y)) {
            return x;
        }
    }
    return 0;
}

int scanFirstValidRight(const cv::Mat& map_x, const cv::Mat& map_y, int center_x, int center_y)
{
    for (int x = center_x; x < map_x.cols; ++x) {
        if (isValidMapPixel(map_x, map_y, x, center_y)) {
            return x;
        }
    }
    return map_x.cols;
}

bool ensureDirectory(const std::string& path)
{
    if (path.empty() || path == "/") {
        return true;
    }
    if (access(path.c_str(), F_OK) == 0) {
        return true;
    }

    const std::string::size_type pos = path.find_last_of("/");
    if (pos != std::string::npos) {
        const std::string parent = pos == 0 ? "/" : path.substr(0, pos);
        if (!parent.empty() && !ensureDirectory(parent)) {
            return false;
        }
    }

    if (mkdir(path.c_str(), 0777) == 0) {
        return true;
    }
    return errno == EEXIST;
}

bool ensureParentDirectory(const std::string& file_path)
{
    const std::string::size_type pos = file_path.find_last_of("/");
    if (pos == std::string::npos) {
        return true;
    }
    return ensureDirectory(pos == 0 ? "/" : file_path.substr(0, pos));
}

double normalizeAngleRad(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

Eigen::Matrix3d cvMatToEigenMatrix3d(const cv::Mat& matrix)
{
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            result(r, c) = matrix.at<double>(r, c);
        }
    }
    return result;
}

cv::Mat eigenMatrix3dToCvMat(const Eigen::Matrix3d& matrix)
{
    cv::Mat result(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            result.at<double>(r, c) = matrix(r, c);
        }
    }
    return result;
}

bool saveImageIfNeeded(const std::string& path, const cv::Mat& image)
{
    if (path.empty() || image.empty()) {
        return true;
    }
    if (ensureParentDirectory(path) == false) {
        return false;
    }
    return cv::imwrite(path, image);
}

std::string joinMessage(const std::vector<std::string>& parts)
{
    std::ostringstream oss;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i > 0) {
            oss << " | ";
        }
        oss << parts[i];
    }
    return oss.str();
}

bool loadYamlFromExistingFile(const std::string& path,
                              YAML::Node& root,
                              std::string& error)
{
    if (path.empty() || access(path.c_str(), F_OK) != 0) {
        return false;
    }

    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        error = e.what();
        return false;
    }
    return true;
}

}  // namespace bev_surround_view
