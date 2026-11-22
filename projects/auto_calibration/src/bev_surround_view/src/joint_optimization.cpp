#include "bev_surround_view/joint_optimization.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>

#include "bev_surround_view/calibration_utils.h"
#include "bev_surround_view/pose_utils.h"
#include "camera_model/mei.h"

namespace bev_surround_view {

// ---------------------------------------------------------------------------
// buildCanonicalBoardObjectPoints
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> buildCanonicalBoardObjectPoints(int rows, int cols)
{
    std::vector<cv::Point3f> object_points;
    object_points.reserve(static_cast<size_t>(rows * cols));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            object_points.emplace_back(-static_cast<float>(r),
                                       -static_cast<float>(c),
                                       0.0f);
        }
    }
    return object_points;
}

// ---------------------------------------------------------------------------
// applyMiddleBoardTopology
// ---------------------------------------------------------------------------
std::vector<cv::Point2f> applyMiddleBoardTopology(const std::vector<cv::Point2f>& ordered_image_points,
                                                  int rows,
                                                  int cols,
                                                  const std::string& topology_name)
{
    cv::Mat grid(rows, cols, CV_32FC2);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            grid.at<cv::Vec2f>(r, c) = cv::Vec2f(
                ordered_image_points[static_cast<size_t>(r * cols + c)].x,
                ordered_image_points[static_cast<size_t>(r * cols + c)].y);
        }
    }

    std::vector<cv::Point2f> candidate;
    candidate.reserve(ordered_image_points.size());
    if (topology_name == "flip_lr") {
        for (int r = 0; r < rows; ++r) {
            for (int c = cols - 1; c >= 0; --c) {
                const cv::Vec2f pt = grid.at<cv::Vec2f>(r, c);
                candidate.emplace_back(pt[0], pt[1]);
            }
        }
    } else if (topology_name == "flip_ud") {
        for (int r = rows - 1; r >= 0; --r) {
            for (int c = 0; c < cols; ++c) {
                const cv::Vec2f pt = grid.at<cv::Vec2f>(r, c);
                candidate.emplace_back(pt[0], pt[1]);
            }
        }
    } else if (topology_name == "rotate_180") {
        for (int r = rows - 1; r >= 0; --r) {
            for (int c = cols - 1; c >= 0; --c) {
                const cv::Vec2f pt = grid.at<cv::Vec2f>(r, c);
                candidate.emplace_back(pt[0], pt[1]);
            }
        }
    } else {
        candidate = ordered_image_points;
    }
    return candidate;
}

// ---------------------------------------------------------------------------
// expectedJointParamCount
// ---------------------------------------------------------------------------
size_t expectedJointParamCount(const JointOptimizationProblem&)
{
    return 18;
}

// ---------------------------------------------------------------------------
// buildJointBoardObjectPoints
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> buildJointBoardObjectPoints(const JointBoardObservation& board,
                                                     double origin_x,
                                                     double origin_y,
                                                     double yaw_rad,
                                                     double spacing_m,
                                                     double ground_z)
{
    std::vector<cv::Point3f> points;
    if (board.rows <= 0 || board.cols <= 0) {
        return points;
    }

    points.reserve(static_cast<size_t>(board.rows * board.cols));
    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);
    const cv::Point2d row_dir(cos_yaw, sin_yaw);
    const cv::Point2d col_dir = board.col_sign >= 0
        ? cv::Point2d(-sin_yaw, cos_yaw)
        : cv::Point2d(sin_yaw, -cos_yaw);

    for (int r = 0; r < board.rows; ++r) {
        for (int c = 0; c < board.cols; ++c) {
            const cv::Point2d p = cv::Point2d(origin_x, origin_y) +
                                  static_cast<double>(r) * spacing_m * row_dir +
                                  static_cast<double>(c) * spacing_m * col_dir;
            points.emplace_back(static_cast<float>(p.x),
                                static_cast<float>(p.y),
                                static_cast<float>(ground_z));
        }
    }
    return points;
}

// ---------------------------------------------------------------------------
// setInitialJointBoardPoseFromObjectPoints
// ---------------------------------------------------------------------------
bool setInitialJointBoardPoseFromObjectPoints(JointBoardObservation& board,
                                              const std::vector<cv::Point3f>& object_points)
{
    const size_t point_count = static_cast<size_t>(board.rows * board.cols);
    if (board.rows <= 0 || board.cols <= 0 ||
        object_points.size() != point_count ||
        object_points.size() < static_cast<size_t>(board.cols + 1)) {
        return false;
    }

    const cv::Point3f& origin = object_points.front();
    const cv::Point3f& row_point = object_points[static_cast<size_t>(board.cols)];
    const double dx = static_cast<double>(row_point.x) - static_cast<double>(origin.x);
    const double dy = static_cast<double>(row_point.y) - static_cast<double>(origin.y);
    if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx, dy) <= 1e-9) {
        return false;
    }

    board.initial_origin_x = static_cast<double>(origin.x);
    board.initial_origin_y = static_cast<double>(origin.y);
    board.initial_yaw = std::atan2(dy, dx);
    return true;
}

// ---------------------------------------------------------------------------
// initJointBoardObservation
// ---------------------------------------------------------------------------
bool initJointBoardObservation(int rows,
                               int cols,
                               const std::vector<cv::Point2f>& image_points_a,
                               const std::vector<cv::Point2f>& image_points_b,
                               JointBoardObservation& board)
{
    if (rows <= 0 || cols <= 0) {
        return false;
    }
    const size_t point_count = static_cast<size_t>(rows * cols);
    if (image_points_a.size() != point_count ||
        image_points_b.size() != point_count) {
        return false;
    }

    board = JointBoardObservation();
    board.rows = rows;
    board.cols = cols;
    board.image_points_a = image_points_a;
    board.image_points_b = image_points_b;
    return true;
}

// ---------------------------------------------------------------------------
// appendJointBoardParams
// ---------------------------------------------------------------------------
void appendJointBoardParams(std::vector<double>& params,
                            const JointBoardObservation& board)
{
    params.push_back(board.initial_origin_x);
    params.push_back(board.initial_origin_y);
    params.push_back(board.initial_yaw);
}

// ---------------------------------------------------------------------------
// packJointPoseParams
// ---------------------------------------------------------------------------
std::vector<double> packJointPoseParams(const cv::Mat& front_T_c_w,
                                        const cv::Mat& left_T_c_w,
                                        const cv::Mat& right_T_c_w,
                                        const cv::Mat& back_T_c_w,
                                        const JointOptimizationProblem& problem)
{
    (void)front_T_c_w;
    std::vector<double> params;
    params.reserve(expectedJointParamCount(problem));

    const cv::Mat poses[3] = {left_T_c_w, right_T_c_w, back_T_c_w};
    for (int pose_idx = 0; pose_idx < 3; ++pose_idx) {
        JointCameraPoseState pose_state;
        if (!extractJointCameraPoseState(poses[pose_idx], pose_state)) {
            return std::vector<double>();
        }
        params.push_back(pose_state.center_x);
        params.push_back(pose_state.center_y);
        params.push_back(pose_state.center_z);
        params.push_back(pose_state.roll_rad);
        params.push_back(pose_state.pitch_rad);
        params.push_back(pose_state.yaw_rad);
    }

    return params;
}

// ---------------------------------------------------------------------------
// unpackJointPoseParams
// ---------------------------------------------------------------------------
bool unpackJointPoseParams(const JointOptimizationProblem& problem,
                           const std::vector<double>& params,
                           JointOptimizationEvalResult& result)
{
    if (params.size() != expectedJointParamCount(problem)) {
        return false;
    }

    result.front_T_c_w = problem.front_T_c_w.clone();

    cv::Mat poses[3];
    for (int pose_idx = 0; pose_idx < 3; ++pose_idx) {
        const size_t base = static_cast<size_t>(pose_idx) * 6;
        JointCameraPoseState pose_state;
        pose_state.center_x = params[base + 0];
        pose_state.center_y = params[base + 1];
        pose_state.center_z = params[base + 2];
        pose_state.roll_rad = normalizeAngleRad(params[base + 3]);
        pose_state.pitch_rad = normalizeAngleRad(params[base + 4]);
        pose_state.yaw_rad = normalizeAngleRad(params[base + 5]);
        poses[pose_idx] = buildPoseMatrixFromJointCameraPose(pose_state);
    }

    result.left_T_c_w = poses[0];
    result.right_T_c_w = poses[1];
    result.back_T_c_w = poses[2];
    return true;
}

// ---------------------------------------------------------------------------
// buildJointBoardObjectPointsFromParams
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> buildJointBoardObjectPointsFromParams(const JointBoardObservation& board,
                                                               const std::vector<double>& params,
                                                               size_t base,
                                                               double spacing_m,
                                                               double ground_z)
{
    if (params.size() < base + 3) {
        return std::vector<cv::Point3f>();
    }
    return buildJointBoardObjectPoints(board,
                                       params[base],
                                       params[base + 1],
                                       params[base + 2],
                                       spacing_m,
                                       ground_z);
}

// ---------------------------------------------------------------------------
// imagePointToUnitRay
// ---------------------------------------------------------------------------
bool imagePointToUnitRay(const FisheyeAutoCameraConfig& cam,
                         const cv::Point2f& image_point,
                         cv::Vec3d& ray)
{
    if (cam.K.empty()) {
        return false;
    }
    const double fx = cam.K.at<double>(0, 0);
    const double fy = cam.K.at<double>(1, 1);
    const double cx = cam.K.at<double>(0, 2);
    const double cy = cam.K.at<double>(1, 2);
    if (std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9) {
        return false;
    }
    ray = cv::Vec3d((static_cast<double>(image_point.x) - cx) / fx,
                    (static_cast<double>(image_point.y) - cy) / fy,
                    1.0);
    const double norm = cv::norm(ray);
    if (norm < 1e-12 || !std::isfinite(norm)) {
        return false;
    }
    ray *= 1.0 / norm;
    return true;
}

// ---------------------------------------------------------------------------
// worldPointToUnitRay
// ---------------------------------------------------------------------------
bool worldPointToUnitRay(const cv::Mat& T_c_w,
                         const cv::Point3f& object_point,
                         cv::Vec3d& ray)
{
    const cv::Mat R = T_c_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t = T_c_w(cv::Rect(3, 0, 1, 3));
    cv::Mat point_w = (cv::Mat_<double>(3, 1) << object_point.x,
                                                object_point.y,
                                                object_point.z);
    cv::Mat point_c = R * point_w + t;
    ray = cv::Vec3d(point_c.at<double>(0, 0),
                    point_c.at<double>(1, 0),
                    point_c.at<double>(2, 0));
    const double norm = cv::norm(ray);
    if (norm < 1e-12 || !std::isfinite(norm)) {
        return false;
    }
    ray *= 1.0 / norm;
    return true;
}

// ---------------------------------------------------------------------------
// appendRayBundleResiduals
// ---------------------------------------------------------------------------
bool appendRayBundleResiduals(const std::vector<cv::Point3f>& object_points,
                              const std::vector<cv::Point2f>& image_points,
                              const FisheyeAutoCameraConfig& cam,
                              const cv::Mat& T_c_w,
                              std::vector<double>& residuals,
                              double& angular_sq_sum_deg,
                              int& angular_count)
{
    if (object_points.size() != image_points.size() || object_points.empty()) {
        return false;
    }

    for (size_t i = 0; i < object_points.size(); ++i) {
        cv::Vec3d predicted_ray;
        cv::Vec3d observed_ray;
        if (!worldPointToUnitRay(T_c_w, object_points[i], predicted_ray) ||
            !imagePointToUnitRay(cam, image_points[i], observed_ray)) {
            return false;
        }

        const cv::Vec3d cross = predicted_ray.cross(observed_ray);
        residuals.push_back(cross[0]);
        residuals.push_back(cross[1]);
        residuals.push_back(cross[2]);

        const double sin_angle = clamp(cv::norm(cross), 0.0, 1.0);
        const double angle_deg = std::asin(sin_angle) * 180.0 / M_PI;
        angular_sq_sum_deg += angle_deg * angle_deg;
        ++angular_count;
    }
    return true;
}

// ---------------------------------------------------------------------------
// appendEpipolar2d2dResiduals
// ---------------------------------------------------------------------------
bool appendEpipolar2d2dResiduals(const std::vector<cv::Point2f>& image_points_a,
                                  const FisheyeAutoCameraConfig& camera_a,
                                  const cv::Mat& T_a_w,
                                  const std::vector<cv::Point2f>& image_points_b,
                                  const FisheyeAutoCameraConfig& camera_b,
                                  const cv::Mat& T_b_w,
                                  std::vector<double>& residuals,
                                  double& angular_sq_sum_a_deg,
                                  int& angular_count_a,
                                  double& angular_sq_sum_b_deg,
                                  int& angular_count_b)
{
    if (image_points_a.size() != image_points_b.size() || image_points_a.empty()) {
        return false;
    }

    const cv::Mat R_a_w = T_a_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t_a_w = T_a_w(cv::Rect(3, 0, 1, 3));
    const cv::Mat R_b_w = T_b_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t_b_w = T_b_w(cv::Rect(3, 0, 1, 3));
    const cv::Mat R_b_a = R_b_w * R_a_w.t();
    const cv::Mat t_b_a_mat = t_b_w - R_b_a * t_a_w;
    const cv::Vec3d t_b_a(t_b_a_mat.at<double>(0, 0),
                          t_b_a_mat.at<double>(1, 0),
                          t_b_a_mat.at<double>(2, 0));
    const double baseline = cv::norm(t_b_a);
    if (baseline < 1e-9 || !std::isfinite(baseline)) {
        return false;
    }

    for (size_t i = 0; i < image_points_a.size(); ++i) {
        cv::Vec3d ray_a;
        cv::Vec3d ray_b;
        if (!imagePointToUnitRay(camera_a, image_points_a[i], ray_a) ||
            !imagePointToUnitRay(camera_b, image_points_b[i], ray_b)) {
            return false;
        }

        cv::Mat ray_a_mat = (cv::Mat_<double>(3, 1) << ray_a[0], ray_a[1], ray_a[2]);
        cv::Mat rotated_ray_a_mat = R_b_a * ray_a_mat;
        const cv::Vec3d rotated_ray_a(rotated_ray_a_mat.at<double>(0, 0),
                                      rotated_ray_a_mat.at<double>(1, 0),
                                      rotated_ray_a_mat.at<double>(2, 0));
        const double residual = ray_b.dot(t_b_a.cross(rotated_ray_a)) / baseline;
        residuals.push_back(residual);

        const double angle_deg = std::asin(clamp(std::abs(residual), 0.0, 1.0)) * 180.0 / M_PI;
        angular_sq_sum_a_deg += angle_deg * angle_deg;
        angular_sq_sum_b_deg += angle_deg * angle_deg;
        ++angular_count_a;
        ++angular_count_b;
    }
    return true;
}

// ---------------------------------------------------------------------------
// appendGround3d3dResiduals
// ---------------------------------------------------------------------------
bool appendGround3d3dResiduals(const std::vector<cv::Point2f>& image_points_a,
                               const FisheyeAutoCameraConfig& camera_a,
                               const cv::Mat& T_a_w,
                               const std::vector<cv::Point2f>& image_points_b,
                               const FisheyeAutoCameraConfig& camera_b,
                               const cv::Mat& T_b_w,
                               double ground_z,
                               std::vector<double>& residuals,
                               double& metric_sq_sum_a_m,
                               int& metric_count_a,
                               double& metric_sq_sum_b_m,
                               int& metric_count_b)
{
    if (image_points_a.size() != image_points_b.size() || image_points_a.empty()) {
        return false;
    }

    const std::vector<cv::Point3f> points_a =
        backProjectToGround(image_points_a, camera_a.K, T_a_w, ground_z);
    const std::vector<cv::Point3f> points_b =
        backProjectToGround(image_points_b, camera_b.K, T_b_w, ground_z);
    if (points_a.size() != image_points_a.size() || points_b.size() != image_points_b.size()) {
        return false;
    }

    for (size_t i = 0; i < points_a.size(); ++i) {
        const cv::Point3f& pa = points_a[i];
        const cv::Point3f& pb = points_b[i];
        if (!std::isfinite(pa.x) || !std::isfinite(pa.y) || !std::isfinite(pa.z) ||
            !std::isfinite(pb.x) || !std::isfinite(pb.y) || !std::isfinite(pb.z)) {
            return false;
        }

        const double dx = static_cast<double>(pa.x) - static_cast<double>(pb.x);
        const double dy = static_cast<double>(pa.y) - static_cast<double>(pb.y);
        residuals.push_back(dx);
        residuals.push_back(dy);

        const double dist_sq = dx * dx + dy * dy;
        metric_sq_sum_a_m += dist_sq;
        metric_sq_sum_b_m += dist_sq;
        ++metric_count_a;
        ++metric_count_b;
    }
    return true;
}

// ---------------------------------------------------------------------------
// distortedImagePointToUnitRay (camera::Mei)
// ---------------------------------------------------------------------------
bool distortedImagePointToUnitRay(const camera::Mei& camera_model,
                                  const cv::Point2f& image_point,
                                  cv::Vec3d& ray)
{
    Eigen::Vector3d point_camera;
    if (!camera_model.projectCamera2dToCamera3d(cv::Point2d(image_point.x, image_point.y),
                                                1.0,
                                                point_camera)) {
        return false;
    }

    const double norm = point_camera.norm();
    if (!std::isfinite(norm) || norm < 1e-12) {
        return false;
    }
    point_camera /= norm;
    ray = cv::Vec3d(point_camera.x(), point_camera.y(), point_camera.z());
    return std::isfinite(ray[0]) && std::isfinite(ray[1]) && std::isfinite(ray[2]);
}

// ---------------------------------------------------------------------------
// distortedImagePointToUnitRay (FisheyeAutoCameraConfig)
// ---------------------------------------------------------------------------
bool distortedImagePointToUnitRay(const FisheyeAutoCameraConfig& cam,
                                  const cv::Point2f& image_point,
                                  cv::Vec3d& ray)
{
    const camera::Mei camera_model = initMeiCamera(cam);
    return distortedImagePointToUnitRay(camera_model, image_point, ray);
}

// ---------------------------------------------------------------------------
// appendRayGround3d3dResiduals
// ---------------------------------------------------------------------------
bool appendRayGround3d3dResiduals(const std::vector<cv::Point2f>& image_points_a,
                                  const FisheyeAutoCameraConfig& camera_a,
                                  const cv::Mat& T_a_w,
                                  const std::vector<cv::Point2f>& image_points_b,
                                  const FisheyeAutoCameraConfig& camera_b,
                                  const cv::Mat& T_b_w,
                                  double ground_z,
                                  std::vector<double>& residuals,
                                  double& metric_sq_sum_a_m,
                                  int& metric_count_a,
                                  double& metric_sq_sum_b_m,
                                  int& metric_count_b,
                                  std::vector<cv::Point3f>* object_points_out)
{
    if (image_points_a.size() != image_points_b.size() || image_points_a.empty()) {
        return false;
    }

    std::vector<cv::Point3f> points_a;
    std::vector<cv::Point3f> points_b;
    if (!backProjectDistortedToGround(image_points_a, camera_a, T_a_w, ground_z, points_a) ||
        !backProjectDistortedToGround(image_points_b, camera_b, T_b_w, ground_z, points_b) ||
        points_a.size() != image_points_a.size() ||
        points_b.size() != image_points_b.size()) {
        return false;
    }

    if (object_points_out != NULL) {
        *object_points_out = points_a;
    }

    for (size_t i = 0; i < points_a.size(); ++i) {
        const double dx = static_cast<double>(points_a[i].x) - static_cast<double>(points_b[i].x);
        const double dy = static_cast<double>(points_a[i].y) - static_cast<double>(points_b[i].y);
        if (!std::isfinite(dx) || !std::isfinite(dy)) {
            return false;
        }

        residuals.push_back(dx);
        residuals.push_back(dy);

        const double dist_sq = dx * dx + dy * dy;
        metric_sq_sum_a_m += dist_sq;
        metric_sq_sum_b_m += dist_sq;
        ++metric_count_a;
        ++metric_count_b;
    }
    return true;
}

// ---------------------------------------------------------------------------
// appendSharedBoardRawReprojectionResiduals
// ---------------------------------------------------------------------------
bool appendSharedBoardRawReprojectionResiduals(const JointBoardObservation& board,
                                               const FisheyeAutoCameraConfig& camera_a,
                                               const cv::Mat& T_a_w,
                                               const FisheyeAutoCameraConfig& camera_b,
                                               const cv::Mat& T_b_w,
                                               const std::vector<double>& params,
                                               size_t board_param_base,
                                               double spacing_m,
                                               double ground_z,
                                               std::vector<double>& residuals,
                                               double& sq_sum_a_px,
                                               int& count_a,
                                               double& sq_sum_b_px,
                                               int& count_b,
                                               std::vector<cv::Point3f>* object_points_out)
{
    if (board.image_points_a.size() != board.image_points_b.size() ||
        board.image_points_a.empty() ||
        params.size() < board_param_base + 3 ||
        spacing_m <= 0.0) {
        return false;
    }

    const std::vector<cv::Point3f> object_points =
        buildJointBoardObjectPointsFromParams(board,
                                              params,
                                              board_param_base,
                                              spacing_m,
                                              ground_z);
    if (object_points.size() != board.image_points_a.size()) {
        return false;
    }
    if (object_points_out != NULL) {
        *object_points_out = object_points;
    }

    for (size_t i = 0; i < object_points.size(); ++i) {
        double dx_a = 0.0;
        double dy_a = 0.0;
        double dx_b = 0.0;
        double dy_b = 0.0;
        rawProjectionResidualOrPenalty(camera_a,
                                       T_a_w,
                                       object_points[i],
                                       board.image_points_a[i],
                                       dx_a,
                                       dy_a);
        rawProjectionResidualOrPenalty(camera_b,
                                       T_b_w,
                                       object_points[i],
                                       board.image_points_b[i],
                                       dx_b,
                                       dy_b);

        residuals.push_back(dx_a);
        residuals.push_back(dy_a);
        residuals.push_back(dx_b);
        residuals.push_back(dy_b);

        sq_sum_a_px += dx_a * dx_a + dy_a * dy_a;
        sq_sum_b_px += dx_b * dx_b + dy_b * dy_b;
        ++count_a;
        ++count_b;
    }
    return true;
}

// ---------------------------------------------------------------------------
// point3fToVec3d
// ---------------------------------------------------------------------------
cv::Vec3d point3fToVec3d(const cv::Point3f& point)
{
    return cv::Vec3d(static_cast<double>(point.x),
                    static_cast<double>(point.y),
                    static_cast<double>(point.z));
}

// ---------------------------------------------------------------------------
// appendMetricResidual
// ---------------------------------------------------------------------------
void appendMetricResidual(double value_m,
                          double sigma_m,
                          double weight_sqrt,
                          std::vector<double>& residuals)
{
    if (std::isfinite(value_m) && sigma_m > 0.0) {
        residuals.push_back(weight_sqrt * value_m / sigma_m);
    }
}

// ---------------------------------------------------------------------------
// appendMiddleBoardMetricResiduals (board + camera)
// ---------------------------------------------------------------------------
bool appendMiddleBoardMetricResiduals(const JointBoardObservation& board,
                                      const FisheyeAutoCameraConfig& camera,
                                      const cv::Mat& T_c_w,
                                      double ground_z,
                                      double cell_size_m,
                                      double weight,
                                      double sigma_m,
                                      std::vector<double>& residuals)
{
    if (board.rows <= 1 || board.cols <= 1 || cell_size_m <= 0.0 || weight < 0.0 || sigma_m <= 0.0) {
        return false;
    }
    const size_t point_count = static_cast<size_t>(board.rows * board.cols);
    if (board.image_points_a.size() != point_count) {
        return false;
    }

    const std::vector<cv::Point3f> points =
        backProjectToGround(board.image_points_a, camera.K, T_c_w, ground_z);
    if (points.size() != point_count) {
        return false;
    }

    const double weight_sqrt = std::sqrt(weight);
    const double diagonal_m = std::sqrt(2.0) * cell_size_m;
    const double inv_cell_size = 1.0 / cell_size_m;
    for (size_t i = 0; i < points.size(); ++i) {
        if (!std::isfinite(points[i].x) ||
            !std::isfinite(points[i].y) ||
            !std::isfinite(points[i].z)) {
            return false;
        }
    }

    for (int r = 0; r < board.rows; ++r) {
        for (int c = 0; c + 1 < board.cols; ++c) {
            const size_t idx0 = static_cast<size_t>(r * board.cols + c);
            const size_t idx1 = static_cast<size_t>(r * board.cols + c + 1);
            appendMetricResidual(cv::norm(points[idx1] - points[idx0]) - cell_size_m,
                                 sigma_m,
                                 weight_sqrt,
                                 residuals);
        }
    }

    for (int r = 0; r + 1 < board.rows; ++r) {
        for (int c = 0; c < board.cols; ++c) {
            const size_t idx0 = static_cast<size_t>(r * board.cols + c);
            const size_t idx1 = static_cast<size_t>((r + 1) * board.cols + c);
            appendMetricResidual(cv::norm(points[idx1] - points[idx0]) - cell_size_m,
                                 sigma_m,
                                 weight_sqrt,
                                 residuals);
        }
    }

    for (int r = 0; r + 1 < board.rows; ++r) {
        for (int c = 0; c + 1 < board.cols; ++c) {
            const size_t idx00 = static_cast<size_t>(r * board.cols + c);
            const size_t idx01 = static_cast<size_t>(r * board.cols + c + 1);
            const size_t idx10 = static_cast<size_t>((r + 1) * board.cols + c);
            const size_t idx11 = static_cast<size_t>((r + 1) * board.cols + c + 1);
            const cv::Vec3d row_vec = point3fToVec3d(points[idx01]) - point3fToVec3d(points[idx00]);
            const cv::Vec3d col_vec = point3fToVec3d(points[idx10]) - point3fToVec3d(points[idx00]);
            appendMetricResidual(row_vec.dot(col_vec) * inv_cell_size,
                                 sigma_m,
                                 weight_sqrt,
                                 residuals);
            appendMetricResidual(cv::norm(points[idx11] - points[idx00]) - diagonal_m,
                                 sigma_m,
                                 weight_sqrt,
                                 residuals);
            appendMetricResidual(cv::norm(points[idx10] - points[idx01]) - diagonal_m,
                                 sigma_m,
                                 weight_sqrt,
                                 residuals);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// appendMiddleBoardMetricResiduals (problem + eval)
// ---------------------------------------------------------------------------
bool appendMiddleBoardMetricResiduals(const JointOptimizationProblem& problem,
                                      const JointOptimizationEvalResult& eval,
                                      std::vector<double>& residuals)
{
    return appendMiddleBoardMetricResiduals(problem.front_middle_board,
                                           problem.front_camera,
                                           eval.front_T_c_w,
                                           problem.ground_z,
                                           problem.grid_spacing_m,
                                           problem.middle_board_metric_weight,
                                           problem.middle_board_metric_sigma_m,
                                           residuals) &&
           appendMiddleBoardMetricResiduals(problem.left_middle_board,
                                           problem.left_camera,
                                           eval.left_T_c_w,
                                           problem.ground_z,
                                           problem.grid_spacing_m,
                                           problem.middle_board_metric_weight,
                                           problem.middle_board_metric_sigma_m,
                                           residuals) &&
           appendMiddleBoardMetricResiduals(problem.right_middle_board,
                                           problem.right_camera,
                                           eval.right_T_c_w,
                                           problem.ground_z,
                                           problem.grid_spacing_m,
                                           problem.middle_board_metric_weight,
                                           problem.middle_board_metric_sigma_m,
                                           residuals) &&
           appendMiddleBoardMetricResiduals(problem.back_middle_board,
                                           problem.back_camera,
                                           eval.back_T_c_w,
                                           problem.ground_z,
                                           problem.grid_spacing_m,
                                           problem.middle_board_metric_weight,
                                           problem.middle_board_metric_sigma_m,
                                           residuals);
}

// ---------------------------------------------------------------------------
// appendJointPriorResiduals
// ---------------------------------------------------------------------------
void appendJointPriorResiduals(const JointOptimizationProblem& problem,
                               const std::vector<double>& params,
                               std::vector<double>& residuals)
{
    const double rotation_scale = std::sqrt(problem.rotation_prior_weight);
    const double translation_scale = std::sqrt(problem.translation_prior_weight);

    for (size_t pose_idx = 0; pose_idx < 3; ++pose_idx) {
        const size_t offset = pose_idx * 6;
        residuals.push_back(translation_scale * (params[offset + 0] - problem.initial_params[offset + 0]));
        residuals.push_back(translation_scale * (params[offset + 1] - problem.initial_params[offset + 1]));
        residuals.push_back(translation_scale * (params[offset + 2] - problem.initial_params[offset + 2]));
        residuals.push_back(rotation_scale *
                            normalizeAngleRad(params[offset + 3] - problem.initial_params[offset + 3]));
        residuals.push_back(rotation_scale *
                            normalizeAngleRad(params[offset + 4] - problem.initial_params[offset + 4]));
        residuals.push_back(rotation_scale *
                            normalizeAngleRad(params[offset + 5] - problem.initial_params[offset + 5]));
    }

    const double board_position_scale = std::sqrt(problem.board_position_prior_weight);
    const double board_yaw_scale = std::sqrt(problem.board_yaw_prior_weight);
    if (params.size() >= 21 && problem.initial_params.size() >= 21) {
        for (size_t board_idx = 0; board_idx < 4; ++board_idx) {
            const size_t offset = 9 + board_idx * 3;
            residuals.push_back(board_position_scale * (params[offset + 0] - problem.initial_params[offset + 0]));
            residuals.push_back(board_position_scale * (params[offset + 1] - problem.initial_params[offset + 1]));
            residuals.push_back(board_yaw_scale *
                                normalizeAngleRad(params[offset + 2] - problem.initial_params[offset + 2]));
        }
    }
}

// ---------------------------------------------------------------------------
// residualCost
// ---------------------------------------------------------------------------
double residualCost(const std::vector<double>& residuals)
{
    double cost = 0.0;
    for (size_t i = 0; i < residuals.size(); ++i) {
        cost += residuals[i] * residuals[i];
    }
    return cost;
}

// ---------------------------------------------------------------------------
// computePoseMetrics
// ---------------------------------------------------------------------------
bool computePoseMetrics(const std::vector<cv::Point3f>& object_points,
                        const std::vector<cv::Point2f>& image_points,
                        const FisheyeAutoCameraConfig& cam,
                        const cv::Mat& T_c_w,
                        double& rmse_px,
                        double& mse_px)
{
    if (object_points.size() != image_points.size() || object_points.empty()) {
        return false;
    }

    double total_sq = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        double dx = 0.0;
        double dy = 0.0;
        rawProjectionResidualOrPenalty(cam, T_c_w, object_points[i], image_points[i], dx, dy);
        const double err = std::sqrt(dx * dx + dy * dy);
        total_sq += err * err;
    }

    mse_px = total_sq / static_cast<double>(image_points.size());
    rmse_px = std::sqrt(mse_px);
    return true;
}

// ---------------------------------------------------------------------------
// accumulateReprojectionError
// ---------------------------------------------------------------------------
bool accumulateReprojectionError(const std::vector<cv::Point3f>& object_points,
                                 const std::vector<cv::Point2f>& image_points,
                                 const FisheyeAutoCameraConfig& cam,
                                 const cv::Mat& T_c_w,
                                 double& total_sq_px,
                                 size_t& total_count,
                                 double* camera_sq_px,
                                 size_t* camera_count)
{
    if (object_points.size() != image_points.size() || object_points.empty()) {
        return false;
    }

    for (size_t i = 0; i < image_points.size(); ++i) {
        double dx = 0.0;
        double dy = 0.0;
        rawProjectionResidualOrPenalty(cam, T_c_w, object_points[i], image_points[i], dx, dy);
        const double err = std::sqrt(dx * dx + dy * dy);
        if (!std::isfinite(err)) {
            return false;
        }
        total_sq_px += err * err;
        if (camera_sq_px != NULL) {
            *camera_sq_px += err * err;
        }
    }
    total_count += image_points.size();
    if (camera_count != NULL) {
        *camera_count += image_points.size();
    }
    return true;
}

// ---------------------------------------------------------------------------
// accumulateSharedBoardReprojectionError
// ---------------------------------------------------------------------------
bool accumulateSharedBoardReprojectionError(const JointBoardObservation& board,
                                            const FisheyeAutoCameraConfig& camera_a,
                                            const cv::Mat& T_a_w,
                                            const FisheyeAutoCameraConfig& camera_b,
                                            const cv::Mat& T_b_w,
                                            double ground_z,
                                            double& total_sq_px,
                                            size_t& total_count,
                                            double* camera_a_sq_px,
                                            size_t* camera_a_count,
                                            double* camera_b_sq_px,
                                            size_t* camera_b_count)
{
    const std::vector<cv::Point3f> object_points =
        backProjectToGround(board.image_points_a, camera_a.K, T_a_w, ground_z);
    if (object_points.size() != board.image_points_a.size()) {
        return false;
    }

    return accumulateReprojectionError(object_points,
                                       board.image_points_a,
                                       camera_a,
                                       T_a_w,
                                       total_sq_px,
                                       total_count,
                                       camera_a_sq_px,
                                       camera_a_count) &&
           accumulateReprojectionError(object_points,
                                       board.image_points_b,
                                       camera_b,
                                       T_b_w,
                                       total_sq_px,
                                       total_count,
                                       camera_b_sq_px,
                                       camera_b_count);
}

// ---------------------------------------------------------------------------
// computeSharedBoardReprojectionRmse
// ---------------------------------------------------------------------------
bool computeSharedBoardReprojectionRmse(const JointOptimizationProblem& problem,
                                        const cv::Mat& front_T_c_w,
                                        const cv::Mat& left_T_c_w,
                                        const cv::Mat& right_T_c_w,
                                        const cv::Mat& back_T_c_w,
                                        double& rmse_px,
                                        double* front_rmse_px,
                                        double* left_rmse_px,
                                        double* right_rmse_px,
                                        double* back_rmse_px)
{
    double total_sq_px = 0.0;
    size_t total_count = 0;
    double front_sq_px = 0.0;
    double left_sq_px = 0.0;
    double right_sq_px = 0.0;
    double back_sq_px = 0.0;
    size_t front_count = 0;
    size_t left_count = 0;
    size_t right_count = 0;
    size_t back_count = 0;
    if (!accumulateSharedBoardReprojectionError(problem.front_left_board,
                                                problem.front_camera,
                                                front_T_c_w,
                                                problem.left_camera,
                                                left_T_c_w,
                                                problem.ground_z,
                                                total_sq_px,
                                                total_count,
                                                &front_sq_px,
                                                &front_count,
                                                &left_sq_px,
                                                &left_count) ||
        !accumulateSharedBoardReprojectionError(problem.front_right_board,
                                                problem.front_camera,
                                                front_T_c_w,
                                                problem.right_camera,
                                                right_T_c_w,
                                                problem.ground_z,
                                                total_sq_px,
                                                total_count,
                                                &front_sq_px,
                                                &front_count,
                                                &right_sq_px,
                                                &right_count) ||
        !accumulateSharedBoardReprojectionError(problem.back_right_board,
                                                problem.right_camera,
                                                right_T_c_w,
                                                problem.back_camera,
                                                back_T_c_w,
                                                problem.ground_z,
                                                total_sq_px,
                                                total_count,
                                                &right_sq_px,
                                                &right_count,
                                                &back_sq_px,
                                                &back_count) ||
        !accumulateSharedBoardReprojectionError(problem.back_left_board,
                                                problem.left_camera,
                                                left_T_c_w,
                                                problem.back_camera,
                                                back_T_c_w,
                                                problem.ground_z,
                                                total_sq_px,
                                                total_count,
                                                &left_sq_px,
                                                &left_count,
                                                &back_sq_px,
                                                &back_count) ||
        total_count == 0) {
        return false;
    }

    rmse_px = std::sqrt(total_sq_px / static_cast<double>(total_count));
    if (front_rmse_px != NULL) {
        *front_rmse_px = front_count > 0 ? std::sqrt(front_sq_px / static_cast<double>(front_count)) : -1.0;
    }
    if (left_rmse_px != NULL) {
        *left_rmse_px = left_count > 0 ? std::sqrt(left_sq_px / static_cast<double>(left_count)) : -1.0;
    }
    if (right_rmse_px != NULL) {
        *right_rmse_px = right_count > 0 ? std::sqrt(right_sq_px / static_cast<double>(right_count)) : -1.0;
    }
    if (back_rmse_px != NULL) {
        *back_rmse_px = back_count > 0 ? std::sqrt(back_sq_px / static_cast<double>(back_count)) : -1.0;
    }
    return std::isfinite(rmse_px);
}

// ---------------------------------------------------------------------------
// computeSharedBoardRawReprojectionRmse
// ---------------------------------------------------------------------------
bool computeSharedBoardRawReprojectionRmse(const JointOptimizationProblem& problem,
                                           const JointOptimizationEvalResult& eval,
                                           double& rmse_px,
                                           double* front_rmse_px,
                                           double* left_rmse_px,
                                           double* right_rmse_px,
                                           double* back_rmse_px)
{
    double total_sq_px = 0.0;
    size_t total_count = 0;
    double front_sq_px = 0.0;
    double left_sq_px = 0.0;
    double right_sq_px = 0.0;
    double back_sq_px = 0.0;
    size_t front_count = 0;
    size_t left_count = 0;
    size_t right_count = 0;
    size_t back_count = 0;

    if (!accumulateReprojectionError(eval.front_left_object_points,
                                     problem.front_left_board.image_points_a,
                                     problem.front_camera,
                                     eval.front_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &front_sq_px,
                                     &front_count) ||
        !accumulateReprojectionError(eval.front_left_object_points,
                                     problem.front_left_board.image_points_b,
                                     problem.left_camera,
                                     eval.left_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &left_sq_px,
                                     &left_count) ||
        !accumulateReprojectionError(eval.front_right_object_points,
                                     problem.front_right_board.image_points_a,
                                     problem.front_camera,
                                     eval.front_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &front_sq_px,
                                     &front_count) ||
        !accumulateReprojectionError(eval.front_right_object_points,
                                     problem.front_right_board.image_points_b,
                                     problem.right_camera,
                                     eval.right_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &right_sq_px,
                                     &right_count) ||
        !accumulateReprojectionError(eval.back_right_object_points,
                                     problem.back_right_board.image_points_a,
                                     problem.right_camera,
                                     eval.right_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &right_sq_px,
                                     &right_count) ||
        !accumulateReprojectionError(eval.back_right_object_points,
                                     problem.back_right_board.image_points_b,
                                     problem.back_camera,
                                     eval.back_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &back_sq_px,
                                     &back_count) ||
        !accumulateReprojectionError(eval.back_left_object_points,
                                     problem.back_left_board.image_points_a,
                                     problem.left_camera,
                                     eval.left_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &left_sq_px,
                                     &left_count) ||
        !accumulateReprojectionError(eval.back_left_object_points,
                                     problem.back_left_board.image_points_b,
                                     problem.back_camera,
                                     eval.back_T_c_w,
                                     total_sq_px,
                                     total_count,
                                     &back_sq_px,
                                     &back_count) ||
        total_count == 0) {
        return false;
    }

    rmse_px = std::sqrt(total_sq_px / static_cast<double>(total_count));
    if (front_rmse_px != NULL) {
        *front_rmse_px = front_count > 0 ? std::sqrt(front_sq_px / static_cast<double>(front_count)) : -1.0;
    }
    if (left_rmse_px != NULL) {
        *left_rmse_px = left_count > 0 ? std::sqrt(left_sq_px / static_cast<double>(left_count)) : -1.0;
    }
    if (right_rmse_px != NULL) {
        *right_rmse_px = right_count > 0 ? std::sqrt(right_sq_px / static_cast<double>(right_count)) : -1.0;
    }
    if (back_rmse_px != NULL) {
        *back_rmse_px = back_count > 0 ? std::sqrt(back_sq_px / static_cast<double>(back_count)) : -1.0;
    }
    return std::isfinite(rmse_px);
}

// ---------------------------------------------------------------------------
// computeFrontSharedBoardReprojectionRmse
// ---------------------------------------------------------------------------
bool computeFrontSharedBoardReprojectionRmse(const JointOptimizationProblem& problem,
                                             const cv::Mat& front_T_c_w,
                                             const cv::Mat& left_T_c_w,
                                             const cv::Mat& right_T_c_w,
                                             double& rmse_px,
                                             double* front_rmse_px,
                                             double* left_rmse_px,
                                             double* right_rmse_px)
{
    double total_sq_px = 0.0;
    size_t total_count = 0;
    double front_sq_px = 0.0;
    double left_sq_px = 0.0;
    double right_sq_px = 0.0;
    size_t front_count = 0;
    size_t left_count = 0;
    size_t right_count = 0;
    if (!accumulateSharedBoardReprojectionError(problem.front_left_board,
                                                problem.front_camera,
                                                front_T_c_w,
                                                problem.left_camera,
                                                left_T_c_w,
                                                problem.ground_z,
                                                total_sq_px,
                                                total_count,
                                                &front_sq_px,
                                                &front_count,
                                                &left_sq_px,
                                                &left_count) ||
        !accumulateSharedBoardReprojectionError(problem.front_right_board,
                                                problem.front_camera,
                                                front_T_c_w,
                                                problem.right_camera,
                                                right_T_c_w,
                                                problem.ground_z,
                                                total_sq_px,
                                                total_count,
                                                &front_sq_px,
                                                &front_count,
                                                &right_sq_px,
                                                &right_count) ||
        total_count == 0) {
        return false;
    }

    rmse_px = std::sqrt(total_sq_px / static_cast<double>(total_count));
    if (front_rmse_px != NULL) {
        *front_rmse_px = front_count > 0 ? std::sqrt(front_sq_px / static_cast<double>(front_count)) : -1.0;
    }
    if (left_rmse_px != NULL) {
        *left_rmse_px = left_count > 0 ? std::sqrt(left_sq_px / static_cast<double>(left_count)) : -1.0;
    }
    if (right_rmse_px != NULL) {
        *right_rmse_px = right_count > 0 ? std::sqrt(right_sq_px / static_cast<double>(right_count)) : -1.0;
    }
    return std::isfinite(rmse_px);
}

// ---------------------------------------------------------------------------
// buildBackSharedObjectPoints
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> buildBackSharedObjectPoints(const JointOptimizationProblem& problem,
                                                     const cv::Mat& left_T_c_w,
                                                     const cv::Mat& right_T_c_w)
{
    const std::vector<cv::Point3f> left_back_points = sortPointsGrid(
        backProjectToGround(problem.left_rear_image_points,
                            problem.left_camera.K,
                            left_T_c_w,
                            problem.ground_z),
        0,
        problem.sort_threshold,
        false,
        false);
    const std::vector<cv::Point3f> right_back_points = sortPointsGrid(
        backProjectToGround(problem.right_rear_image_points,
                            problem.right_camera.K,
                            right_T_c_w,
                            problem.ground_z),
        0,
        problem.sort_threshold,
        false,
        false);

    std::vector<cv::Point3f> back_object_points = right_back_points;
    back_object_points.insert(back_object_points.end(),
                              left_back_points.begin(),
                              left_back_points.end());
    return back_object_points;
}

// ---------------------------------------------------------------------------
// evaluateJointOptimization
// ---------------------------------------------------------------------------
bool evaluateJointOptimization(const JointOptimizationProblem& problem,
                               const std::vector<double>& params,
                               JointOptimizationEvalResult& result)
{
    result = JointOptimizationEvalResult();
    if (params.size() != expectedJointParamCount(problem) ||
        problem.initial_params.size() != expectedJointParamCount(problem)) {
        return false;
    }

    if (unpackJointPoseParams(problem, params, result) == false) {
        return false;
    }

    std::vector<double> residuals;
    residuals.reserve(256);

    double front_metric_sq_m = 0.0;
    double left_metric_sq_m = 0.0;
    double right_metric_sq_m = 0.0;
    double back_metric_sq_m = 0.0;
    int front_count = 0;
    int left_count = 0;
    int right_count = 0;
    int back_count = 0;

    if (!appendRayGround3d3dResiduals(problem.front_left_board.image_points_a,
                                     problem.front_camera,
                                     result.front_T_c_w,
                                     problem.front_left_board.image_points_b,
                                     problem.left_camera,
                                     result.left_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     front_metric_sq_m,
                                     front_count,
                                     left_metric_sq_m,
                                     left_count,
                                     &result.front_left_object_points) ||
        !appendRayGround3d3dResiduals(problem.front_right_board.image_points_a,
                                     problem.front_camera,
                                     result.front_T_c_w,
                                     problem.front_right_board.image_points_b,
                                     problem.right_camera,
                                     result.right_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     front_metric_sq_m,
                                     front_count,
                                     right_metric_sq_m,
                                     right_count,
                                     &result.front_right_object_points) ||
        !appendRayGround3d3dResiduals(problem.back_right_board.image_points_a,
                                     problem.right_camera,
                                     result.right_T_c_w,
                                     problem.back_right_board.image_points_b,
                                     problem.back_camera,
                                     result.back_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     right_metric_sq_m,
                                     right_count,
                                     back_metric_sq_m,
                                     back_count,
                                     &result.back_right_object_points) ||
        !appendRayGround3d3dResiduals(problem.back_left_board.image_points_a,
                                     problem.left_camera,
                                     result.left_T_c_w,
                                     problem.back_left_board.image_points_b,
                                     problem.back_camera,
                                     result.back_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     left_metric_sq_m,
                                     left_count,
                                     back_metric_sq_m,
                                     back_count,
                                     &result.back_left_object_points)) {
        return false;
    }

    appendJointPriorResiduals(problem, params, residuals);

    result.front_rmse = front_count > 0 ? std::sqrt(front_metric_sq_m / static_cast<double>(front_count)) : -1.0;
    result.left_rmse = left_count > 0 ? std::sqrt(left_metric_sq_m / static_cast<double>(left_count)) : -1.0;
    result.right_rmse = right_count > 0 ? std::sqrt(right_metric_sq_m / static_cast<double>(right_count)) : -1.0;
    result.back_rmse = back_count > 0 ? std::sqrt(back_metric_sq_m / static_cast<double>(back_count)) : -1.0;
    result.back_object_points = result.back_right_object_points;
    result.back_object_points.insert(result.back_object_points.end(),
                                     result.back_left_object_points.begin(),
                                     result.back_left_object_points.end());
    result.total_cost = residualCost(residuals);
    result.valid = true;
    return true;
}

// ---------------------------------------------------------------------------
// buildJointBundleResiduals
// ---------------------------------------------------------------------------
bool buildJointBundleResiduals(const JointOptimizationProblem& problem,
                               const std::vector<double>& params,
                               JointOptimizationEvalResult& eval,
                               std::vector<double>& residuals)
{
    if (unpackJointPoseParams(problem, params, eval) == false) {
        return false;
    }

    residuals.clear();
    double unused_sq_a = 0.0;
    double unused_sq_b = 0.0;
    int unused_count_a = 0;
    int unused_count_b = 0;
    if (!appendRayGround3d3dResiduals(problem.front_left_board.image_points_a,
                                     problem.front_camera,
                                     eval.front_T_c_w,
                                     problem.front_left_board.image_points_b,
                                     problem.left_camera,
                                     eval.left_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     unused_sq_a,
                                     unused_count_a,
                                     unused_sq_b,
                                     unused_count_b,
                                     &eval.front_left_object_points) ||
        !appendRayGround3d3dResiduals(problem.front_right_board.image_points_a,
                                     problem.front_camera,
                                     eval.front_T_c_w,
                                     problem.front_right_board.image_points_b,
                                     problem.right_camera,
                                     eval.right_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     unused_sq_a,
                                     unused_count_a,
                                     unused_sq_b,
                                     unused_count_b,
                                     &eval.front_right_object_points) ||
        !appendRayGround3d3dResiduals(problem.back_right_board.image_points_a,
                                     problem.right_camera,
                                     eval.right_T_c_w,
                                     problem.back_right_board.image_points_b,
                                     problem.back_camera,
                                     eval.back_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     unused_sq_a,
                                     unused_count_a,
                                     unused_sq_b,
                                     unused_count_b,
                                     &eval.back_right_object_points) ||
        !appendRayGround3d3dResiduals(problem.back_left_board.image_points_a,
                                     problem.left_camera,
                                     eval.left_T_c_w,
                                     problem.back_left_board.image_points_b,
                                     problem.back_camera,
                                     eval.back_T_c_w,
                                     problem.ground_z,
                                     residuals,
                                     unused_sq_a,
                                     unused_count_a,
                                     unused_sq_b,
                                     unused_count_b,
                                     &eval.back_left_object_points)) {
        return false;
    }
    eval.back_object_points = eval.back_right_object_points;
    eval.back_object_points.insert(eval.back_object_points.end(),
                                   eval.back_left_object_points.begin(),
                                   eval.back_left_object_points.end());
    appendJointPriorResiduals(problem, params, residuals);
    return true;
}

// ---------------------------------------------------------------------------
// JointBundleCeresCost (implementation detail)
// ---------------------------------------------------------------------------
struct JointBundleCeresCost {
    explicit JointBundleCeresCost(const JointOptimizationProblem& problem)
        : problem_(problem) {}

    bool operator()(double const* const* parameters, double* residuals) const
    {
        std::vector<double> params = problem_.initial_params;
        if (params.size() != expectedJointParamCount(problem_)) {
            return false;
        }
        for (size_t i = 0; i < params.size(); ++i) {
            params[i] = parameters[0][i];
        }
        for (size_t pose_idx = 0; pose_idx < 3; ++pose_idx) {
            const size_t base = pose_idx * 6;
            params[base + 3] = normalizeAngleRad(params[base + 3]);
            params[base + 4] = normalizeAngleRad(params[base + 4]);
            params[base + 5] = normalizeAngleRad(params[base + 5]);
        }

        JointOptimizationEvalResult eval;
        std::vector<double> residual_vec;
        if (!buildJointBundleResiduals(problem_, params, eval, residual_vec)) {
            return false;
        }
        for (size_t i = 0; i < residual_vec.size(); ++i) {
            residuals[i] = residual_vec[i];
        }
        return true;
    }

    JointOptimizationProblem problem_;
};

// ---------------------------------------------------------------------------
// optimizeJointLeftRightBack
// ---------------------------------------------------------------------------
bool optimizeJointLeftRightBack(const JointOptimizationProblem& problem,
                                cv::Mat& front_T_c_w,
                                cv::Mat& left_T_c_w,
                                cv::Mat& right_T_c_w,
                                cv::Mat& back_T_c_w,
                                double& front_rmse,
                                double& left_rmse,
                                double& right_rmse,
                                double& back_rmse,
                                double& total_cost,
                                JointOptimizationEvalResult* final_eval)
{
    const size_t param_count = expectedJointParamCount(problem);
    if (problem.initial_params.size() != param_count) {
        return false;
    }

    std::vector<double> params = problem.initial_params;
    JointOptimizationEvalResult initial_eval;
    std::vector<double> initial_residuals;
    if (!buildJointBundleResiduals(problem, params, initial_eval, initial_residuals)) {
        return false;
    }
    if (!evaluateJointOptimization(problem, params, initial_eval)) {
        return false;
    }
    const double initial_cost = residualCost(initial_residuals);
    if (initial_residuals.empty()) {
        return false;
    }

    std::vector<double> optimized_params = params;
    for (size_t i = 0; i < optimized_params.size(); ++i) {
        optimized_params[i] = params[i];
    }

    ceres::Problem ceres_problem;
    JointBundleCeresCost* cost = new JointBundleCeresCost(problem);
    ceres::DynamicNumericDiffCostFunction<JointBundleCeresCost, ceres::CENTRAL>* cost_function =
        new ceres::DynamicNumericDiffCostFunction<JointBundleCeresCost, ceres::CENTRAL>(cost);
    cost_function->AddParameterBlock(static_cast<int>(param_count));
    cost_function->SetNumResiduals(static_cast<int>(initial_residuals.size()));
    ceres_problem.AddResidualBlock(cost_function, new ceres::HuberLoss(3.0), optimized_params.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 80;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &ceres_problem, &summary);

    for (size_t i = 0; i < params.size(); ++i) {
        params[i] = optimized_params[i];
    }
    for (size_t pose_idx = 0; pose_idx < 3; ++pose_idx) {
        const size_t base = pose_idx * 6;
        params[base + 3] = normalizeAngleRad(params[base + 3]);
        params[base + 4] = normalizeAngleRad(params[base + 4]);
        params[base + 5] = normalizeAngleRad(params[base + 5]);
    }

    JointOptimizationEvalResult solved_eval;
    std::vector<double> solved_residuals;
    if (!buildJointBundleResiduals(problem, params, solved_eval, solved_residuals)) {
        return false;
    }
    if (!evaluateJointOptimization(problem, params, solved_eval)) {
        return false;
    }
    const double solved_cost = residualCost(solved_residuals);
    const bool any_improved = solved_cost + 1e-12 < initial_cost;
    const JointOptimizationEvalResult& output_eval = any_improved ? solved_eval : initial_eval;
    const double output_cost = any_improved ? solved_cost : initial_cost;

    front_T_c_w = output_eval.front_T_c_w.clone();
    left_T_c_w = output_eval.left_T_c_w.clone();
    right_T_c_w = output_eval.right_T_c_w.clone();
    back_T_c_w = output_eval.back_T_c_w.clone();
    front_rmse = output_eval.front_rmse;
    left_rmse = output_eval.left_rmse;
    right_rmse = output_eval.right_rmse;
    back_rmse = output_eval.back_rmse;
    total_cost = output_cost;
    if (final_eval != NULL) {
        *final_eval = output_eval;
    }
    return any_improved;
}

}  // namespace bev_surround_view
