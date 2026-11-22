#include "bev_surround_view/pose_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>

#include "bev_surround_view/calibration_utils.h"
#include "camera_model/mei.h"

namespace bev_surround_view {

// Forward declarations for functions defined in other modules (not declared in headers).
// buildCanonicalBoardObjectPoints and applyMiddleBoardTopology are used by
// solveMiddleBoardPoseForPlane; they originate from the anonymous namespace in
// the monolithic source and will be moved to a suitable home later.
std::vector<cv::Point3f> buildCanonicalBoardObjectPoints(int rows, int cols);
std::vector<cv::Point2f> applyMiddleBoardTopology(const std::vector<cv::Point2f>& ordered_image_points,
                                                  int rows, int cols,
                                                  const std::string& topology_name);

// distortedImagePointToUnitRay (camera::Mei overload) is defined in the
// joint_optimization module.
bool distortedImagePointToUnitRay(const camera::Mei& camera_model,
                                  const cv::Point2f& image_point,
                                  cv::Vec3d& ray);

// ---------------------------------------------------------------------------
// buildPoseMatrix
// ---------------------------------------------------------------------------
cv::Mat buildPoseMatrix(const cv::Mat& rvec, const cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

// ---------------------------------------------------------------------------
// solvePose
// ---------------------------------------------------------------------------
bool solvePose(const std::vector<cv::Point3f>& object_points,
               const std::vector<cv::Point2f>& image_points,
               const FisheyeAutoCameraConfig& cam,
               cv::Mat& T_c_w,
               double& reproj_rmse_px)
{
    if (object_points.size() != image_points.size() || object_points.size() < 4) {
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    const cv::Mat zero_dist = cv::Mat::zeros(1, 4, CV_64F);
    const bool ok = cv::solvePnP(object_points,
                                 image_points,
                                 cam.K,
                                 zero_dist,
                                 rvec,
                                 tvec,
                                 false,
                                 cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        return false;
    }

    T_c_w = buildPoseMatrix(rvec, tvec);

    std::vector<cv::Point2f> reproj;
    cv::projectPoints(object_points, rvec, tvec, cam.K, zero_dist, reproj);
    double total_sq = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        const double err = cv::norm(reproj[i] - image_points[i]);
        total_sq += err * err;
    }
    reproj_rmse_px = std::sqrt(total_sq / static_cast<double>(image_points.size()));
    return true;
}

// ---------------------------------------------------------------------------
// cameraCenterFromPose
// ---------------------------------------------------------------------------
cv::Vec3d cameraCenterFromPose(const cv::Mat& T_c_w)
{
    const cv::Mat R = T_c_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t = T_c_w(cv::Rect(3, 0, 1, 3));
    const cv::Mat camera_center = -R.t() * t;
    return cv::Vec3d(camera_center.at<double>(0, 0),
                     camera_center.at<double>(1, 0),
                     camera_center.at<double>(2, 0));
}

// ---------------------------------------------------------------------------
// computeVehicleCenterX
// ---------------------------------------------------------------------------
double computeVehicleCenterX(const cv::Mat& front_T_c_w,
                             const cv::Mat& back_T_c_w)
{
    const cv::Vec3d front_center = cameraCenterFromPose(front_T_c_w);
    const cv::Vec3d back_center = cameraCenterFromPose(back_T_c_w);
    return 0.5 * (front_center[0] + back_center[0]);
}

// ---------------------------------------------------------------------------
// backProjectToGround
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> backProjectToGround(const std::vector<cv::Point2f>& points,
                                             const cv::Mat& K,
                                             const cv::Mat& T_c_w,
                                             double z_world)
{
    const cv::Mat R = T_c_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t = T_c_w(cv::Rect(3, 0, 1, 3));
    const cv::Mat R_inv = R.t();
    const cv::Mat K_inv = K.inv();
    const cv::Mat camera_center = -R_inv * t;

    std::vector<cv::Point3f> result;
    result.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const cv::Mat p_pixel = (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, 1.0);
        const cv::Mat dir_cam = K_inv * p_pixel;
        const cv::Mat dir_world = R_inv * dir_cam;
        const double dz = dir_world.at<double>(2, 0);
        if (std::abs(dz) < 1e-9) {
            result.emplace_back(0.0f, 0.0f, static_cast<float>(z_world));
            continue;
        }
        const double s = (z_world - camera_center.at<double>(2, 0)) / dz;
        const cv::Mat p_world = camera_center + s * dir_world;
        result.emplace_back(static_cast<float>(p_world.at<double>(0, 0)),
                            static_cast<float>(p_world.at<double>(1, 0)),
                            static_cast<float>(p_world.at<double>(2, 0)));
    }
    return result;
}

// ---------------------------------------------------------------------------
// sortPointsGrid
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> sortPointsGrid(const std::vector<cv::Point3f>& points,
                                        int primary_axis,
                                        double threshold,
                                        bool reverse_primary,
                                        bool reverse_secondary)
{
    std::vector<cv::Point3f> pts = points;
    if (pts.empty()) {
        return pts;
    }

    const int secondary_axis = primary_axis == 0 ? 1 : 0;
    std::sort(pts.begin(), pts.end(),
              [primary_axis, reverse_primary](const cv::Point3f& lhs, const cv::Point3f& rhs) {
                  const double lv = primary_axis == 0 ? lhs.x : lhs.y;
                  const double rv = primary_axis == 0 ? rhs.x : rhs.y;
                  return reverse_primary ? lv > rv : lv < rv;
              });

    std::vector<cv::Point3f> sorted_points;
    std::vector<cv::Point3f> bucket;
    bucket.push_back(pts.front());
    for (size_t i = 1; i < pts.size(); ++i) {
        const double current = primary_axis == 0 ? pts[i].x : pts[i].y;
        const double last = primary_axis == 0 ? bucket.back().x : bucket.back().y;
        if (std::abs(current - last) < threshold) {
            bucket.push_back(pts[i]);
        } else {
            std::sort(bucket.begin(), bucket.end(),
                      [secondary_axis, reverse_secondary](const cv::Point3f& lhs, const cv::Point3f& rhs) {
                          const double lv = secondary_axis == 0 ? lhs.x : lhs.y;
                          const double rv = secondary_axis == 0 ? rhs.x : rhs.y;
                          return reverse_secondary ? lv > rv : lv < rv;
                      });
            sorted_points.insert(sorted_points.end(), bucket.begin(), bucket.end());
            bucket.clear();
            bucket.push_back(pts[i]);
        }
    }

    std::sort(bucket.begin(), bucket.end(),
              [secondary_axis, reverse_secondary](const cv::Point3f& lhs, const cv::Point3f& rhs) {
                  const double lv = secondary_axis == 0 ? lhs.x : lhs.y;
                  const double rv = secondary_axis == 0 ? rhs.x : rhs.y;
                  return reverse_secondary ? lv > rv : lv < rv;
              });
    sorted_points.insert(sorted_points.end(), bucket.begin(), bucket.end());
    return sorted_points;
}

// ---------------------------------------------------------------------------
// poseMatrixToRt
// ---------------------------------------------------------------------------
void poseMatrixToRt(const cv::Mat& T_c_w, cv::Mat& rvec, cv::Mat& tvec)
{
    cv::Rodrigues(T_c_w(cv::Rect(0, 0, 3, 3)), rvec);
    tvec = T_c_w(cv::Rect(3, 0, 1, 3)).clone();
}

// ---------------------------------------------------------------------------
// buildRotationMatrixFromOpenCvRpy
// ---------------------------------------------------------------------------
cv::Mat buildRotationMatrixFromOpenCvRpy(double roll_rad,
                                         double pitch_rad,
                                         double yaw_rad)
{
    const double cr = std::cos(roll_rad);
    const double sr = std::sin(roll_rad);
    const double cp = std::cos(pitch_rad);
    const double sp = std::sin(pitch_rad);
    const double cy = std::cos(yaw_rad);
    const double sy = std::sin(yaw_rad);

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    R.at<double>(0, 0) = cy * cp;
    R.at<double>(0, 1) = cy * sp * sr - sy * cr;
    R.at<double>(0, 2) = cy * sp * cr + sy * sr;
    R.at<double>(1, 0) = sy * cp;
    R.at<double>(1, 1) = sy * sp * sr + cy * cr;
    R.at<double>(1, 2) = sy * sp * cr - cy * sr;
    R.at<double>(2, 0) = -sp;
    R.at<double>(2, 1) = cp * sr;
    R.at<double>(2, 2) = cp * cr;
    return R;
}

// ---------------------------------------------------------------------------
// rotationMatrixToOpenCvRpy
// ---------------------------------------------------------------------------
bool rotationMatrixToOpenCvRpy(const cv::Mat& R,
                               double& roll_rad,
                               double& pitch_rad,
                               double& yaw_rad)
{
    if (R.empty() || R.rows != 3 || R.cols != 3) {
        return false;
    }

    const double r00 = R.at<double>(0, 0);
    const double r10 = R.at<double>(1, 0);
    const double r20 = R.at<double>(2, 0);
    const double r21 = R.at<double>(2, 1);
    const double r22 = R.at<double>(2, 2);
    const double r01 = R.at<double>(0, 1);
    const double r11 = R.at<double>(1, 1);

    pitch_rad = std::atan2(-r20, std::sqrt(r00 * r00 + r10 * r10));
    const double cos_pitch = std::cos(pitch_rad);
    if (std::abs(cos_pitch) > 1e-9) {
        roll_rad = std::atan2(r21, r22);
        yaw_rad = std::atan2(r10, r00);
    } else {
        roll_rad = 0.0;
        yaw_rad = std::atan2(-r01, r11);
    }

    roll_rad = normalizeAngleRad(roll_rad);
    pitch_rad = normalizeAngleRad(pitch_rad);
    yaw_rad = normalizeAngleRad(yaw_rad);
    return std::isfinite(roll_rad) &&
           std::isfinite(pitch_rad) &&
           std::isfinite(yaw_rad);
}

// ---------------------------------------------------------------------------
// buildPoseMatrixWithFixedCenter
// ---------------------------------------------------------------------------
cv::Mat buildPoseMatrixWithFixedCenter(const cv::Mat& rvec,
                                       const cv::Vec3d& camera_center_w)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat center = (cv::Mat_<double>(3, 1) << camera_center_w[0],
                                                 camera_center_w[1],
                                                 camera_center_w[2]);
    cv::Mat tvec = -R * center;
    return buildPoseMatrix(rvec, tvec);
}

// ---------------------------------------------------------------------------
// extractJointCameraPoseState
// ---------------------------------------------------------------------------
bool extractJointCameraPoseState(const cv::Mat& T_c_w,
                                 JointCameraPoseState& pose_state)
{
    const cv::Vec3d center = cameraCenterFromPose(T_c_w);
    if (!rotationMatrixToOpenCvRpy(T_c_w(cv::Rect(0, 0, 3, 3)),
                                   pose_state.roll_rad,
                                   pose_state.pitch_rad,
                                   pose_state.yaw_rad)) {
        return false;
    }

    pose_state.center_x = center[0];
    pose_state.center_y = center[1];
    pose_state.center_z = center[2];
    return true;
}

// ---------------------------------------------------------------------------
// buildPoseMatrixFromJointCameraPose
// ---------------------------------------------------------------------------
cv::Mat buildPoseMatrixFromJointCameraPose(const JointCameraPoseState& pose_state)
{
    const cv::Mat R = buildRotationMatrixFromOpenCvRpy(pose_state.roll_rad,
                                                       pose_state.pitch_rad,
                                                       pose_state.yaw_rad);
    const cv::Mat center = (cv::Mat_<double>(3, 1) << pose_state.center_x,
                                                       pose_state.center_y,
                                                       pose_state.center_z);
    const cv::Mat tvec = -R * center;
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

// ---------------------------------------------------------------------------
// solveUndistortedPinholePoseForSorting
// ---------------------------------------------------------------------------
bool solveUndistortedPinholePoseForSorting(const std::vector<cv::Point3f>& object_points,
                                           const std::vector<cv::Point2f>& image_points,
                                           const FisheyeAutoCameraConfig& cam,
                                           cv::Mat& T_c_w,
                                           double& reproj_rmse_px)
{
    if (object_points.size() != image_points.size() || object_points.size() < 4) {
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    const cv::Mat zero_dist = cv::Mat::zeros(1, 4, CV_64F);
    const bool ok = cv::solvePnP(object_points,
                                 image_points,
                                 cam.K,
                                 zero_dist,
                                 rvec,
                                 tvec,
                                 false,
                                 cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
        return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    T_c_w = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T_c_w(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T_c_w(cv::Rect(3, 0, 1, 3)));

    std::vector<cv::Point2f> reproj;
    cv::projectPoints(object_points, rvec, tvec, cam.K, zero_dist, reproj);
    double total_sq = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        const double err = cv::norm(reproj[i] - image_points[i]);
        total_sq += err * err;
    }
    reproj_rmse_px = std::sqrt(total_sq / static_cast<double>(image_points.size()));
    return std::isfinite(reproj_rmse_px);
}

// ---------------------------------------------------------------------------
// solveMiddleBoardPoseForPlane
// ---------------------------------------------------------------------------
bool solveMiddleBoardPoseForPlane(const std::vector<cv::Point2f>& ordered_image_points,
                                  int rows,
                                  int cols,
                                  const FisheyeAutoCameraConfig& cam,
                                  cv::Mat& homography)
{
    homography.release();
    const std::vector<cv::Point3f> object_points = buildCanonicalBoardObjectPoints(rows, cols);
    const std::array<std::string, 4> topology_names = {{"identity", "flip_lr", "flip_ud", "rotate_180"}};

    bool found = false;
    double best_rmse = std::numeric_limits<double>::max();
    cv::Mat best_pose;
    for (size_t i = 0; i < topology_names.size(); ++i) {
        const std::vector<cv::Point2f> candidate_image_points =
            applyMiddleBoardTopology(ordered_image_points, rows, cols, topology_names[i]);
        cv::Mat pose;
        double rmse = -1.0;
        if (solveUndistortedPinholePoseForSorting(object_points, candidate_image_points, cam, pose, rmse) == false) {
            continue;
        }
        const cv::Vec3d camera_center = cameraCenterFromPose(pose);
        if (camera_center[2] <= 0.0) {
            continue;
        }
        if (rmse < best_rmse) {
            best_rmse = rmse;
            best_pose = pose.clone();
            found = true;
        }
    }

    if (!found) {
        return false;
    }

    const cv::Mat R = best_pose(cv::Rect(0, 0, 3, 3)).clone();
    const cv::Mat t = best_pose(cv::Rect(3, 0, 1, 3)).clone();
    cv::Mat plane_transform(3, 3, CV_64F);
    R.col(0).copyTo(plane_transform.col(0));
    R.col(1).copyTo(plane_transform.col(1));
    t.copyTo(plane_transform.col(2));
    homography = cam.K * plane_transform;
    return !homography.empty();
}

// ---------------------------------------------------------------------------
// imagePointsToPlane
// ---------------------------------------------------------------------------
std::vector<cv::Point3f> imagePointsToPlane(const std::vector<cv::Point2f>& image_points,
                                            const cv::Mat& homography)
{
    std::vector<cv::Point3f> plane_points;
    if (image_points.empty() || homography.empty()) {
        return plane_points;
    }

    const cv::Mat homography_inv = homography.inv();
    plane_points.reserve(image_points.size());
    for (size_t i = 0; i < image_points.size(); ++i) {
        cv::Mat image_h = (cv::Mat_<double>(3, 1) << image_points[i].x, image_points[i].y, 1.0);
        cv::Mat plane_h = homography_inv * image_h;
        const double w = plane_h.at<double>(2, 0);
        if (std::abs(w) <= 1e-12) {
            plane_points.clear();
            return plane_points;
        }
        plane_points.emplace_back(static_cast<float>(plane_h.at<double>(0, 0) / w),
                                  static_cast<float>(plane_h.at<double>(1, 0) / w),
                                  0.0f);
    }
    return plane_points;
}

// ---------------------------------------------------------------------------
// recoverPlaneGridTopologyIndices
// ---------------------------------------------------------------------------
std::vector<size_t> recoverPlaneGridTopologyIndices(const std::vector<cv::Point3f>& plane_points,
                                                    int rows,
                                                    int cols)
{
    const size_t n = plane_points.size();
    std::vector<size_t> identity(n);
    for (size_t i = 0; i < n; ++i) {
        identity[i] = i;
    }
    if (rows <= 0 || cols <= 0 || n != static_cast<size_t>(rows * cols)) {
        return identity;
    }

    std::vector<cv::Point2d> xy(n);
    for (size_t i = 0; i < n; ++i) {
        xy[i] = cv::Point2d(plane_points[i].x, plane_points[i].y);
    }

    const cv::Point2d target_top_left(
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest());
    int top_left_index = -1;
    double best_tl_distance = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        max_x = std::max(max_x, xy[i].x);
        max_y = std::max(max_y, xy[i].y);
    }
    for (size_t i = 0; i < n; ++i) {
        const double dx = xy[i].x - max_x;
        const double dy = xy[i].y - max_y;
        const double distance = dx * dx + dy * dy;
        if (distance < best_tl_distance) {
            best_tl_distance = distance;
            top_left_index = static_cast<int>(i);
        }
    }
    if (top_left_index < 0) {
        return identity;
    }

    std::vector<size_t> neighbor_indices(identity.begin(), identity.end());
    std::sort(neighbor_indices.begin(), neighbor_indices.end(),
              [&](size_t lhs, size_t rhs) {
                  const double dl = cv::norm(xy[lhs] - xy[static_cast<size_t>(top_left_index)]);
                  const double dr = cv::norm(xy[rhs] - xy[static_cast<size_t>(top_left_index)]);
                  return dl < dr;
              });

    int best_right_index = -1;
    double best_right_score = std::numeric_limits<double>::max();
    for (size_t k = 0; k < neighbor_indices.size() && k < 8; ++k) {
        const int idx = static_cast<int>(neighbor_indices[k]);
        if (idx == top_left_index) {
            continue;
        }
        const cv::Point2d rel = xy[static_cast<size_t>(idx)] - xy[static_cast<size_t>(top_left_index)];
        const double distance = cv::norm(rel);
        if (distance <= 1.0e-9) {
            continue;
        }
        const cv::Point2d direction(rel.x / distance, rel.y / distance);
        const double alignment = -direction.y;
        if (alignment <= 0.0) {
            continue;
        }
        const double score = (1.0 - alignment) + 0.05 * distance;
        if (score < best_right_score) {
            best_right_score = score;
            best_right_index = idx;
        }
    }
    if (best_right_index < 0) {
        for (size_t k = 0; k < neighbor_indices.size(); ++k) {
            if (static_cast<int>(neighbor_indices[k]) != top_left_index) {
                best_right_index = static_cast<int>(neighbor_indices[k]);
                break;
            }
        }
    }
    if (best_right_index < 0) {
        return identity;
    }

    const cv::Point2d right_vector = xy[static_cast<size_t>(best_right_index)] - xy[static_cast<size_t>(top_left_index)];
    const double rotation_angle = -std::atan2(right_vector.y, right_vector.x);
    const double c = std::cos(rotation_angle);
    const double s = std::sin(rotation_angle);

    struct RotatedPoint {
        size_t index;
        double x;
        double y;
        int row_label;
        int col_label;
    };
    std::vector<RotatedPoint> rotated_points;
    rotated_points.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const cv::Point2d rel = xy[i] - xy[static_cast<size_t>(top_left_index)];
        rotated_points.push_back(RotatedPoint{
            i,
            rel.x * c - rel.y * s,
            rel.x * s + rel.y * c,
            -1,
            -1});
    }

    std::vector<size_t> sorted_by_row(identity.begin(), identity.end());
    std::stable_sort(sorted_by_row.begin(), sorted_by_row.end(),
                     [&](size_t lhs, size_t rhs) {
                         return rotated_points[lhs].y > rotated_points[rhs].y;
                     });
    for (int row = 0; row < rows; ++row) {
        for (int i = row * cols; i < (row + 1) * cols; ++i) {
            rotated_points[sorted_by_row[static_cast<size_t>(i)]].row_label = row;
        }
    }

    std::vector<size_t> sorted_by_col(identity.begin(), identity.end());
    std::stable_sort(sorted_by_col.begin(), sorted_by_col.end(),
                     [&](size_t lhs, size_t rhs) {
                         return rotated_points[lhs].x < rotated_points[rhs].x;
                     });
    for (int col = 0; col < cols; ++col) {
        for (int i = col * rows; i < (col + 1) * rows; ++i) {
            rotated_points[sorted_by_col[static_cast<size_t>(i)]].col_label = col;
        }
    }

    std::vector<size_t> ordered_indices(identity.begin(), identity.end());
    std::stable_sort(ordered_indices.begin(), ordered_indices.end(),
                     [&](size_t lhs, size_t rhs) {
                         if (rotated_points[lhs].row_label != rotated_points[rhs].row_label) {
                             return rotated_points[lhs].row_label < rotated_points[rhs].row_label;
                         }
                         return rotated_points[lhs].col_label < rotated_points[rhs].col_label;
                     });
    return ordered_indices;
}

// ---------------------------------------------------------------------------
// backProjectDistortedToGround
// ---------------------------------------------------------------------------
bool backProjectDistortedToGround(const std::vector<cv::Point2f>& image_points,
                                  const FisheyeAutoCameraConfig& cam,
                                  const cv::Mat& T_c_w,
                                  double ground_z,
                                  std::vector<cv::Point3f>& ground_points)
{
    ground_points.clear();
    if (image_points.empty() || T_c_w.empty() || T_c_w.rows != 4 || T_c_w.cols != 4) {
        return false;
    }

    const cv::Mat R = T_c_w(cv::Rect(0, 0, 3, 3));
    const cv::Mat t = T_c_w(cv::Rect(3, 0, 1, 3));
    const cv::Mat R_inv = R.t();
    const cv::Mat camera_center = -R_inv * t;
    const camera::Mei camera_model = initMeiCamera(cam);

    ground_points.reserve(image_points.size());
    for (size_t i = 0; i < image_points.size(); ++i) {
        cv::Vec3d ray_cam;
        if (!distortedImagePointToUnitRay(camera_model, image_points[i], ray_cam)) {
            return false;
        }

        const cv::Mat dir_cam = (cv::Mat_<double>(3, 1) << ray_cam[0], ray_cam[1], ray_cam[2]);
        const cv::Mat dir_world = R_inv * dir_cam;
        const double dz = dir_world.at<double>(2, 0);
        if (!std::isfinite(dz) || std::abs(dz) < 1e-9) {
            return false;
        }

        const double s = (ground_z - camera_center.at<double>(2, 0)) / dz;
        if (!std::isfinite(s)) {
            return false;
        }
        const cv::Mat point_world = camera_center + s * dir_world;
        const double x = point_world.at<double>(0, 0);
        const double y = point_world.at<double>(1, 0);
        const double z = point_world.at<double>(2, 0);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            return false;
        }
        ground_points.emplace_back(static_cast<float>(x),
                                   static_cast<float>(y),
                                   static_cast<float>(z));
    }
    return true;
}

}  // namespace bev_surround_view
