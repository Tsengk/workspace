#include "image_lidar_calibration/calibration_solver.h"

#include <ros/ros.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <pcl/common/centroid.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace image_lidar_calibration {
namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

struct IndexedPoint {
    Eigen::Vector3d vcs;
    int idx = -1;
};

struct IndexedCameraPoint {
    Eigen::Vector3d cam;
    cv::Point2f uv;
    int idx = -1;
};

struct SceneObservation {
    std::vector<Eigen::Vector3d> camera_centers;
    std::vector<cv::Point2f> camera_pixels;
    std::vector<Eigen::Vector3d> lidar_points;
    std::vector<int> raw_indices;
    std::array<bool, 4> inliers;
    int inlier_count = 0;
    int scene_input_index = -1;

    SceneObservation() {
        inliers.fill(false);
    }
};

bool reorderCameraByImagePixels(const std::vector<Eigen::Vector3d>& cam_pts,
                                const std::vector<cv::Point2f>& image_pts,
                                std::vector<Eigen::Vector3d>& ordered_cam,
                                std::vector<cv::Point2f>& ordered_pixels,
                                std::vector<int>& ordered_indices,
                                double& row_gap,
                                double& col_gap) {
    ordered_cam.clear();
    ordered_pixels.clear();
    ordered_indices.clear();
    row_gap = 0.0;
    col_gap = 0.0;

    if (cam_pts.size() != 4 || image_pts.size() != 4) {
        return false;
    }

    std::vector<IndexedCameraPoint> items;
    items.reserve(4);
    for (int i = 0; i < 4; ++i) {
        IndexedCameraPoint item;
        item.cam = cam_pts[i];
        item.uv = image_pts[i];
        item.idx = i;
        items.push_back(item);
        ROS_DEBUG("[CalibrationSolver]   raw_cam[%d] uv=(%.2f, %.2f) cam3d=(%.4f, %.4f, %.4f)",
                 i,
                 item.uv.x,
                 item.uv.y,
                 item.cam.x(),
                 item.cam.y(),
                 item.cam.z());
    }

    std::sort(items.begin(), items.end(), [](const IndexedCameraPoint& a, const IndexedCameraPoint& b) {
        return a.uv.y < b.uv.y;
    });

    std::vector<IndexedCameraPoint> top(items.begin(), items.begin() + 2);
    std::vector<IndexedCameraPoint> bottom(items.begin() + 2, items.end());

    std::sort(top.begin(), top.end(), [](const IndexedCameraPoint& a, const IndexedCameraPoint& b) {
        return a.uv.x < b.uv.x;
    });
    std::sort(bottom.begin(), bottom.end(), [](const IndexedCameraPoint& a, const IndexedCameraPoint& b) {
        return a.uv.x < b.uv.x;
    });

    row_gap = 0.5 * ((bottom[0].uv.y - top[0].uv.y) +
                     (bottom[1].uv.y - top[1].uv.y));
    col_gap = 0.5 * ((top[1].uv.x - top[0].uv.x) +
                     (bottom[1].uv.x - bottom[0].uv.x));

    ordered_cam.push_back(top[0].cam);
    ordered_cam.push_back(top[1].cam);
    ordered_cam.push_back(bottom[1].cam);
    ordered_cam.push_back(bottom[0].cam);

    ordered_pixels.push_back(top[0].uv);
    ordered_pixels.push_back(top[1].uv);
    ordered_pixels.push_back(bottom[1].uv);
    ordered_pixels.push_back(bottom[0].uv);

    ordered_indices.push_back(top[0].idx);
    ordered_indices.push_back(top[1].idx);
    ordered_indices.push_back(bottom[1].idx);
    ordered_indices.push_back(bottom[0].idx);

    static const char* kLabels[4] = {"LT", "RT", "RB", "LB"};
    ROS_DEBUG("[CalibrationSolver] Camera hard-order enabled: v_asc => top, u_asc => left");
    ROS_DEBUG("[CalibrationSolver] Camera hard-order row_gap=%.2f col_gap=%.2f raw_idx=[%d %d %d %d]",
             row_gap,
             col_gap,
             ordered_indices[0],
             ordered_indices[1],
             ordered_indices[2],
             ordered_indices[3]);
    for (int i = 0; i < 4; ++i) {
        ROS_DEBUG("[CalibrationSolver]   %s <= raw_cam[%d] uv=(%.2f, %.2f) cam3d=(%.4f, %.4f, %.4f)",
                 kLabels[i],
                 ordered_indices[i],
                 ordered_pixels[i].x,
                 ordered_pixels[i].y,
                 ordered_cam[i].x(),
                 ordered_cam[i].y(),
                 ordered_cam[i].z());
    }

    return true;
}

bool reorderSceneFrontByYZ(const std::vector<Eigen::Vector3d>& vcs_pts,
                           std::vector<Eigen::Vector3d>& ordered_vcs,
                           std::vector<int>& ordered_indices,
                           double& row_gap,
                           double& col_gap) {
    ordered_vcs.clear();
    ordered_indices.clear();
    row_gap = 0.0;
    col_gap = 0.0;

    if (vcs_pts.size() != 4) {
        return false;
    }

    double min_x = vcs_pts[0].x();
    double max_x = vcs_pts[0].x();
    double min_y = vcs_pts[0].y();
    double max_y = vcs_pts[0].y();

    std::vector<IndexedPoint> items;
    items.reserve(4);
    for (int i = 0; i < 4; ++i) {
        IndexedPoint item;
        item.vcs = vcs_pts[i];
        item.idx = i;
        items.push_back(item);
        min_x = std::min(min_x, item.vcs.x());
        max_x = std::max(max_x, item.vcs.x());
        min_y = std::min(min_y, item.vcs.y());
        max_y = std::max(max_y, item.vcs.y());
        ROS_DEBUG("[CalibrationSolver]   raw_vcs[%d] = (%.4f, %.4f, %.4f)",
                 i,
                 item.vcs.x(),
                 item.vcs.y(),
                 item.vcs.z());
    }

    const double span_x = max_x - min_x;
    const double span_y = max_y - min_y;
    const bool use_xz = span_x > span_y;

    std::sort(items.begin(), items.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
        return a.vcs.z() > b.vcs.z();
    });

    std::vector<IndexedPoint> top(items.begin(), items.begin() + 2);
    std::vector<IndexedPoint> bottom(items.begin() + 2, items.end());

    if (use_xz) {
        std::sort(top.begin(), top.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
            return a.vcs.x() > b.vcs.x();
        });
        std::sort(bottom.begin(), bottom.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
            return a.vcs.x() > b.vcs.x();
        });
    } else {
        std::sort(top.begin(), top.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
            return a.vcs.y() > b.vcs.y();
        });
        std::sort(bottom.begin(), bottom.end(), [](const IndexedPoint& a, const IndexedPoint& b) {
            return a.vcs.y() > b.vcs.y();
        });
    }

    row_gap = 0.5 * ((top[0].vcs.z() - bottom[0].vcs.z()) +
                     (top[1].vcs.z() - bottom[1].vcs.z()));
    if (use_xz) {
        col_gap = 0.5 * ((top[0].vcs.x() - top[1].vcs.x()) +
                         (bottom[0].vcs.x() - bottom[1].vcs.x()));
    } else {
        col_gap = 0.5 * ((top[0].vcs.y() - top[1].vcs.y()) +
                         (bottom[0].vcs.y() - bottom[1].vcs.y()));
    }

    ordered_vcs.push_back(top[0].vcs);
    ordered_vcs.push_back(top[1].vcs);
    ordered_vcs.push_back(bottom[1].vcs);
    ordered_vcs.push_back(bottom[0].vcs);

    ordered_indices.push_back(top[0].idx);
    ordered_indices.push_back(top[1].idx);
    ordered_indices.push_back(bottom[1].idx);
    ordered_indices.push_back(bottom[0].idx);

    static const char* kLabels[4] = {"LT", "RT", "RB", "LB"};
    ROS_DEBUG("[CalibrationSolver] Front hard-order enabled: z_desc => top, %s_desc => left, span_x=%.4f span_y=%.4f",
             use_xz ? "x" : "y",
             span_x,
             span_y);
    ROS_DEBUG("[CalibrationSolver] Front hard-order row_gap=%.4f col_gap=%.4f raw_idx=[%d %d %d %d]",
             row_gap,
             col_gap,
             ordered_indices[0],
             ordered_indices[1],
             ordered_indices[2],
             ordered_indices[3]);
    for (int i = 0; i < 4; ++i) {
        ROS_DEBUG("[CalibrationSolver]   %s <= raw[%d] = (%.4f, %.4f, %.4f)",
                 kLabels[i],
                 ordered_indices[i],
                 ordered_vcs[i].x(),
                 ordered_vcs[i].y(),
                 ordered_vcs[i].z());
    }

    return true;
}
Eigen::Matrix3d eulerDegToRotation(double roll_deg,
                                   double pitch_deg,
                                   double yaw_deg) {
    const Eigen::AngleAxisd rz(yaw_deg * kDegToRad, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd ry(pitch_deg * kDegToRad, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rx(roll_deg * kDegToRad, Eigen::Vector3d::UnitX());
    return (rz * ry * rx).toRotationMatrix();
}

double rotationAngleDeg(const Eigen::Matrix3d& R1,
                        const Eigen::Matrix3d& R2) {
    const Eigen::Matrix3d dR = R1.transpose() * R2;
    const double cos_val = std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5));
    return std::acos(cos_val) * kRadToDeg;
}

Eigen::Vector3d transformPoint(const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& t,
                               const Eigen::Vector3d& p) {
    return R * p + t;
}

double computeRMSEFromVectors(const std::vector<Eigen::Vector3d>& src,
                              const std::vector<Eigen::Vector3d>& dst) {
    if (src.size() != dst.size() || src.empty()) {
        return -1.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < src.size(); ++i) {
        sum += (src[i] - dst[i]).squaredNorm();
    }
    return std::sqrt(sum / static_cast<double>(src.size()));
}

bool reorderSceneByPriorPnP(const std::vector<Eigen::Vector3d>& vcs_pts,
                            const std::vector<Eigen::Vector3d>& cam_pts,
                            const std::vector<cv::Point2f>& cam_pixels,
                            const cv::Mat& K,
                            const cv::Mat& D,
                            const Eigen::Vector3d& prior_t_c2v,
                            std::vector<Eigen::Vector3d>& ordered_vcs,
                            std::vector<int>& ordered_indices) {
    ordered_vcs.clear();
    ordered_indices.clear();

    if (vcs_pts.size() != 4 || cam_pts.size() != 4 || cam_pixels.size() != 4) {
        return false;
    }

    // After directional sorting, only identity and left-right flip need checking.
    const std::array<std::array<int, 4>, 2> kPerms = {{
        {{0, 1, 2, 3}},  // identity
        {{1, 0, 3, 2}}   // left-right swap
    }};

    double best_translation_diff = std::numeric_limits<double>::infinity();
    std::array<int, 4> best_perm = {{0, 1, 2, 3}};

    for (const auto& perm : kPerms) {
        std::vector<cv::Point3f> object_points(4);
        std::vector<cv::Point2f> image_points(4);
        for (int i = 0; i < 4; ++i) {
            object_points[i] = cv::Point3f(
                static_cast<float>(vcs_pts[perm[i]].x()),
                static_cast<float>(vcs_pts[perm[i]].y()),
                static_cast<float>(vcs_pts[perm[i]].z()));
            image_points[i] = cam_pixels[i];
        }

        cv::Mat rvec, tvec;
        if (!cv::solvePnP(object_points, image_points, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            continue;
        }

        cv::Mat R_v2c_cv;
        cv::Rodrigues(rvec, R_v2c_cv);
        Eigen::Matrix3d R_v2c = Eigen::Matrix3d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_v2c(r, c) = R_v2c_cv.at<double>(r, c);
            }
        }

        const Eigen::Vector3d t_v2c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        const Eigen::Matrix3d R_c2v = R_v2c.transpose();
        const Eigen::Vector3d t_c2v = -R_c2v * t_v2c;

        const double dx = t_c2v.x() - prior_t_c2v.x();
        const double dy = t_c2v.y() - prior_t_c2v.y();
        const double dz = t_c2v.z() - prior_t_c2v.z();
        const double diff = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (diff < best_translation_diff) {
            best_translation_diff = diff;
            best_perm = perm;
        }
    }

    for (int i = 0; i < 4; ++i) {
        ordered_vcs.push_back(vcs_pts[best_perm[i]]);
        ordered_indices.push_back(best_perm[i]);
    }

    static const char* kLabels[4] = {"LT", "RT", "RB", "LB"};
    ROS_DEBUG("[CalibrationSolver] Prior PnP pairing best_translation_diff=%.6f m raw_idx=[%d %d %d %d]",
             best_translation_diff,
             best_perm[0],
             best_perm[1],
             best_perm[2],
             best_perm[3]);
    for (int i = 0; i < 4; ++i) {
        ROS_DEBUG("[CalibrationSolver]   %s raw[%d] -> (%.4f, %.4f, %.4f)",
                 kLabels[i],
                 ordered_indices[i],
                 ordered_vcs[i].x(),
                 ordered_vcs[i].y(),
                 ordered_vcs[i].z());
    }

    return std::isfinite(best_translation_diff);
}

double computeReprojectionRMSE(const std::vector<cv::Point2f>& measured,
                               const std::vector<cv::Point2f>& projected) {
    if (measured.size() != projected.size() || measured.empty()) {
        return -1.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < measured.size(); ++i) {
        const double dx = static_cast<double>(measured[i].x - projected[i].x);
        const double dy = static_cast<double>(measured[i].y - projected[i].y);
        sum += dx * dx + dy * dy;
    }
    return std::sqrt(sum / static_cast<double>(measured.size()));
}

void buildPnPInputs(const std::vector<SceneObservation>& scenes,
                    bool use_only_inliers,
                    std::vector<cv::Point3f>& object_points,
                    std::vector<cv::Point2f>& image_points,
                    std::vector<Eigen::Vector3d>& camera_points_3d,
                    std::vector<std::pair<int, int> >& scene_point_refs) {
    object_points.clear();
    image_points.clear();
    camera_points_3d.clear();
    scene_point_refs.clear();

    for (size_t scene_idx = 0; scene_idx < scenes.size(); ++scene_idx) {
        const SceneObservation& scene = scenes[scene_idx];
        for (int i = 0; i < 4; ++i) {
            if (use_only_inliers && !scene.inliers[i]) {
                continue;
            }
            object_points.push_back(cv::Point3f(static_cast<float>(scene.lidar_points[i].x()),
                                                static_cast<float>(scene.lidar_points[i].y()),
                                                static_cast<float>(scene.lidar_points[i].z())));
            image_points.push_back(scene.camera_pixels[i]);
            camera_points_3d.push_back(scene.camera_centers[i]);
            scene_point_refs.push_back(std::make_pair(static_cast<int>(scene_idx), i));
        }
    }
}

void resetAllInliers(std::vector<SceneObservation>& scenes, bool value) {
    for (size_t scene_idx = 0; scene_idx < scenes.size(); ++scene_idx) {
        scenes[scene_idx].inliers.fill(value);
        scenes[scene_idx].inlier_count = value ? 4 : 0;
    }
}

void assignInliersFromPnP(std::vector<SceneObservation>& scenes,
                          const std::vector<std::pair<int, int> >& refs,
                          const cv::Mat& inliers) {
    resetAllInliers(scenes, false);
    if (inliers.empty()) {
        return;
    }
    for (int i = 0; i < inliers.rows; ++i) {
        const int flat_idx = inliers.at<int>(i, 0);
        if (flat_idx < 0 || flat_idx >= static_cast<int>(refs.size())) {
            continue;
        }
        const int scene_idx = refs[flat_idx].first;
        const int point_idx = refs[flat_idx].second;
        scenes[scene_idx].inliers[point_idx] = true;
    }
    for (size_t scene_idx = 0; scene_idx < scenes.size(); ++scene_idx) {
        scenes[scene_idx].inlier_count = 0;
        for (int i = 0; i < 4; ++i) {
            if (scenes[scene_idx].inliers[i]) {
                ++scenes[scene_idx].inlier_count;
            }
        }
    }
}

bool refinePnP(const std::vector<cv::Point3f>& object_points,
               const std::vector<cv::Point2f>& image_points,
               const cv::Mat& K,
               const cv::Mat& D,
               cv::Mat& rvec,
               cv::Mat& tvec) {
    if (object_points.size() < 4 || image_points.size() != object_points.size()) {
        return false;
    }
    return cv::solvePnP(object_points,
                        image_points,
                        K,
                        D,
                        rvec,
                        tvec,
                        true,
                        cv::SOLVEPNP_ITERATIVE);
}

// ============================================================
// Ceres optimization helpers (ray-space error)
// ============================================================

void pixelToUnitRay(const cv::Point2f& pixel,
                    const cv::Mat& K,
                    const cv::Mat& D,
                    cv::Vec3d& ray) {
    const double fx = K.at<double>(0, 0);
    const double fy = K.at<double>(1, 1);
    const double cx = K.at<double>(0, 2);
    const double cy = K.at<double>(1, 2);

    const bool has_distortion = !D.empty() && D.cols >= 4 &&
        (std::abs(D.at<double>(0, 0)) > 1e-12 ||
         std::abs(D.at<double>(0, 1)) > 1e-12 ||
         std::abs(D.at<double>(0, 2)) > 1e-12 ||
         std::abs(D.at<double>(0, 3)) > 1e-12);

    double xn = 0.0;
    double yn = 0.0;
    if (has_distortion) {
        const double xd = (static_cast<double>(pixel.x) - cx) / fx;
        const double yd = (static_cast<double>(pixel.y) - cy) / fy;
        const double k1 = D.at<double>(0, 0);
        const double k2 = D.at<double>(0, 1);
        const double p1 = D.at<double>(0, 2);
        const double p2 = D.at<double>(0, 3);
        xn = xd;
        yn = yd;
        for (int iter = 0; iter < 30; ++iter) {
            const double r2 = xn * xn + yn * yn;
            const double r4 = r2 * r2;
            const double radial = 1.0 + k1 * r2 + k2 * r4;
            xn = (xd - 2.0 * p1 * xn * yn - p2 * (r2 + 2.0 * xn * xn)) / radial;
            yn = (yd - p1 * (r2 + 2.0 * yn * yn) - 2.0 * p2 * xn * yn) / radial;
        }
    } else {
        xn = (static_cast<double>(pixel.x) - cx) / fx;
        yn = (static_cast<double>(pixel.y) - cy) / fy;
    }

    ray = cv::Vec3d(xn, yn, 1.0);
    const double norm = cv::norm(ray);
    if (norm > 1e-12) {
        ray *= 1.0 / norm;
    }
}

struct RaySpaceError {
    RaySpaceError(double obs_ray_x, double obs_ray_y, double obs_ray_z,
                  double point_x, double point_y, double point_z)
        : obs_ray_x_(obs_ray_x), obs_ray_y_(obs_ray_y), obs_ray_z_(obs_ray_z),
          point_x_(point_x), point_y_(point_y), point_z_(point_z) {}

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        T point_vcs[3] = {T(point_x_), T(point_y_), T(point_z_)};
        T point_shifted[3] = {
            point_vcs[0] - pose[3],
            point_vcs[1] - pose[4],
            point_vcs[2] - pose[5]
        };
        T inv_rvec[3] = {-pose[0], -pose[1], -pose[2]};
        T point_cam[3];
        ceres::AngleAxisRotatePoint(inv_rvec, point_shifted, point_cam);

        T norm = ceres::sqrt(point_cam[0] * point_cam[0] +
                             point_cam[1] * point_cam[1] +
                             point_cam[2] * point_cam[2]);

        if (point_cam[2] <= T(1e-6) || norm <= T(1e-12)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            residuals[2] = T(1000.0) * (T(1.0) + T(1e-6) - point_cam[2]);
            return true;
        }

        T pred_x = point_cam[0] / norm;
        T pred_y = point_cam[1] / norm;
        T pred_z = point_cam[2] / norm;

        // Cross product: predicted_ray × observed_ray = sin(θ) * axis
        residuals[0] = pred_y * T(obs_ray_z_) - pred_z * T(obs_ray_y_);
        residuals[1] = pred_z * T(obs_ray_x_) - pred_x * T(obs_ray_z_);
        residuals[2] = pred_x * T(obs_ray_y_) - pred_y * T(obs_ray_x_);

        return true;
    }

private:
    double obs_ray_x_, obs_ray_y_, obs_ray_z_;
    double point_x_, point_y_, point_z_;
};

void clampTranslationToPrior(double prior_x,
                             double prior_y,
                             double prior_z,
                             double delta_limit,
                             bool separate_axis,
                             double* xyz) {
    if (xyz == NULL || delta_limit <= 0.0) {
        return;
    }

    if (separate_axis) {
        xyz[0] = std::max(prior_x - delta_limit, std::min(prior_x + delta_limit, xyz[0]));
        xyz[1] = std::max(prior_y - delta_limit, std::min(prior_y + delta_limit, xyz[1]));
        xyz[2] = std::max(prior_z - delta_limit, std::min(prior_z + delta_limit, xyz[2]));
        return;
    }

    const double dx = xyz[0] - prior_x;
    const double dy = xyz[1] - prior_y;
    const double dz = xyz[2] - prior_z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist <= delta_limit || dist <= 1e-12) {
        return;
    }

    const double scale = delta_limit / dist;
    xyz[0] = prior_x + dx * scale;
    xyz[1] = prior_y + dy * scale;
    xyz[2] = prior_z + dz * scale;
}

#if defined(CERES_VERSION_MAJOR) && (CERES_VERSION_MAJOR >= 2)
struct BoundedPoseParameterization : public ceres::Manifold {
public:
    BoundedPoseParameterization(double prior_x, double prior_y, double prior_z,
                                double delta_limit, bool separate_axis)
        : prior_x_(prior_x), prior_y_(prior_y), prior_z_(prior_z),
          delta_limit_(delta_limit), separate_axis_(separate_axis) {}

    virtual ~BoundedPoseParameterization() {}

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
        for (int i = 0; i < 6; ++i) {
            x_plus_delta[i] = x[i] + delta[i];
        }
        clampTranslationToPrior(prior_x_, prior_y_, prior_z_, delta_limit_, separate_axis_, x_plus_delta + 3);
        return true;
    }

    virtual bool PlusJacobian(const double* x, double* jacobian) const {
        (void)x;
        std::fill(jacobian, jacobian + 36, 0.0);
        for (int i = 0; i < 6; ++i) {
            jacobian[i * 6 + i] = 1.0;
        }
        return true;
    }

    virtual bool Minus(const double* y, const double* x, double* y_minus_x) const {
        for (int i = 0; i < 6; ++i) {
            y_minus_x[i] = y[i] - x[i];
        }
        return true;
    }

    virtual bool MinusJacobian(const double* x, double* jacobian) const {
        (void)x;
        std::fill(jacobian, jacobian + 36, 0.0);
        for (int i = 0; i < 6; ++i) {
            jacobian[i * 6 + i] = 1.0;
        }
        return true;
    }

    virtual int AmbientSize() const { return 6; }
    virtual int TangentSize() const { return 6; }

private:
    double prior_x_, prior_y_, prior_z_;
    double delta_limit_;
    bool separate_axis_;
};
#else
struct BoundedPoseParameterization : public ceres::LocalParameterization {
public:
    BoundedPoseParameterization(double prior_x, double prior_y, double prior_z,
                                double delta_limit, bool separate_axis)
        : prior_x_(prior_x), prior_y_(prior_y), prior_z_(prior_z),
          delta_limit_(delta_limit), separate_axis_(separate_axis) {}

    virtual ~BoundedPoseParameterization() {}

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
        for (int i = 0; i < 6; ++i) {
            x_plus_delta[i] = x[i] + delta[i];
        }
        clampTranslationToPrior(prior_x_, prior_y_, prior_z_, delta_limit_, separate_axis_, x_plus_delta + 3);
        return true;
    }

    virtual bool ComputeJacobian(const double* x, double* jacobian) const {
        (void)x;
        std::fill(jacobian, jacobian + 36, 0.0);
        for (int i = 0; i < 6; ++i) {
            jacobian[i * 6 + i] = 1.0;
        }
        return true;
    }

    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }

private:
    double prior_x_, prior_y_, prior_z_;
    double delta_limit_;
    bool separate_axis_;
};
#endif

bool optimizeWithCeres(const std::vector<SceneObservation>& scenes,
                       const CameraIntrinsics& cam,
                       const Eigen::Matrix3d& prior_R_c2v,
                       const Eigen::Vector3d& prior_t_c2v,
                       const CalibrationSolver::Params& params,
                       Eigen::Matrix3d& R_c2v,
                       Eigen::Vector3d& t_c2v) {
    (void)prior_R_c2v;
    if (!params.use_ceres_optimization) {
        return true;
    }

    ROS_INFO("[CalibrationSolver] Using Ceres optimization with xyz delta_limit=%.3f m in c2v frame (mode=%s)",
             params.xyz_delta_limit_m,
             params.separate_axis_constraint ? "axis" : "norm");

    double pose[6];
    {
        cv::Mat R_cv(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_cv.at<double>(r, c) = R_c2v(r, c);
            }
        }
        cv::Mat rvec;
        cv::Rodrigues(R_cv, rvec);
        pose[0] = rvec.at<double>(0);
        pose[1] = rvec.at<double>(1);
        pose[2] = rvec.at<double>(2);
        pose[3] = t_c2v.x();
        pose[4] = t_c2v.y();
        pose[5] = t_c2v.z();
    }

    clampTranslationToPrior(prior_t_c2v.x(),
                            prior_t_c2v.y(),
                            prior_t_c2v.z(),
                            params.xyz_delta_limit_m,
                            params.separate_axis_constraint,
                            pose + 3);

    const cv::Mat K = cam.getPnPCameraMatrix();
    const cv::Mat D = cam.getPnPDistCoeffs();

    // Pre-compute observed unit rays from image points (pixel -> undistort -> unit ray)
    struct IndexedRay {
        cv::Vec3d ray;
        int scene_idx;
        int pt_idx;
    };
    std::vector<IndexedRay> obs_rays;
    for (size_t si = 0; si < scenes.size(); ++si) {
        for (int i = 0; i < 4; ++i) {
            if (!scenes[si].inliers[i]) {
                continue;
            }
            IndexedRay ir;
            pixelToUnitRay(scenes[si].camera_pixels[i], K, D, ir.ray);
            ir.scene_idx = static_cast<int>(si);
            ir.pt_idx = i;
            obs_rays.push_back(ir);
        }
    }

    ceres::Problem problem;
    if (params.xyz_delta_limit_m > 0.0 && params.separate_axis_constraint) {
        problem.AddParameterBlock(pose, 6);
        problem.SetParameterLowerBound(pose, 3, prior_t_c2v.x() - params.xyz_delta_limit_m);
        problem.SetParameterUpperBound(pose, 3, prior_t_c2v.x() + params.xyz_delta_limit_m);
        problem.SetParameterLowerBound(pose, 4, prior_t_c2v.y() - params.xyz_delta_limit_m);
        problem.SetParameterUpperBound(pose, 4, prior_t_c2v.y() + params.xyz_delta_limit_m);
        problem.SetParameterLowerBound(pose, 5, prior_t_c2v.z() - params.xyz_delta_limit_m);
        problem.SetParameterUpperBound(pose, 5, prior_t_c2v.z() + params.xyz_delta_limit_m);
    } else if (params.xyz_delta_limit_m > 0.0) {
#if defined(CERES_VERSION_MAJOR) && (CERES_VERSION_MAJOR >= 2)
        problem.AddParameterBlock(pose, 6);
        problem.SetManifold(pose,
                            new BoundedPoseParameterization(prior_t_c2v.x(),
                                                            prior_t_c2v.y(),
                                                            prior_t_c2v.z(),
                                                            params.xyz_delta_limit_m,
                                                            false));
#else
        problem.AddParameterBlock(pose,
                                  6,
                                  new BoundedPoseParameterization(prior_t_c2v.x(),
                                                                  prior_t_c2v.y(),
                                                                  prior_t_c2v.z(),
                                                                  params.xyz_delta_limit_m,
                                                                  false));
#endif
    } else {
        problem.AddParameterBlock(pose, 6);
    }

    for (const auto& ir : obs_rays) {
        const auto& scene = scenes[ir.scene_idx];
        const int i = ir.pt_idx;

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<RaySpaceError, 3, 6>(
                new RaySpaceError(ir.ray[0], ir.ray[1], ir.ray[2],
                                  scene.lidar_points[i].x(),
                                  scene.lidar_points[i].y(),
                                  scene.lidar_points[i].z()));

        // Huber loss at sin(2deg) ≈ 0.035, matching ~2deg angular error outlier threshold
        problem.AddResidualBlock(cost_function,
                                 new ceres::HuberLoss(0.035),
                                 pose);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = params.optimization_max_iterations;
    options.function_tolerance = 1e-6;
    options.gradient_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
        ROS_WARN("[CalibrationSolver] Ceres solution unusable: %s", summary.BriefReport().c_str());
        return false;
    }

    ROS_INFO("[CalibrationSolver] Ceres optimization completed (ray-space): %d iterations, final cost=%.6f",
             static_cast<int>(summary.iterations.size()), summary.final_cost);

    {
        cv::Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0) = pose[0];
        rvec.at<double>(1) = pose[1];
        rvec.at<double>(2) = pose[2];

        cv::Mat R_cv(3, 3, CV_64F);
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3d R_c2v_result = Eigen::Matrix3d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_c2v_result(r, c) = R_cv.at<double>(r, c);
            }
        }

        R_c2v = R_c2v_result;
        t_c2v = Eigen::Vector3d(pose[3], pose[4], pose[5]);
    }

    // Compute final angular RMSE
    {
        const Eigen::Matrix3d R_v2c_final = R_c2v.transpose();
        const Eigen::Vector3d t_v2c_final = -R_v2c_final * t_c2v;
        double angular_sq_sum = 0.0;
        int angular_count = 0;
        for (const auto& ir : obs_rays) {
            const auto& scene = scenes[ir.scene_idx];
            const int i = ir.pt_idx;
            const Eigen::Vector3d lidar_pt = scene.lidar_points[i];
            const Eigen::Vector3d pred_cam = transformPoint(R_v2c_final, t_v2c_final, lidar_pt);
            const double pred_norm = pred_cam.norm();
            if (pred_norm > 1e-12 && pred_cam.z() > 1e-6) {
                const Eigen::Vector3d pred_ray = pred_cam / pred_norm;
                const Eigen::Vector3d obs_ray(ir.ray[0], ir.ray[1], ir.ray[2]);
                const Eigen::Vector3d cross = pred_ray.cross(obs_ray);
                const double sin_angle = std::min(1.0, cross.norm());
                const double angle_deg = std::asin(sin_angle) * 180.0 / M_PI;
                angular_sq_sum += angle_deg * angle_deg;
                ++angular_count;
            }
        }
        if (angular_count > 0) {
            const double angular_rmse_deg = std::sqrt(angular_sq_sum / angular_count);
            ROS_INFO("[CalibrationSolver] Final ray-space angular RMSE: %.4f deg (%d rays)",
                     angular_rmse_deg, angular_count);
        }
    }

    const double final_dx = t_c2v.x() - prior_t_c2v.x();
    const double final_dy = t_c2v.y() - prior_t_c2v.y();
    const double final_dz = t_c2v.z() - prior_t_c2v.z();
    const double final_max_abs = std::max(std::fabs(final_dx), std::max(std::fabs(final_dy), std::fabs(final_dz)));
    const double final_dist = std::sqrt(final_dx * final_dx + final_dy * final_dy + final_dz * final_dz);
    ROS_INFO("[CalibrationSolver] After Ceres: final c2v xyz delta from prior = (%.6f, %.6f, %.6f)m max_abs=%.6f dist=%.6f (limit=%.3f m)",
             final_dx,
             final_dy,
             final_dz,
             final_max_abs,
             final_dist,
             params.xyz_delta_limit_m);

    return true;
}


}  // namespace

CalibrationSolver::CalibrationSolver() {}

void CalibrationSolver::setParams(const Params& p) {
    params_ = p;
}

void CalibrationSolver::setCamera(const CameraIntrinsics& cam) {
    cam_ = cam;
    camera_ready_ = true;
}

void CalibrationSolver::sortPatternCenters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& out,
                                           const std::string& axis_mode) {
    if (!in || in->size() != 4) {
        ROS_ERROR("[CalibrationSolver::sortPatternCenters] Input must have exactly 4 points (got %zu).",
                  in ? in->size() : 0);
        out = in;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr work(new pcl::PointCloud<pcl::PointXYZ>);

    if (axis_mode == "vcs") {
        for (const auto& p : in->points) {
            pcl::PointXYZ pt;
            pt.x = -p.y;
            pt.y = -p.z;
            pt.z = p.x;
            work->push_back(pt);
        }
    } else {
        *work = *in;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*work, centroid);

    std::vector<std::pair<float, int> > angle_idx;
    for (size_t i = 0; i < work->size(); ++i) {
        const float dx = work->points[i].x - centroid[0];
        const float dy = work->points[i].y - centroid[1];
        angle_idx.push_back(std::make_pair(std::atan2(dy, dx), static_cast<int>(i)));
    }
    std::sort(angle_idx.begin(), angle_idx.end());

    out.reset(new pcl::PointCloud<pcl::PointXYZ>);
    out->resize(4);
    for (int i = 0; i < 4; ++i) {
        (*out)[i] = work->points[angle_idx[i].second];
    }

    const pcl::PointXYZ& p0 = out->points[0];
    const pcl::PointXYZ& p1 = out->points[1];
    const pcl::PointXYZ& p2 = out->points[2];
    const Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0.0f);
    const Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0.0f);
    if (v01.cross(v12).z() > 0.0f) {
        std::swap((*out)[1], (*out)[3]);
    }

    if (axis_mode == "vcs") {
        for (size_t i = 0; i < out->points.size(); ++i) {
            pcl::PointXYZ& pt = out->points[i];
            const float x = pt.z;
            const float y = -pt.x;
            const float z = -pt.y;
            pt.x = x;
            pt.y = y;
            pt.z = z;
        }
    }
}

double CalibrationSolver::computeRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt) {
    if (src->size() != tgt->size() || src->empty()) {
        return -1.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < src->size(); ++i) {
        const double dx = src->points[i].x - tgt->points[i].x;
        const double dy = src->points[i].y - tgt->points[i].y;
        const double dz = src->points[i].z - tgt->points[i].z;
        sum += dx * dx + dy * dy + dz * dz;
    }
    return std::sqrt(sum / static_cast<double>(src->size()));
}

void CalibrationSolver::extractEulerZYX(const Eigen::Matrix3d& R,
                                        double& roll,
                                        double& pitch,
                                        double& yaw) {
    pitch = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));

    roll *= kRadToDeg;
    pitch *= kRadToDeg;
    yaw *= kRadToDeg;
}

bool CalibrationSolver::solve(const std::vector<SceneData>& scenes,
                              CalibResult& result) {
    result = CalibResult();

    if (!camera_ready_) {
        result.message = "Camera intrinsics not set for PnP solver.";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    std::vector<const SceneData*> valid;
    for (size_t i = 0; i < scenes.size(); ++i) {
        const SceneData& s = scenes[i];
        if (s.valid &&
            s.camera_centers.size() == 4 &&
            s.camera_circle_pixels.size() == 4 &&
            s.lidar_centers.size() == 4) {
            valid.push_back(&s);
        }
    }

    result.solver_input_scenes = static_cast<int>(valid.size());
    if (static_cast<int>(valid.size()) < params_.min_scenes) {
        result.message = "Not enough valid scenes: " + std::to_string(valid.size()) +
                         " < " + std::to_string(params_.min_scenes);
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    const cv::Mat K = cam_.getPnPCameraMatrix();
    const cv::Mat D = cam_.getPnPDistCoeffs();

    const Eigen::Matrix3d prior_R_c2v = eulerDegToRotation(params_.prior_roll_deg,
                                                           params_.prior_pitch_deg,
                                                           params_.prior_yaw_deg);
    const Eigen::Vector3d prior_t_c2v(params_.prior_x_m,
                                      params_.prior_y_m,
                                      params_.prior_z_m);
    const Eigen::Matrix3d prior_R_v2c = prior_R_c2v.transpose();
    const Eigen::Vector3d prior_t_v2c = -prior_R_v2c * prior_t_c2v;

    std::vector<SceneObservation> obs_scenes;
    obs_scenes.reserve(valid.size());
    static const char* kLabels[4] = {"LT", "RT", "RB", "LB"};
    for (size_t scene_idx = 0; scene_idx < valid.size(); ++scene_idx) {
        const SceneData* s = valid[scene_idx];
        SceneObservation scene;
        scene.lidar_points = s->lidar_centers;
        scene.raw_indices = {0, 1, 2, 3};
        scene.scene_input_index = static_cast<int>(scene_idx);

        std::vector<int> camera_raw_indices;
        double cam_row_gap = 0.0;
        double cam_col_gap = 0.0;
        if (!reorderCameraByImagePixels(s->camera_centers,
                                        s->camera_circle_pixels,
                                        scene.camera_centers,
                                        scene.camera_pixels,
                                        camera_raw_indices,
                                        cam_row_gap,
                                        cam_col_gap)) {
            result.message = "Failed to reorder camera circle centers by image pixels.";
            ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
            return false;
        }

        double row_gap = 0.0;
        double col_gap = 0.0;
        // Sort LiDAR centers by spatial direction into canonical LT/RT/RB/LB order
        if (!reorderSceneFrontByYZ(s->lidar_centers,
                                   scene.lidar_points,
                                   scene.raw_indices,
                                   row_gap,
                                   col_gap)) {
            result.message = "Failed to reorder LiDAR centers by front hard-order.";
            ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
            return false;
        }
        // Resolve remaining flip ambiguities via PnP + prior check (only 4 permutations)
        if (params_.use_prior_pairing) {
            const std::vector<Eigen::Vector3d> sorted_lidar_pts = scene.lidar_points;
            const std::vector<int> sorted_raw_indices = scene.raw_indices;
            if (!reorderSceneByPriorPnP(sorted_lidar_pts,
                                        scene.camera_centers,
                                        scene.camera_pixels,
                                        K,
                                        D,
                                        prior_t_c2v,
                                        scene.lidar_points,
                                        scene.raw_indices)) {
                result.message = "Failed to reorder LiDAR centers by PnP prior pairing.";
                ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
                return false;
            }
            // Compose indices: raw_indices from PnP are into sorted array,
            // map through sorted_raw_indices back to original raw data
            for (int i = 0; i < 4; ++i) {
                const int sorted_idx = scene.raw_indices[i];
                if (sorted_idx >= 0 && sorted_idx < 4) {
                    scene.raw_indices[i] = sorted_raw_indices[sorted_idx];
                }
            }
        }

        ROS_INFO("[CalibrationSolver] Scene %zu 2D-3D matches:", scene_idx + 1);
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector3d& pc = scene.camera_centers[i];
            const cv::Point2f& uv = scene.camera_pixels[i];
            const Eigen::Vector3d& pv = scene.lidar_points[i];
            ROS_INFO("[CalibrationSolver]   %s 2d=(%.2f, %.2f) cam3d=(%.4f, %.4f, %.4f) <-> lidar3d=(%.4f, %.4f, %.4f) raw_idx=%d",
                     kLabels[i],
                     uv.x,
                     uv.y,
                     pc.x(),
                     pc.y(),
                     pc.z(),
                     pv.x(),
                     pv.y(),
                     pv.z(),
                     scene.raw_indices[i]);
        }

        obs_scenes.push_back(scene);
    }

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    std::vector<Eigen::Vector3d> camera_points_3d;
    std::vector<std::pair<int, int> > refs;
    buildPnPInputs(obs_scenes, false, object_points, image_points, camera_points_3d, refs);

    const int total_correspondences = static_cast<int>(object_points.size());
    ROS_DEBUG("[CalibrationSolver] Total PnP correspondences before RANSAC: %d", total_correspondences);
    if (total_correspondences < 4) {
        result.message = "Not enough total PnP correspondences.";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec(3, 1, CV_64F);
    {
        cv::Mat R_cv(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_cv.at<double>(r, c) = prior_R_v2c(r, c);
            }
        }
        cv::Rodrigues(R_cv, rvec);
        tvec.at<double>(0) = prior_t_v2c.x();
        tvec.at<double>(1) = prior_t_v2c.y();
        tvec.at<double>(2) = prior_t_v2c.z();
    }

    cv::Mat inliers;
    if (params_.use_ransac_rejection) {
        const int iterations = std::max(50, params_.optimization_max_iterations * 4);
        const bool ok = cv::solvePnPRansac(object_points,
                                           image_points,
                                           K,
                                           D,
                                           rvec,
                                           tvec,
                                           true,
                                           iterations,
                                           static_cast<float>(params_.ransac_reproj_threshold_px),
                                           0.99,
                                           inliers,
                                           cv::SOLVEPNP_ITERATIVE);
        if (!ok || inliers.rows < std::max(4, params_.ransac_min_inliers)) {
            result.message = "solvePnPRansac failed or inliers not enough.";
            ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
            return false;
        }
        assignInliersFromPnP(obs_scenes, refs, inliers);
        ROS_INFO("[CalibrationSolver] solvePnPRansac keep=%d/%d correspondences, reproj_thresh=%.2f px",
                 inliers.rows,
                 total_correspondences,
                 params_.ransac_reproj_threshold_px);
    } else {
        resetAllInliers(obs_scenes, true);
    }

    const int min_scene_inliers = std::max(1, std::min(4, params_.ransac_min_scene_inliers));
    std::vector<SceneObservation> filtered_scenes;
    filtered_scenes.reserve(obs_scenes.size());
    for (size_t scene_idx = 0; scene_idx < obs_scenes.size(); ++scene_idx) {
        const SceneObservation& scene = obs_scenes[scene_idx];
        ROS_INFO("[CalibrationSolver] Scene %zu PnP inliers=%d/4 mask=[%d %d %d %d]",
                 scene_idx + 1,
                 scene.inlier_count,
                 scene.inliers[0] ? 1 : 0,
                 scene.inliers[1] ? 1 : 0,
                 scene.inliers[2] ? 1 : 0,
                 scene.inliers[3] ? 1 : 0);
        if (scene.inlier_count >= min_scene_inliers) {
            filtered_scenes.push_back(scene);
        } else {
            ROS_WARN("[CalibrationSolver] Drop scene %zu after PnP-RANSAC: inliers=%d < %d",
                     scene_idx + 1,
                     scene.inlier_count,
                     min_scene_inliers);
        }
    }
    obs_scenes.swap(filtered_scenes);
    result.optimization_scenes = static_cast<int>(obs_scenes.size());
    result.solver_dropped_scenes = std::max(0, result.solver_input_scenes - result.optimization_scenes);

    if (static_cast<int>(obs_scenes.size()) < params_.min_scenes) {
        result.message = "Not enough scenes after PnP-RANSAC: " + std::to_string(obs_scenes.size()) +
                         " < " + std::to_string(params_.min_scenes);
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    buildPnPInputs(obs_scenes, true, object_points, image_points, camera_points_3d, refs);
    if (static_cast<int>(object_points.size()) < std::max(4, params_.ransac_min_inliers)) {
        result.message = "Not enough inlier correspondences after scene filtering.";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    if (!refinePnP(object_points, image_points, K, D, rvec, tvec)) {
        result.message = "solvePnP iterative refinement failed.";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, K, D, projected_points);
    const double reproj_rmse_px = computeReprojectionRMSE(image_points, projected_points);

    cv::Mat R_v2c_cv;
    cv::Rodrigues(rvec, R_v2c_cv);
    Eigen::Matrix3d R_v2c = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R_v2c(r, c) = R_v2c_cv.at<double>(r, c);
        }
    }
    const Eigen::Vector3d t_v2c(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    std::vector<Eigen::Vector3d> predicted_cam_points;
    predicted_cam_points.reserve(object_points.size());
    for (size_t i = 0; i < object_points.size(); ++i) {
        const Eigen::Vector3d lidar_pt(object_points[i].x, object_points[i].y, object_points[i].z);
        predicted_cam_points.push_back(transformPoint(R_v2c, t_v2c, lidar_pt));
    }
    const double rmse_3d = computeRMSEFromVectors(predicted_cam_points, camera_points_3d);

    const Eigen::Matrix3d final_R_c2v = R_v2c.transpose();
    const Eigen::Vector3d final_t_c2v = -final_R_c2v * t_v2c;

    const double dx_from_prior = final_t_c2v.x() - params_.prior_x_m;
    const double dy_from_prior = final_t_c2v.y() - params_.prior_y_m;
    const double dz_from_prior = final_t_c2v.z() - params_.prior_z_m;
    const double max_abs_delta_from_prior = std::max(std::fabs(dx_from_prior),
                                                     std::max(std::fabs(dy_from_prior),
                                                              std::fabs(dz_from_prior)));
    const double rotation_delta_from_prior_deg = rotationAngleDeg(prior_R_c2v, final_R_c2v);
    ROS_INFO("[CalibrationSolver] PnP done: reproj_rmse=%.4f px, pseudo-3d RMSE=%.6f m, inliers=%zu/%d",
             reproj_rmse_px,
             rmse_3d,
             object_points.size(),
             total_correspondences);
    ROS_INFO("[CalibrationSolver] Pose prior check: pos_delta=(%.4f, %.4f, %.4f)m max_abs=%.4fm rot_delta=%.3fdeg",
             dx_from_prior, dy_from_prior, dz_from_prior,
             max_abs_delta_from_prior, rotation_delta_from_prior_deg);

    if (params_.max_translation_delta_from_prior_m > 0.0 &&
        max_abs_delta_from_prior > params_.max_translation_delta_from_prior_m) {
        result.message = "PnP pose prior check failed: translation delta too large: " + std::to_string(max_abs_delta_from_prior) +
                         "m > " + std::to_string(params_.max_translation_delta_from_prior_m) + "m";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    if (params_.max_rotation_delta_from_prior_deg > 0.0 &&
        rotation_delta_from_prior_deg > params_.max_rotation_delta_from_prior_deg) {
        result.message = "PnP pose prior check failed: rotation delta too large: " + std::to_string(rotation_delta_from_prior_deg) +
                         "deg > " + std::to_string(params_.max_rotation_delta_from_prior_deg) + "deg";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    if (params_.max_pairing_rmse_m > 0.0 && rmse_3d > params_.max_pairing_rmse_m) {
        result.message = "PnP pseudo-3d RMSE too large: " + std::to_string(rmse_3d) +
                         "m > " + std::to_string(params_.max_pairing_rmse_m) + "m";
        ROS_WARN("[CalibrationSolver] %s", result.message.c_str());
        return false;
    }

    Eigen::Matrix3d result_R_c2v = final_R_c2v;
    Eigen::Vector3d result_t_c2v = final_t_c2v;
    if (params_.use_ceres_optimization) {
        Eigen::Matrix3d optimized_R_c2v = final_R_c2v;
        Eigen::Vector3d optimized_t_c2v = final_t_c2v;

        if (optimizeWithCeres(obs_scenes, cam_, prior_R_c2v, prior_t_c2v, params_,
                              optimized_R_c2v, optimized_t_c2v)) {
            result_R_c2v = optimized_R_c2v;
            result_t_c2v = optimized_t_c2v;
        } else {
            ROS_WARN("[CalibrationSolver] Ceres optimization failed, keep refined solvePnP result.");
        }
    }

    result.R = result_R_c2v;
    result.t = result_t_c2v;
    result.T = Eigen::Matrix4d::Identity();
    result.T.block<3, 3>(0, 0) = result.R;
    result.T.block<3, 1>(0, 3) = result.t;
    extractEulerZYX(result.R, result.roll_deg, result.pitch_deg, result.yaw_deg);
    result.x_m = result.t.x();
    result.y_m = result.t.y();
    result.z_m = result.t.z();
    result.num_scenes = static_cast<int>(obs_scenes.size());
    result.optimization_scenes = result.num_scenes;
    result.optimization_scene_indices.clear();
    result.optimization_scene_indices.reserve(obs_scenes.size());
    for (size_t i = 0; i < obs_scenes.size(); ++i) {
        result.optimization_scene_indices.push_back(obs_scenes[i].scene_input_index);
    }
    result.valid = true;

    const Eigen::Matrix3d result_R_v2c = result.R.transpose();
    const Eigen::Vector3d result_t_v2c = -result_R_v2c * result.t;

    cv::Mat final_rvec(3, 1, CV_64F);
    cv::Mat final_R_v2c_cv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            final_R_v2c_cv.at<double>(r, c) = result_R_v2c(r, c);
        }
    }
    cv::Rodrigues(final_R_v2c_cv, final_rvec);
    cv::Mat final_tvec = (cv::Mat_<double>(3, 1) << result_t_v2c.x(), result_t_v2c.y(), result_t_v2c.z());

    std::vector<Eigen::Vector3d> final_predicted_cam_points;
    final_predicted_cam_points.reserve(object_points.size());
    for (size_t i = 0; i < object_points.size(); ++i) {
        const Eigen::Vector3d lidar_pt(object_points[i].x, object_points[i].y, object_points[i].z);
        final_predicted_cam_points.push_back(transformPoint(result_R_v2c, result_t_v2c, lidar_pt));
    }
    const double final_rmse_3d = computeRMSEFromVectors(final_predicted_cam_points, camera_points_3d);

    std::vector<cv::Point2f> final_projected_points;
    cv::projectPoints(object_points, final_rvec, final_tvec, K, D, final_projected_points);
    const double final_reproj_rmse_px = computeReprojectionRMSE(image_points, final_projected_points);

    result.rmse_m = final_rmse_3d;
    result.raw_rmse_m = final_rmse_3d;
    result.reproj_rmse_px = final_reproj_rmse_px;

    if (result.rmse_m < 0.01 && result.num_scenes >= 3) {
        result.quality_grade = "EXCELLENT";
        result.confidence = 1.0;
    } else if (result.rmse_m < 0.02 && result.num_scenes >= 2) {
        result.quality_grade = "GOOD";
        result.confidence = 0.85;
    } else if (result.rmse_m < 0.05) {
        result.quality_grade = "ACCEPTABLE";
        result.confidence = 0.6;
    } else {
        result.quality_grade = "POOR";
        result.confidence = 0.3;
    }

    result.message = "Calibration OK. Grade=" + result.quality_grade +
                     " RMSE=" + std::to_string(result.rmse_m) + "m" +
                     " reproj=" + std::to_string(result.reproj_rmse_px) + "px" +
                     " scenes=" + std::to_string(result.num_scenes) +
                     " inliers=" + std::to_string(object_points.size()) + "/" + std::to_string(total_correspondences);

    ROS_INFO("[CalibrationSolver] %s", result.message.c_str());
    ROS_INFO("[CalibrationSolver] Final pose: xyz=(%.4f, %.4f, %.4f)m rpy=(%.3f, %.3f, %.3f)deg",
             result.x_m, result.y_m, result.z_m,
             result.roll_deg, result.pitch_deg, result.yaw_deg);
    ROS_INFO("[CalibrationSolver] Final metrics: reproj_rmse=%.4f px pseudo-3d RMSE=%.6f m",
             result.reproj_rmse_px,
             result.rmse_m);
    ROS_INFO("[CalibrationSolver] T_cam_to_VCS (P_VCS = R*P_cam + t):\n"
             "  R: [%.4f %.4f %.4f | %.4f %.4f %.4f | %.4f %.4f %.4f]\n"
             "  t(cam pos in VCS FLU): [x=%.4f, y=%.4f, z=%.4f] m",
             result.R(0, 0), result.R(0, 1), result.R(0, 2),
             result.R(1, 0), result.R(1, 1), result.R(1, 2),
             result.R(2, 0), result.R(2, 1), result.R(2, 2),
             result.x_m, result.y_m, result.z_m);
    ROS_INFO("[CalibrationSolver] Euler ZYX (cam in VCS): roll=%.3f deg pitch=%.3f deg yaw=%.3f deg",
             result.roll_deg,
             result.pitch_deg,
             result.yaw_deg);
    ROS_DEBUG("[CalibrationSolver] Rotation diff vs prior: %.3f deg",
              rotation_delta_from_prior_deg);

    return true;
}

}  // namespace image_lidar_calibration
