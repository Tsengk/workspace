#include "image_lidar_calibration/lidar_processor.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

namespace image_lidar_calibration {
namespace {

struct PlaneMetrics {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double width_m = 0.0;
    double height_m = 0.0;
    double short_side_m = 0.0;
    double long_side_m = 0.0;
    double area_m2 = 0.0;
    double vertical_score = 0.0;
};

struct BoardFrame2D {
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Matrix2d basis = Eigen::Matrix2d::Identity();
    double width_m = 0.0;
    double height_m = 0.0;
    bool valid = false;
};

static double percentileValue(std::vector<double> values, double ratio)
{
    if (values.empty()) return 0.0;

    ratio = std::max(0.0, std::min(1.0, ratio));
    std::sort(values.begin(), values.end());

    const double pos = ratio * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));
    if (lo == hi) return values[lo];

    const double alpha = pos - static_cast<double>(lo);
    return values[lo] * (1.0 - alpha) + values[hi] * alpha;
}

static PlaneMetrics computePlaneMetrics(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::ModelCoefficients::Ptr& coefficients)
{
    PlaneMetrics metrics;
    if (!cloud || cloud->empty() || !coefficients || coefficients->values.size() < 4) {
        return metrics;
    }

    Eigen::Vector3d normal(coefficients->values[0],
                           coefficients->values[1],
                           coefficients->values[2]);
    if (normal.norm() < 1e-9) return metrics;
    metrics.normal = normal.normalized();

    for (const auto& pt : cloud->points) {
        metrics.centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    metrics.centroid /= static_cast<double>(cloud->size());

    Eigen::Vector3d axis_u = metrics.normal.unitOrthogonal();
    if (axis_u.norm() < 1e-9) axis_u = Eigen::Vector3d::UnitX();
    axis_u.normalize();
    Eigen::Vector3d axis_v = metrics.normal.cross(axis_u);
    if (axis_v.norm() < 1e-9) axis_v = Eigen::Vector3d::UnitY();
    axis_v.normalize();

    std::vector<double> us;
    std::vector<double> vs;
    us.reserve(cloud->size());
    vs.reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const Eigen::Vector3d d = p - metrics.centroid;
        us.push_back(d.dot(axis_u));
        vs.push_back(d.dot(axis_v));
    }

    const double u_min = percentileValue(us, 0.10);
    const double u_max = percentileValue(us, 0.90);
    const double v_min = percentileValue(vs, 0.10);
    const double v_max = percentileValue(vs, 0.90);

    metrics.width_m = std::max(0.0, u_max - u_min);
    metrics.height_m = std::max(0.0, v_max - v_min);
    metrics.short_side_m = std::min(metrics.width_m, metrics.height_m);
    metrics.long_side_m = std::max(metrics.width_m, metrics.height_m);
    metrics.area_m2 = metrics.width_m * metrics.height_m;
    metrics.vertical_score = 1.0 - std::abs(metrics.normal.z());
    return metrics;
}

static double scoreBoardPlane(const PlaneMetrics& metrics,
                              size_t inlier_count,
                              double expected_w,
                              double expected_h)
{
    if (inlier_count < 30) return -1.0;
    if (metrics.vertical_score < 0.55) return -1.0;

    const double expected_long = std::max(expected_w, expected_h);
    const double expected_short = std::min(expected_w, expected_h);
    const double expected_area = expected_w * expected_h;

    if (metrics.long_side_m < expected_long * 0.45) return -1.0;
    if (metrics.short_side_m < expected_short * 0.45) return -1.0;
    if (metrics.long_side_m > expected_long * 2.50) return -1.0;
    if (metrics.short_side_m > expected_short * 2.50) return -1.0;
    if (metrics.area_m2 < expected_area * 0.20) return -1.0;
    if (metrics.area_m2 > expected_area * 4.00) return -1.0;

    const double long_err = std::abs(metrics.long_side_m - expected_long) /
                            std::max(expected_long, 1e-6);
    const double short_err = std::abs(metrics.short_side_m - expected_short) /
                             std::max(expected_short, 1e-6);
    const double area_err = std::abs(metrics.area_m2 - expected_area) /
                            std::max(expected_area, 1e-6);
    const double inlier_bonus = std::min(2.0, static_cast<double>(inlier_count) / 120.0);

    return 3.0 * metrics.vertical_score + inlier_bonus - long_err - short_err - 0.3 * area_err;
}

static double rectangleError(const std::vector<Eigen::Vector2d>& pts,
                             double w, double h)
{
    if (pts.size() != 4) return std::numeric_limits<double>::infinity();

    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    for (const auto& p : pts) c += p;
    c /= 4.0;

    std::vector<std::pair<double, int>> angles;
    for (int i = 0; i < 4; ++i) {
        angles.emplace_back(std::atan2(pts[i].y() - c.y(), pts[i].x() - c.x()), i);
    }
    std::sort(angles.begin(), angles.end());

    std::vector<Eigen::Vector2d> sorted;
    for (const auto& item : angles) sorted.push_back(pts[item.second]);

    const double s01 = (sorted[1] - sorted[0]).norm();
    const double s12 = (sorted[2] - sorted[1]).norm();
    const double s23 = (sorted[3] - sorted[2]).norm();
    const double s30 = (sorted[0] - sorted[3]).norm();

    const double err_layout1 =
        std::abs(s01 - w) / w + std::abs(s12 - h) / h +
        std::abs(s23 - w) / w + std::abs(s30 - h) / h;
    const double err_layout2 =
        std::abs(s01 - h) / h + std::abs(s12 - w) / w +
        std::abs(s23 - h) / h + std::abs(s30 - w) / w;

    const double diag = std::sqrt(w * w + h * h);
    const double d02 = (sorted[2] - sorted[0]).norm();
    const double d13 = (sorted[3] - sorted[1]).norm();
    const double diag_err = std::abs(d02 - diag) / diag + std::abs(d13 - diag) / diag;

    return std::min(err_layout1, err_layout2) + diag_err;
}

static bool fitCircleOnCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                               double expected_radius,
                               Eigen::Vector2d& center,
                               double& score)
{
    score = -1.0;
    if (!cluster || cluster->size() < 8) return false;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.015);
    seg.setOptimizeCoefficients(true);
    seg.setMaxIterations(1000);
    seg.setRadiusLimits(std::max(0.02, expected_radius - 0.05), expected_radius + 0.05);
    seg.setInputCloud(cluster);

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.size() < 6 || coeff->values.size() < 3) return false;

    const double radius = coeff->values[2];
    double residual = 0.0;
    for (int idx : inliers->indices) {
        const double dx = cluster->points[idx].x - coeff->values[0];
        const double dy = cluster->points[idx].y - coeff->values[1];
        const double dist = std::sqrt(dx * dx + dy * dy);
        residual += std::abs(dist - radius);
    }
    residual /= static_cast<double>(inliers->indices.size());

    const double radius_err = std::abs(radius - expected_radius);
    if (radius_err > 0.05 || residual > 0.02) return false;

    center = Eigen::Vector2d(coeff->values[0], coeff->values[1]);
    score = static_cast<double>(inliers->indices.size()) - 80.0 * residual - 40.0 * radius_err;
    return true;
}

static int countPointsInBand(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const Eigen::Vector2d& center,
                             double r_min,
                             double r_max)
{
    if (!cloud || cloud->empty()) return 0;

    int count = 0;
    for (const auto& pt : cloud->points) {
        const double dx = pt.x - center.x();
        const double dy = pt.y - center.y();
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d >= r_min && d <= r_max) ++count;
    }
    return count;
}

static int countPointsInDisk(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const Eigen::Vector2d& center,
                             double radius)
{
    if (!cloud || cloud->empty()) return 0;

    int count = 0;
    const double radius2 = radius * radius;
    for (const auto& pt : cloud->points) {
        const double dx = pt.x - center.x();
        const double dy = pt.y - center.y();
        const double d2 = dx * dx + dy * dy;
        if (d2 <= radius2) ++count;
    }
    return count;
}

static bool computeBoardFrame2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud,
                                double expected_w,
                                double expected_h,
                                BoardFrame2D& frame)
{
    frame = BoardFrame2D();
    if (!plane_cloud || plane_cloud->size() < 20) return false;

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& pt : plane_cloud->points) {
        centroid += Eigen::Vector2d(pt.x, pt.y);
    }
    centroid /= static_cast<double>(plane_cloud->size());

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& pt : plane_cloud->points) {
        const Eigen::Vector2d d(pt.x - centroid.x(), pt.y - centroid.y());
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(plane_cloud->size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() != Eigen::Success) return false;

    Eigen::Vector2d axis_a = solver.eigenvectors().col(1).normalized();
    Eigen::Vector2d axis_b = solver.eigenvectors().col(0).normalized();

    auto evaluate_axes = [&](const Eigen::Vector2d& axis_x,
                             const Eigen::Vector2d& axis_y,
                             double& width_out,
                             double& height_out,
                             Eigen::Vector2d& center_out,
                             double& error_out) {
        std::vector<double> xs;
        std::vector<double> ys;
        xs.reserve(plane_cloud->size());
        ys.reserve(plane_cloud->size());

        for (const auto& pt : plane_cloud->points) {
            const Eigen::Vector2d d(pt.x - centroid.x(), pt.y - centroid.y());
            xs.push_back(d.dot(axis_x));
            ys.push_back(d.dot(axis_y));
        }

        const double x_min = percentileValue(xs, 0.05);
        const double x_max = percentileValue(xs, 0.95);
        const double y_min = percentileValue(ys, 0.05);
        const double y_max = percentileValue(ys, 0.95);

        width_out = std::max(0.0, x_max - x_min);
        height_out = std::max(0.0, y_max - y_min);
        center_out = centroid + axis_x * (0.5 * (x_min + x_max)) + axis_y * (0.5 * (y_min + y_max));
        error_out = std::abs(width_out - expected_w) / std::max(expected_w, 1e-6) +
                    std::abs(height_out - expected_h) / std::max(expected_h, 1e-6);
    };

    double width_ab = 0.0, height_ab = 0.0, err_ab = 0.0;
    double width_ba = 0.0, height_ba = 0.0, err_ba = 0.0;
    Eigen::Vector2d center_ab = Eigen::Vector2d::Zero();
    Eigen::Vector2d center_ba = Eigen::Vector2d::Zero();

    evaluate_axes(axis_a, axis_b, width_ab, height_ab, center_ab, err_ab);
    evaluate_axes(axis_b, axis_a, width_ba, height_ba, center_ba, err_ba);

    if (err_ab <= err_ba) {
        frame.basis.col(0) = axis_a;
        frame.basis.col(1) = axis_b;
        frame.center = center_ab;
        frame.width_m = width_ab;
        frame.height_m = height_ab;
    } else {
        frame.basis.col(0) = axis_b;
        frame.basis.col(1) = axis_a;
        frame.center = center_ba;
        frame.width_m = width_ba;
        frame.height_m = height_ba;
    }

    frame.valid = true;
    return true;
}

static bool refineCenterNearPrediction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_cloud,
                                       const Eigen::Vector2d& predicted_center,
                                       double expected_radius,
                                       const LidarProcessor::Params& params,
                                       Eigen::Vector2d& refined_center,
                                       double& support_score)
{
    support_score = -1.0;
    if (!edge_cloud || edge_cloud->empty()) return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
    const double search_radius = expected_radius + params.hole_search_radius_margin;
    const double max_center_shift = params.hole_max_center_shift;

    for (const auto& pt : edge_cloud->points) {
        const double dx = pt.x - predicted_center.x();
        const double dy = pt.y - predicted_center.y();
        if (std::sqrt(dx * dx + dy * dy) <= search_radius) {
            local->push_back(pt);
        }
    }

    if (local->size() < 8) return false;

    double fit_score = -1.0;
    Eigen::Vector2d fitted_center;
    if (!fitCircleOnCluster(local, expected_radius, fitted_center, fit_score)) {
        return false;
    }

    if ((fitted_center - predicted_center).norm() > max_center_shift) {
        return false;
    }

    refined_center = fitted_center;
    support_score = fit_score;
    return true;
}

static bool detectCentersByBoardPrior(const pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud,
                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_cloud,
                                      double delta_width_circles,
                                      double delta_height_circles,
                                      double circle_radius,
                                      const LidarProcessor::Params& params,
                                      std::vector<Eigen::Vector2d>& centers_z0)
{
    centers_z0.clear();

    const double expected_board_w = delta_width_circles + 2.0 * circle_radius;
    const double expected_board_h = delta_height_circles + 2.0 * circle_radius;

    BoardFrame2D frame;
    if (!computeBoardFrame2D(plane_cloud,
                             expected_board_w,
                             expected_board_h,
                             frame)) {
        return false;
    }

    ROS_DEBUG("[LidarProcessor] Board frame(raw): center=(%.3f, %.3f) size=%.3f x %.3f",
             frame.center.x(), frame.center.y(), frame.width_m, frame.height_m);

    auto cropByFrame = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                           double margin_x,
                           double margin_y) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr dst(new pcl::PointCloud<pcl::PointXYZ>);
        if (!src || src->empty()) return dst;

        const double half_w = 0.5 * expected_board_w + margin_x;
        const double half_h = 0.5 * expected_board_h + margin_y;
        for (const auto& pt : src->points) {
            const Eigen::Vector2d p(pt.x, pt.y);
            const Eigen::Vector2d local = frame.basis.transpose() * (p - frame.center);
            if (std::abs(local.x()) <= half_w && std::abs(local.y()) <= half_h) {
                dst->push_back(pt);
            }
        }
        return dst;
    };

    auto computeInsideRatio = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                                 double margin_x,
                                 double margin_y) {
        if (!src || src->empty()) return 0.0;
        int inside = 0;
        const double half_w = 0.5 * expected_board_w + margin_x;
        const double half_h = 0.5 * expected_board_h + margin_y;
        for (const auto& pt : src->points) {
            const Eigen::Vector2d p(pt.x, pt.y);
            const Eigen::Vector2d local = frame.basis.transpose() * (p - frame.center);
            if (std::abs(local.x()) <= half_w && std::abs(local.y()) <= half_h) ++inside;
        }
        return static_cast<double>(inside) / static_cast<double>(src->size());
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_filtered = cropByFrame(plane_cloud, params.board_filter_margin_x, params.board_filter_margin_y);
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_filtered = cropByFrame(edge_cloud, params.edge_filter_margin_x, params.edge_filter_margin_y);
    if (!plane_filtered || plane_filtered->size() < 30 || !edge_filtered || edge_filtered->size() < 12) {
        ROS_WARN("[LidarProcessor] Board contamination filter left too few points: plane=%zu edge=%zu",
                 plane_filtered ? plane_filtered->size() : 0, edge_filtered ? edge_filtered->size() : 0);
        return false;
    }

    const double plane_keep_ratio = static_cast<double>(plane_filtered->size()) /
                                    static_cast<double>(plane_cloud->size());
    const double edge_keep_ratio = static_cast<double>(edge_filtered->size()) /
                                   static_cast<double>(edge_cloud->size());

    BoardFrame2D filtered_frame;
    if (!computeBoardFrame2D(plane_filtered,
                             expected_board_w,
                             expected_board_h,
                             filtered_frame)) {
        return false;
    }
    frame = filtered_frame;

    const double tight_inside_ratio = computeInsideRatio(plane_filtered, 0.05, 0.05);
    const double isolation_score = 0.45 * plane_keep_ratio + 0.20 * edge_keep_ratio + 0.35 * tight_inside_ratio;

    ROS_DEBUG("[LidarProcessor] Board frame(filtered): center=(%.3f, %.3f) size=%.3f x %.3f keep_ratio=(plane=%.3f edge=%.3f) tight=%.3f score=%.3f",
             frame.center.x(), frame.center.y(), frame.width_m, frame.height_m,
             plane_keep_ratio, edge_keep_ratio, tight_inside_ratio, isolation_score);

    if (isolation_score < params.contamination_min_score ||
        plane_keep_ratio < params.contamination_min_plane_keep_ratio ||
        tight_inside_ratio < params.contamination_min_tight_inside_ratio) {
        ROS_WARN("[LidarProcessor] Board contamination score too low.");
        return false;
    }

    const std::vector<Eigen::Vector2d> local_holes = {
        Eigen::Vector2d(-0.5 * delta_width_circles,  0.5 * delta_height_circles),
        Eigen::Vector2d( 0.5 * delta_width_circles,  0.5 * delta_height_circles),
        Eigen::Vector2d( 0.5 * delta_width_circles, -0.5 * delta_height_circles),
        Eigen::Vector2d(-0.5 * delta_width_circles, -0.5 * delta_height_circles)
    };

    double best_score = -std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector2d> best_predicted;

    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            Eigen::Matrix2d basis = frame.basis;
            basis.col(0) *= static_cast<double>(sx);
            basis.col(1) *= static_cast<double>(sy);

            std::vector<Eigen::Vector2d> predicted;
            double score = 0.0;

            for (const auto& local : local_holes) {
                const Eigen::Vector2d c = frame.center + basis * local;
                predicted.push_back(c);

                const int annulus_pts = countPointsInBand(edge_filtered,
                                                          c,
                                                          std::max(0.01, circle_radius - 0.03),
                                                          circle_radius + 0.03);
                const int inner_pts = countPointsInDisk(plane_filtered, c, std::max(0.01, circle_radius - 0.04));
                score += 2.5 * static_cast<double>(annulus_pts) - 2.0 * static_cast<double>(inner_pts);
            }

            if (score > best_score) {
                best_score = score;
                best_predicted = predicted;
            }
        }
    }

    if (best_predicted.size() != 4) {
        return false;
    }

    ROS_DEBUG("[LidarProcessor] Prior-based hole hypothesis score=%.3f", best_score);

    std::vector<Eigen::Vector2d> refined;
    refined.reserve(4);
    int refined_ok = 0;
    for (const auto& predicted : best_predicted) {
        Eigen::Vector2d center;
        double local_score = -1.0;
        if (!refineCenterNearPrediction(edge_filtered, predicted, circle_radius, params, center, local_score)) {
            ROS_DEBUG("[LidarProcessor] Local arc refinement failed near predicted hole center.");
            return false;
        }

        const int annulus_pts = countPointsInBand(edge_filtered,
                                                  center,
                                                  std::max(0.01, circle_radius - 0.03),
                                                  circle_radius + 0.03);
        const int inner_pts = countPointsInDisk(plane_filtered, center, std::max(0.01, circle_radius - 0.04));
        if (annulus_pts < params.hole_min_annulus_points ||
            inner_pts > params.hole_max_inner_points ||
            local_score < params.hole_min_local_score) {
            ROS_DEBUG("[LidarProcessor] Local hole support too weak: annulus=%d inner=%d score=%.3f",
                     annulus_pts, inner_pts, local_score);
            return false;
        }

        ++refined_ok;
        refined.push_back(center);
    }

    ROS_DEBUG("[LidarProcessor] Prior-based refinement: %d/4 centers fitted from local arcs.", refined_ok);

    if (!std::all_of(refined.begin(), refined.end(), [](const Eigen::Vector2d& p) {
            return std::isfinite(p.x()) && std::isfinite(p.y());
        })) {
        return false;
    }

    if (rectangleError(refined, delta_width_circles, delta_height_circles) > 0.30) {
        ROS_WARN("[LidarProcessor] Refined centers violate rectangle geometry.");
        return false;
    }

    centers_z0 = refined;
    return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// 内部：几何一致性检查（等价于 common_lib.h Square::is_valid()）
// 检查若干候选圆心中是否有4个构成 w×h 矩形
// ─────────────────────────────────────────────────────────────────────────────
static bool squareIsValid2D(const std::vector<Eigen::Vector2d>& pts,
                             double w, double h)
{
    if (pts.size() != 4) return false;

    const double GEOM_TOL = 0.08;
    const double diag     = std::sqrt(w * w + h * h);

    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    for (const auto& p : pts) c += p;
    c /= 4.0;

    for (const auto& p : pts) {
        double d = (p - c).norm();
        if (std::fabs(d - diag / 2.0) / (diag / 2.0) > GEOM_TOL * 2.0)
            return false;
    }

    std::vector<std::pair<double, int>> angles;
    for (int i = 0; i < 4; ++i)
        angles.emplace_back(std::atan2(pts[i].y() - c.y(), pts[i].x() - c.x()), i);
    std::sort(angles.begin(), angles.end());

    std::vector<Eigen::Vector2d> sorted;
    for (const auto& a : angles) sorted.push_back(pts[a.second]);

    double s01 = (sorted[1] - sorted[0]).norm();
    double s12 = (sorted[2] - sorted[1]).norm();
    double s23 = (sorted[3] - sorted[2]).norm();
    double s30 = (sorted[0] - sorted[3]).norm();

    bool ok1 = (std::fabs(s01 - w) / w < GEOM_TOL) &&
               (std::fabs(s12 - h) / h < GEOM_TOL) &&
               (std::fabs(s23 - w) / w < GEOM_TOL) &&
               (std::fabs(s30 - h) / h < GEOM_TOL);
    bool ok2 = (std::fabs(s01 - h) / h < GEOM_TOL) &&
               (std::fabs(s12 - w) / w < GEOM_TOL) &&
               (std::fabs(s23 - h) / h < GEOM_TOL) &&
               (std::fabs(s30 - w) / w < GEOM_TOL);
    if (!ok1 && !ok2) return false;

    double perim = s01 + s12 + s23 + s30;
    double ideal = 2.0 * (w + h);
    return std::fabs(perim - ideal) / ideal <= GEOM_TOL;
}

LidarProcessor::LidarProcessor() {}

void LidarProcessor::setParams(const Params& p) { params_ = p; }

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessor::filterROI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(static_cast<float>(params_.roi_x_min),
                         static_cast<float>(params_.roi_x_max));
    pass.filter(*tmp);

    pass.setInputCloud(tmp);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(static_cast<float>(params_.roi_y_min),
                         static_cast<float>(params_.roi_y_max));
    pass.filter(*out);

    pass.setInputCloud(out);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(params_.roi_z_min),
                         static_cast<float>(params_.roi_z_max));
    pass.filter(*tmp);

    ROS_DEBUG("[LidarProcessor] ROI filter: %zu -> %zu pts",
             cloud->size(), tmp->size());
    return tmp;
}

pcl::PointCloud<PointXYZIR>::Ptr LidarProcessor::filterROIMech(
    const pcl::PointCloud<PointXYZIR>::Ptr& cloud) const
{
    pcl::PointCloud<PointXYZIR>::Ptr tmp(new pcl::PointCloud<PointXYZIR>);
    pcl::PointCloud<PointXYZIR>::Ptr out(new pcl::PointCloud<PointXYZIR>);

    pcl::PassThrough<PointXYZIR> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(static_cast<float>(params_.roi_x_min),
                         static_cast<float>(params_.roi_x_max));
    pass.filter(*tmp);

    pass.setInputCloud(tmp);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(static_cast<float>(params_.roi_y_min),
                         static_cast<float>(params_.roi_y_max));
    pass.filter(*out);

    pass.setInputCloud(out);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(params_.roi_z_min),
                         static_cast<float>(params_.roi_z_max));
    pass.filter(*tmp);

    return tmp;
}

bool LidarProcessor::fitPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                               pcl::ModelCoefficients::Ptr&                coefficients,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr&       plane_cloud)
{
    if (!cloud || cloud->size() < 10) {
        ROS_WARN("[LidarProcessor] fitPlane: too few points (%zu).",
                 cloud ? cloud->size() : 0);
        return false;
    }

    const double plane_thresh = std::max(0.01, std::min(params_.ransac_threshold, 0.05));
    if (std::abs(plane_thresh - params_.ransac_threshold) > 1e-9) {
        ROS_WARN("[LidarProcessor] Clamp lidar/ransac_threshold from %.3f to %.3f.",
                 params_.ransac_threshold, plane_thresh);
    }

    const double expected_w = params_.delta_width_circles + 2.0 * params_.circle_radius;
    const double expected_h = params_.delta_height_circles + 2.0 * params_.circle_radius;

    pcl::PointCloud<pcl::PointXYZI>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZI>(*cloud));

    double best_score = -1.0;
    pcl::ModelCoefficients::Ptr best_coeffs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr best_plane;

    for (int candidate_idx = 0; candidate_idx < 6 && remaining->size() >= 30; ++candidate_idx) {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_thresh);
        seg.setMaxIterations(params_.max_iterations);
        seg.setOptimizeCoefficients(true);
        seg.setInputCloud(remaining);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coeffs);

        if (inliers->indices.empty()) {
            ROS_WARN("[LidarProcessor] fitPlane: no more plane candidates.");
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);

        pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_plane(new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*candidate_plane);

        const PlaneMetrics metrics = computePlaneMetrics(candidate_plane, coeffs);
        const double score = scoreBoardPlane(metrics, candidate_plane->size(), expected_w, expected_h);

        ROS_DEBUG("[LidarProcessor] Plane candidate %d: inliers=%zu normal=(%.3f, %.3f, %.3f) size=%.3fm x %.3fm vertical=%.3f score=%.3f",
                 candidate_idx + 1,
                 candidate_plane->size(),
                 metrics.normal.x(), metrics.normal.y(), metrics.normal.z(),
                 metrics.long_side_m, metrics.short_side_m,
                 metrics.vertical_score, score);

        if (score > best_score) {
            best_score = score;
            best_coeffs = coeffs;
            best_plane = candidate_plane;
        }

        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZI>::Ptr next_remaining(new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*next_remaining);
        remaining.swap(next_remaining);
    }

    if (best_score < 0.0 || !best_coeffs || !best_plane) {
        ROS_WARN("[LidarProcessor] fitPlane: no board-like plane found in ROI.");
        return false;
    }

    coefficients = best_coeffs;
    plane_cloud = best_plane;
    ROS_DEBUG("[LidarProcessor] Selected board plane: %zu pts, score=%.3f",
             plane_cloud->size(), best_score);
    return true;
}

bool LidarProcessor::fitPlaneMech(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                                   pcl::ModelCoefficients::Ptr&            coefficients,
                                   pcl::PointCloud<PointXYZIR>::Ptr&       plane_cloud)
{
    if (!cloud || cloud->size() < 10) return false;

    const double plane_thresh = std::max(0.01, std::min(params_.ransac_threshold, 0.05));

    pcl::SACSegmentation<PointXYZIR> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_thresh);
    seg.setMaxIterations(params_.max_iterations);
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    coefficients.reset(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) return false;

    pcl::ExtractIndices<PointXYZIR> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    plane_cloud.reset(new pcl::PointCloud<PointXYZIR>);
    extract.filter(*plane_cloud);
    return !plane_cloud->empty();
}

void LidarProcessor::alignToZPlane(
    const Eigen::Vector3d&                      normal,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
    pcl::PointCloud<pcl::PointXYZ>::Ptr&        aligned,
    Eigen::Matrix3d&                            R_align,
    double&                                     avg_z)
{
    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d n = normal.normalized();
    if (n.dot(z_axis) < 0) n = -n;

    Eigen::Vector3d axis = n.cross(z_axis);
    const double cos_angle = std::max(-1.0, std::min(1.0, n.dot(z_axis)));
    const double angle = std::acos(cos_angle);

    if (axis.norm() < 1e-9) {
        R_align = Eigen::Matrix3d::Identity();
    } else {
        Eigen::AngleAxisd rot(angle, axis.normalized());
        R_align = rot.toRotationMatrix();
    }

    aligned.reset(new pcl::PointCloud<pcl::PointXYZ>);
    aligned->reserve(src->size());
    avg_z = 0.0;
    for (const auto& pt : src->points) {
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const Eigen::Vector3d ap = R_align * p;
        avg_z += ap.z();
        aligned->push_back(pcl::PointXYZ(
            static_cast<float>(ap.x()),
            static_cast<float>(ap.y()),
            0.0f));
    }
    if (!src->empty()) avg_z /= static_cast<double>(src->size());
}

void LidarProcessor::alignEdgesToZPlane(
    const Eigen::Vector3d&                   normal,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
    pcl::PointCloud<pcl::PointXYZ>::Ptr&       aligned,
    Eigen::Matrix3d&                           R_align,
    double&                                    avg_z)
{
    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d n = normal.normalized();
    if (n.dot(z_axis) < 0) n = -n;

    Eigen::Vector3d axis = n.cross(z_axis);
    const double cos_angle = std::max(-1.0, std::min(1.0, n.dot(z_axis)));
    const double angle = std::acos(cos_angle);

    if (axis.norm() < 1e-9) {
        R_align = Eigen::Matrix3d::Identity();
    } else {
        Eigen::AngleAxisd rot(angle, axis.normalized());
        R_align = rot.toRotationMatrix();
    }

    aligned.reset(new pcl::PointCloud<pcl::PointXYZ>);
    aligned->reserve(src->size());
    avg_z = 0.0;
    for (const auto& pt : src->points) {
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const Eigen::Vector3d ap = R_align * p;
        avg_z += ap.z();
        aligned->push_back(pcl::PointXYZ(
            static_cast<float>(ap.x()),
            static_cast<float>(ap.y()),
            0.0f));
    }
    if (!src->empty()) avg_z /= static_cast<double>(src->size());
}

bool LidarProcessor::detectCircleCenters(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    std::vector<Eigen::Vector3d>&                circle_centers,
    pcl::PointCloud<pcl::PointXYZ>::Ptr          edge_cloud_debug,
    pcl::PointCloud<pcl::PointXYZ>::Ptr          plane_cloud_debug)
{
    circle_centers.clear();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered = filterROI(cloud);
    if (!filtered || filtered->size() < 20) {
        ROS_WARN("[LidarProcessor][Solid] Too few points after ROI (%zu).",
                 filtered ? filtered->size() : 0);
        return false;
    }

    {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(filtered);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
        vg.filter(*ds);
        filtered = ds;
    }
    ROS_DEBUG("[LidarProcessor][Solid] After voxel: %zu pts", filtered->size());

    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud;
    if (!fitPlane(filtered, coefficients, plane_cloud)) return false;
    if (!plane_cloud || plane_cloud->size() < 20) {
        ROS_WARN("[LidarProcessor][Solid] Board plane too small after segmentation.");
        return false;
    }

    Eigen::Vector3d normal(coefficients->values[0],
                           coefficients->values[1],
                           coefficients->values[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
    Eigen::Matrix3d R_align;
    double avg_z = 0.0;
    alignToZPlane(normal, plane_cloud, aligned_cloud, R_align, avg_z);
    ROS_DEBUG("[LidarProcessor][Solid] Aligned to Z=0, avg_z=%.4f", avg_z);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>);
    search->setInputCloud(aligned_cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(aligned_cloud);
    ne.setSearchMethod(search);
    ne.setRadiusSearch(0.05);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(aligned_cloud);
    be.setInputNormals(normals);
    be.setSearchMethod(search);
    be.setRadiusSearch(0.05);
    be.setAngleThreshold(static_cast<float>(M_PI / 4.0));
    pcl::PointCloud<pcl::Boundary> boundaries;
    be.compute(boundaries);

    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < aligned_cloud->size() && i < boundaries.size(); ++i) {
        if (boundaries.points[i].boundary_point > 0) {
            edge_cloud->push_back(aligned_cloud->points[i]);
        }
    }

    if (edge_cloud->size() < 20) {
        ROS_WARN("[LidarProcessor][Solid] BoundaryEstimation edge points only %zu, fallback to plane cloud.",
                 edge_cloud->size());
        *edge_cloud = *aligned_cloud;
    }
    ROS_DEBUG("[LidarProcessor][Solid] Edge points: %zu", edge_cloud->size());

    if (plane_cloud_debug) {
        plane_cloud_debug->clear();
        plane_cloud_debug->reserve(plane_cloud->size());
        for (const auto& pt : plane_cloud->points) {
            plane_cloud_debug->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
    }

    Eigen::Matrix3d R_inv = R_align.inverse();
    std::vector<Eigen::Vector2d> centers_z0;

    if (!detectCentersByBoardPrior(aligned_cloud,
                                   edge_cloud,
                                   params_.delta_width_circles,
                                   params_.delta_height_circles,
                                   params_.circle_radius,
                                   params_,
                                   centers_z0)) {
        ROS_DEBUG("[LidarProcessor][Solid] Prior-based hole search failed.");
        return false;
    }

    if (centers_z0.size() != 4) {
        ROS_WARN("[LidarProcessor][Solid] Expected 4 circle centers, got %zu.",
                 centers_z0.size());
        return false;
    }

    ROS_DEBUG("[LidarProcessor][Solid] Circle candidates: %zu", centers_z0.size());

    if (edge_cloud_debug) {
        edge_cloud_debug->clear();
        edge_cloud_debug->reserve(edge_cloud->size());
        for (const auto& pt : edge_cloud->points) {
            const Eigen::Vector3d aligned_pt(pt.x, pt.y, avg_z);
            const Eigen::Vector3d orig = R_inv * aligned_pt;
            edge_cloud_debug->push_back(pcl::PointXYZ(static_cast<float>(orig.x()),
                                                      static_cast<float>(orig.y()),
                                                      static_cast<float>(orig.z())));
        }
    }

    circle_centers.reserve(4);
    for (const auto& c2d : centers_z0) {
        Eigen::Vector3d aligned_pt(c2d.x(), c2d.y(), avg_z);
        Eigen::Vector3d orig = R_inv * aligned_pt;
        circle_centers.push_back(orig);
    }

    ROS_DEBUG("[LidarProcessor][Solid] Detected %zu circle centers (LiDAR frame).",
             circle_centers.size());
    return circle_centers.size() == 4;
}

bool LidarProcessor::detectCircleCentersMech(
    const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
    std::vector<Eigen::Vector3d>&            circle_centers,
    pcl::PointCloud<pcl::PointXYZ>::Ptr      edge_cloud_debug,
    pcl::PointCloud<pcl::PointXYZ>::Ptr      plane_cloud_debug)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_xyzi->reserve(cloud ? cloud->size() : 0);

    if (cloud) {
        for (const auto& pt : cloud->points) {
            pcl::PointXYZI out;
            out.x = pt.x;
            out.y = pt.y;
            out.z = pt.z;
            out.intensity = pt.intensity;
            cloud_xyzi->push_back(out);
        }
        cloud_xyzi->width = static_cast<uint32_t>(cloud_xyzi->size());
        cloud_xyzi->height = 1;
        cloud_xyzi->is_dense = cloud->is_dense;
    }

    ROS_DEBUG("[LidarProcessor][Mech] Ignore ring field for now, use geometry-only detection.");
    return detectCircleCenters(cloud_xyzi, circle_centers, edge_cloud_debug, plane_cloud_debug);
}

} // namespace image_lidar_calibration
