#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <array>
#include <limits>
#include <numeric>

namespace calib{

template <typename PointT>
std::vector<size_t> forceReorderCornerIndices(const std::vector<PointT>& points, int cols, int rows)
{
    const size_t n = points.size();
    std::vector<size_t> identity(n);
    std::iota(identity.begin(), identity.end(), 0);
    if (cols <= 0 || rows <= 0 || n != static_cast<size_t>(cols * rows)) {
        return identity;
    }

    cv::Mat point_mat(static_cast<int>(n), 2, CV_64F);
    cv::Point2d mean(0.0, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const double x = static_cast<double>(points[i].x);
        const double y = static_cast<double>(points[i].y);
        point_mat.at<double>(static_cast<int>(i), 0) = x;
        point_mat.at<double>(static_cast<int>(i), 1) = y;
        mean.x += x;
        mean.y += y;
    }
    mean.x /= static_cast<double>(n);
    mean.y /= static_cast<double>(n);

    cv::Mat centered = point_mat.clone();
    for (int i = 0; i < centered.rows; ++i) {
        centered.at<double>(i, 0) -= mean.x;
        centered.at<double>(i, 1) -= mean.y;
    }

    cv::SVD svd(centered, cv::SVD::FULL_UV);
    if (svd.vt.rows < 2 || svd.vt.cols < 2) {
        return identity;
    }

    cv::Mat planar = centered * svd.vt.rowRange(0, 2).t();
    double min_u = std::numeric_limits<double>::max();
    double min_v = std::numeric_limits<double>::max();
    double max_u = -std::numeric_limits<double>::max();
    double max_v = -std::numeric_limits<double>::max();
    for (int i = 0; i < planar.rows; ++i) {
        const double u = planar.at<double>(i, 0);
        const double v = planar.at<double>(i, 1);
        min_u = std::min(min_u, u);
        min_v = std::min(min_v, v);
        max_u = std::max(max_u, u);
        max_v = std::max(max_v, v);
    }

    const std::array<cv::Point2d, 4> target_corners = {{
        cv::Point2d(min_u, min_v),
        cv::Point2d(max_u, min_v),
        cv::Point2d(min_u, max_v),
        cv::Point2d(max_u, max_v),
    }};

    std::array<int, 4> chosen_indices = {-1, -1, -1, -1};
    std::vector<bool> remaining(n, true);
    for (size_t target_idx = 0; target_idx < target_corners.size(); ++target_idx) {
        double best_distance = std::numeric_limits<double>::max();
        int best_index = -1;
        for (size_t point_idx = 0; point_idx < n; ++point_idx) {
            if (!remaining[point_idx]) {
                continue;
            }
            const double du = planar.at<double>(static_cast<int>(point_idx), 0) - target_corners[target_idx].x;
            const double dv = planar.at<double>(static_cast<int>(point_idx), 1) - target_corners[target_idx].y;
            const double distance = du * du + dv * dv;
            if (distance < best_distance) {
                best_distance = distance;
                best_index = static_cast<int>(point_idx);
            }
        }
        if (best_index < 0) {
            return identity;
        }
        chosen_indices[target_idx] = best_index;
        remaining[static_cast<size_t>(best_index)] = false;
    }

    std::vector<cv::Point2f> outer_corners(4);
    for (size_t i = 0; i < 4; ++i) {
        outer_corners[i] = cv::Point2f(
            static_cast<float>(point_mat.at<double>(chosen_indices[i], 0)),
            static_cast<float>(point_mat.at<double>(chosen_indices[i], 1)));
    }
    std::vector<cv::Point2f> canonical_corners = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(cols - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(rows - 1)),
        cv::Point2f(static_cast<float>(cols - 1), static_cast<float>(rows - 1)),
    };
    const cv::Mat homography = cv::getPerspectiveTransform(outer_corners, canonical_corners);
    if (homography.empty()) {
        return identity;
    }

    std::vector<cv::Point2f> src_points(static_cast<size_t>(n));
    for (size_t i = 0; i < n; ++i) {
        src_points[i] = cv::Point2f(
            static_cast<float>(point_mat.at<double>(static_cast<int>(i), 0)),
            static_cast<float>(point_mat.at<double>(static_cast<int>(i), 1)));
    }
    std::vector<cv::Point2f> warped;
    cv::perspectiveTransform(src_points, warped, homography);
    if (warped.size() != n) {
        return identity;
    }

    std::vector<size_t> sorted_by_y(n);
    std::iota(sorted_by_y.begin(), sorted_by_y.end(), 0);
    std::stable_sort(sorted_by_y.begin(), sorted_by_y.end(),
                     [&](size_t lhs, size_t rhs) {
                         return warped[lhs].y < warped[rhs].y;
                     });

    std::vector<size_t> ordered_indices;
    ordered_indices.reserve(n);
    for (int row = 0; row < rows; ++row) {
        auto row_begin = sorted_by_y.begin() + static_cast<std::ptrdiff_t>(row * cols);
        auto row_end = row_begin + cols;
        std::stable_sort(row_begin, row_end,
                         [&](size_t lhs, size_t rhs) {
                             return warped[lhs].x < warped[rhs].x;
                         });
        ordered_indices.insert(ordered_indices.end(), row_begin, row_end);
    }

    return ordered_indices;
}
}
