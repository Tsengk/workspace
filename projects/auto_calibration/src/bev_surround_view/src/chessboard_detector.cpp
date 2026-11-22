#include "bev_surround_view/chessboard_detector.h"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <sstream>
#include <unistd.h>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "bev_surround_view/calibration_utils.h"
#include "bev_surround_view/corners_sort.h"
#include "camera_model/mei.h"
#include "CornerDetAC.h"
#include "ChessboradStruct.h"

namespace bev_surround_view {

// Forward declarations for functions defined in other translation units
void drawUndistortedBoundary(cv::Mat& debug,
                             camera::Mei& camera_model,
                             const FisheyeAutoCameraConfig& cam,
                             int debug_boundary_scale);

bool solveMiddleBoardPoseForPlane(const std::vector<cv::Point2f>& ordered_image_points,
                                  int rows,
                                  int cols,
                                  const FisheyeAutoCameraConfig& cam,
                                  cv::Mat& homography);

std::vector<cv::Point3f> imagePointsToPlane(const std::vector<cv::Point2f>& image_points,
                                            const cv::Mat& homography);

std::vector<size_t> recoverPlaneGridTopologyIndices(const std::vector<cv::Point3f>& plane_points,
                                                    int rows,
                                                    int cols);

// ---------------------------------------------------------------------------
// Layout helpers
// ---------------------------------------------------------------------------

size_t groupPointCount(const ChessboardGroupConfig& group)
{
    return static_cast<size_t>(group.corner_rows * group.corner_cols);
}

bool loadChessboardGroups(const YAML::Node& board_node,
                          std::vector<ChessboardGroupConfig>& groups,
                          std::string& error)
{
    if (board_node.IsNull() ||
        board_node["chessboard_groups"].IsNull() ||
        board_node["chessboard_groups"].IsSequence() == false) {
        error = "chessboard_groups is missing or invalid";
        return false;
    }

    const YAML::Node list = board_node["chessboard_groups"];
    groups.clear();
    for (size_t i = 0; i < list.size(); ++i) {
        ChessboardGroupConfig group;
        if (list[i]["corner_count"].IsNull() == false &&
            list[i]["corner_count"].IsSequence() &&
            list[i]["corner_count"].size() >= 2) {
            group.corner_rows = list[i]["corner_count"][0].as<int>();
            group.corner_cols = list[i]["corner_count"][1].as<int>();
        } else if (list[i]["rows"].IsNull() == false && list[i]["cols"].IsNull() == false) {
            group.corner_rows = list[i]["rows"].as<int>() - 1;
            group.corner_cols = list[i]["cols"].as<int>() - 1;
        } else {
            std::ostringstream oss;
            oss << "group " << i << " missing corner_count/rows/cols";
            error = oss.str();
            return false;
        }

        if (group.corner_rows <= 0 || group.corner_cols <= 0) {
            std::ostringstream oss;
            oss << "group " << i << " has invalid corner size";
            error = oss.str();
            return false;
        }
        groups.push_back(group);
    }

    if (groups.empty()) {
        error = "chessboard_groups is empty";
        return false;
    }
    return true;
}

ChessboardLayoutConfig legacyChessboardLayoutConfig()
{
    ChessboardLayoutConfig config;
    config.unit_size = 0.4;
    config.top_bottom_groups.push_back(ChessboardGroupConfig{4, 4});
    config.top_bottom_groups.push_back(ChessboardGroupConfig{6, 4});
    config.top_bottom_groups.push_back(ChessboardGroupConfig{4, 4});
    config.left_right_groups.push_back(ChessboardGroupConfig{4, 4});
    config.left_right_groups.push_back(ChessboardGroupConfig{4, 6});
    config.left_right_groups.push_back(ChessboardGroupConfig{4, 4});
    return config;
}

bool loadChessboardLayoutConfig(const std::string& path,
                                ChessboardLayoutConfig& config,
                                std::string& error)
{
    if (path.empty() || access(path.c_str(), F_OK) != 0) {
        config = legacyChessboardLayoutConfig();
        error.clear();
        return true;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        error = e.what();
        return false;
    }

    config = ChessboardLayoutConfig();
    try {
        if (root["unit_size"].IsNull() == false) {
            config.unit_size = root["unit_size"].as<double>();
        }
        if (config.unit_size <= 0.0) {
            error = "unit_size must be positive";
            return false;
        }
        if (loadChessboardGroups(root["top_bottom_board"], config.top_bottom_groups, error) == false) {
            error = std::string("top_bottom_board: ") + error;
            return false;
        }
        if (loadChessboardGroups(root["left_right_board"], config.left_right_groups, error) == false) {
            error = std::string("left_right_board: ") + error;
            return false;
        }
    } catch (const YAML::Exception& e) {
        error = e.what();
        return false;
    }

    return true;
}

const std::vector<ChessboardGroupConfig>& groupsForRole(const ChessboardLayoutConfig& config,
                                                        const std::string& role)
{
    if (role == "front" || role == "back") {
        return config.top_bottom_groups;
    }
    return config.left_right_groups;
}

bool validateCurrentLayout(const ChessboardLayoutConfig& config, std::string& error)
{
    if (config.top_bottom_groups.size() != 3) {
        error = "top_bottom_board requires exactly 3 chessboard_groups";
        return false;
    }
    if (config.left_right_groups.size() != 3) {
        error = "left_right_board requires exactly 3 chessboard_groups";
        return false;
    }
    return true;
}

std::vector<std::pair<int, int> > buildDetectPatterns(const std::vector<ChessboardGroupConfig>& groups)
{
    std::vector<std::pair<int, int> > patterns;
    patterns.reserve(groups.size());
    for (size_t i = 0; i < groups.size(); ++i) {
        patterns.push_back(std::make_pair(groups[i].corner_cols, groups[i].corner_rows));
    }
    return patterns;
}

// ---------------------------------------------------------------------------
// 3D point generation
// ---------------------------------------------------------------------------

std::vector<cv::Point3f> generateFrontPoints(const ChessboardLayoutConfig& config)
{
    const float square_size = static_cast<float>(config.unit_size);
    const float start_y[] = {0.0f, -7.75f * square_size, -17.5f * square_size};
    std::vector<cv::Point3f> points;

    for (size_t g = 0; g < config.top_bottom_groups.size() && g < 3; ++g) {
        for (int c = 0; c < config.top_bottom_groups[g].corner_rows; ++c) {
            for (int r = 0; r < config.top_bottom_groups[g].corner_cols; ++r) {
                points.emplace_back(-c * square_size, start_y[g] - r * square_size, 0.0f);
            }
        }
    }
    return points;
}

std::vector<cv::Point3f> generateBackPoints(const ChessboardLayoutConfig& config)
{
    const float square_size = static_cast<float>(config.unit_size);
    const float start_y[] = {0.0f, 7.75f * square_size, 17.5f * square_size};
    std::vector<cv::Point3f> points;

    for (size_t g = 0; g < config.top_bottom_groups.size() && g < 3; ++g) {
        for (int c = 0; c < config.top_bottom_groups[g].corner_rows; ++c) {
            for (int r = 0; r < config.top_bottom_groups[g].corner_cols; ++r) {
                points.emplace_back(c * square_size, start_y[g] + r * square_size, 0.0f);
            }
        }
    }
    return points;
}

std::vector<cv::Point3f> generateLeftPoints(const ChessboardLayoutConfig& config)
{
    const float square_size = static_cast<float>(config.unit_size);
    const float start_x[] = {5.0f * square_size, 0.0f, -3.0f * square_size};
    std::vector<cv::Point3f> points;

    for (size_t g = 0; g < config.left_right_groups.size() && g < 3; ++g) {
        for (int r = 0; r < config.left_right_groups[g].corner_rows; ++r) {
            for (int c = 0; c < config.left_right_groups[g].corner_cols; ++c) {
                points.emplace_back(start_x[g] + c * square_size, -r * square_size, 0.0f);
            }
        }
    }
    return points;
}

std::vector<cv::Point3f> generateRightPoints(const ChessboardLayoutConfig& config)
{
    const float square_size = static_cast<float>(config.unit_size);
    const float start_y[] = {-20.5f * square_size, 0.0f, 5.0f * square_size};
    std::vector<cv::Point3f> points;

    for (size_t g = 0; g < config.left_right_groups.size() && g < 3; ++g) {
        for (int r = 0; r < config.left_right_groups[g].corner_rows; ++r) {
            for (int c = 0; c < config.left_right_groups[g].corner_cols; ++c) {
                points.emplace_back(-c * square_size, start_y[g] + r * square_size, 0.0f);
            }
        }
    }
    return points;
}

// ---------------------------------------------------------------------------
// Detection
// ---------------------------------------------------------------------------

std::vector<DetectedChessboard> detectChessboards(const cv::Mat& gray, double score)
{
    std::vector<cv::Point> corners_p;
    Corners corners_s;
    std::vector<cv::Mat> chessboards;

    cv::Mat gray_copy = gray.clone();
    CornerDetAC corner_detector(gray_copy);
    corner_detector.detectCorners(gray_copy, corners_p, corners_s, 0.01);

    ChessboradStruct chessboard_struct;
    chessboard_struct.chessboardsFromCorners(corners_s, chessboards, static_cast<float>(score));

    std::vector<DetectedChessboard> results;
    results.reserve(chessboards.size());
    for (size_t i = 0; i < chessboards.size(); ++i) {
        DetectedChessboard board;
        const cv::Mat& cb = chessboards[i];
        board.corners.reserve(static_cast<size_t>(cb.rows * cb.cols));
        for (int r = 0; r < cb.rows; ++r) {
            for (int c = 0; c < cb.cols; ++c) {
                const int idx = cb.at<int>(r, c);
                if (idx >= 0 && idx < static_cast<int>(corners_s.p.size())) {
                    board.corners.emplace_back(static_cast<float>(corners_s.p[idx].x),
                                               static_cast<float>(corners_s.p[idx].y));
                }
            }
        }
        results.push_back(board);
    }
    return results;
}

double boardMinY(const DetectedChessboard& board)
{
    double min_y = std::numeric_limits<double>::max();
    for (size_t i = 0; i < board.corners.size(); ++i) {
        min_y = std::min(min_y, static_cast<double>(board.corners[i].y));
    }
    return min_y;
}

double boardMinX(const DetectedChessboard& board)
{
    double min_x = std::numeric_limits<double>::max();
    for (size_t i = 0; i < board.corners.size(); ++i) {
        min_x = std::min(min_x, static_cast<double>(board.corners[i].x));
    }
    return min_x;
}

std::vector<DetectedChessboard> selectTopBoards(std::vector<DetectedChessboard> boards, size_t count)
{
    std::sort(boards.begin(), boards.end(),
              [](const DetectedChessboard& lhs, const DetectedChessboard& rhs) {
                  return boardMinY(lhs) < boardMinY(rhs);
              });
    if (boards.size() > count) {
        boards.resize(count);
    }
    std::sort(boards.begin(), boards.end(),
              [](const DetectedChessboard& lhs, const DetectedChessboard& rhs) {
                  return boardMinX(lhs) < boardMinX(rhs);
              });
    return boards;
}

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

double polygonAreaFromOrderedPoints(const std::vector<cv::Point2f>& points)
{
    if (points.size() < 3) {
        return 0.0;
    }

    double twice_area = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const cv::Point2f& a = points[i];
        const cv::Point2f& b = points[(i + 1) % points.size()];
        twice_area += static_cast<double>(a.x) * static_cast<double>(b.y) -
                      static_cast<double>(a.y) * static_cast<double>(b.x);
    }
    return 0.5 * std::abs(twice_area);
}

std::vector<cv::Point2f> orderPolygonPoints(const std::vector<cv::Point2f>& points)
{
    std::vector<cv::Point2f> ordered = points;
    if (ordered.size() < 3) {
        return ordered;
    }

    cv::Point2d center(0.0, 0.0);
    for (size_t i = 0; i < ordered.size(); ++i) {
        center.x += static_cast<double>(ordered[i].x);
        center.y += static_cast<double>(ordered[i].y);
    }
    center.x /= static_cast<double>(ordered.size());
    center.y /= static_cast<double>(ordered.size());

    std::stable_sort(ordered.begin(), ordered.end(),
                     [&](const cv::Point2f& lhs, const cv::Point2f& rhs) {
                         const double lhs_angle = std::atan2(static_cast<double>(lhs.y) - center.y,
                                                             static_cast<double>(lhs.x) - center.x);
                         const double rhs_angle = std::atan2(static_cast<double>(rhs.y) - center.y,
                                                             static_cast<double>(rhs.x) - center.x);
                         return lhs_angle < rhs_angle;
                     });
    return ordered;
}

std::vector<size_t> estimateOuterContourIndices(const std::vector<cv::Point2f>& corners)
{
    std::vector<size_t> best_indices;
    if (corners.size() < 4) {
        return best_indices;
    }

    double best_area = -1.0;
    for (size_t i = 0; i + 3 < corners.size(); ++i) {
        for (size_t j = i + 1; j + 2 < corners.size(); ++j) {
            for (size_t k = j + 1; k + 1 < corners.size(); ++k) {
                for (size_t m = k + 1; m < corners.size(); ++m) {
                    std::vector<cv::Point2f> candidate_points;
                    candidate_points.push_back(corners[i]);
                    candidate_points.push_back(corners[j]);
                    candidate_points.push_back(corners[k]);
                    candidate_points.push_back(corners[m]);
                    const double area = polygonAreaFromOrderedPoints(orderPolygonPoints(candidate_points));
                    if (area > best_area) {
                        best_area = area;
                        best_indices = {i, j, k, m};
                    }
                }
            }
        }
    }
    return best_indices;
}

bool selectTopLeftAndReferenceNeighborIndices(const std::vector<cv::Point2f>& corners,
                                              size_t& top_left_index,
                                              size_t& reference_index)
{
    const std::vector<size_t> outer_indices = estimateOuterContourIndices(corners);
    if (outer_indices.size() != 4) {
        return false;
    }

    std::vector<cv::Point2f> outer_points;
    outer_points.reserve(4);
    for (size_t i = 0; i < outer_indices.size(); ++i) {
        outer_points.push_back(corners[outer_indices[i]]);
    }

    std::vector<size_t> y_indices = {0, 1, 2, 3};
    std::stable_sort(y_indices.begin(), y_indices.end(),
                     [&](size_t lhs, size_t rhs) {
                         return outer_points[lhs].y < outer_points[rhs].y;
                     });
    std::vector<size_t> x_indices = {0, 1, 2, 3};
    std::stable_sort(x_indices.begin(), x_indices.end(),
                     [&](size_t lhs, size_t rhs) {
                         return outer_points[lhs].x < outer_points[rhs].x;
                     });

    int top_left_local_index = -1;
    for (size_t yi = 0; yi < 2; ++yi) {
        for (size_t xi = 0; xi < 2; ++xi) {
            if (y_indices[yi] == x_indices[xi]) {
                top_left_local_index = static_cast<int>(y_indices[yi]);
                break;
            }
        }
        if (top_left_local_index >= 0) {
            break;
        }
    }

    if (top_left_local_index < 0) {
        double best_score = std::numeric_limits<double>::max();
        for (size_t i = 0; i < outer_points.size(); ++i) {
            const double score = static_cast<double>(outer_points[i].x) +
                                 static_cast<double>(outer_points[i].y);
            if (score < best_score) {
                best_score = score;
                top_left_local_index = static_cast<int>(i);
            }
        }
    }
    if (top_left_local_index < 0) {
        return false;
    }

    top_left_index = outer_indices[static_cast<size_t>(top_left_local_index)];
    const cv::Point2f top_left = corners[top_left_index];

    std::vector<size_t> neighbor_indices;
    neighbor_indices.reserve(corners.size());
    for (size_t i = 0; i < corners.size(); ++i) {
        const double dx = static_cast<double>(corners[i].x - top_left.x);
        const double dy = static_cast<double>(corners[i].y - top_left.y);
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq > 1.0e-12) {
            neighbor_indices.push_back(i);
        }
    }
    std::stable_sort(neighbor_indices.begin(), neighbor_indices.end(),
                     [&](size_t lhs, size_t rhs) {
                         const double dlx = static_cast<double>(corners[lhs].x - top_left.x);
                         const double dly = static_cast<double>(corners[lhs].y - top_left.y);
                         const double drx = static_cast<double>(corners[rhs].x - top_left.x);
                         const double dry = static_cast<double>(corners[rhs].y - top_left.y);
                         return dlx * dlx + dly * dly < drx * drx + dry * dry;
                     });
    if (neighbor_indices.empty()) {
        return false;
    }

    reference_index = neighbor_indices[0];
    double best_score = std::numeric_limits<double>::max();
    const size_t candidate_count = std::min<size_t>(3, neighbor_indices.size());
    for (size_t i = 0; i < candidate_count; ++i) {
        const size_t idx = neighbor_indices[i];
        const cv::Point2f vector = corners[idx] - top_left;
        const double norm = std::sqrt(static_cast<double>(vector.x) * static_cast<double>(vector.x) +
                                      static_cast<double>(vector.y) * static_cast<double>(vector.y));
        if (norm <= 1.0e-12) {
            continue;
        }
        const double dir_x = static_cast<double>(vector.x) / norm;
        const double score = 1.0 - dir_x;
        if (score < best_score) {
            best_score = score;
            reference_index = idx;
        }
    }
    return true;
}

std::vector<size_t> recoverMiddleBoardTopologyIndices(const std::vector<cv::Point2f>& corners,
                                                      int cols,
                                                      int rows)
{
    std::vector<size_t> identity(corners.size());
    for (size_t i = 0; i < corners.size(); ++i) {
        identity[i] = i;
    }
    if (cols <= 0 || rows <= 0 || corners.size() != static_cast<size_t>(cols * rows)) {
        return identity;
    }

    size_t top_left_index = 0;
    size_t reference_index = 0;
    if (!selectTopLeftAndReferenceNeighborIndices(corners, top_left_index, reference_index)) {
        return identity;
    }

    const cv::Point2f top_left = corners[top_left_index];
    const cv::Point2f horizontal_axis = corners[reference_index] - top_left;
    const double rotation_angle = -std::atan2(static_cast<double>(horizontal_axis.y),
                                              static_cast<double>(horizontal_axis.x));
    const double c = std::cos(rotation_angle);
    const double s = std::sin(rotation_angle);

    struct RotatedPoint {
        size_t index;
        double x;
        double y;
    };
    std::vector<RotatedPoint> rotated_points;
    rotated_points.reserve(corners.size());
    for (size_t i = 0; i < corners.size(); ++i) {
        const cv::Point2f rel = corners[i] - top_left;
        rotated_points.push_back(RotatedPoint{
            i,
            static_cast<double>(rel.x) * c - static_cast<double>(rel.y) * s,
            static_cast<double>(rel.x) * s + static_cast<double>(rel.y) * c});
    }

    std::vector<size_t> indices = identity;
    std::stable_sort(indices.begin(), indices.end(),
                     [&](size_t lhs, size_t rhs) {
                         return rotated_points[lhs].y < rotated_points[rhs].y;
                     });

    std::vector<size_t> ordered_indices;
    ordered_indices.reserve(static_cast<size_t>(cols * rows));
    for (int row = 0; row < rows; ++row) {
        const int start = row * cols;
        const int end = std::min(start + cols, static_cast<int>(indices.size()));
        std::vector<size_t> line(indices.begin() + start, indices.begin() + end);
        std::stable_sort(line.begin(), line.end(),
                         [&](size_t lhs, size_t rhs) {
                             return rotated_points[lhs].x < rotated_points[rhs].x;
                         });
        ordered_indices.insert(ordered_indices.end(), line.begin(), line.end());
    }
    return ordered_indices;
}

// ---------------------------------------------------------------------------
// Visualization
// ---------------------------------------------------------------------------

cv::Mat drawCorners(const cv::Mat& image, const std::vector<cv::Point2f>& points)
{
    cv::Mat vis = image.clone();
    const int n_pts = static_cast<int>(points.size());
    if (n_pts <= 0) {
        return vis;
    }

    for (int i = 0; i < n_pts; ++i) {
        const cv::Point2f& pt = points[i];
        const cv::Scalar color(0.0,
                               255.0 * static_cast<double>(i) / static_cast<double>(n_pts),
                               255.0 * static_cast<double>(n_pts - i) / static_cast<double>(n_pts));
        cv::circle(vis, pt, 6, color, -1);
        cv::putText(vis,
                    std::to_string(i),
                    pt + cv::Point2f(5.0f, -5.0f),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.4,
                    cv::Scalar(255, 255, 255),
                    1);
        if (i > 0) {
            cv::line(vis, points[i - 1], pt, color, 3);
        }
    }
    return vis;
}

bool bilinearSample(const cv::Mat& image, const cv::Point2d& uv, cv::Vec3b& value)
{
    if (image.empty() || image.channels() != 3) {
        return false;
    }
    if (uv.x < 0.0 || uv.y < 0.0 ||
        uv.x >= static_cast<double>(image.cols - 1) ||
        uv.y >= static_cast<double>(image.rows - 1)) {
        return false;
    }

    const int x0 = static_cast<int>(uv.x);
    const int y0 = static_cast<int>(uv.y);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const double dx = uv.x - static_cast<double>(x0);
    const double dy = uv.y - static_cast<double>(y0);

    const cv::Vec3b c00 = image.at<cv::Vec3b>(y0, x0);
    const cv::Vec3b c01 = image.at<cv::Vec3b>(y0, x1);
    const cv::Vec3b c10 = image.at<cv::Vec3b>(y1, x0);
    const cv::Vec3b c11 = image.at<cv::Vec3b>(y1, x1);

    cv::Vec3d sample(0.0, 0.0, 0.0);
    for (int ch = 0; ch < 3; ++ch) {
        sample[ch] =
            static_cast<double>(c00[ch]) * (1.0 - dx) * (1.0 - dy) +
            static_cast<double>(c01[ch]) * dx * (1.0 - dy) +
            static_cast<double>(c10[ch]) * (1.0 - dx) * dy +
            static_cast<double>(c11[ch]) * dx * dy;
    }

    value = cv::Vec3b(static_cast<unsigned char>(clamp(sample[0], 0.0, 255.0)),
                      static_cast<unsigned char>(clamp(sample[1], 0.0, 255.0)),
                      static_cast<unsigned char>(clamp(sample[2], 0.0, 255.0)));
    return true;
}

// ---------------------------------------------------------------------------
// Detection pipeline
// ---------------------------------------------------------------------------

bool extractCameraDetection(const FisheyeAutoCameraConfig& cam,
                            camera::Mei& camera_model,
                            const cv::Mat& image,
                            const ChessboardLayoutConfig& chessboard_config,
                            double detector_score,
                            int debug_boundary_scale,
                            CameraDetection& detection,
                            std::string& error)
{
    const std::vector<ChessboardGroupConfig>& groups = groupsForRole(chessboard_config, cam.role);
    if (groups.empty()) {
        error = cam.camera_name + ": chessboard group config is empty";
        return false;
    }
    const std::vector<std::pair<int, int> > patterns = buildDetectPatterns(groups);

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    detection.debug_image = image.clone();
    drawUndistortedBoundary(detection.debug_image, camera_model, cam, debug_boundary_scale);

    const std::vector<DetectedChessboard> boards = detectChessboards(gray, detector_score);
    if (boards.empty()) {
        error = cam.camera_name + ": no chessboard detected";
        return false;
    }

    std::vector<DetectedChessboard> selected = selectTopBoards(boards, patterns.size());
    if (selected.size() != patterns.size()) {
        std::ostringstream oss;
        oss << cam.camera_name << ": expected " << patterns.size()
            << " chessboards, got " << selected.size();
        error = oss.str();
        return false;
    }
    printf("cam %s \n" ,cam.role.c_str());

    std::vector<std::vector<cv::Point2f> > undistorted_boards(selected.size());
    for (size_t i = 0; i < selected.size(); ++i) {
        const int cols = patterns[i].first;
        const int rows = patterns[i].second;
        if (selected[i].corners.size() != static_cast<size_t>(cols * rows)) {
            std::ostringstream oss;
            oss << cam.camera_name << ": board " << i << " point count mismatch, expected "
                << cols * rows << ", got " << selected[i].corners.size();
            error = oss.str();
            return false;
        }

        std::vector<cv::Point2f> undistorted_corners;
        undistorted_corners.reserve(selected[i].corners.size());
        for (size_t j = 0; j < selected[i].corners.size(); ++j) {
            cv::Point2d undistorted;
            if (!camera_model.undistort(selected[i].corners[j], undistorted, cam.K)) {
                std::ostringstream oss;
                oss << cam.camera_name << ": undistort failed at point " << j;
                error = oss.str();
                return false;
            }
            undistorted_corners.push_back(cv::Point2f(static_cast<float>(undistorted.x),
                                                      static_cast<float>(undistorted.y)));
        }
        undistorted_boards[i] = undistorted_corners;
    }

    std::vector<std::vector<size_t> > board_ordered_indices(selected.size());
    const bool use_three_board_recovery =
        selected.size() == 3 &&
        patterns.size() == 3;
    if (use_three_board_recovery) {
        const size_t middle_index = 1;
        const int middle_cols = patterns[middle_index].first;
        const int middle_rows = patterns[middle_index].second;
        board_ordered_indices[middle_index] =
            recoverMiddleBoardTopologyIndices(undistorted_boards[middle_index], middle_cols, middle_rows);

        std::vector<cv::Point2f> ordered_middle_undistorted;
        ordered_middle_undistorted.reserve(board_ordered_indices[middle_index].size());
        for (size_t j = 0; j < board_ordered_indices[middle_index].size(); ++j) {
            ordered_middle_undistorted.push_back(
                undistorted_boards[middle_index][board_ordered_indices[middle_index][j]]);
        }

        cv::Mat middle_homography;
        if (!solveMiddleBoardPoseForPlane(ordered_middle_undistorted,
                                          middle_rows,
                                          middle_cols,
                                          cam,
                                          middle_homography)) {
            error = cam.camera_name + ": middle board pose recovery failed";
            return false;
        }

        for (size_t board_index = 0; board_index < selected.size(); ++board_index) {
            if (board_index == middle_index) {
                continue;
            }
            const int cols = patterns[board_index].first;
            const int rows = patterns[board_index].second;
            const std::vector<cv::Point3f> plane_points =
                imagePointsToPlane(undistorted_boards[board_index], middle_homography);
            if (plane_points.size() != undistorted_boards[board_index].size()) {
                error = cam.camera_name + ": plane projection failed";
                return false;
            }
            board_ordered_indices[board_index] =
                recoverPlaneGridTopologyIndices(plane_points, rows, cols);
        }
    } else {
        for (size_t i = 0; i < selected.size(); ++i) {
            const int cols = patterns[i].first;
            const int rows = patterns[i].second;
            board_ordered_indices[i] = calib::forceReorderCornerIndices(undistorted_boards[i], cols, rows);
        }
    }

    std::vector<cv::Point2f> ordered_distorted;
    std::vector<cv::Point2f> ordered_undistorted;
    for (size_t i = 0; i < selected.size(); ++i) {
        for (size_t j = 0; j < board_ordered_indices[i].size(); ++j) {
            ordered_distorted.push_back(selected[i].corners[board_ordered_indices[i][j]]);
            ordered_undistorted.push_back(undistorted_boards[i][board_ordered_indices[i][j]]);
        }
    }

    detection.distorted_points = ordered_distorted;
    detection.undistorted_points = ordered_undistorted;

    printf("%s: undistorted corners for sorting (%zu pts):\n", cam.camera_name.c_str(), ordered_distorted.size());
    for (size_t i = 0; i < ordered_distorted.size(); ++i) {
        printf("  [%zu] dist=(%.1f,%.1f) -> undist=(%.1f,%.1f)\n",
               i, ordered_distorted[i].x, ordered_distorted[i].y,
               ordered_undistorted[i].x, ordered_undistorted[i].y);
    }

    // Draw corners on distorted image
    cv::Mat debug = drawCorners(image, ordered_distorted);
    drawUndistortedBoundary(debug, camera_model, cam, debug_boundary_scale);

    detection.debug_image = debug;

    return true;
}

}  // namespace bev_surround_view
