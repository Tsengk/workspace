#pragma once

#include <string>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include "bev_surround_view/fisheye_auto_calibrator.h"

namespace bev_surround_view {

struct ChessboardGroupConfig {
    int corner_rows = 0;
    int corner_cols = 0;
};

struct ChessboardLayoutConfig {
    double unit_size = 0.4;
    std::vector<ChessboardGroupConfig> top_bottom_groups;
    std::vector<ChessboardGroupConfig> left_right_groups;
};

struct DetectedChessboard {
    std::vector<cv::Point2f> corners;
};

struct CameraDetection {
    std::vector<cv::Point2f> undistorted_points;
    std::vector<cv::Point2f> distorted_points;
    cv::Mat debug_image;
};

// Layout
size_t groupPointCount(const ChessboardGroupConfig& group);
const std::vector<ChessboardGroupConfig>& groupsForRole(const ChessboardLayoutConfig& config, const std::string& role);
bool loadChessboardGroups(const YAML::Node& board_node, std::vector<ChessboardGroupConfig>& groups, std::string& error);
bool loadChessboardLayoutConfig(const std::string& path, ChessboardLayoutConfig& config, std::string& error);
bool validateCurrentLayout(const ChessboardLayoutConfig& config, std::string& error);
std::vector<std::pair<int, int>> buildDetectPatterns(const std::vector<ChessboardGroupConfig>& groups);

// Point generation
std::vector<cv::Point3f> generateFrontPoints(const ChessboardLayoutConfig& config);
std::vector<cv::Point3f> generateBackPoints(const ChessboardLayoutConfig& config);
std::vector<cv::Point3f> generateLeftPoints(const ChessboardLayoutConfig& config);
std::vector<cv::Point3f> generateRightPoints(const ChessboardLayoutConfig& config);

// Detection
std::vector<DetectedChessboard> detectChessboards(const cv::Mat& gray, double score);
double boardMinY(const DetectedChessboard& board);
double boardMinX(const DetectedChessboard& board);
std::vector<DetectedChessboard> selectTopBoards(std::vector<DetectedChessboard> boards, size_t count);

// Topology
double polygonAreaFromOrderedPoints(const std::vector<cv::Point2f>& points);
std::vector<cv::Point2f> orderPolygonPoints(const std::vector<cv::Point2f>& points);
std::vector<size_t> estimateOuterContourIndices(const std::vector<cv::Point2f>& corners);
bool selectTopLeftAndReferenceNeighborIndices(const std::vector<cv::Point2f>& corners,
                                              size_t& top_left_index,
                                              size_t& reference_index);
std::vector<size_t> recoverMiddleBoardTopologyIndices(const std::vector<cv::Point2f>& corners, int cols, int rows);

// Visualization
cv::Mat drawCorners(const cv::Mat& image, const std::vector<cv::Point2f>& points);
bool bilinearSample(const cv::Mat& image, const cv::Point2d& uv, cv::Vec3b& value);

// Detection pipeline
bool extractCameraDetection(const FisheyeAutoCameraConfig& cam,
                            camera::Mei& camera_model,
                            const cv::Mat& image,
                            const ChessboardLayoutConfig& chessboard_config,
                            double detector_score,
                            int debug_boundary_scale,
                            CameraDetection& detection,
                            std::string& error);

}  // namespace bev_surround_view
