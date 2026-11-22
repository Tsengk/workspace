#include "bev_surround_view/fisheye_auto_calibrator.h"
#include "bev_surround_view/corners_sort.h"

// New module headers
#include "bev_surround_view/calibration_utils.h"
#include "bev_surround_view/chessboard_detector.h"
#include "bev_surround_view/pose_utils.h"
#include "bev_surround_view/joint_optimization.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include "camera_model/mei.h"
#include "ChessboradStruct.h"
#include "CornerDetAC.h"
#include "HeaderCB.h"

namespace bev_surround_view {

bool configureIpmCamera(const FisheyeAutoCameraConfig& cam,
                        const FisheyePoseConfig& pose,
                        double ground_z,
                        int bev_width,
                        int bev_height,
                        double bev_scale,
                        double origin_offset_x,
                        double origin_offset_y,
                        double center_x_m,
                        double center_y_m,
                        int roi_top,
                        int roi_bottom,
                        int roi_left,
                        int roi_right,
                        camera::Mei& camera_model)
{
    if (cam.K.empty() || cam.width <= 0 || cam.height <= 0 || cam.t.size() < 3 || cam.r_euler.size() < 3 ||
        pose.t_values.size() < 3 || pose.r_values.size() < 3 || bev_scale <= 0.0) {
        return false;
    }

    const int image_width = camera_model.getImageWidth() > 0 ? camera_model.getImageWidth() : cam.width;
    const int image_height = camera_model.getImageHeight() > 0 ? camera_model.getImageHeight() : cam.height;
    camera_model.setIntrinsics(cam.epsilon,
                               cam.K,
                               cam.D,
                               image_width,
                               image_height);
    camera_model.setGroundHeight(ground_z);
    camera_model.setExtrinsics(toEigenVector3d(pose.t_values, cam.vcs_to_ground_height),
                               toEigenEulerRadians(pose.r_values));
    camera_model.setIpmInfo(bev_width,
                            bev_height,
                            bev_scale,
                            origin_offset_x - static_cast<double>(std::lround(center_x_m * bev_scale)),
                            origin_offset_y - static_cast<double>(std::lround(center_y_m * bev_scale)),
                            roi_top,
                            roi_bottom,
                            roi_left,
                            roi_right);
    return true;
}

void resolveBevCropRoi(const std::string& role,
                       int middle_top_px,
                       int middle_bottom_px,
                       int left_crop_px,
                       int right_crop_px,
                       int& roi_top,
                       int& roi_bottom,
                       int& roi_left,
                       int& roi_right)
{
    roi_top = 0;
    roi_bottom = 0;
    roi_left = 0;
    roi_right = 0;

    if (role == "front") {
        roi_bottom = middle_top_px;
        return;
    }
    if (role == "back") {
        roi_top = middle_bottom_px;
        return;
    }
    if (role == "left") {
        roi_right = left_crop_px;
        return;
    }
    if (role == "right") {
        roi_left = right_crop_px;
    }
}

cv::Mat fuseBev(const std::map<std::string, cv::Mat>& bev_images)
{
    if (bev_images.empty()) {
        return cv::Mat();
    }

    const cv::Mat& ref = bev_images.begin()->second;
    if (ref.empty()) {
        return cv::Mat();
    }

    cv::Mat fused = cv::Mat::zeros(ref.rows, ref.cols, CV_32FC3);
    const float weight = 0.7f;

    for (std::map<std::string, cv::Mat>::const_iterator it = bev_images.begin();
         it != bev_images.end();
         ++it) {
        const cv::Mat& bev = it->second;
        if (bev.empty() || bev.size() != ref.size() || bev.type() != CV_8UC3) {
            continue;
        }
        for (int y = 0; y < bev.rows; ++y) {
            for (int x = 0; x < bev.cols; ++x) {
                const cv::Vec3b pixel = bev.at<cv::Vec3b>(y, x);
                fused.at<cv::Vec3f>(y, x) += cv::Vec3f(pixel[0], pixel[1], pixel[2]) * weight;
            }
        }
    }

    cv::Mat output(ref.rows, ref.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < output.rows; ++y) {
        for (int x = 0; x < output.cols; ++x) {
            const cv::Vec3f value = fused.at<cv::Vec3f>(y, x);
            output.at<cv::Vec3b>(y, x) = cv::Vec3b(
                static_cast<unsigned char>(clamp(value[0], 0.0, 255.0)),
                static_cast<unsigned char>(clamp(value[1], 0.0, 255.0)),
                static_cast<unsigned char>(clamp(value[2], 0.0, 255.0)));
        }
    }
    return output;
}

Eigen::Vector3d FisheyeAutoCalibrator::getCameraEulerRadians(const std::string& camera_name, const cv::Mat& R_c_w)
{
    camera::Mei& camera_model = ipm_cameras_.at(camera_name);
    camera_model.setWorldToCameraR(cvMatToEigenMatrix3d(R_c_w));
    return camera_model.getREuler();
}

FisheyePoseConfig FisheyeAutoCalibrator::resolvePoseConfig(const std::string& camera_name,
                                                                                  const cv::Mat& T_c_w,
                                                                                  const cv::Mat& T_init,
                                                                                  double vehicle_center_x_m,
                                                                                  double ground_offset_z_m,
                                                                                  double reproj_rmse_px)
{
    FisheyePoseConfig pose;
    pose.T_c_w = T_c_w.clone();
    pose.reproj_rmse_px = reproj_rmse_px;
    pose.valid = true;

    const Eigen::Vector3d euler_rad = getCameraEulerRadians(camera_name, T_c_w(cv::Rect(0, 0, 3, 3)));
    const double roll_deg = round2(deg(euler_rad.x()));
    const double pitch_deg = round2(deg(euler_rad.y()));
    const double yaw_deg = round2(deg(euler_rad.z()));

    const cv::Vec3d t_std = cameraCenterFromPose(T_c_w);
    const cv::Vec3d t_init_std = cameraCenterFromPose(T_init);

    const double x = t_std[0] - vehicle_center_x_m;
    const double y = t_std[1] - t_init_std[1];
    const double z = t_std[2] - ground_offset_z_m;

    pose.t_values.push_back(round2(y));
    pose.t_values.push_back(round2(z));
    pose.t_values.push_back(round2(-x));

    pose.r_values.push_back(roll_deg);
    pose.r_values.push_back(pitch_deg);
    pose.r_values.push_back(yaw_deg);
    return pose;
}

void drawUndistortedBoundary(cv::Mat& debug,
                             camera::Mei& camera_model,
                             const FisheyeAutoCameraConfig& cam,
                             int debug_boundary_scale)
{
    if (debug.empty() || cam.K.empty()) {
        return;
    }

    const int boundary_scale = (debug_boundary_scale > 0) ? debug_boundary_scale : 4;
    const int undistort_width = cam.width * boundary_scale;
    const int undistort_height = cam.height * boundary_scale;
    cv::Mat undistort_K = cam.K.clone();
    undistort_K.at<double>(0, 0) = cam.width / 2.0;
    undistort_K.at<double>(1, 1) = cam.width / 2.0;
    undistort_K.at<double>(0, 2) = undistort_width / 2.0;
    undistort_K.at<double>(1, 2) = undistort_height / 2.0;

    const double undist_fx = undistort_K.at<double>(0, 0);
    const double undist_fy = undistort_K.at<double>(1, 1);
    const double undist_cx = undistort_K.at<double>(0, 2);
    const double undist_cy = undistort_K.at<double>(1, 2);
    const int step = std::max(1, boundary_scale * 8);
    std::vector<cv::Point> poly;
    auto addBoundaryPt = [&](int ux, int uy) {
        const double xn = (static_cast<double>(ux) - undist_cx) / undist_fx;
        const double yn = (static_cast<double>(uy) - undist_cy) / undist_fy;
        cv::Point2d distorted_pt;
        if (camera_model.projectCamera3dTo2d(Eigen::Vector3d(xn, yn, 1.0), distorted_pt)) {
            poly.push_back(cv::Point(cvRound(distorted_pt.x), cvRound(distorted_pt.y)));
        }
    };
    for (int x = 0; x < undistort_width; x += step) addBoundaryPt(x, 0);
    for (int y = 0; y < undistort_height; y += step) addBoundaryPt(undistort_width - 1, y);
    for (int x = undistort_width - 1; x >= 0; x -= step) addBoundaryPt(x, undistort_height - 1);
    for (int y = undistort_height - 1; y >= 0; y -= step) addBoundaryPt(0, y);
    if (!poly.empty()) {
        cv::polylines(debug, poly, true, cv::Scalar(0, 255, 0), 2);
    }
}

std::vector<cv::Point2f> slicePoints(const std::vector<cv::Point2f>& points,
                                     size_t begin,
                                     size_t count)
{
    if (begin >= points.size()) {
        return std::vector<cv::Point2f>();
    }
    const size_t end = std::min(points.size(), begin + count);
    return std::vector<cv::Point2f>(points.begin() + begin, points.begin() + end);
}

std::vector<cv::Point3f> slicePoints3(const std::vector<cv::Point3f>& points,
                                      size_t begin,
                                      size_t count)
{
    if (begin >= points.size()) {
        return std::vector<cv::Point3f>();
    }
    const size_t end = std::min(points.size(), begin + count);
    return std::vector<cv::Point3f>(points.begin() + begin, points.begin() + end);
}

size_t tailBegin(size_t size, size_t count)
{
    return size > count ? size - count : 0;
}

template <typename PointT>
std::vector<PointT> rotateGridPoints90Impl(const std::vector<PointT>& points, int rows, int cols, bool clockwise)
{
    if (points.size() != static_cast<size_t>(rows * cols)) {
        return points;
    }
    std::vector<PointT> rotated(points.size());
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int src_idx = r * cols + c;
            int dst_idx;
            if (clockwise) {
                dst_idx = c * rows + (rows - 1 - r);
            } else {
                dst_idx = (cols - 1 - c) * rows + r;
            }
            rotated[dst_idx] = points[src_idx];
        }
    }
    return rotated;
}

std::vector<cv::Point2f> rotateGridPoints90(const std::vector<cv::Point2f>& points, int rows, int cols, bool clockwise)
{
    return rotateGridPoints90Impl(points, rows, cols, clockwise);
}

std::vector<cv::Point3f> rotateGridPoints90_3f(const std::vector<cv::Point3f>& points, int rows, int cols, bool clockwise)
{
    return rotateGridPoints90Impl(points, rows, cols, clockwise);
}

bool FisheyeAutoCalibrator::detect(const FisheyeAutoCameraConfig& camera,
                                   const cv::Mat& image,
                                   const Options& options,
                                   FisheyeDetectionResult& detection,
                                   std::string& error) const
{
    detection = FisheyeDetectionResult();
    error.clear();

    if (image.empty()) {
        error = camera.camera_name + ": input image is empty";
        return false;
    }

    ChessboardLayoutConfig chessboard_config;
    if (loadChessboardLayoutConfig(options.chessboard_yaml_path, chessboard_config, error) == false) {
        error = camera.camera_name + ": load chessboard yaml failed: " + error;
        return false;
    }

    FisheyeAutoCameraConfig cam = camera;
    cam.width = image.cols;
    cam.height = image.rows;
    if (cam.new_width <= 0) {
        cam.new_width = cam.width;
    }
    if (cam.new_height <= 0) {
        cam.new_height = cam.height;
    }

camera::Mei camera_model = initMeiCamera(cam);

    CameraDetection internal_detection;
    if (extractCameraDetection(cam,
                               camera_model,
                               image,
                               chessboard_config,
                               options.detector_score,
                               options.debug_boundary_scale,
                               internal_detection,
                               error) == false) {
        detection.debug_image = internal_detection.debug_image;
        return false;
    }

    detection.valid = true;
    detection.undistorted_points = internal_detection.undistorted_points;
    detection.distorted_points = internal_detection.distorted_points;
    detection.debug_image = internal_detection.debug_image;
    return true;
}

#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <iostream>

void save(const std::string& name,
          const std::vector<std::vector<cv::Point2f>>& p2ds,
          const std::vector<std::vector<cv::Point3f>>& p3ds,
          const std::string& filename = "/home/idriver/work/agent_space/test/save.txt")
{
    if (p2ds.size() != p3ds.size()) {
        std::cerr << "Error: The number of 2D and 3D point sets must be the same!" << std::endl;
        return;
    }

    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return;
    }

    file << name << " = [\n";

    for (size_t i = 0; i < p2ds.size(); ++i) {
        const auto& p2d_set = p2ds[i];
        const auto& p3d_set = p3ds[i];

        file << "    {\"p2d\": [";
        for (size_t j = 0; j < p2d_set.size(); ++j) {
            const auto& p2d = p2d_set[j];
            file << "[" << p2d.x << ", " << p2d.y << "]";
            if (j != p2d_set.size() - 1) file << ", ";
        }
        file << "], \"p3d\": [";
        for (size_t j = 0; j < p3d_set.size(); ++j) {
            const auto& p3d = p3d_set[j];
            file << "[" << p3d.x << ", " << p3d.y << ", " << p3d.z << "]";
            if (j != p3d_set.size() - 1) file << ", ";
        }
        file << "]},\n";
    }

    file << "]\n";

    file.close();
}

bool FisheyeAutoCalibrator::calibrate(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                                      const std::array<FisheyeDetectionResult, 4>& detections,
                                      const Options& options,
                                      Output& output)
{
    output = Output();

    ChessboardLayoutConfig chessboard_config;
    std::string layout_error;
    if (loadChessboardLayoutConfig(options.chessboard_yaml_path, chessboard_config, layout_error) == false) {
        output.message = "load chessboard yaml failed: " + layout_error;
        return false;
    }
    if (validateCurrentLayout(chessboard_config, layout_error) == false) {
        output.message = layout_error;
        return false;
    }

    static const size_t kFront = 0;
    static const size_t kBack = 1;
    static const size_t kLeft = 2;
    static const size_t kRight = 3;

    for (size_t i = 0; i < cameras.size(); ++i) {
        if (detections[i].valid == false || detections[i].distorted_points.empty()) {
            output.message = cameras[i].camera_name + ": detection is invalid";
            return false;
        }
        output.corner_debug_images[cameras[i].camera_name] = detections[i].debug_image;
    }

    const std::vector<cv::Point3f> front_points = generateFrontPoints(chessboard_config);
    const std::vector<cv::Point3f> back_points = generateBackPoints(chessboard_config);
    const std::vector<cv::Point3f> left_points = generateLeftPoints(chessboard_config);
    const std::vector<cv::Point3f> right_points = generateRightPoints(chessboard_config);

    const size_t top_group0_count = groupPointCount(chessboard_config.top_bottom_groups[0]);
    const size_t top_group1_count = groupPointCount(chessboard_config.top_bottom_groups[1]);
    const size_t top_group2_count = groupPointCount(chessboard_config.top_bottom_groups[2]);
    const size_t side_group0_count = groupPointCount(chessboard_config.left_right_groups[0]);
    const size_t side_group1_count = groupPointCount(chessboard_config.left_right_groups[1]);
    const size_t side_group2_count = groupPointCount(chessboard_config.left_right_groups[2]);
    const int top_group0_rows = chessboard_config.top_bottom_groups[0].corner_rows;
    const int top_group0_cols = chessboard_config.top_bottom_groups[0].corner_cols;
    const int top_group1_rows = chessboard_config.top_bottom_groups[1].corner_rows;
    const int top_group1_cols = chessboard_config.top_bottom_groups[1].corner_cols;
    const int top_group2_rows = chessboard_config.top_bottom_groups[2].corner_rows;
    const int top_group2_cols = chessboard_config.top_bottom_groups[2].corner_cols;
    const int side_group0_rows = chessboard_config.left_right_groups[0].corner_rows;
    const int side_group0_cols = chessboard_config.left_right_groups[0].corner_cols;
    const int side_group1_rows = chessboard_config.left_right_groups[1].corner_rows;
    const int side_group1_cols = chessboard_config.left_right_groups[1].corner_cols;
    const int side_group2_rows = chessboard_config.left_right_groups[2].corner_rows;
    const int side_group2_cols = chessboard_config.left_right_groups[2].corner_cols;

    const std::vector<cv::Point3f> front_middle_object_points =
        slicePoints3(front_points, top_group0_count, top_group1_count);
    const std::vector<cv::Point3f> left_middle_object_points =
        slicePoints3(left_points, side_group0_count, side_group1_count);
    const std::vector<cv::Point3f> right_middle_object_points =
        slicePoints3(right_points, side_group0_count, side_group1_count);
    const std::vector<cv::Point3f> back_middle_object_points =
        slicePoints3(back_points, top_group0_count, top_group1_count);
    const std::vector<cv::Point2f> front_middle_image_points =
        slicePoints(detections[kFront].undistorted_points, top_group0_count, top_group1_count);
    const std::vector<cv::Point2f> left_middle_image_points =
        slicePoints(detections[kLeft].undistorted_points, side_group0_count, side_group1_count);
    const std::vector<cv::Point2f> right_middle_image_points =
        slicePoints(detections[kRight].undistorted_points, side_group0_count, side_group1_count);
    const std::vector<cv::Point2f> back_middle_image_points =
        slicePoints(detections[kBack].undistorted_points, top_group0_count, top_group1_count);

    cv::Mat front_T_c_w;
    double front_rmse = -1.0;
    if (solvePose(front_middle_object_points,
                  front_middle_image_points,
                  cameras[kFront],
                  front_T_c_w,
                  front_rmse) == false) {
        output.message = "front solvePnP failed";
        return false;
    }
    printf("front_rmse %f\n",front_rmse);
    save("front_middle", {front_middle_image_points}, {front_middle_object_points});

    cv::Mat left_middle_T_c_w;
    double left_middle_rmse = -1.0;
    if (solvePose(left_middle_object_points,
                  left_middle_image_points,
                  cameras[kLeft],
                  left_middle_T_c_w,
                  left_middle_rmse) == false) {
        output.message = "left middle-board solvePnP failed";
        return false;
    }
    save("left_middle", {left_middle_image_points}, {left_middle_object_points});

    cv::Mat right_middle_T_c_w;
    double right_middle_rmse = -1.0;
    if (solvePose(right_middle_object_points,
                  right_middle_image_points,
                  cameras[kRight],
                  right_middle_T_c_w,
                  right_middle_rmse) == false) {
        output.message = "right middle-board solvePnP failed";
        return false;
    }
    save("right_middle", {right_middle_image_points}, {right_middle_object_points});

    cv::Mat back_middle_T_c_w;
    double back_middle_rmse = -1.0;
    if (solvePose(back_middle_object_points,
                  back_middle_image_points,
                  cameras[kBack],
                  back_middle_T_c_w,
                  back_middle_rmse) == false) {
        output.message = "back middle-board solvePnP failed";
        return false;
    }
    save("back_middle", {back_middle_image_points}, {back_middle_object_points});

    const std::vector<cv::Point2f> front_left_top_image_points =
        slicePoints(detections[kFront].undistorted_points, 0, top_group0_count);
    const std::vector<cv::Point2f> front_right_top_image_points =
        slicePoints(detections[kFront].undistorted_points,
                    top_group0_count + top_group1_count,
                    top_group2_count);
    const std::vector<cv::Point2f> front_left_top_raw_image_points =
        slicePoints(detections[kFront].distorted_points, 0, top_group0_count);
    const std::vector<cv::Point2f> front_right_top_raw_image_points =
        slicePoints(detections[kFront].distorted_points,
                    top_group0_count + top_group1_count,
                    top_group2_count);
    const std::vector<cv::Point2f>& left_front_shared_front_image_points =
        front_left_top_image_points;
    const std::vector<cv::Point2f>& right_front_shared_front_image_points =
        front_right_top_image_points;
    const std::vector<cv::Point3f> left_front_object_points =
        backProjectToGround(left_front_shared_front_image_points,
                            cameras[kFront].K,
                            front_T_c_w,
                            options.solve_ground_z);
    const std::vector<cv::Point2f> left_front_image_points_raw =
        slicePoints(detections[kLeft].undistorted_points,
                    tailBegin(detections[kLeft].undistorted_points.size(), side_group2_count),
                    side_group2_count);
    const std::vector<cv::Point2f> left_front_image_points =
        rotateGridPoints90(left_front_image_points_raw, side_group2_rows, side_group2_cols, false);
    const std::vector<cv::Point2f> left_front_raw_image_points_raw =
        slicePoints(detections[kLeft].distorted_points,
                    tailBegin(detections[kLeft].distorted_points.size(), side_group2_count),
                    side_group2_count);
    const std::vector<cv::Point2f> left_front_raw_image_points =
        rotateGridPoints90(left_front_raw_image_points_raw, side_group2_rows, side_group2_cols, false);
    save("left_front", {left_front_image_points}, {left_front_object_points});

    cv::Mat left_T_c_w;
    double left_rmse = -1.0;
    if (solvePose(left_front_object_points,
                  left_front_image_points,
                  cameras[kLeft],
                  left_T_c_w,
                  left_rmse) == false) {
        output.message = "left front-shared solvePnP failed";
        return false;
    }
    printf("left_rmse %f\n",left_rmse);
    JointCameraPoseState left_initial_pose_state;
    if (!extractJointCameraPoseState(left_T_c_w, left_initial_pose_state)) {
        output.message = "extract left initial xyzrpy failed";
        return false;
    }

    const std::vector<cv::Point3f> right_front_object_points =
        backProjectToGround(right_front_shared_front_image_points,
                            cameras[kFront].K,
                            front_T_c_w,
                            options.solve_ground_z);
    const std::vector<cv::Point2f> right_front_image_points_raw =
        slicePoints(detections[kRight].undistorted_points, 0, side_group0_count);
    const std::vector<cv::Point2f> right_front_image_points =
        rotateGridPoints90(right_front_image_points_raw, side_group0_rows, side_group0_cols, true);
    const std::vector<cv::Point2f> right_front_raw_image_points_raw =
        slicePoints(detections[kRight].distorted_points, 0, side_group0_count);
    const std::vector<cv::Point2f> right_front_raw_image_points =
        rotateGridPoints90(right_front_raw_image_points_raw, side_group0_rows, side_group0_cols, true);
    save("right_front", {right_front_image_points}, {right_front_object_points});

    cv::Mat right_T_c_w;
    double right_rmse = -1.0;
    if (solvePose(right_front_object_points,
                  right_front_image_points,
                  cameras[kRight],
                  right_T_c_w,
                  right_rmse) == false) {
        output.message = "right front-shared solvePnP failed";
        return false;
    }
    printf("right_rmse %f\n",right_rmse);
    JointCameraPoseState right_initial_pose_state;
    if (!extractJointCameraPoseState(right_T_c_w, right_initial_pose_state)) {
        output.message = "extract right initial xyzrpy failed";
        return false;
    }

    const std::vector<cv::Point2f> left_rear_image_points_raw =
        slicePoints(detections[kLeft].undistorted_points, 0, side_group0_count);
    const std::vector<cv::Point2f> left_rear_image_points =
        rotateGridPoints90(left_rear_image_points_raw, side_group0_rows, side_group0_cols, true);
    const std::vector<cv::Point2f> left_rear_raw_image_points_raw =
        slicePoints(detections[kLeft].distorted_points, 0, side_group0_count);
    const std::vector<cv::Point2f> left_rear_raw_image_points =
        rotateGridPoints90(left_rear_raw_image_points_raw, side_group0_rows, side_group0_cols, true);
    save("left_rear", {left_rear_image_points}, {std::vector<cv::Point3f>()});
    const std::vector<cv::Point2f> right_rear_image_points_raw =
        slicePoints(detections[kRight].undistorted_points,
                    tailBegin(detections[kRight].undistorted_points.size(), side_group2_count),
                    side_group2_count);
    const std::vector<cv::Point2f> right_rear_image_points =
        rotateGridPoints90(right_rear_image_points_raw, side_group2_rows, side_group2_cols, false);
    const std::vector<cv::Point2f> right_rear_raw_image_points_raw =
        slicePoints(detections[kRight].distorted_points,
                    tailBegin(detections[kRight].distorted_points.size(), side_group2_count),
                    side_group2_count);
    const std::vector<cv::Point2f> right_rear_raw_image_points =
        rotateGridPoints90(right_rear_raw_image_points_raw, side_group2_rows, side_group2_cols, false);
    save("right_rear", {right_rear_image_points}, {std::vector<cv::Point3f>()});
    const std::vector<cv::Point3f> back_right_object_points =
        backProjectToGround(right_rear_image_points,
                            cameras[kRight].K,
                            right_T_c_w,
                            options.solve_ground_z);
    const std::vector<cv::Point3f> back_left_object_points =
        backProjectToGround(left_rear_image_points,
                            cameras[kLeft].K,
                            left_T_c_w,
                            options.solve_ground_z);

    const std::vector<cv::Point2f>  back_left_image_points=
        slicePoints(detections[kBack].undistorted_points,
                    tailBegin(detections[kBack].undistorted_points.size(), top_group2_count),
                    top_group2_count);
    const std::vector<cv::Point2f> back_right_image_points =
        slicePoints(detections[kBack].undistorted_points, 0, top_group0_count);
    const std::vector<cv::Point2f>  back_left_raw_image_points=
        slicePoints(detections[kBack].distorted_points,
                    tailBegin(detections[kBack].distorted_points.size(), top_group2_count),
                    top_group2_count);
    const std::vector<cv::Point2f> back_right_raw_image_points =
        slicePoints(detections[kBack].distorted_points, 0, top_group0_count);
    std::vector<cv::Point3f> back_object_points = back_right_object_points;
    back_object_points.insert(back_object_points.end(),
                              back_left_object_points.begin(),
                              back_left_object_points.end());
    std::vector<cv::Point2f> back_image_points = back_right_image_points;
    back_image_points.insert(back_image_points.end(),
                             back_left_image_points.begin(),
                             back_left_image_points.end());

    if (back_object_points.size() < back_image_points.size() || back_object_points.size() > back_image_points.size()) {
        output.message = "back shared point count mismatch";
        return false;
    }

    cv::Mat back_T_c_w;
    double back_rmse = -1.0;
    if (solvePose(back_object_points,
                  back_image_points,
                  cameras[kBack],
                  back_T_c_w,
                  back_rmse) == false) {
        output.message = "back shared solvePnP failed";
        return false;
    }
    save("back_shared", {back_image_points}, {back_object_points});
    printf("back_rmse %f\n", back_rmse);
    JointCameraPoseState back_initial_pose_state;
    if (!extractJointCameraPoseState(back_T_c_w, back_initial_pose_state)) {
        output.message = "extract back initial xyzrpy failed";
        return false;
    }

    JointOptimizationProblem joint_problem;
    joint_problem.front_camera = cameras[kFront];
    joint_problem.left_camera = cameras[kLeft];
    joint_problem.right_camera = cameras[kRight];
    joint_problem.back_camera = cameras[kBack];
    joint_problem.front_T_c_w = front_T_c_w.clone();
    joint_problem.front_center_w = cameraCenterFromPose(front_T_c_w);
    joint_problem.grid_spacing_m = chessboard_config.unit_size;
    joint_problem.left_object_points = left_front_object_points;
    joint_problem.left_image_points = left_front_image_points;
    joint_problem.right_object_points = right_front_object_points;
    joint_problem.right_image_points = right_front_image_points;
    joint_problem.left_rear_image_points = left_rear_image_points;
    joint_problem.right_rear_image_points = right_rear_image_points;
    joint_problem.back_image_points = back_image_points;
    joint_problem.ground_z = options.solve_ground_z;
    joint_problem.sort_threshold = options.back_sort_threshold;
    if (initJointBoardObservation(top_group0_rows,
                                  top_group0_cols,
                                  front_left_top_raw_image_points,
                                  left_front_raw_image_points,
                                  joint_problem.front_left_board) == false ||
        initJointBoardObservation(top_group2_rows,
                                  top_group2_cols,
                                  front_right_top_raw_image_points,
                                  right_front_raw_image_points,
                                  joint_problem.front_right_board) == false ||
        initJointBoardObservation(top_group0_rows,
                                  top_group0_cols,
                                  right_rear_raw_image_points,
                                  back_right_raw_image_points,
                                  joint_problem.back_right_board) == false ||
        initJointBoardObservation(top_group2_rows,
                                  top_group2_cols,
                                  left_rear_raw_image_points,
                                  back_left_raw_image_points,
                                  joint_problem.back_left_board) == false ||
        initJointBoardObservation(top_group1_rows,
                                  top_group1_cols,
                                  front_middle_image_points,
                                  front_middle_image_points,
                                  joint_problem.front_middle_board) == false ||
        initJointBoardObservation(side_group1_rows,
                                  side_group1_cols,
                                  left_middle_image_points,
                                  left_middle_image_points,
                                  joint_problem.left_middle_board) == false ||
        initJointBoardObservation(side_group1_rows,
                                  side_group1_cols,
                                  right_middle_image_points,
                                  right_middle_image_points,
                                  joint_problem.right_middle_board) == false ||
        initJointBoardObservation(top_group1_rows,
                                  top_group1_cols,
                                  back_middle_image_points,
                                  back_middle_image_points,
                                  joint_problem.back_middle_board) == false) {
        output.message = "joint board observation init failed";
        return false;
    }

    if (!setInitialJointBoardPoseFromObjectPoints(joint_problem.front_left_board,
                                                  left_front_object_points) ||
        !setInitialJointBoardPoseFromObjectPoints(joint_problem.front_right_board,
                                                  right_front_object_points) ||
        !setInitialJointBoardPoseFromObjectPoints(joint_problem.back_right_board,
                                                  back_right_object_points) ||
        !setInitialJointBoardPoseFromObjectPoints(joint_problem.back_left_board,
                                                  back_left_object_points)) {
        output.message = "joint board initial pose init failed";
        return false;
    }

    joint_problem.initial_params = packJointPoseParams(front_T_c_w,
                                                       left_T_c_w,
                                                       right_T_c_w,
                                                       back_T_c_w,
                                                       joint_problem);
    if (joint_problem.initial_params.size() != expectedJointParamCount(joint_problem)) {
        output.message = "build initial xyzrpy params failed";
        return false;
    }

    JointOptimizationEvalResult init_joint_eval;
    double init_joint_cost = std::numeric_limits<double>::max();
    if (evaluateJointOptimization(joint_problem,
                                  joint_problem.initial_params,
                                  init_joint_eval) == false) {
        output.message = "initial xyzrpy joint evaluation failed";
        return false;
    }

    init_joint_cost = init_joint_eval.total_cost;
    front_T_c_w = init_joint_eval.front_T_c_w.clone();
    left_T_c_w = init_joint_eval.left_T_c_w.clone();
    right_T_c_w = init_joint_eval.right_T_c_w.clone();
    back_T_c_w = init_joint_eval.back_T_c_w.clone();
    front_rmse = init_joint_eval.front_rmse;
    left_rmse = init_joint_eval.left_rmse;
    right_rmse = init_joint_eval.right_rmse;
    back_rmse = init_joint_eval.back_rmse;
    if (!init_joint_eval.back_object_points.empty()) {
        back_object_points = init_joint_eval.back_object_points;
    }

    double selected_shared_reproj_rmse = std::numeric_limits<double>::max();
    if (!computeSharedBoardRawReprojectionRmse(joint_problem,
                                               init_joint_eval,
                                               selected_shared_reproj_rmse,
                                               NULL,
                                               NULL,
                                               NULL,
                                               NULL)) {
        output.message = "initial shared-board raw reprojection metric failed";
        return false;
    }

    cv::Mat optimized_front_T_c_w = front_T_c_w.clone();
    cv::Mat optimized_left_T_c_w = left_T_c_w.clone();
    cv::Mat optimized_right_T_c_w = right_T_c_w.clone();
    cv::Mat optimized_back_T_c_w = back_T_c_w.clone();
    double optimized_front_rmse = front_rmse;
    double optimized_left_rmse = left_rmse;
    double optimized_right_rmse = right_rmse;
    double optimized_back_rmse = back_rmse;
    double optimized_joint_cost = init_joint_cost;
    JointOptimizationEvalResult optimized_joint_eval = init_joint_eval;

    if (options.enable_joint_optimization) {
        if (optimizeJointLeftRightBack(joint_problem,
                                       optimized_front_T_c_w,
                                       optimized_left_T_c_w,
                                       optimized_right_T_c_w,
                                       optimized_back_T_c_w,
                                       optimized_front_rmse,
                                       optimized_left_rmse,
                                       optimized_right_rmse,
                                       optimized_back_rmse,
                                       optimized_joint_cost,
                                       &optimized_joint_eval)) {
            double optimized_shared_reproj_rmse = std::numeric_limits<double>::max();
            const bool optimized_reproj_valid =
                computeSharedBoardRawReprojectionRmse(joint_problem,
                                                      optimized_joint_eval,
                                                      optimized_shared_reproj_rmse,
                                                      NULL,
                                                      NULL,
                                                      NULL,
                                                      NULL);
            std::cout << "[FisheyeAutoCalibrator] shared-board xyzrpy ceres optimize with front fixed cost "
                      << init_joint_cost << " -> " << optimized_joint_cost
                      << ", shared-board raw reproj rmse "
                      << selected_shared_reproj_rmse << " -> "
                      << optimized_shared_reproj_rmse << std::endl;
            if (optimized_reproj_valid &&
                optimized_shared_reproj_rmse + 1e-12 < selected_shared_reproj_rmse) {
                front_T_c_w = optimized_front_T_c_w;
                left_T_c_w = optimized_left_T_c_w;
                right_T_c_w = optimized_right_T_c_w;
                back_T_c_w = optimized_back_T_c_w;
                if (!optimized_joint_eval.back_object_points.empty()) {
                    back_object_points = optimized_joint_eval.back_object_points;
                }
                front_rmse = optimized_front_rmse;
                left_rmse = optimized_left_rmse;
                right_rmse = optimized_right_rmse;
                back_rmse = optimized_back_rmse;
                selected_shared_reproj_rmse = optimized_shared_reproj_rmse;
            } else {
                std::cout << "[FisheyeAutoCalibrator] reject xyzrpy ceres result by shared-board raw reproj rmse"
                          << std::endl;
            }
        }
    } else {
        std::cout << "[FisheyeAutoCalibrator] skip joint optimization by config" << std::endl;
    }

    const double vehicle_center_x_m = computeVehicleCenterX(front_T_c_w, back_T_c_w);
    std::cout << "[FisheyeAutoCalibrator] computed vehicle center x offset = "
              << vehicle_center_x_m << std::endl;

    static double last_shared_reproj_rmse = std::numeric_limits<double>::max();
    if (last_shared_reproj_rmse > selected_shared_reproj_rmse) {
        last_shared_reproj_rmse = selected_shared_reproj_rmse;
        output.shared_reproj_rmse_px = selected_shared_reproj_rmse;

    output.poses[cameras[kFront].camera_name] = resolvePoseConfig(cameras[kFront].camera_name,
                                                                  front_T_c_w,
                                                                  front_T_c_w,
                                                                  vehicle_center_x_m,
                                                                  options.ground_offset_z_m,
                                                                  front_rmse);
    output.poses[cameras[kLeft].camera_name] = resolvePoseConfig(cameras[kLeft].camera_name,
                                                                 left_T_c_w,
                                                                 front_T_c_w,
                                                                 vehicle_center_x_m,
                                                                 options.ground_offset_z_m,
                                                                 left_rmse);
    output.poses[cameras[kRight].camera_name] = resolvePoseConfig(cameras[kRight].camera_name,
                                                                  right_T_c_w,
                                                                  front_T_c_w,
                                                                  vehicle_center_x_m,
                                                                  options.ground_offset_z_m,
                                                                  right_rmse);
    output.poses[cameras[kBack].camera_name] = resolvePoseConfig(cameras[kBack].camera_name,
                                                                 back_T_c_w,
                                                                 front_T_c_w,
                                                                 vehicle_center_x_m,
                                                                 options.ground_offset_z_m,
                                                                 back_rmse);
    }

    std::vector<std::string> summaries;
    for (size_t i = 0; i < cameras.size(); ++i) {
        const FisheyePoseConfig& pose = output.poses[cameras[i].camera_name];
        std::ostringstream oss;
        oss << cameras[i].camera_name << " ground_ray_rmse=" << std::fixed << std::setprecision(3)
            << pose.reproj_rmse_px << "m";
        summaries.push_back(oss.str());
    }

    // Write all 3D object points to a single txt file (overwrite mode)
    if (!options.debug_dir.empty()) {
        const std::string pts_path = options.debug_dir + "/calibration_3d_points.txt";
        std::ofstream pts_file(pts_path, std::ios::out | std::ios::trunc);
        if (pts_file.is_open()) {
            auto write_pts = [&](const std::string& name, const std::vector<cv::Point3f>& pts) {
                for (const auto& p : pts) {
                    pts_file << name << " " << p.x << " " << p.y << " " << p.z << "\n";
                }
            };
            write_pts(cameras[kFront].camera_name, front_middle_object_points);
            write_pts(cameras[kLeft].camera_name,  left_front_object_points);
            write_pts(cameras[kRight].camera_name, right_front_object_points);
            write_pts(cameras[kBack].camera_name,  back_object_points);
        }
    }
    output.valid = true;
    output.message = joinMessage(summaries);
    return true;
}

FisheyeAutoCalibrator::FisheyeAutoCalibrator(const std::array<FisheyeAutoCameraConfig, 4>& cameras)
{
    for (size_t i = 0; i < cameras.size(); ++i) {
        ipm_cameras_.emplace(cameras[i].camera_name, cameras[i].rotation_order);
    }
}

bool FisheyeAutoCalibrator::refreshIpmCameras(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                                              const Output& calibration_output,
                                              Options& options,
                                              std::string& error)
{
    static const size_t kFront = 0;
    static const size_t kBack = 1;
    static const size_t kLeft = 2;
    static const size_t kRight = 3;

    const std::map<std::string, FisheyePoseConfig>::const_iterator front_it =
        calibration_output.poses.find(cameras[kFront].camera_name);
    const std::map<std::string, FisheyePoseConfig>::const_iterator back_it =
        calibration_output.poses.find(cameras[kBack].camera_name);
    const std::map<std::string, FisheyePoseConfig>::const_iterator left_it =
        calibration_output.poses.find(cameras[kLeft].camera_name);
    const std::map<std::string, FisheyePoseConfig>::const_iterator right_it =
        calibration_output.poses.find(cameras[kRight].camera_name);
    if (front_it == calibration_output.poses.end() ||
        back_it == calibration_output.poses.end() ||
        left_it == calibration_output.poses.end() ||
        right_it == calibration_output.poses.end()) {
        error = "missing pose for bev render";
        return false;
    }

    const FisheyePoseConfig& front_pose = front_it->second;
    const FisheyePoseConfig& back_pose = back_it->second;
    const FisheyePoseConfig& left_pose = left_it->second;
    const FisheyePoseConfig& right_pose = right_it->second;

    double bev_center_x_m = 0.0;
    double bev_center_y_m = 0.0;
    if (front_pose.t_values.size() >= 3 && back_pose.t_values.size() >= 3) {
        bev_center_x_m = 0.5 * (-front_pose.t_values[2] - back_pose.t_values[2]);
    }
    if (left_pose.t_values.size() >= 1 && right_pose.t_values.size() >= 1) {
        bev_center_y_m = 0.5 * (left_pose.t_values[0] + right_pose.t_values[0]);
    }
    if (!std::isfinite(bev_center_x_m)) {
        bev_center_x_m = 0.0;
    }
    if (!std::isfinite(bev_center_y_m)) {
        bev_center_y_m = 0.0;
    }

    const double origin_y_px = 0.5 * static_cast<double>(options.bev_height) - options.origin_offset_x;
    const double origin_x_px = 0.5 * static_cast<double>(options.bev_width) - options.origin_offset_y;
    const int center_y_px = static_cast<int>(std::lround(clamp(origin_y_px, 0.0, static_cast<double>(options.bev_height - 1))));
    const int center_x_px = static_cast<int>(std::lround(clamp(origin_x_px, 0.0, static_cast<double>(options.bev_width - 1))));

    if (!options.enable_roi_crop) {
        for (size_t i = 0; i < cameras.size(); ++i) {
            camera::Mei& camera_model = ipm_cameras_[cameras[i].camera_name];
            if (!configureIpmCamera(cameras[i],
                                    calibration_output.poses.at(cameras[i].camera_name),
                                    options.solve_ground_z,
                                    options.bev_width,
                                    options.bev_height,
                                    options.bev_scale,
                                    options.origin_offset_x,
                                    options.origin_offset_y,
                                    bev_center_x_m,
                                    bev_center_y_m,
                                    0,
                                    0,
                                    0,
                                    0,
                                    camera_model)) {
                error = cameras[i].camera_name + ": configure ipm camera failed";
                return false;
            }
        }
        error.clear();
        return true;
    }
    const bool needs_auto_crop = options.bev_range_front_auto ||
                                 options.bev_range_rear_auto ||
                                 options.bev_range_left_auto ||
                                 options.bev_range_right_auto;

    std::array<cv::Mat, 4> auto_map_x_images;
    std::array<cv::Mat, 4> auto_map_y_images;
    if (needs_auto_crop) {
        for (size_t i = 0; i < cameras.size(); ++i) {
            camera::Mei& camera_model = ipm_cameras_[cameras[i].camera_name];
            if (!configureIpmCamera(cameras[i],
                                    calibration_output.poses.at(cameras[i].camera_name),
                                    options.solve_ground_z,
                                    options.bev_width,
                                    options.bev_height,
                                    options.bev_scale,
                                    options.origin_offset_x,
                                    options.origin_offset_y,
                                    bev_center_x_m,
                                    bev_center_y_m,
                                    0,
                                    0,
                                    0,
                                    0,
                                    camera_model)) {
                error = cameras[i].camera_name + ": configure ipm camera failed";
                return false;
            }
            if (!camera_model.generateIpmMap(&auto_map_x_images[i], &auto_map_y_images[i])) {
                error = cameras[i].camera_name + ": generate ipm map failed";
                return false;
            }
        }
    }

    const int front_crop_px = options.bev_range_front_auto
        ? scanFirstValidUp(auto_map_x_images[kFront], auto_map_y_images[kFront], center_x_px, center_y_px)
        : static_cast<int>(std::lround(clamp(options.bev_range_front, 0.0, static_cast<double>(options.bev_height))));
    const int rear_crop_px = options.bev_range_rear_auto
        ? scanFirstValidDown(auto_map_x_images[kBack], auto_map_y_images[kBack], center_x_px, center_y_px)
        : static_cast<int>(std::lround(clamp(options.bev_range_rear, 0.0, static_cast<double>(options.bev_height))));
    const int left_crop_px = options.bev_range_left_auto
        ? scanFirstValidLeft(auto_map_x_images[kLeft], auto_map_y_images[kLeft], center_x_px, center_y_px)
        : static_cast<int>(std::lround(clamp(options.bev_range_left, 0.0, static_cast<double>(options.bev_width))));
    const int right_crop_px = options.bev_range_right_auto
        ? scanFirstValidRight(auto_map_x_images[kRight], auto_map_y_images[kRight], center_x_px, center_y_px)
        : static_cast<int>(std::lround(clamp(options.bev_range_right, 0.0, static_cast<double>(options.bev_width))));

    options.bev_range_front = static_cast<double>(front_crop_px);
    options.bev_range_rear = static_cast<double>(rear_crop_px);
    options.bev_range_left = static_cast<double>(left_crop_px);
    options.bev_range_right = static_cast<double>(right_crop_px);

    const int middle_top_px = std::min(front_crop_px, rear_crop_px);
    const int middle_bottom_px = std::max(front_crop_px, rear_crop_px);

    for (size_t i = 0; i < cameras.size(); ++i) {
        int roi_top = 0;
        int roi_bottom = 0;
        int roi_left = 0;
        int roi_right = 0;
        resolveBevCropRoi(cameras[i].role,
                          middle_top_px,
                          middle_bottom_px,
                          left_crop_px,
                          right_crop_px,
                          roi_top,
                          roi_bottom,
                          roi_left,
                          roi_right);

        camera::Mei& camera_model = ipm_cameras_[cameras[i].camera_name];
        if (!configureIpmCamera(cameras[i],
                                calibration_output.poses.at(cameras[i].camera_name),
                                options.solve_ground_z,
                                options.bev_width,
                                options.bev_height,
                                options.bev_scale,
                                options.origin_offset_x,
                                options.origin_offset_y,
                                bev_center_x_m,
                                bev_center_y_m,
                                roi_top,
                                roi_bottom,
                                roi_left,
                                roi_right,
                                camera_model)) {
            error = cameras[i].camera_name + ": configure ipm camera failed";
            return false;
        }
    }

    error.clear();
    return true;
}

bool FisheyeAutoCalibrator::renderBevImages(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                                            const Output& calibration_output,
                                            Options& options,
                                            const std::array<cv::Mat, 4>& images,
                                            bool refresh_ipm_info,
                                            FisheyeBevRenderOutput& render_output,
                                            std::string& error)
{
    render_output = FisheyeBevRenderOutput();
    error.clear();

    if (refresh_ipm_info || ipm_cameras_.size() != cameras.size()) {
        if (!refreshIpmCameras(cameras, calibration_output, options, error)) {
            return false;
        }
    }

    for (size_t i = 0; i < cameras.size(); ++i) {
        if (images[i].empty()) {
            error = cameras[i].camera_name + ": input image is empty";
            return false;
        }

        std::map<std::string, camera::Mei>::iterator camera_it = ipm_cameras_.find(cameras[i].camera_name);
        if (camera_it == ipm_cameras_.end()) {
            error = cameras[i].camera_name + ": ipm camera missing";
            return false;
        }

        camera_it->second.setImageWidth(images[i].cols);
        camera_it->second.setImageHeight(images[i].rows);

        cv::Mat bev;
        if (!camera_it->second.generateIpmImage(images[i], bev)) {
            error = cameras[i].camera_name + ": generate ipm image failed";
            return false;
        }

        render_output.bev_images[cameras[i].camera_name] = bev;
    }

    render_output.fused_bev_image = fuseBev(render_output.bev_images);
    render_output.valid = !render_output.fused_bev_image.empty();
    return render_output.valid;
}

bool FisheyeAutoCalibrator::savePoseYaml(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                                         const Output& output,
                                         const Options& options,
                                         std::string& error) const
{
    const double bev_resolution = options.bev_scale > 0.0 ? 1.0 / options.bev_scale : 0.0;

    auto outputPathForRole = [&options](const std::string& role) -> std::string {
        if (role == "front") {
            return options.output_front_yaml_path;
        }
        if (role == "back") {
            return options.output_rear_yaml_path;
        }
        if (role == "left") {
            return options.output_left_yaml_path;
        }
        if (role == "right") {
            return options.output_right_yaml_path;
        }
        return std::string();
    };
    for (size_t i = 0; i < cameras.size(); ++i) {
        const FisheyeAutoCameraConfig& camera = cameras[i];
        const std::string camera_output_path = outputPathForRole(camera.role);
        if (camera_output_path.empty()) {
            error = "output yaml path is empty for role: " + camera.role;
            return false;
        }

        YAML::Node root(YAML::NodeType::Map);
        YAML::Node node(YAML::NodeType::Map);

        node["role"] = camera.role;

        YAML::Node size(YAML::NodeType::Sequence);
        size.push_back(camera.width);
        size.push_back(camera.height);
        node["Size"] = size;

        YAML::Node k(YAML::NodeType::Sequence);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                k.push_back(camera.K.at<double>(r, c));
            }
        }
        node["K"] = k;

        YAML::Node d(YAML::NodeType::Sequence);
        for (int j = 0; j < 4; ++j) {
            d.push_back(distortionCoeff(camera.D, j));
        }
        node["D"] = d;
        node["Epsilon"] = camera.epsilon;

        std::vector<double> t_values = camera.t;
        std::vector<double> r_values = camera.r_euler;
        std::map<std::string, FisheyePoseConfig>::const_iterator pose_it = output.poses.find(camera.camera_name);
        if (pose_it != output.poses.end()) {
            t_values = pose_it->second.t_values;
            r_values = pose_it->second.r_values;
        }

        YAML::Node t(YAML::NodeType::Sequence);
        for (size_t j = 0; j < t_values.size(); ++j) {
            t.push_back(t_values[j]);
        }
        node["t"] = t;

        YAML::Node r_euler(YAML::NodeType::Sequence);
        for (size_t j = 0; j < r_values.size(); ++j) {
            r_euler.push_back(r_values[j]);
        }
        node["R_euler"] = r_euler;

        node["vcs_z_offset"] = options.ground_offset_z_m;
        node["bev_width"] = options.bev_width;
        node["bev_height"] = options.bev_height;
        node["bev_resolution"] = bev_resolution;
        node["bev_range_front"] = options.bev_range_front;
        node["bev_range_rear"] = options.bev_range_rear;
        node["bev_range_left"] = options.bev_range_left;
        node["bev_range_right"] = options.bev_range_right;
        node["bev_origin_offset_x"] = options.origin_offset_x;
        node["bev_origin_offset_y"] = options.origin_offset_y;

        root[camera.camera_name] = node;

        if (ensureParentDirectory(camera_output_path) == false) {
            error = "failed to create output yaml parent directory";
            return false;
        }

        YAML::Emitter emitter;
        emitter.SetIndent(2);
        emitter << root;
        if (emitter.good() == false) {
            error = emitter.GetLastError();
            return false;
        }

        std::ofstream ofs(camera_output_path.c_str(), std::ios::out | std::ios::trunc);
        if (ofs.is_open() == false) {
            error = "failed to open output yaml: " + camera_output_path;
            return false;
        }
        ofs << emitter.c_str();
        if (ofs.good() == false) {
            error = "failed to write output yaml: " + camera_output_path;
            return false;
        }
    }

    error.clear();
    return true;
}

bool FisheyeAutoCalibrator::getIpmMaps(const std::string& camera_name, cv::Mat& map_x, cv::Mat& map_y)
{
    auto it = ipm_cameras_.find(camera_name);
    if (it == ipm_cameras_.end()) {
        return false;
    }
    return it->second.generateIpmMap(&map_x, &map_y);
}

bool FisheyeAutoCalibrator::getIpmRoi(const std::string& camera_name, IpmRoiInfo& roi_info)
{
    auto it = ipm_cameras_.find(camera_name);
    if (it == ipm_cameras_.end()) {
        return false;
    }
    const camera::IpmInfo ipm_info = it->second.getIpmInfo();
    const int roi_left = static_cast<int>(ipm_info.getRoiLeft());
    const int roi_top = static_cast<int>(ipm_info.getRoiTop());
    const int roi_right = ipm_info.getRoiRight() > 0 ? static_cast<int>(ipm_info.getRoiRight()) : ipm_info.getWidth();
    const int roi_bottom = ipm_info.getRoiBottom() > 0 ? static_cast<int>(ipm_info.getRoiBottom()) : ipm_info.getHeight();
    roi_info.x = roi_left;
    roi_info.y = roi_top;
    roi_info.width = roi_right - roi_left;
    roi_info.height = roi_bottom - roi_top;
    return true;
}

}  // namespace bev_surround_view
