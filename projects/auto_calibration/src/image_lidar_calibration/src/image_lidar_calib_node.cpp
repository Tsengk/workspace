#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>

#include "image_lidar_calibration/types.h"
#include "image_lidar_calibration/fastlivo2_detector.h"
#include "image_lidar_calibration/lidar_processor.h"
#include "image_lidar_calibration/calibration_solver.h"

#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <thread>
#include <mutex>
#include <deque>
#include <algorithm>
#include <limits>
#include <sys/stat.h>

namespace image_lidar_calibration {

static void ensureDir(const std::string& dir) {
    struct stat st;
    if (stat(dir.c_str(), &st) != 0) {
        mkdir(dir.c_str(), 0755);
    }
}

static pcl::PointCloud<pcl::PointXYZI>::Ptr convertCloudToGeometryCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->points.reserve(static_cast<size_t>(msg->width) * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        pcl::PointXYZI pt;
        pt.x = *iter_x;
        pt.y = *iter_y;
        pt.z = *iter_z;
        pt.intensity = 0.0f;
        cloud->points.push_back(pt);
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

static cv::Scalar distanceToRainbowBgr(double distance_m)
{
    int i = static_cast<int>(distance_m * 5.0);
    if (i < 0) i = 0;
    if (i > 255) i = 255;
    const int idx = i * 5;

    if (idx < 256) return cv::Scalar(0, idx, 255);
    if (idx < 512) return cv::Scalar(0, 255, 511 - idx);
    if (idx < 768) return cv::Scalar(idx - 512, 255, 0);
    if (idx < 1024) return cv::Scalar(255, 1023 - idx, 0);
    return cv::Scalar(255, 0, idx - 1024);
}

static bool projectRawPointToPixel(const CameraIntrinsics& ci,
                                   const Eigen::Vector3d& p_cam,
                                   cv::Point2f& uv)
{
    const double x = p_cam.x();
    const double y = p_cam.y();
    const double z = p_cam.z();
    if (z <= 0.0) {
        return false;
    }

    double xu = 0.0;
    double yu = 0.0;
    if (ci.camera_model == MEI) {
        const double norm = std::sqrt(x * x + y * y + z * z);
        const double denom = z + ci.xi * norm;
        if (std::abs(denom) < 1e-6) {
            return false;
        }
        xu = x / denom;
        yu = y / denom;
    } else {
        xu = x / z;
        yu = y / z;
    }

    const double r2 = xu * xu + yu * yu;
    const double r4 = r2 * r2;
    const double radial = 1.0 + ci.k1 * r2 + ci.k2 * r4;
    const double xd = xu * radial + 2.0 * ci.p1 * xu * yu + ci.p2 * (r2 + 2.0 * xu * xu);
    const double yd = yu * radial + ci.p1 * (r2 + 2.0 * yu * yu) + 2.0 * ci.p2 * xu * yu;

    uv.x = static_cast<float>(ci.fx * xd + ci.cx);
    uv.y = static_cast<float>(ci.fy * yd + ci.cy);
    return true;
}

static Eigen::Matrix3d getDInitRotation();

static cv::Mat mergeCloudToRawImage(const CameraIntrinsics& ci,
                                    const cv::Mat& raw_image,
                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzi,
                                    const Eigen::Matrix3d& R_d,
                                    const Eigen::Vector3d& t_d,
                                    double min_depth_m,
                                    double max_depth_m)
{
    if (raw_image.empty() || cloud_xyzi.get() == NULL) {
        return cv::Mat();
    }

    cv::Size display_size = raw_image.size();
    if (raw_image.cols != 960) {
        const double scale = 960.0 / static_cast<double>(raw_image.cols);
        display_size = cv::Size(960, cvRound(raw_image.rows * scale));
    }

    cv::Mat base_image;
    if (raw_image.size() == display_size) {
        base_image = raw_image;
    } else {
        cv::resize(raw_image, base_image, display_size, 0, 0, cv::INTER_AREA);
    }

    cv::Mat overlay(display_size, base_image.type(), cv::Scalar::all(0));
    const double scale_x = ci.width > 0 ? static_cast<double>(display_size.width) / static_cast<double>(ci.width) : 1.0;
    const double scale_y = ci.height > 0 ? static_cast<double>(display_size.height) / static_cast<double>(ci.height) : 1.0;
    const Eigen::Matrix3d R0 = getDInitRotation();

    struct ProjectedPoint {
        cv::Point2f uv;
        float depth;
        cv::Scalar color;
        int pixel_x;
        int pixel_y;
        int pixel_index;
    };

    std::vector<ProjectedPoint> projected_points;
    projected_points.reserve(cloud_xyzi->points.size());
    std::vector<float> zbuffer(static_cast<size_t>(display_size.width) * static_cast<size_t>(display_size.height), 1e9f);
    const float kOcclusionDepthMarginMin = 0.2f;
    const float kOcclusionDepthMarginRatio = 0.03f;

    for (size_t i = 0; i < cloud_xyzi->points.size(); ++i) {
        const pcl::PointXYZI& p = cloud_xyzi->points[i];
        Eigen::Vector3d p0(p.x, p.y, p.z);
        p0 = R0 * p0;
        if (((p0.x() * p0.x()) + (p0.z() * p0.z())) > 22500.0) {
            continue;
        }

        const Eigen::Vector3d point_camera = R_d * (p0 + t_d);
        const float depth = static_cast<float>(point_camera.z());
        if (depth <= static_cast<float>(min_depth_m) || depth >= static_cast<float>(max_depth_m)) {
            continue;
        }

        cv::Point2f uv_raw;
        if (projectRawPointToPixel(ci, point_camera, uv_raw) == false) {
            continue;
        }

        const cv::Point2f uv_display(static_cast<float>(uv_raw.x * scale_x),
                                     static_cast<float>(uv_raw.y * scale_y));
        const int pixel_x = cvRound(uv_display.x);
        const int pixel_y = cvRound(uv_display.y);
        if (pixel_x < 0 || pixel_x >= overlay.cols || pixel_y < 0 || pixel_y >= overlay.rows) {
            continue;
        }

        const int pixel_index = pixel_y * overlay.cols + pixel_x;
        if (depth < zbuffer[static_cast<size_t>(pixel_index)]) {
            zbuffer[static_cast<size_t>(pixel_index)] = depth;
        }

        ProjectedPoint projected_point;
        projected_point.uv = uv_display;
        projected_point.depth = depth;
        projected_point.color = distanceToRainbowBgr(depth);
        projected_point.pixel_x = pixel_x;
        projected_point.pixel_y = pixel_y;
        projected_point.pixel_index = pixel_index;
        projected_points.push_back(projected_point);
    }

    for (size_t i = 0; i < projected_points.size(); ++i) {
        const ProjectedPoint& projected_point = projected_points[i];
        float neighborhood_min_depth = zbuffer[static_cast<size_t>(projected_point.pixel_index)];
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int neighbor_x = projected_point.pixel_x + dx;
                const int neighbor_y = projected_point.pixel_y + dy;
                if (neighbor_x < 0 || neighbor_x >= overlay.cols || neighbor_y < 0 || neighbor_y >= overlay.rows) {
                    continue;
                }
                const int neighbor_index = neighbor_y * overlay.cols + neighbor_x;
                const float neighbor_depth = zbuffer[static_cast<size_t>(neighbor_index)];
                if (neighbor_depth < neighborhood_min_depth) {
                    neighborhood_min_depth = neighbor_depth;
                }
            }
        }

        const float depth_margin = std::max(kOcclusionDepthMarginMin,
                                            kOcclusionDepthMarginRatio * neighborhood_min_depth);
        if (projected_point.depth <= neighborhood_min_depth + depth_margin) {
            cv::circle(overlay, projected_point.uv, 1, projected_point.color, 1, cv::LINE_AA);
        }
    }

    cv::Mat result;
    cv::addWeighted(base_image, 0.8, overlay, 0.3, 0.0, result, -1);
    return result;
}

static cv::Size getSceneDisplaySize(const CameraIntrinsics& ci,
                                    const cv::Mat& raw_image)
{
    if (raw_image.empty()) {
        return cv::Size(std::max(1, ci.width), std::max(1, ci.height));
    }

    cv::Size display_size = raw_image.size();
    if (raw_image.cols != 960) {
        const double scale = 960.0 / static_cast<double>(raw_image.cols);
        display_size = cv::Size(960, cvRound(raw_image.rows * scale));
    }
    return display_size;
}

static cv::Mat resizeSceneImage(const cv::Mat& image,
                                const cv::Size& display_size)
{
    if (image.empty()) {
        return cv::Mat(display_size, CV_8UC3, cv::Scalar(24, 24, 24));
    }
    if (image.size() == display_size) {
        return image.clone();
    }

    cv::Mat resized;
    cv::resize(image, resized, display_size, 0, 0, cv::INTER_AREA);
    return resized;
}

static cv::Mat makeTitledScenePanel(const cv::Mat& image,
                                    const std::string& title,
                                    const cv::Size& panel_size)
{
    cv::Mat panel = resizeSceneImage(image, panel_size);
    cv::rectangle(panel, cv::Rect(0, 0, panel.cols, 28), cv::Scalar(16, 16, 16), cv::FILLED);
    cv::putText(panel,
                title,
                cv::Point(12, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(255, 255, 255),
                1,
                cv::LINE_AA);
    return panel;
}
static cv::Mat buildSceneOverviewImage(const cv::Mat& raw_image,
                                       const cv::Mat& qr_detect_image,
                                       const cv::Mat& lidar_detect_image,
                                       const cv::Mat& fusion_image)
{
    const cv::Size panel_size = raw_image.empty() ? cv::Size(960, 540) : getSceneDisplaySize(CameraIntrinsics(), raw_image);

    const cv::Mat raw_panel = makeTitledScenePanel(raw_image, "raw_image", panel_size);
    const cv::Mat qr_panel = makeTitledScenePanel(qr_detect_image, "image_code_detect", panel_size);
    const cv::Mat lidar_panel = makeTitledScenePanel(lidar_detect_image, "lidar_detect_overlay", panel_size);
    const cv::Mat fusion_panel = makeTitledScenePanel(fusion_image, "fusion_overlay", panel_size);

    cv::Mat top_row;
    cv::Mat bottom_row;
    cv::hconcat(raw_panel, qr_panel, top_row);
    cv::hconcat(lidar_panel, fusion_panel, bottom_row);

    cv::Mat overview;
    cv::vconcat(top_row, bottom_row, overview);
    return overview;
}
static void extractEulerZYXDeg(const Eigen::Matrix3d& R,
                               double& roll_deg,
                               double& pitch_deg,
                               double& yaw_deg)
{
    double pitch = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    double roll = std::atan2(R(2, 1), R(2, 2));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    const double r2d = 180.0 / M_PI;
    roll_deg = roll * r2d;
    pitch_deg = pitch * r2d;
    yaw_deg = yaw * r2d;
}
static Eigen::Matrix3d getDInitRotation()
{
    Eigen::Matrix3d R0;
    // d/camera_calibration.h: init_rot.Init(0,0,0, roll=90deg, pitch=-90deg, yaw=0)
    // 对原始 FLU 点云先做一次固定旋转，得到前视相机默认光学轴对齐坐标：[-y, -z, x]
    R0 << 0.0, -1.0,  0.0,
          0.0,  0.0, -1.0,
          1.0,  0.0,  0.0;
    return R0;
}

static void computeDCompatiblePose(const CalibResult& r,
                                   Eigen::Matrix3d& R_d,
                                   Eigen::Vector3d& t_d,
                                   Eigen::Vector3d& Pcv,
                                   double& roll_deg,
                                   double& pitch_deg,
                                   double& yaw_deg)
{
    const Eigen::Matrix3d Rcv = r.R.transpose();
    const Eigen::Vector3d Pcv_current = -Rcv * r.t;
    const Eigen::Matrix3d R0 = getDInitRotation();

    // d 的完整链路：
    //   p0    = R0 * p_raw
    //   p_cam = R_d * (p0 + t_d) = R_d * R0 * p_raw + R_d * t_d
    // 需要与当前包投影链路一致：
    //   p_cam = Rcv * p_raw + Pcv_current
    // 因此：R_d = Rcv * R0^T,  t_d = R_d^T * Pcv_current
    R_d = Rcv * R0.transpose();
    t_d = R_d.transpose() * Pcv_current;
    Pcv = R_d * t_d;
    extractEulerZYXDeg(R_d, roll_deg, pitch_deg, yaw_deg);
}

enum LidarHorizontalAxis { LIDAR_HORIZONTAL_Y = 0, LIDAR_HORIZONTAL_X = 1 };

static double lidarHorizontalCoord(const Eigen::Vector3d& p,
                                   LidarHorizontalAxis axis)
{
    return axis == LIDAR_HORIZONTAL_X ? p.x() : p.y();
}

static LidarHorizontalAxis chooseLidarHorizontalAxis(const std::vector<Eigen::Vector3d>& lidar_centers)
{
    if (lidar_centers.empty()) {
        return LIDAR_HORIZONTAL_Y;
    }

    double min_x = lidar_centers[0].x();
    double max_x = lidar_centers[0].x();
    double min_y = lidar_centers[0].y();
    double max_y = lidar_centers[0].y();
    for (size_t i = 1; i < lidar_centers.size(); ++i) {
        min_x = std::min(min_x, lidar_centers[i].x());
        max_x = std::max(max_x, lidar_centers[i].x());
        min_y = std::min(min_y, lidar_centers[i].y());
        max_y = std::max(max_y, lidar_centers[i].y());
    }

    const double span_x = max_x - min_x;
    const double span_y = max_y - min_y;
    return span_x > span_y ? LIDAR_HORIZONTAL_X : LIDAR_HORIZONTAL_Y;
}

static bool estimateLidarBoardPose(const std::vector<Eigen::Vector3d>& lidar_centers,
                                    BoardPose& pose)
{
    pose = BoardPose();
    if (lidar_centers.size() != 4) return false;

    const LidarHorizontalAxis horizontal_axis = chooseLidarHorizontalAxis(lidar_centers);

    std::vector<Eigen::Vector3d> sorted = lidar_centers;
    std::sort(sorted.begin(), sorted.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a.z() > b.z();
    });

    std::vector<Eigen::Vector3d> top(sorted.begin(), sorted.begin() + 2);
    std::vector<Eigen::Vector3d> bottom(sorted.begin() + 2, sorted.end());

    std::sort(top.begin(), top.end(), [horizontal_axis](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return lidarHorizontalCoord(a, horizontal_axis) > lidarHorizontalCoord(b, horizontal_axis);
    });
    std::sort(bottom.begin(), bottom.end(), [horizontal_axis](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return lidarHorizontalCoord(a, horizontal_axis) > lidarHorizontalCoord(b, horizontal_axis);
    });

    const Eigen::Vector3d lt = top[0];
    const Eigen::Vector3d rt = top[1];
    const Eigen::Vector3d rb = bottom[1];
    const Eigen::Vector3d lb = bottom[0];

    const Eigen::Vector3d center = 0.25 * (lt + rt + rb + lb);
    Eigen::Vector3d board_x = 0.5 * ((rt - lt) + (rb - lb));
    Eigen::Vector3d board_y = -0.5 * ((lb - lt) + (rb - rt));
    if (board_x.norm() < 1e-6 || board_y.norm() < 1e-6) return false;

    board_x.normalize();
    board_y.normalize();
    Eigen::Vector3d board_z = board_x.cross(board_y);
    if (board_z.norm() < 1e-6) return false;
    board_z.normalize();

    if (board_z.x() < 0.0) {
        board_x = -board_x;
        board_z = -board_z;
    }
    board_y = board_z.cross(board_x);
    if (board_y.norm() < 1e-6) return false;
    board_y.normalize();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.col(0) = board_x;
    R.col(1) = board_y;
    R.col(2) = board_z;

    pose.x = center.x();
    pose.y = center.y();
    pose.z = center.z();
    extractEulerZYXDeg(R, pose.roll_deg, pose.pitch_deg, pose.yaw_deg);
    return true;
}

static void fillSceneStatistics(int accepted_scene_count,
                                 int history_filtered_scenes,
                                 CalibResult& result)
{
    result.accepted_scenes = std::max(0, accepted_scene_count);
    result.history_filtered_scenes = std::max(0, history_filtered_scenes);
    result.total_keyframes = result.accepted_scenes + result.history_filtered_scenes;
    if (result.solver_input_scenes <= 0) {
        result.solver_input_scenes = result.accepted_scenes;
    }
    if (result.optimization_scenes <= 0 && result.num_scenes > 0) {
        result.optimization_scenes = result.num_scenes;
    }
}

static bool writeCalibFile(const CalibOutput& out, const cv::Mat& debug_image)
{
    ensureDir(out.output_dir);
    std::string dir = out.output_dir;
    if (!dir.empty() && dir.back() != '/') dir += '/';

    const CalibResult& r = out.result;
    const CameraIntrinsics& ci = out.intrinsics;

    Eigen::Matrix3d R_d;
    Eigen::Vector3d t_d;
    Eigen::Vector3d Pcv;
    double d_roll_deg = 0.0;
    double d_pitch_deg = 0.0;
    double d_yaw_deg = 0.0;
    computeDCompatiblePose(r, R_d, t_d, Pcv, d_roll_deg, d_pitch_deg, d_yaw_deg);

    std::ofstream ofs(dir + "single_calib_result.txt");
    if (!ofs.is_open()) {
        ROS_ERROR("[Node] Cannot open %s", (dir + "single_calib_result.txt").c_str());
        return false;
    }

    std::time_t t = std::time(nullptr);
    char tbuf[64];
    std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    ofs << "# Camera-LiDAR extrinsic calibration\n";
    ofs << "# Generated: " << tbuf << "\n";
    ofs << "# Quality: " << r.quality_grade << "  RMSE=" << r.rmse_m << "m"
        << "  raw_rmse=" << r.raw_rmse_m << "m"
        << "  reproj=" << r.reproj_rmse_px << "px"
        << "  scenes=" << r.num_scenes << "\n";
    ofs << "# Scene stats: total_keyframes=" << r.total_keyframes
        << "  accepted=" << r.accepted_scenes
        << "  filtered=" << (r.history_filtered_scenes + r.solver_dropped_scenes)
        << "  history_skip=" << r.history_filtered_scenes
        << "  solver_in=" << r.solver_input_scenes
        << "  solver_drop=" << r.solver_dropped_scenes
        << "  optimize=" << r.optimization_scenes << "\n";
    ofs << "# T_cam_to_VCS: P_VCS = R * P_cam + t\n";
    ofs << "# d pipeline: p0 = R0 * p_raw, p_cam = R_d * (p0 + t_d), R0=[[-y],[-z],[x]]\n\n";

    if (ci.camera_model == MEI) {
        ofs << "cam_model: MEI\n";
        ofs << std::fixed << std::setprecision(8);
        ofs << "xi: " << ci.xi << "\n";
    } else {
        ofs << "cam_model: Pinhole\n";
    }
    ofs << "cam_width: " << ci.width << "\n";
    ofs << "cam_height: " << ci.height << "\n";
    ofs << "scale: 1.0\n";
    ofs << std::fixed << std::setprecision(8);
    ofs << "cam_fx: " << ci.fx << "\n";
    ofs << "cam_fy: " << ci.fy << "\n";
    ofs << "cam_cx: " << ci.cx << "\n";
    ofs << "cam_cy: " << ci.cy << "\n";
    ofs << "cam_d0: " << ci.k1 << "\n";
    ofs << "cam_d1: " << ci.k2 << "\n";
    ofs << "cam_d2: " << ci.p1 << "\n";
    ofs << "cam_d3: " << ci.p2 << "\n\n";

    ofs << "R_c2v: [" << std::setw(12) << r.R(0,0) << ", "
                      << std::setw(12) << r.R(0,1) << ", "
                      << std::setw(12) << r.R(0,2) << ",\n";
    ofs << "        " << std::setw(12) << r.R(1,0) << ", "
                      << std::setw(12) << r.R(1,1) << ", "
                      << std::setw(12) << r.R(1,2) << ",\n";
    ofs << "        " << std::setw(12) << r.R(2,0) << ", "
                      << std::setw(12) << r.R(2,1) << ", "
                      << std::setw(12) << r.R(2,2) << "]\n";
    ofs << "t_c2v: [" << std::setw(12) << r.x_m << ", "
                      << std::setw(12) << r.y_m << ", "
                      << std::setw(12) << r.z_m << "]\n";

    ofs << "R_d: [" << std::setw(12) << R_d(0,0) << ", "
                    << std::setw(12) << R_d(0,1) << ", "
                    << std::setw(12) << R_d(0,2) << ",\n";
    ofs << "     " << std::setw(12) << R_d(1,0) << ", "
                    << std::setw(12) << R_d(1,1) << ", "
                    << std::setw(12) << R_d(1,2) << ",\n";
    ofs << "     " << std::setw(12) << R_d(2,0) << ", "
                    << std::setw(12) << R_d(2,1) << ", "
                    << std::setw(12) << R_d(2,2) << "]\n";
    ofs << "Pcv_d: [" << std::setw(12) << Pcv(0) << ", "
                      << std::setw(12) << Pcv(1) << ", "
                      << std::setw(12) << Pcv(2) << "]\n";
    ofs << "d_xyz: [" << std::setw(12) << t_d(0) << ", "
                      << std::setw(12) << t_d(1) << ", "
                      << std::setw(12) << t_d(2) << "]\n";
    ofs << "d_rpy_deg: [" << std::setw(12) << d_roll_deg << ", "
                          << std::setw(12) << d_pitch_deg << ", "
                          << std::setw(12) << d_yaw_deg << "]\n";
    ofs << "d_note: d applies p0 = R0 * p_raw, then p_cam = R_d * (p0 + t).\n";
    ofs << "d_note_frame: R0 maps raw FLU lidar to d pre-rotated frame p0=[-y,-z,x].\n";
    ofs << "d_note_param: d_xyz here equals the yaml field t in that pre-rotated frame.\n";
    ofs.close();
    ROS_INFO("[Node] Saved calibration result to: %ssingle_calib_result.txt", dir.c_str());

    const std::string camera_name = out.installation.camera_name.empty() ? std::string("camera")
                                                                         : out.installation.camera_name;
    std::string yaml_path = out.yaml_path;
    if (yaml_path.empty()) {
        yaml_path = dir + "d_camera_params.yaml";
    }
    std::ofstream d_ofs(yaml_path);
    if (!d_ofs.is_open()) {
        ROS_ERROR("[Node] Cannot open %s", yaml_path.c_str());
        return false;
    }
    d_ofs << camera_name << ":\n";
    d_ofs << "  Size: [" << ci.width << ", " << ci.height << "]\n";
    d_ofs << std::fixed << std::setprecision(10);
    d_ofs << "  K: [" << ci.fx << ", 0.0, " << ci.cx
          << ", 0.0, " << ci.fy << ", " << ci.cy
          << ", 0.0, 0.0, 1.0]\n";
    d_ofs << "  D: [" << ci.k1 << ", " << ci.k2 << ", " << ci.p1 << ", " << ci.p2 << "]\n";
    d_ofs << "  Epsilon: " << (ci.camera_model == MEI ? ci.xi : 0.0) << "\n";
    d_ofs << "  R_euler: [" << d_roll_deg << ", " << d_pitch_deg << ", " << d_yaw_deg << "]\n";
    d_ofs << "  t: [" << t_d(0) << ", " << t_d(1) << ", " << t_d(2) << "]\n";
    d_ofs << "  # Generated from image_lidar_calibration with d init_rot compensated.\n";
    d_ofs << "  # d pipeline: p0 = [[0,-1,0],[0,0,-1],[1,0,0]] * p_raw, then p_cam = R * (p0 + t).\n";
    d_ofs.close();
    ROS_INFO("[Node] Saved d-compatible camera params to: %s", yaml_path.c_str());

    if (!debug_image.empty()) {
        cv::imwrite(dir + "qr_detect.png", debug_image);
        ROS_INFO("[Node] Saved debug image to: %sqr_detect.png", dir.c_str());
    }
    return true;
}

static int qualityGradeRank(const std::string& grade)
{
    if (grade == "EXCELLENT") return 4;
    if (grade == "GOOD") return 3;
    if (grade == "ACCEPTABLE") return 2;
    return 1;
}

static bool isBetterCalibResult(const CalibResult& candidate,
                                const CalibResult& current)
{
    if (!current.valid) return candidate.valid;
    if (!candidate.valid) return false;

    const int candidate_rank = qualityGradeRank(candidate.quality_grade);
    const int current_rank = qualityGradeRank(current.quality_grade);
    if (candidate_rank != current_rank) {
        return candidate_rank > current_rank;
    }
    if (std::fabs(candidate.rmse_m - current.rmse_m) > 1e-6) {
        return candidate.rmse_m < current.rmse_m;
    }
    if (std::fabs(candidate.reproj_rmse_px - current.reproj_rmse_px) > 1e-6) {
        return candidate.reproj_rmse_px < current.reproj_rmse_px;
    }
    if (std::fabs(candidate.raw_rmse_m - current.raw_rmse_m) > 1e-6) {
        return candidate.raw_rmse_m < current.raw_rmse_m;
    }
    if (candidate.num_scenes != current.num_scenes) {
        return candidate.num_scenes > current.num_scenes;
    }
    return candidate.confidence > current.confidence;
}

static std::string buildCalibResultSummary(const CalibResult& result)
{
    Eigen::Matrix3d R_d;
    Eigen::Vector3d t_d;
    Eigen::Vector3d Pcv;
    double d_roll_deg = 0.0;
    double d_pitch_deg = 0.0;
    double d_yaw_deg = 0.0;
    computeDCompatiblePose(result, R_d, t_d, Pcv, d_roll_deg, d_pitch_deg, d_yaw_deg);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "grade=" << result.quality_grade
        << " rmse=" << result.rmse_m << "m"
        << " raw_rmse=" << result.raw_rmse_m << "m"
        << " reproj=" << result.reproj_rmse_px << "px"
        << " scenes=" << result.num_scenes
        << " total_keyframes=" << result.total_keyframes
        << " accepted=" << result.accepted_scenes
        << " filtered=" << (result.history_filtered_scenes + result.solver_dropped_scenes)
        << " history_skip=" << result.history_filtered_scenes
        << " solver_in=" << result.solver_input_scenes
        << " solver_drop=" << result.solver_dropped_scenes
        << " optimize=" << result.optimization_scenes
        << " c2v_xyz=(" << result.x_m << ", " << result.y_m << ", " << result.z_m << ")m"
        << " c2v_rpy=(" << result.roll_deg << ", " << result.pitch_deg << ", " << result.yaw_deg << ")deg"
        << " d_t(R0*raw)=(" << t_d.x() << ", " << t_d.y() << ", " << t_d.z() << ")m"
        << " d_rpy=(" << d_roll_deg << ", " << d_pitch_deg << ", " << d_yaw_deg << ")deg";
    return oss.str();
}

static std::string compactServiceText(std::string text)
{
    for (size_t pos = text.find(": "); pos != std::string::npos; pos = text.find(": ", pos)) {
        text.replace(pos, 2, ",");
    }
    for (size_t pos = text.find(" > "); pos != std::string::npos; pos = text.find(" > ", pos)) {
        text.replace(pos, 3, ",gt,");
    }
    for (size_t pos = text.find(" < "); pos != std::string::npos; pos = text.find(" < ", pos)) {
        text.replace(pos, 3, ",lt,");
    }
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == ' ') {
            text[i] = '_';
        }
    }
    return text;
}

static std::string buildCalibResultServiceSummary(const CalibResult& result)
{
    Eigen::Matrix3d R_d;
    Eigen::Vector3d t_d;
    Eigen::Vector3d Pcv;
    double d_roll_deg = 0.0;
    double d_pitch_deg = 0.0;
    double d_yaw_deg = 0.0;
    computeDCompatiblePose(result, R_d, t_d, Pcv, d_roll_deg, d_pitch_deg, d_yaw_deg);

    std::ostringstream oss;
    oss << std::fixed
        << "detected_keyframes=" << result.total_keyframes
        << ",cached_scenes=" << result.accepted_scenes
        << ",ransac_scenes=" << result.optimization_scenes
        << std::setprecision(4)
        << ",c2v_xyz=(" << result.x_m << "," << result.y_m << "," << result.z_m << ")"
        << std::setprecision(2)
        << ",c2v_rpy=(" << result.roll_deg << "," << result.pitch_deg << "," << result.yaw_deg << ")"
        << std::setprecision(4)
        << ",d_t=(" << t_d.x() << "," << t_d.y() << "," << t_d.z() << ")"
        << std::setprecision(2)
        << ",d_rpy=(" << d_roll_deg << "," << d_pitch_deg << "," << d_yaw_deg << ")"
        << ",grade=" << result.quality_grade
        << std::setprecision(4)
        << ",rmse=" << result.rmse_m
        << ",raw_rmse=" << result.raw_rmse_m
        << ",reproj=" << result.reproj_rmse_px;
    return oss.str();
}

static std::string buildCalibFailureServiceSummary(const std::string& message)
{
    return std::string("failure_reason=") + compactServiceText(message);
}

static void logStageDebugReport(const std::string& tag,
                                const CalibResult& result)
{
    if (result.stage_debug_report.empty()) return;
    std::istringstream iss(result.stage_debug_report);
    std::string line;
    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        ROS_INFO("%s %s", tag.c_str(), line.c_str());
    }
}


class ImageLidarCalibNode {
public:
    explicit ImageLidarCalibNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh),
          camera_info_received_(false), latest_result_valid_(false)
    {
        loadParams();
        setupROS();
        calibration_thread_ = std::thread(&ImageLidarCalibNode::calibrationLoop, this);
        calibration_thread_.detach();
        projection_thread_ = std::thread(&ImageLidarCalibNode::projectionLoop, this);
        projection_thread_.detach();
        ROS_INFO("[Node] Online mode: realtime_calibration=%s, waiting for sensor data.",
                 realtime_calibration_ ? "true" : "false");
    }

private:


    struct CachedImageDetection {
        sensor_msgs::Image::ConstPtr image_msg;
        std_msgs::Header header;
        cv::Mat raw_image;
    };

    struct CachedScenePair {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        sensor_msgs::Image::ConstPtr image_msg;
        cv::Mat raw_image;
        std::vector<Eigen::Vector3d> camera_centers;
        std::vector<cv::Point2f> camera_circle_pixels;
        std::vector<Eigen::Vector3d> lidar_centers;
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_edge_cloud;
        cv::Mat debug_image;
        BoardPose board_pose;
        bool valid = false;
    };

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    FastLivo2Detector detector_;
    LidarProcessor lidar_proc_;
    CalibrationSolver solver_;

    std::vector<CachedScenePair> scene_pairs_;
    size_t max_scene_pairs_ = 100;
    size_t scene_cache_version_ = 0;
    size_t calibrated_scene_cache_version_ = 0;
    CachedScenePair latest_render_scene_;
    bool latest_render_scene_valid_ = false;
    CameraIntrinsics cam_intrinsics_;
    bool camera_info_received_;

    std::deque<CachedImageDetection> image_buffer_;
    size_t max_image_buffer_size_ = 20;
    double max_sensor_time_diff_sec_ = 0.05;
    cv::Mat latest_debug_image_;
    std::mutex image_mutex_;
    std::mutex scene_mutex_;
    std::mutex render_scene_mutex_;
    std::mutex result_mutex_;

    std::string output_dir_;
    std::string yaml_path_;
    InstallParams install_params_;

    bool realtime_calibration_ = true;
    bool auto_save_realtime_result_ = false;
    CalibResult latest_result_;
    bool latest_result_valid_;
    double projection_min_depth_m_ = 0.3;
    double projection_max_depth_m_ = 30.0;
    double projection_rate_hz_ = 10.0;
    double calibration_rate_hz_ = 10.0;
    std::thread calibration_thread_;
    std::thread projection_thread_;

    ros::Publisher pub_status_;
    ros::Publisher pub_colored_cloud_;
    ros::Publisher pub_board_plane_cloud_;
    ros::Publisher pub_board_edge_cloud_;
    ros::Publisher pub_board_circle_centers_cloud_;
    image_transport::Publisher pub_debug_image_;
    image_transport::Publisher pub_lidar_projection_image_;
    ros::ServiceServer srv_trigger_;
    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_save_;
    ros::Subscriber sub_lidar_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_cam_info_;

    double min_rotation_deg_ = 5.0;
    double min_position_m_ = 0.5;
    int history_pose_filtered_count_ = 0;

    void loadParams();
    void refreshLidarProcessorParams();
    void setupROS();
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool getClosestImageDetection(const ros::Time& cloud_stamp,
                                  CachedImageDetection& image_detection,
                                  double& time_diff_sec);
    bool getLatestRenderScene(CachedScenePair& scene_pair);
    bool getLatestResultSnapshot(CalibResult& result);
    void calibrationLoop();
    void projectionLoop();
    void publishXYZCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         const std_msgs::Header& header,
                         ros::Publisher& pub);
    void publishCircleCentersCloud(const std::vector<Eigen::Vector3d>& centers,
                                   const std_msgs::Header& header);
    void publishStatus(const std::string& msg);
    bool solveScenes(CalibResult& result);
    bool saveResultToFile(const CalibResult& result);
    bool saveSceneVisualizations(const CalibResult& result);
    bool updateRealtimeCalibration();
    void publishLidarProjectionImage(const sensor_msgs::Image::ConstPtr& image_msg,
                                     const cv::Mat& raw_image,
                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzi,
                                     const CalibResult& result);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    bool srvTrigger(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
    bool srvReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
    bool srvSave(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
};
void ImageLidarCalibNode::loadParams()
{
    output_dir_ = "/tmp/image_lidar_calibration";
    pnh_.param<std::string>("output/yaml_path", yaml_path_, "");
    pnh_.param<bool>("realtime_calibration", realtime_calibration_, true);
    pnh_.param<bool>("auto_save_realtime_result", auto_save_realtime_result_, false);
    pnh_.param<double>("projection/min_depth_m", projection_min_depth_m_, 0.3);
    pnh_.param<double>("projection/max_depth_m", projection_max_depth_m_, 30.0);
    pnh_.param<double>("projection/rate_hz", projection_rate_hz_, 10.0);
    pnh_.param<double>("calibration/rate_hz", calibration_rate_hz_, 10.0);
    int scene_pair_cache_size = 100;
    pnh_.param<int>("calibration/max_scene_pairs", scene_pair_cache_size, 100);
    max_scene_pairs_ = scene_pair_cache_size > 0 ? static_cast<size_t>(scene_pair_cache_size) : static_cast<size_t>(1);

    std::string camera_file;
    pnh_.param<std::string>("camera_file", camera_file, "");
    if (camera_file.empty()) {
        ROS_WARN("[Node] camera_file not set.");
    } else if (cam_intrinsics_.loadFromFile(camera_file) == false) {
        ROS_ERROR("[Node] Failed to load camera intrinsics from: %s", camera_file.c_str());
    } else {
        ROS_INFO("[Node] ===== Camera Parameters =====");
        ROS_INFO("[Node] Camera loaded: model=%s", (cam_intrinsics_.camera_model == MEI ? "MEI" : "Pinhole"));
        ROS_INFO("[Node]   width=%d height=%d", cam_intrinsics_.width, cam_intrinsics_.height);
        ROS_INFO("[Node]   fx=%.6f fy=%.6f", cam_intrinsics_.fx, cam_intrinsics_.fy);
        ROS_INFO("[Node]   cx=%.6f cy=%.6f", cam_intrinsics_.cx, cam_intrinsics_.cy);
        ROS_INFO("[Node]   k1=%.8f k2=%.8f", cam_intrinsics_.k1, cam_intrinsics_.k2);
        ROS_INFO("[Node]   p1=%.10f p2=%.10f", cam_intrinsics_.p1, cam_intrinsics_.p2);
        ROS_INFO("[Node]   xi=%.8f", cam_intrinsics_.xi);
        ROS_INFO("[Node] ==============================");
    }
    detector_.setCamera(cam_intrinsics_);
    solver_.setCamera(cam_intrinsics_);

    pnh_.param<std::string>("installation/camera_name", install_params_.camera_name, "front_camera");
    pnh_.param<double>("installation/x", install_params_.x, 0.0);
    pnh_.param<double>("installation/y", install_params_.y, 0.0);
    pnh_.param<double>("installation/z", install_params_.z, 0.0);
    pnh_.param<double>("installation/roll_deg", install_params_.roll_deg, 0.0);
    pnh_.param<double>("installation/pitch_deg", install_params_.pitch_deg, 0.0);
    pnh_.param<double>("installation/yaw_deg", install_params_.yaw_deg, 0.0);

    TargetParams target;
    pnh_.param<double>("target/marker_size", target.marker_size, 0.20);
    pnh_.param<double>("target/delta_width_qr_center", target.delta_width_qr_center, 0.55);
    pnh_.param<double>("target/delta_height_qr_center", target.delta_height_qr_center, 0.35);
    pnh_.param<double>("target/delta_width_circles", target.delta_width_circles, 0.50);
    pnh_.param<double>("target/delta_height_circles", target.delta_height_circles, 0.40);
    pnh_.param<double>("target/circle_radius", target.circle_radius, 0.12);
    pnh_.param<int>("target/min_detected_markers", target.min_detected_markers, 3);
    detector_.setTarget(target);
    refreshLidarProcessorParams();

    CalibrationSolver::Params sp;
    pnh_.param<int>("solver/min_scenes", sp.min_scenes, 1);
    pnh_.param<bool>("solver/use_prior_pairing", sp.use_prior_pairing, true);
    sp.prior_x_m = install_params_.x;
    sp.prior_y_m = install_params_.y;
    sp.prior_z_m = install_params_.z;
    sp.prior_roll_deg = install_params_.roll_deg;
    sp.prior_pitch_deg = install_params_.pitch_deg;
    sp.prior_yaw_deg = install_params_.yaw_deg;
    sp.target_delta_width_circles_m = target.delta_width_circles;
    sp.target_delta_height_circles_m = target.delta_height_circles;
    pnh_.param<double>("solver/max_pairing_rmse_m", sp.max_pairing_rmse_m, 1.0);
    pnh_.param<double>("solver/max_translation_delta_from_prior_m",
                       sp.max_translation_delta_from_prior_m,
                       0.05);
    pnh_.param<double>("solver/max_rotation_delta_from_prior_deg",
                       sp.max_rotation_delta_from_prior_deg,
                       0.0);
    
    // Ceres 优化参数
    pnh_.param<bool>("solver/use_ceres_optimization",
                     sp.use_ceres_optimization,
                     true);
    pnh_.param<int>("solver/optimization_max_iterations",
                    sp.optimization_max_iterations,
                    200);
    
    // xyz 边界约束参数
    pnh_.param<double>("solver/xyz_delta_limit_m",
                       sp.xyz_delta_limit_m,
                       0.03);
    pnh_.param<bool>("solver/separate_axis_constraint",
                     sp.separate_axis_constraint,
                     false);
    
    pnh_.param<double>("solver/lidar_point_prior_weight",
                       sp.lidar_point_prior_weight,
                       1.0);
    pnh_.param<double>("solver/lidar_geometry_weight",
                       sp.lidar_geometry_weight,
                       10.0);
    pnh_.param<double>("solver/pose_rotation_prior_weight",
                       sp.pose_rotation_prior_weight,
                       0.0);
    pnh_.param<bool>("solver/use_ransac_rejection",
                       sp.use_ransac_rejection,
                       true);
    pnh_.param<double>("solver/ransac_reproj_threshold_px",
                       sp.ransac_reproj_threshold_px,
                       6.0);
    pnh_.param<int>("solver/ransac_min_inliers",
                    sp.ransac_min_inliers,
                    4);
    pnh_.param<int>("solver/ransac_min_scene_inliers",
                    sp.ransac_min_scene_inliers,
                    3);
    solver_.setParams(sp);
    pnh_.param<double>("pose_filter/min_rotation_deg", min_rotation_deg_, 5.0);
    pnh_.param<double>("pose_filter/min_position_m", min_position_m_, min_position_m_);
    int image_buffer_size = 20;
    pnh_.param<int>("sync/image_buffer_size", image_buffer_size, 20);
    pnh_.param<double>("sync/max_time_diff_sec", max_sensor_time_diff_sec_, 0.05);
    max_image_buffer_size_ = image_buffer_size > 0 ? static_cast<size_t>(image_buffer_size) : static_cast<size_t>(1);

    ROS_INFO("[Node] Params loaded. output_dir=%s", output_dir_.c_str());
    ROS_INFO("[Node] Pose filter(lidar history): min_rotation=%.1f deg, min_xyz=%.2f m, compare=all_history", min_rotation_deg_, min_position_m_);
    ROS_INFO("[Node] Realtime calibration: enabled=%s auto_save=%s projection_mode=d_style_raw_forward_threaded",
             realtime_calibration_ ? "true" : "false",
             auto_save_realtime_result_ ? "true" : "false");
    ROS_INFO("[Node] Calibration thread: rate=%.1fHz max_scene_pairs=%zu",
             calibration_rate_hz_,
             max_scene_pairs_);
    ROS_INFO("[Node] Projection thread: rate=%.1fHz depth=[%.2f, %.2f]m",
             projection_rate_hz_,
             projection_min_depth_m_,
             projection_max_depth_m_);
    ROS_INFO("[Node] Sync buffer: image=%zu max_dt=%.3fs",
             max_image_buffer_size_,
             max_sensor_time_diff_sec_);
}
void ImageLidarCalibNode::refreshLidarProcessorParams()
{
    TargetParams target;
    pnh_.param<double>("target/marker_size", target.marker_size, 0.20);
    pnh_.param<double>("target/delta_width_qr_center", target.delta_width_qr_center, 0.55);
    pnh_.param<double>("target/delta_height_qr_center", target.delta_height_qr_center, 0.35);
    pnh_.param<double>("target/delta_width_circles", target.delta_width_circles, 0.50);
    pnh_.param<double>("target/delta_height_circles", target.delta_height_circles, 0.40);
    pnh_.param<double>("target/circle_radius", target.circle_radius, 0.12);

    LidarProcessor::Params lp;
    lp.circle_radius = target.circle_radius;
    lp.delta_width_circles = target.delta_width_circles;
    lp.delta_height_circles = target.delta_height_circles;
    pnh_.param<double>("lidar/roi_x_min", lp.roi_x_min, lp.roi_x_min);
    pnh_.param<double>("lidar/roi_x_max", lp.roi_x_max, lp.roi_x_max);
    pnh_.param<double>("lidar/roi_y_min", lp.roi_y_min, lp.roi_y_min);
    pnh_.param<double>("lidar/roi_y_max", lp.roi_y_max, lp.roi_y_max);
    pnh_.param<double>("lidar/roi_z_min", lp.roi_z_min, lp.roi_z_min);
    pnh_.param<double>("lidar/roi_z_max", lp.roi_z_max, lp.roi_z_max);
    pnh_.param<double>("lidar/ransac_threshold", lp.ransac_threshold, lp.ransac_threshold);
    pnh_.param<int>("lidar/max_iterations", lp.max_iterations, lp.max_iterations);
    pnh_.param<double>("lidar/board_filter_margin_x", lp.board_filter_margin_x, lp.board_filter_margin_x);
    pnh_.param<double>("lidar/board_filter_margin_y", lp.board_filter_margin_y, lp.board_filter_margin_y);
    pnh_.param<double>("lidar/edge_filter_margin_x", lp.edge_filter_margin_x, lp.edge_filter_margin_x);
    pnh_.param<double>("lidar/edge_filter_margin_y", lp.edge_filter_margin_y, lp.edge_filter_margin_y);
    pnh_.param<double>("lidar/contamination_min_score", lp.contamination_min_score, lp.contamination_min_score);
    pnh_.param<double>("lidar/contamination_min_plane_keep_ratio", lp.contamination_min_plane_keep_ratio, lp.contamination_min_plane_keep_ratio);
    pnh_.param<double>("lidar/contamination_min_tight_inside_ratio", lp.contamination_min_tight_inside_ratio, lp.contamination_min_tight_inside_ratio);
    pnh_.param<double>("lidar/hole_search_radius_margin", lp.hole_search_radius_margin, lp.hole_search_radius_margin);
    pnh_.param<double>("lidar/hole_max_center_shift", lp.hole_max_center_shift, lp.hole_max_center_shift);
    pnh_.param<int>("lidar/hole_min_annulus_points", lp.hole_min_annulus_points, lp.hole_min_annulus_points);
    pnh_.param<int>("lidar/hole_max_inner_points", lp.hole_max_inner_points, lp.hole_max_inner_points);
    pnh_.param<double>("lidar/hole_min_local_score", lp.hole_min_local_score, lp.hole_min_local_score);
    lidar_proc_.setParams(lp);
}

void ImageLidarCalibNode::setupROS()
{
    std::string lidar_topic, image_topic, cam_info_topic;
    std::string status_topic, colored_cloud_topic, board_plane_cloud_topic;
    std::string board_edge_cloud_topic, board_circle_centers_topic;
    std::string debug_image_topic, lidar_projection_image_topic;

    pnh_.param<std::string>("lidar_topic", lidar_topic, "/rslidar_points");
    pnh_.param<std::string>("image_topic", image_topic, "/camera/image_raw");
    pnh_.param<std::string>("camera_info_topic", cam_info_topic, "/camera/camera_info");
    pnh_.param<std::string>("status_topic", status_topic, "status");
    pnh_.param<std::string>("colored_cloud_topic", colored_cloud_topic, "colored_cloud");
    pnh_.param<std::string>("board_plane_cloud_topic", board_plane_cloud_topic, "board_plane_cloud");
    pnh_.param<std::string>("board_edge_cloud_topic", board_edge_cloud_topic, "board_edge_cloud");
    pnh_.param<std::string>("board_circle_centers_topic", board_circle_centers_topic, "board_circle_centers");
    pnh_.param<std::string>("debug_image_topic", debug_image_topic, "debug_image");
    pnh_.param<std::string>("lidar_projection_image_topic", lidar_projection_image_topic, "lidar_projection_image");

    sub_lidar_ = nh_.subscribe(lidar_topic, 1, &ImageLidarCalibNode::lidarCallback, this);
    sub_image_ = it_.subscribe(image_topic, 1, &ImageLidarCalibNode::imageCallback, this);
    sub_cam_info_ = nh_.subscribe(cam_info_topic, 1, &ImageLidarCalibNode::camInfoCallback, this);

    pub_status_ = nh_.advertise<std_msgs::String>(status_topic, 10);
    pub_colored_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(colored_cloud_topic, 1, true);
    pub_board_plane_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(board_plane_cloud_topic, 1, true);
    pub_board_edge_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(board_edge_cloud_topic, 1, true);
    pub_board_circle_centers_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(board_circle_centers_topic, 1, true);
    pub_debug_image_ = it_.advertise(debug_image_topic, 1);
    pub_lidar_projection_image_ = it_.advertise(lidar_projection_image_topic, 1);

    srv_trigger_ = nh_.advertiseService("trigger_calibration", &ImageLidarCalibNode::srvTrigger, this);
    srv_reset_ = nh_.advertiseService("reset_calibration", &ImageLidarCalibNode::srvReset, this);
    srv_save_ = nh_.advertiseService("save_calibration", &ImageLidarCalibNode::srvSave, this);
}

void ImageLidarCalibNode::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (camera_info_received_) return;

    std::string camera_file;
    pnh_.param<std::string>("camera_file", camera_file, "");
    if (camera_file.empty() == false) {
        sub_cam_info_.shutdown();
        return;
    }

    cam_intrinsics_.fx = msg->K[0];
    cam_intrinsics_.fy = msg->K[4];
    cam_intrinsics_.cx = msg->K[2];
    cam_intrinsics_.cy = msg->K[5];
    cam_intrinsics_.width = static_cast<int>(msg->width);
    cam_intrinsics_.height = static_cast<int>(msg->height);
    if (msg->D.size() >= 4) {
        cam_intrinsics_.k1 = msg->D[0];
        cam_intrinsics_.k2 = msg->D[1];
        cam_intrinsics_.p1 = msg->D[2];
        cam_intrinsics_.p2 = msg->D[3];
    }
    detector_.setCamera(cam_intrinsics_);
    solver_.setCamera(cam_intrinsics_);
    camera_info_received_ = true;
    ROS_INFO("[Node] Camera info from topic: fx=%.1f fy=%.1f", cam_intrinsics_.fx, cam_intrinsics_.fy);
    sub_cam_info_.shutdown();
}

void ImageLidarCalibNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi = convertCloudToGeometryCloud(msg);

    CachedScenePair render_scene;
    CachedImageDetection matched_image;
    double image_time_diff_sec = 0.0;

    refreshLidarProcessorParams();

    if (getClosestImageDetection(msg->header.stamp, matched_image, image_time_diff_sec)) {
        render_scene = CachedScenePair();
        render_scene.cloud = cloud_xyzi;
        render_scene.image_msg = matched_image.image_msg;
        render_scene.raw_image = matched_image.raw_image.clone();
        render_scene.valid = true;
        {
            std::lock_guard<std::mutex> lock(render_scene_mutex_);
            latest_render_scene_ = render_scene;
            latest_render_scene_valid_ = true;
        }
        ROS_DEBUG("[Node] Latest render frame updated. image_dt=%.4f s", image_time_diff_sec);
    }

    std::vector<Eigen::Vector3d> lidar_centers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_edge_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const bool lidar_ok = lidar_proc_.detectCircleCenters(cloud_xyzi, lidar_centers, lidar_edge_cloud, lidar_plane_cloud);

    publishXYZCloud(lidar_plane_cloud, msg->header, pub_board_plane_cloud_);
    publishXYZCloud(lidar_edge_cloud, msg->header, pub_board_edge_cloud_);
    publishXYZCloud(lidar_edge_cloud, msg->header, pub_colored_cloud_);
    publishCircleCentersCloud(lidar_centers, msg->header);
    if (lidar_ok == false) return;

    ROS_DEBUG("[Node] LiDAR keyframe accepted: 4 circle centers at stamp %.6f", msg->header.stamp.toSec());

    if (matched_image.image_msg.get() == NULL || matched_image.raw_image.empty()) {
        ROS_DEBUG_THROTTLE(2.0, "[Node] No time-matched raw image for LiDAR keyframe. max_time_diff=%.3fs", max_sensor_time_diff_sec_);
        return;
    }

    ROS_DEBUG("[Node] Matched LiDAR keyframe/image time diff: %.4f s", image_time_diff_sec);

    BoardPose lidar_board_pose;
    if (!estimateLidarBoardPose(lidar_centers, lidar_board_pose)) {
        ROS_DEBUG("[Node] Failed to estimate LiDAR board pose from detected circle centers.");
        return;
    }

    ROS_DEBUG("[Node] LiDAR board pose: xyz=(%.3f, %.3f, %.3f)m rpy=(%.2f, %.2f, %.2f)deg",
             lidar_board_pose.x,
             lidar_board_pose.y,
             lidar_board_pose.z,
             lidar_board_pose.roll_deg,
             lidar_board_pose.pitch_deg,
             lidar_board_pose.yaw_deg);

    size_t nearest_scene_idx = 0;
    double nearest_max_abs_xyz_m = std::numeric_limits<double>::infinity();
    double nearest_rotation_deg = std::numeric_limits<double>::infinity();
    bool same_cluster = false;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        double best_score = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < scene_pairs_.size(); ++i) {
            if (!scene_pairs_[i].valid) continue;
            const BoardPose& history = scene_pairs_[i].board_pose;
            const double dx = std::fabs(lidar_board_pose.x - history.x);
            const double dy = std::fabs(lidar_board_pose.y - history.y);
            const double dz = std::fabs(lidar_board_pose.z - history.z);
            const double max_abs_xyz_m = std::max(dx, std::max(dy, dz));
            const double rotation_deg = lidar_board_pose.rotationDiffDeg(history);
            const double score = (min_position_m_ > 1e-9 ? (max_abs_xyz_m / min_position_m_) : max_abs_xyz_m) +
                                 (min_rotation_deg_ > 1e-9 ? (rotation_deg / min_rotation_deg_) : rotation_deg);
            if (score < best_score) {
                best_score = score;
                nearest_scene_idx = i;
                nearest_max_abs_xyz_m = max_abs_xyz_m;
                nearest_rotation_deg = rotation_deg;
            }
            if (max_abs_xyz_m < min_position_m_ && rotation_deg < min_rotation_deg_) {
                same_cluster = true;
            }
        }
    }
    if (std::isfinite(nearest_max_abs_xyz_m) && std::isfinite(nearest_rotation_deg)) {
        ROS_DEBUG("[Node] LiDAR pose nearest history scene %zu: max_abs_xyz=%.3f m rot=%.2f deg cluster=%s",
                 nearest_scene_idx + 1,
                 nearest_max_abs_xyz_m,
                 nearest_rotation_deg,
                 same_cluster ? "hit" : "new");
    }

    if (same_cluster) {
        ROS_DEBUG("[Node] LiDAR pose too close to history scene %zu (max_abs_xyz=%.3f<%.2f m, rot=%.2f<%.1f deg). Skipping.",
                 nearest_scene_idx + 1,
                 nearest_max_abs_xyz_m,
                 min_position_m_,
                 nearest_rotation_deg,
                 min_rotation_deg_);
        ++history_pose_filtered_count_;
        return;
    }

    std::vector<Eigen::Vector3d> camera_centers;
    std::vector<cv::Point2f> camera_circle_pixels;
    BoardPose image_board_pose;
    cv::Mat debug_img;
    if (!detector_.detectWithPose(matched_image.raw_image,
                                  camera_centers,
                                  camera_circle_pixels,
                                  image_board_pose,
                                  debug_img)) {
        ROS_DEBUG("[Node] Image circle detection failed for LiDAR keyframe. Skip caching this scene.");
        return;
    }

    if (!debug_img.empty()) {
        cv_bridge::CvImage debug_out(matched_image.image_msg->header, "bgr8", debug_img);
        pub_debug_image_.publish(debug_out.toImageMsg());
    }

    CachedScenePair scene_pair = render_scene;
    scene_pair.camera_centers = camera_centers;
    scene_pair.camera_circle_pixels = camera_circle_pixels;
    scene_pair.lidar_centers = lidar_centers;
    scene_pair.lidar_edge_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*lidar_edge_cloud));
    scene_pair.debug_image = debug_img.clone();
    scene_pair.board_pose = lidar_board_pose;
    scene_pair.valid = true;

    size_t scene_pair_count = 0;
    bool dropped_oldest = false;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        scene_pairs_.push_back(scene_pair);
        ++scene_cache_version_;
        if (max_scene_pairs_ > 0 && scene_pairs_.size() > max_scene_pairs_) {
            scene_pairs_.erase(scene_pairs_.begin());
            dropped_oldest = true;
        }
        scene_pair_count = scene_pairs_.size();
    }

    std::ostringstream status_stream;
    status_stream << "Scene pair #" << scene_pair_count << " cached.";
    if (dropped_oldest) {
        status_stream << " oldest scene dropped by max_scene_pairs=" << max_scene_pairs_ << ".";
    }

    const std::string status = status_stream.str();
    publishStatus(status);
    ROS_DEBUG("[Node] %s", status.c_str());
}

bool ImageLidarCalibNode::getClosestImageDetection(const ros::Time& cloud_stamp,
                                                   CachedImageDetection& image_detection,
                                                   double& time_diff_sec)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (image_buffer_.empty()) return false;

    size_t best_idx = 0;
    double best_dt = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < image_buffer_.size(); ++i) {
        const double dt = std::fabs((image_buffer_[i].header.stamp - cloud_stamp).toSec());
        if (dt < best_dt) {
            best_dt = dt;
            best_idx = i;
        }
    }

    if (std::isfinite(best_dt) == false || best_dt > max_sensor_time_diff_sec_) {
        return false;
    }

    image_detection = image_buffer_[best_idx];
    time_diff_sec = best_dt;
    return image_detection.image_msg.get() != NULL && image_detection.raw_image.empty() == false;
}

bool ImageLidarCalibNode::getLatestRenderScene(CachedScenePair& scene_pair)
{
    std::lock_guard<std::mutex> lock(render_scene_mutex_);
    if (!latest_render_scene_valid_) return false;
    scene_pair = latest_render_scene_;
    return scene_pair.valid && scene_pair.image_msg.get() != NULL &&
           scene_pair.cloud.get() != NULL && !scene_pair.raw_image.empty();
}

bool ImageLidarCalibNode::getLatestResultSnapshot(CalibResult& result)
{
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (latest_result_valid_ == false) return false;
        result = latest_result_;
    }

    int accepted_scene_pairs = 0;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        accepted_scene_pairs = static_cast<int>(scene_pairs_.size());
    }
    fillSceneStatistics(accepted_scene_pairs, history_pose_filtered_count_, result);
    return true;
}

void ImageLidarCalibNode::publishXYZCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          const std_msgs::Header& header,
                                          ros::Publisher& pub)
{
    if (cloud.get() == NULL || cloud->empty()) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header = header;
    pub.publish(msg);
}

void ImageLidarCalibNode::publishCircleCentersCloud(const std::vector<Eigen::Vector3d>& centers,
                                                    const std_msgs::Header& header)
{
    if (centers.empty()) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(centers.size());
    for (size_t i = 0; i < centers.size(); ++i) {
        const Eigen::Vector3d& c = centers[i];
        cloud.push_back(pcl::PointXYZ(static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z())));
    }
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    pub_board_circle_centers_cloud_.publish(msg);
}

void ImageLidarCalibNode::publishStatus(const std::string& msg)
{
    std_msgs::String s;
    s.data = msg;
    pub_status_.publish(s);
}

bool ImageLidarCalibNode::solveScenes(CalibResult& result)
{
    result = CalibResult();

    std::vector<CachedScenePair> scene_pairs_snapshot;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        scene_pairs_snapshot = scene_pairs_;
    }

    if (scene_pairs_snapshot.empty()) {
        result.message = "No cached scene pairs yet.";
        return false;
    }

    std::vector<SceneData> scenes;
    scenes.reserve(scene_pairs_snapshot.size());
    cv::Mat latest_debug_image;
    int invalid_cached_scene_count = 0;

    for (size_t i = 0; i < scene_pairs_snapshot.size(); ++i) {
        const CachedScenePair& pair = scene_pairs_snapshot[i];
        if (!pair.valid ||
            pair.camera_centers.size() != 4 ||
            pair.camera_circle_pixels.size() != 4 ||
            pair.lidar_centers.size() != 4) {
            ++invalid_cached_scene_count;
            continue;
        }

        SceneData scene;
        scene.camera_centers = pair.camera_centers;
        scene.camera_circle_pixels = pair.camera_circle_pixels;
        scene.lidar_centers = pair.lidar_centers;
        scene.board_pose = pair.board_pose;
        scene.valid = true;
        scenes.push_back(scene);
        if (!pair.debug_image.empty()) {
            latest_debug_image = pair.debug_image.clone();
        }
    }

    if (scenes.empty()) {
        std::ostringstream oss;
        oss << "No frozen cached scenes. total_pairs=" << scene_pairs_snapshot.size()
            << " invalid_cached=" << invalid_cached_scene_count;
        result.message = oss.str();
        fillSceneStatistics(static_cast<int>(scene_pairs_snapshot.size()), history_pose_filtered_count_, result);
        return false;
    }

    if (solver_.solve(scenes, result) == false || result.valid == false) {
        fillSceneStatistics(static_cast<int>(scene_pairs_snapshot.size()), history_pose_filtered_count_, result);
        return false;
    }

    if (!latest_debug_image.empty()) {
        latest_debug_image_ = latest_debug_image;
    }
    fillSceneStatistics(static_cast<int>(scene_pairs_snapshot.size()), history_pose_filtered_count_, result);
    return true;
}

bool ImageLidarCalibNode::saveResultToFile(const CalibResult& result)
{
    CalibOutput out;
    out.installation = install_params_;
    out.intrinsics = cam_intrinsics_;
    out.result = result;
    out.output_dir = output_dir_;
    out.yaml_path = yaml_path_;

    if (!writeCalibFile(out, latest_debug_image_)) {
        return false;
    }
    return saveSceneVisualizations(result);
}

bool ImageLidarCalibNode::saveSceneVisualizations(const CalibResult& result)
{
    std::vector<CachedScenePair> scene_pairs_snapshot;
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        scene_pairs_snapshot = scene_pairs_;
    }

    if (scene_pairs_snapshot.empty()) {
        return true;
    }

    int solver_candidate_count = 0;
    for (size_t i = 0; i < scene_pairs_snapshot.size(); ++i) {
        const CachedScenePair& pair = scene_pairs_snapshot[i];
        if (pair.valid &&
            pair.camera_centers.size() == 4 &&
            pair.camera_circle_pixels.size() == 4 &&
            pair.lidar_centers.size() == 4) {
            ++solver_candidate_count;
        }
    }

    bool use_ransac_prefix = false;
    std::vector<bool> optimized_scene_flags;
    if (solver_candidate_count > 0 &&
        !result.optimization_scene_indices.empty() &&
        result.solver_input_scenes == solver_candidate_count) {
        use_ransac_prefix = true;
        optimized_scene_flags.assign(static_cast<size_t>(solver_candidate_count), false);
        for (size_t i = 0; i < result.optimization_scene_indices.size(); ++i) {
            const int idx = result.optimization_scene_indices[i];
            if (idx >= 0 && idx < solver_candidate_count) {
                optimized_scene_flags[static_cast<size_t>(idx)] = true;
            }
        }
    }

    std::string dir = output_dir_;
    if (!dir.empty() && dir.back() != '/') {
        dir += '/';
    }
    ensureDir(dir);
    const std::string scene_dir = dir + "scene_visualizations";
    ensureDir(scene_dir);

    Eigen::Matrix3d R_d;
    Eigen::Vector3d t_d;
    Eigen::Vector3d Pcv;
    double d_roll_deg = 0.0;
    double d_pitch_deg = 0.0;
    double d_yaw_deg = 0.0;
    computeDCompatiblePose(result, R_d, t_d, Pcv, d_roll_deg, d_pitch_deg, d_yaw_deg);

    bool ok = true;
    int exported_count = 0;
    int solver_scene_order = 0;
    for (size_t i = 0; i < scene_pairs_snapshot.size(); ++i) {
        const CachedScenePair& pair = scene_pairs_snapshot[i];

        const bool is_solver_candidate = pair.valid &&
                                         pair.camera_centers.size() == 4 &&
                                         pair.camera_circle_pixels.size() == 4 &&
                                         pair.lidar_centers.size() == 4;
        bool dropped_by_ransac = false;
        if (is_solver_candidate) {
            if (use_ransac_prefix && solver_scene_order < static_cast<int>(optimized_scene_flags.size())) {
                dropped_by_ransac = !optimized_scene_flags[static_cast<size_t>(solver_scene_order)];
            }
            ++solver_scene_order;
        }

        if (!pair.valid || pair.raw_image.empty()) {
            continue;
        }

        const cv::Size display_size = getSceneDisplaySize(cam_intrinsics_, pair.raw_image);
        const cv::Mat raw_image = resizeSceneImage(pair.raw_image, display_size);
        const cv::Mat qr_detect_image = resizeSceneImage(pair.debug_image.empty() ? pair.raw_image : pair.debug_image,
                                                         display_size);
        cv::Mat fusion_image;
        if (pair.cloud.get() != NULL) {
            const cv::Mat fusion_base = pair.debug_image.empty() ? pair.raw_image : pair.debug_image;
            fusion_image = mergeCloudToRawImage(cam_intrinsics_,
                                                fusion_base,
                                                pair.cloud,
                                                R_d,
                                                t_d,
                                                projection_min_depth_m_,
                                                projection_max_depth_m_);
        }
        const cv::Mat lidar_detect_image = fusion_image.empty() ? raw_image.clone() : fusion_image.clone();
        if (fusion_image.empty()) {
            fusion_image = lidar_detect_image.clone();
        }

        const cv::Mat overview_image = buildSceneOverviewImage(raw_image,
                                                               qr_detect_image,
                                                               lidar_detect_image,
                                                               fusion_image);

        std::ostringstream prefix;
        prefix << scene_dir << "/" << (dropped_by_ransac ? "bad_scene_" : "scene_")
               << std::setw(3) << std::setfill('0') << (i + 1);
        const std::string scene_prefix = prefix.str();

        bool scene_ok = true;
        scene_ok = cv::imwrite(scene_prefix + "_overview.png", overview_image) && scene_ok;

        if (!scene_ok) {
            ok = false;
            ROS_ERROR("[Node] Failed to save scene overview: %s", scene_prefix.c_str());
            continue;
        }
        ++exported_count;
    }

    ROS_INFO("[Node] Saved scene visualizations: %d/%zu to %s",
             exported_count,
             scene_pairs_snapshot.size(),
             scene_dir.c_str());
    return ok;
}
bool ImageLidarCalibNode::updateRealtimeCalibration()
{
    CalibResult result;
    if (solveScenes(result) == false) {
        const std::string msg = "Realtime calibration failed: " + result.message;
        publishStatus(msg);
        ROS_DEBUG("[Node] %s", msg.c_str());
        return false;
    }

    bool replace_latest = false;
    CalibResult kept_result;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (!latest_result_valid_ || isBetterCalibResult(result, latest_result_)) {
            latest_result_ = result;
            latest_result_valid_ = true;
            replace_latest = true;
        }
        kept_result = latest_result_;
    }

    if (replace_latest) {
        publishStatus(result.message);
        ROS_DEBUG("[Node] Realtime accepted %s", result.message.c_str());
        if (auto_save_realtime_result_) {
            saveResultToFile(result);
        }
    } else {
        ROS_DEBUG("[Node] Realtime candidate rejected. keep_best=%s", buildCalibResultSummary(kept_result).c_str());
        publishStatus("Realtime candidate rejected. keep_best: " + buildCalibResultSummary(kept_result));
    }
    return true;
}
void ImageLidarCalibNode::calibrationLoop()
{
    const double rate_hz = calibration_rate_hz_ > 0.0 ? calibration_rate_hz_ : 10.0;
    ros::Rate rate(rate_hz);
    while (ros::ok()) {
        if (!realtime_calibration_) {
            rate.sleep();
            continue;
        }

        size_t pending_scene_cache_version = 0;
        {
            std::lock_guard<std::mutex> lock(scene_mutex_);
            if (scene_cache_version_ <= calibrated_scene_cache_version_) {
                pending_scene_cache_version = 0;
            } else {
                pending_scene_cache_version = scene_cache_version_;
                calibrated_scene_cache_version_ = scene_cache_version_;
            }
        }

        if (pending_scene_cache_version == 0) {
            rate.sleep();
            continue;
        }

        ROS_DEBUG("[Node] Realtime calibration triggered by cached scene version %zu", pending_scene_cache_version);
        updateRealtimeCalibration();
        rate.sleep();
    }
}
void ImageLidarCalibNode::projectionLoop()
{
    const double rate_hz = projection_rate_hz_ > 0.0 ? projection_rate_hz_ : 10.0;
    ros::Rate rate(rate_hz);
    while (ros::ok()) {
        if (pub_lidar_projection_image_.getNumSubscribers() == 0) {
            rate.sleep();
            continue;
        }

        CalibResult result_snapshot;
        if (getLatestResultSnapshot(result_snapshot) == false) {
            rate.sleep();
            continue;
        }

        // ===== 点云雷达渲染策略: 从最新渲染场景读取 =====
        CachedScenePair scene_pair;
        if (getLatestRenderScene(scene_pair) == false) {
            rate.sleep();
            continue;
        }

        publishLidarProjectionImage(scene_pair.image_msg,
                                    scene_pair.raw_image,
                                    scene_pair.cloud,
                                    result_snapshot);
        rate.sleep();
    }
}

void ImageLidarCalibNode::publishLidarProjectionImage(const sensor_msgs::Image::ConstPtr& image_msg,
                                                      const cv::Mat& raw_image,
                                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzi,
                                                      const CalibResult& result)
{
    if (image_msg.get() == NULL || cloud_xyzi.get() == NULL || raw_image.empty()) return;

    Eigen::Matrix3d R_d;
    Eigen::Vector3d t_d;
    Eigen::Vector3d Pcv;
    double d_roll_deg = 0.0;
    double d_pitch_deg = 0.0;
    double d_yaw_deg = 0.0;
    computeDCompatiblePose(result, R_d, t_d, Pcv, d_roll_deg, d_pitch_deg, d_yaw_deg);

    cv::Mat overlay = mergeCloudToRawImage(cam_intrinsics_,
                                           raw_image,
                                           cloud_xyzi,
                                           R_d,
                                           t_d,
                                           projection_min_depth_m_,
                                           projection_max_depth_m_);
    if (overlay.empty()) return;

    cv_bridge::CvImage out(image_msg->header, "bgr8", overlay);
    pub_lidar_projection_image_.publish(out.toImageMsg());
}

void ImageLidarCalibNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_img;
    try {
        cv_img = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("[Node] cv_bridge: %s", e.what());
        return;
    }


    CachedImageDetection cached;
    cached.image_msg = msg;
    cached.header = msg->header;
    cached.raw_image = cv_img->image.clone();
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_buffer_.push_back(cached);
        while (image_buffer_.size() > max_image_buffer_size_) {
            image_buffer_.pop_front();
        }
    }
}

bool ImageLidarCalibNode::srvTrigger(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    CalibResult result;
    if (solveScenes(result) == false) {
        CalibResult visualize_result;
        bool have_visualize_result = false;
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            if (latest_result_valid_) {
                visualize_result = latest_result_;
                have_visualize_result = true;
            }
        }
        if (have_visualize_result) {
            saveSceneVisualizations(visualize_result);
        }

        res.success = false;
        res.message = buildCalibFailureServiceSummary(result.message);
        logStageDebugReport("[Node][Trigger]", result);
        publishStatus(res.message);
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        latest_result_ = result;
        latest_result_valid_ = true;
    }
    logStageDebugReport("[Node][Trigger]", result);

    const std::string summary = buildCalibResultServiceSummary(result);
    res.success = saveResultToFile(result);
    res.message = res.success ? summary
                              : "save_failed," + summary;
    publishStatus(res.message);
    return true;
}

bool ImageLidarCalibNode::srvReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    {
        std::lock_guard<std::mutex> scene_lock(scene_mutex_);
        scene_pairs_.clear();
        scene_cache_version_ = 0;
        calibrated_scene_cache_version_ = 0;
    }
    {
        std::lock_guard<std::mutex> render_lock(render_scene_mutex_);
        latest_render_scene_ = CachedScenePair();
        latest_render_scene_valid_ = false;
    }
    history_pose_filtered_count_ = 0;
    {
        std::lock_guard<std::mutex> image_lock(image_mutex_);
        image_buffer_.clear();
    }
    {
        std::lock_guard<std::mutex> result_lock(result_mutex_);
        latest_result_ = CalibResult();
        latest_result_valid_ = false;
    }
    latest_debug_image_.release();
    res.success = true;
    res.message = "reset_ok,scene_pair_cache_cleared,image_cache_cleared,render_scene_cleared,realtime_result_cleared";
    publishStatus(res.message);
    return true;
}
bool ImageLidarCalibNode::srvSave(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res)
{
    CalibResult result;
    if (solveScenes(result) == false) {
        res.success = false;
        res.message = buildCalibFailureServiceSummary(result.message);
        logStageDebugReport("[Node][Save]", result);
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        latest_result_ = result;
        latest_result_valid_ = true;
    }
    logStageDebugReport("[Node][Save]", result);

    const std::string summary = buildCalibResultServiceSummary(result);
    res.success = saveResultToFile(result);
    res.message = res.success ? summary
                              : "save_failed," + summary;
    publishStatus(res.message);
    return true;
}
} // namespace image_lidar_calibration

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_lidar_calib_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_lidar_calibration::ImageLidarCalibNode node(nh, pnh);
    ros::spin();
    return 0;
}
