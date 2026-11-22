/**
 * lidar_secondary_rp_cali_node.cpp
 *
 * 辅激光雷达 Roll/Pitch/Z 手动标定节点
 *
 * 前提条件：
 *   - 主激光雷达的 roll/pitch/z 已标定完毕
 *   - 两个雷达的驱动均已将点云变换到 VCS（车辆坐标系，FLU）
 *
 * 标定原理：
 *   步骤1 — 收集主雷达地面点：
 *     用户在 RViz 中使用 "Publish Point" 工具点击主雷达点云上的地面点，
 *     节点收集点击点并拟合主雷达地面平面（法线 n_prim, z_gnd_prim）。
 *   步骤2 — confirm_primary 服务：
 *     主雷达平面已确定，切换到辅雷达点击阶段。
 *   步骤3 — 收集辅雷达地面点：
 *     用户切换 RViz 显示到辅雷达点云，继续使用 "Publish Point" 点击地面点。
 *   步骤4 — save_calibration 服务：
 *     拟合辅雷达地面平面，计算与主雷达平面的差异：
 *       delta_roll  = roll(n_sec)  - roll(n_prim)     [消除实际地面坡度影响]
 *       delta_pitch = pitch(n_sec) - pitch(n_prim)
 *       z_calib     = z_install - (z_gnd_sec + z_offset)
 *     四元数融合 R_fused = R(delta) * R_install，输出标定 YAML。
 *
 * 订阅：
 *   /clicked_point  (geometry_msgs/PointStamped)  — RViz Publish Point 点击点
 *
 * 发布：
 *   ~primary_cloud   (sensor_msgs/PointCloud2)      — 已收集的主雷达点击点
 *   ~secondary_cloud (sensor_msgs/PointCloud2)      — 已收集的辅雷达点击点
 *   ~result          (geometry_msgs/Vector3Stamped)  — 辅雷达 roll_deg, pitch_deg, conf
 *   ~status          (std_msgs/String)               — 文字状态信息
 *
 * 服务：
 *   ~confirm_primary   — 确认主雷达点采集完成，切换到辅雷达阶段
 *   ~reset_calibration — 清空全部数据，回到阶段1
 *   ~save_calibration  — 计算并保存辅雷达标定结果到 YAML 文件
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "lidar_rp_cali/plane_fitter.h"
#include "lidar_rp_cali/calibration_calculator.h"
#include "lidar_rp_cali/types.h"
#include "lidar_shm_param.h"

namespace lidar_secondary_rp_cali {

enum class Phase {
    PRIMARY,    // 等待用户点击主雷达地面点
    SECONDARY   // 主雷达已确认，等待用户点击辅雷达地面点
};

class LidarSecondaryRpCaliNode {
public:
    LidarSecondaryRpCaliNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), min_clicked_points_(4), phase_(Phase::PRIMARY)
    {
        loadParams();

        sub_clicked_ = nh_.subscribe(
            "/clicked_point", 10,
            &LidarSecondaryRpCaliNode::clickedPointCallback, this);

        pub_primary_cloud_   = pnh_.advertise<sensor_msgs::PointCloud2>("primary_cloud",   1, true);
        pub_secondary_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("secondary_cloud", 1, true);
        pub_result_          = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_          = pnh_.advertise<std_msgs::String>("status", 10);

        srv_confirm_primary_ = pnh_.advertiseService(
            "confirm_primary", &LidarSecondaryRpCaliNode::confirmPrimary, this);
        srv_reset_ = pnh_.advertiseService(
            "reset_calibration", &LidarSecondaryRpCaliNode::resetCalibration, this);
        srv_save_ = pnh_.advertiseService(
            "save_calibration", &LidarSecondaryRpCaliNode::saveCalibration, this);

        ROS_INFO("[SecondaryRpCali] Initialization complete");
        ROS_INFO("[SecondaryRpCali] Secondary lidar installation params: z=%.3f roll=%.2f deg pitch=%.2f deg yaw=%.2f deg z_offset=%.3f",
                 install_params_.z, install_params_.roll_deg,
                 install_params_.pitch_deg, install_params_.yaw_deg,
                 install_params_.z_offset);
        ROS_INFO("[SecondaryRpCali] Minimum %d points per phase required, output file: %s",
                 min_clicked_points_, calib_output_file_.c_str());
        ROS_INFO("[SecondaryRpCali] === Phase 1/2: Display [PRIMARY LIDAR] point cloud in RViz,");
        ROS_INFO("[SecondaryRpCali]     use 'Publish Point' tool to click ground points (suggest >= %d)",
                 min_clicked_points_);
        ROS_INFO("[SecondaryRpCali]     When done, call: rosservice call /%s/confirm_primary",
                 ros::this_node::getName().c_str() + 1);

        publishStatus("[Phase 1/2] Click ground points on primary lidar cloud (collected: 0 / required: " +
                      std::to_string(min_clicked_points_) + ")");
    }

private:
    // -------------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("primary_lidar_frame",   primary_frame_,   "primary_lidar");
        pnh_.param<std::string>("secondary_lidar_frame", secondary_frame_,  "secondary_lidar");
        pnh_.param<std::string>("base_frame",            base_frame_,       "base_link");
        pnh_.param<std::string>("calibration_output_file", calib_output_file_,
                                std::string("/tmp/secondary_lidar_rp_calibration.yaml"));
        pnh_.param("min_clicked_points", min_clicked_points_, 4);

        pnh_.param<std::string>("installation/sensor_name",
                                install_params_.sensor_name, std::string("secondary_lidar"));
        pnh_.param("installation/x",         install_params_.x,         0.0);
        pnh_.param("installation/y",         install_params_.y,         0.0);
        pnh_.param("installation/z",         install_params_.z,         1.8);
        pnh_.param("installation/roll_deg",  install_params_.roll_deg,  0.0);
        pnh_.param("installation/pitch_deg", install_params_.pitch_deg, 0.0);
        pnh_.param("installation/yaw_deg",   install_params_.yaw_deg,   0.0);
        pnh_.param("installation/z_offset",  install_params_.z_offset,  0.37);

        lidar_rp_cali::PlaneFitter::Params pf;
        std::string method;
        pnh_.param<std::string>("plane/method", method, "LEAST_SQUARES");
        if (method == "RANSAC")
            pf.method = lidar_rp_cali::FitMethod::RANSAC;
        else if (method == "PCA")
            pf.method = lidar_rp_cali::FitMethod::PCA;
        else
            pf.method = lidar_rp_cali::FitMethod::LEAST_SQUARES;
        pnh_.param("plane/ransac_distance_threshold", pf.ransac_distance_threshold, 0.05f);
        pnh_.param("plane/ransac_max_iterations",     pf.ransac_max_iterations,     200);
        plane_fitter_.setParams(pf);

        lidar_rp_cali::CalibrationCalculator::Params cc;
        pnh_.param("cali/max_roll_deg",  cc.max_roll_deg,  15.0);
        pnh_.param("cali/max_pitch_deg", cc.max_pitch_deg, 15.0);
        cc.window_size    = 50;
        cc.min_confidence = 0.0;
        pnh_.param("cali/rms_weight", cc.rms_weight, 0.5f);
        cali_calc_.setParams(cc);
    }

    // -------------------------------------------------------------------------
    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
    {
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(msg->point.x);
        pt.y = static_cast<float>(msg->point.y);
        pt.z = static_cast<float>(msg->point.z);

        if (phase_ == Phase::PRIMARY) {
            pt.intensity = static_cast<float>(primary_pts_.size() + 1);
            primary_pts_.push_back(pt);
            int n = static_cast<int>(primary_pts_.size());
            ROS_INFO("[SecondaryRpCali][Primary] Received point #%d [%.3f, %.3f, %.3f]",
                     n, pt.x, pt.y, pt.z);
            publishPrimaryCloud(msg->header);

            if (n < min_clicked_points_) {
                publishStatus("[Phase 1/2 Primary] Collected " + std::to_string(n) +
                              " / " + std::to_string(min_clicked_points_) +
                              " points, keep clicking ground...");
            } else {

                // 实时预览拟合结果
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(primary_pts_));

                lidar_rp_cali::PlaneModel plane;
                if (plane_fitter_.fitPlane(cloud, plane)) {
                    double zg = planeToZGround(plane);
                    lidar_rp_cali::CalibrationResult r = cali_calc_.calculate(plane);
                    ROS_INFO("[SecondaryRpCali][primary preview] normal=[%.5f, %.5f, %.5f] d=%.5f z_gnd=%.5f rms=%.5f curv=%.6f inliers=%d roll=%.5f pitch=%.5f conf=%.3f",
                             plane.normal.x(), plane.normal.y(), plane.normal.z(), plane.coefficients[3],
                             zg, plane.rms_error, plane.curvature, plane.inliers_count,
                             r.roll_deg, r.pitch_deg, r.confidence);
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(3)
                        << "[Phase 1/2 Primary] " << n << " points"
                        << " | roll=" << r.roll_deg << " deg"
                        << " pitch=" << r.pitch_deg << " deg"
                        << " z_gnd=" << zg << "m"
                        << " rms=" << plane.rms_error << "m"
                        << " | When satisfied, call confirm_primary service";
                    publishStatus(oss.str());
                } else {
                    publishStatus("[Phase 1/2 Primary] Collected " + std::to_string(n) +
                                  " points (fitting plane...), keep adding points");
                }
            }
        } else {
            // Phase::SECONDARY
            pt.intensity = static_cast<float>(secondary_pts_.size() + 1);
            secondary_pts_.push_back(pt);
            int n = static_cast<int>(secondary_pts_.size());
            ROS_INFO("[SecondaryRpCali][Secondary] Received point #%d [%.3f, %.3f, %.3f]",
                     n, pt.x, pt.y, pt.z);
            publishSecondaryCloud(msg->header);

            if (n < min_clicked_points_) {
                publishStatus("[Phase 2/2 Secondary] Collected " + std::to_string(n) +
                              " / " + std::to_string(min_clicked_points_) +
                              " points, keep clicking ground...");
            } else {

                // 实时预览拟合结果
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(secondary_pts_));

                lidar_rp_cali::PlaneModel plane;
                if (plane_fitter_.fitPlane(cloud, plane)) {
                    double zg = planeToZGround(plane);
                    lidar_rp_cali::CalibrationResult r = cali_calc_.calculate(plane);
                    double dr = r.roll_deg  - primary_result_.roll_deg;
                    double dp = r.pitch_deg - primary_result_.pitch_deg;
                    ROS_INFO("[SecondaryRpCali][secondary preview] normal=[%.5f, %.5f, %.5f] d=%.5f z_gnd=%.5f rms=%.5f curv=%.6f inliers=%d roll=%.5f pitch=%.5f conf=%.3f delta_roll=%.5f delta_pitch=%.5f",
                             plane.normal.x(), plane.normal.y(), plane.normal.z(), plane.coefficients[3],
                             zg, plane.rms_error, plane.curvature, plane.inliers_count,
                             r.roll_deg, r.pitch_deg, r.confidence, dr, dp);
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(3)
                        << "[Phase 2/2 Secondary] " << n << " points"
                        << " | delta_roll=" << dr << " deg delta_pitch=" << dp << " deg"
                        << " z_gnd=" << zg << "m"
                        << " rms=" << plane.rms_error << "m"
                        << " | When satisfied, call save_calibration service";
                    publishStatus(oss.str());
                    publishResult(r, msg->header.stamp);
                } else {
                    publishStatus("[Phase 2/2 Secondary] Collected " + std::to_string(n) +
                                  " points (fitting plane...), keep adding points");
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    bool confirmPrimary(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        if (phase_ != Phase::PRIMARY) {
            res.success = false;
            res.message = "Already in secondary phase; call reset_calibration to restart";
            return true;
        }
        int n = static_cast<int>(primary_pts_.size());
        if (n < min_clicked_points_) {
            res.success = false;
            res.message = "Not enough primary lidar points (" + std::to_string(n) +
                          " / " + std::to_string(min_clicked_points_) + "), keep clicking ground";
            return true;
        }

        // 拟合主雷达地面平面
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(primary_pts_));
        if (!plane_fitter_.fitPlane(cloud, primary_plane_)) {
            res.success = false;
            res.message = "Primary lidar plane fit failed; add more ground points and retry";
            return true;
        }

        // 从主雷达法线提取 roll/pitch（用于后续差分消除地面坡度）
        primary_result_ = cali_calc_.calculate(primary_plane_);
        cali_calc_.reset();  // 重置窗口，准备辅雷达阶段

        double z_prim = planeToZGround(primary_plane_);
        ROS_INFO("[SecondaryRpCali] primary plane confirmed: normal=[%.5f, %.5f, %.5f] d=%.5f z_gnd=%.5f rms=%.5f curv=%.6f inliers=%d roll=%.5f pitch=%.5f conf=%.3f",
                 primary_plane_.normal.x(), primary_plane_.normal.y(), primary_plane_.normal.z(), primary_plane_.coefficients[3],
                 z_prim, primary_plane_.rms_error, primary_plane_.curvature, primary_plane_.inliers_count,
                 primary_result_.roll_deg, primary_result_.pitch_deg, primary_result_.confidence);
        phase_ = Phase::SECONDARY;

        res.success = true;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "Primary lidar plane confirmed | points=" << n
            << " roll=" << primary_result_.roll_deg << " deg"
            << " pitch=" << primary_result_.pitch_deg << " deg"
            << " z_gnd=" << z_prim << "m"
            << " rms=" << primary_plane_.rms_error << "m"
            << " | Switch to secondary lidar cloud in RViz and click ground points";
        res.message = oss.str();
        ROS_INFO("[SecondaryRpCali] confirm_primary: %s", res.message.c_str());

        publishStatus("[Phase 2/2 Secondary] Primary confirmed; click ground points on secondary lidar cloud (need >= " +
                      std::to_string(min_clicked_points_) + ")");
        return true;
    }

    // -------------------------------------------------------------------------
    bool resetCalibration(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        primary_pts_.clear();
        secondary_pts_.clear();
        primary_plane_ = lidar_rp_cali::PlaneModel();
        primary_result_ = lidar_rp_cali::CalibrationResult();
        cali_calc_.reset();
        phase_ = Phase::PRIMARY;

        res.success = true;
        res.message = "All data reset, back to phase 1 (primary lidar clicking)";
        ROS_INFO("[SecondaryRpCali] %s", res.message.c_str());
        publishStatus("[Phase 1/2] Reset complete; click ground points on primary lidar cloud (need >= " +
                      std::to_string(min_clicked_points_) + ")");
        return true;
    }

    // -------------------------------------------------------------------------
    bool saveCalibration(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        if (phase_ != Phase::SECONDARY) {
            res.success = false;
            res.message = "Primary lidar not yet confirmed (call confirm_primary first)";
            return true;
        }
        int n_sec = static_cast<int>(secondary_pts_.size());
        if (n_sec < min_clicked_points_) {
            res.success = false;
            res.message = "Not enough secondary lidar points (" + std::to_string(n_sec) +
                          " / " + std::to_string(min_clicked_points_) + ")";
            return true;
        }

        // === 求解输入打印 ===
        ROS_INFO("[SecondaryRpCali][solve-input] installation: sensor_name=%s x=%.5f y=%.5f z=%.5f roll_deg=%.5f pitch_deg=%.5f yaw_deg=%.5f z_offset=%.5f",
                 install_params_.sensor_name.c_str(),
                 install_params_.x, install_params_.y, install_params_.z,
                 install_params_.roll_deg, install_params_.pitch_deg,
                 install_params_.yaw_deg, install_params_.z_offset);
        ROS_INFO("[SecondaryRpCali][solve-input] frames: primary=%s secondary=%s base=%s | min_clicked_points=%d | output_file=%s",
                 primary_frame_.c_str(), secondary_frame_.c_str(),
                 base_frame_.c_str(), min_clicked_points_,
                 calib_output_file_.c_str());
        ROS_INFO("[SecondaryRpCali][solve-input] primary_plane (from confirm_primary): normal=[%.5f, %.5f, %.5f] d=%.5f rms=%.5f curv=%.6f inliers=%d roll=%.5f pitch=%.5f conf=%.3f",
                 primary_plane_.normal.x(), primary_plane_.normal.y(), primary_plane_.normal.z(),
                 primary_plane_.coefficients[3],
                 primary_plane_.rms_error, primary_plane_.curvature, primary_plane_.inliers_count,
                 primary_result_.roll_deg, primary_result_.pitch_deg, primary_result_.confidence);

        ROS_INFO("[SecondaryRpCali][solve-input] primary_pts count=%zu", primary_pts_.size());
        for (size_t i = 0; i < primary_pts_.size(); ++i) {
            const auto& p = primary_pts_[i];
            ROS_INFO("[SecondaryRpCali][solve-input] primary_pt[%zu] = [%.5f, %.5f, %.5f] intensity=%.1f",
                     i, p.x, p.y, p.z, p.intensity);
        }
        ROS_INFO("[SecondaryRpCali][solve-input] secondary_pts count=%zu", secondary_pts_.size());
        for (size_t i = 0; i < secondary_pts_.size(); ++i) {
            const auto& p = secondary_pts_[i];
            ROS_INFO("[SecondaryRpCali][solve-input] secondary_pt[%zu] = [%.5f, %.5f, %.5f] intensity=%.1f",
                     i, p.x, p.y, p.z, p.intensity);
        }

        // 拟合辅雷达地面平面
        pcl::PointCloud<pcl::PointXYZI>::Ptr sec_cloud(new pcl::PointCloud<pcl::PointXYZI>(secondary_pts_));
        lidar_rp_cali::PlaneModel sec_plane;
        if (!plane_fitter_.fitPlane(sec_cloud, sec_plane)) {
            res.success = false;
            res.message = "Secondary lidar plane fit failed; add more ground points and retry";
            return true;
        }

        lidar_rp_cali::CalibrationResult sec_result = cali_calc_.calculate(sec_plane);
        if (!sec_result.valid) {
            res.success = false;
            res.message = "Secondary lidar calibration computation failed: " + sec_result.message;
            return true;
        }

        // 差分消除地面坡度影响
        double delta_roll_deg  = sec_result.roll_deg  - primary_result_.roll_deg;
        double delta_pitch_deg = sec_result.pitch_deg - primary_result_.pitch_deg;

        // 地面高度
        double z_gnd_prim = planeToZGround(primary_plane_);
        double z_gnd_sec  = planeToZGround(sec_plane);

        // z 标定：用辅雷达自己的地面测量做绝对修正（与主节点公式一致）
        // 不能用 (z_gnd_prim - z_gnd_sec) 差分，因为那会把主雷达驱动参数的残差耦合进来
        double z_calib = install_params_.z - (z_gnd_sec + install_params_.z_offset);

        // 四元数融合
        double roll_fused, pitch_fused, yaw_fused;
        computeFusedEuler(install_params_, delta_roll_deg, delta_pitch_deg,
                          roll_fused, pitch_fused, yaw_fused);

        ROS_INFO("[SecondaryRpCali] final solve: primary_z=%.5f secondary_z=%.5f expected_ground=%.5f abs_delta_z=%.5f rel_delta_z=%.5f install_z=%.5f z_calib=%.5f delta_roll=%.5f delta_pitch=%.5f roll_fused=%.5f pitch_fused=%.5f yaw_fused=%.5f",
                 z_gnd_prim, z_gnd_sec, -install_params_.z_offset, -(z_gnd_sec + install_params_.z_offset),
                 (z_gnd_prim - z_gnd_sec), install_params_.z, z_calib,
                 delta_roll_deg, delta_pitch_deg, roll_fused, pitch_fused, yaw_fused);

        std::string timestamp = getCurrentTimestamp();
        int n_prim = static_cast<int>(primary_pts_.size());

        if (!writeCalibrationFile(n_prim, n_sec,
                                  z_gnd_prim, z_gnd_sec,
                                  delta_roll_deg, delta_pitch_deg,
                                  z_calib, roll_fused, pitch_fused, yaw_fused,
                                  primary_plane_.rms_error, sec_plane.rms_error,
                                  sec_result.confidence, timestamp)) {
            res.success = false;
            res.message = "Failed to write file: " + calib_output_file_;
            return true;
        }

        res.success = true;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "Saved to " << calib_output_file_
            << " | z=" << z_calib
            << " roll=" << roll_fused << " deg"
            << " pitch=" << pitch_fused << " deg"
            << " | delta_roll=" << delta_roll_deg << " deg"
            << " delta_pitch=" << delta_pitch_deg << " deg"
            << " | primary(" << n_prim << " pts, rms=" << primary_plane_.rms_error << "m)"
            << " secondary(" << n_sec << " pts, rms=" << sec_plane.rms_error << "m)";
        res.message = oss.str();
        ROS_INFO("[SecondaryRpCali] save: %s", res.message.c_str());
        publishStatus("[Done] " + res.message);

        std::cout << "zzzzzzzzzzzzzzzzzzzzzz"<<std::endl;
        LidarShmParam::SetModuleParam_roll_pitch_z(install_params_.sensor_name, z_calib, roll_fused, pitch_fused);
        std::cout << "zzzzzzzzzzzzzzzzzzzzzz "<< install_params_.sensor_name<<std::endl;
        return true;
    }

    // -------------------------------------------------------------------------
    // 从平面模型计算 VCS 地面高度 z_ground = -d / n.z
    static double planeToZGround(const lidar_rp_cali::PlaneModel& plane)
    {
        if (plane.normal.z() < 1e-3f) return 0.0;
        return -static_cast<double>(plane.coefficients[3]) /
                static_cast<double>(plane.normal.z());
    }

    // -------------------------------------------------------------------------
    // 四元数合成：R_fused = R(delta_pitch, delta_roll) * R_install
    static void computeFusedEuler(const lidar_rp_cali::InstallationParams& ip,
                                  double delta_roll_deg, double delta_pitch_deg,
                                  double& roll_fused, double& pitch_fused, double& yaw_fused)
    {
        static constexpr double D2R = M_PI / 180.0;
        static constexpr double R2D = 180.0 / M_PI;

        Eigen::Quaterniond q_install =
            Eigen::AngleAxisd(ip.yaw_deg   * D2R, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ip.pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ip.roll_deg  * D2R, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q_delta =
            Eigen::AngleAxisd(delta_pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(delta_roll_deg  * D2R, Eigen::Vector3d::UnitX());

        Eigen::Matrix3d R = (q_delta * q_install).normalized().toRotationMatrix();

        pitch_fused = std::atan2(-R(2, 0),
                                  std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))) * R2D;
        roll_fused  = std::atan2(R(2, 1), R(2, 2)) * R2D;
        yaw_fused   = std::atan2(R(1, 0), R(0, 0)) * R2D;
    }

    // -------------------------------------------------------------------------
    bool writeCalibrationFile(int n_prim, int n_sec,
                              double z_gnd_prim, double z_gnd_sec,
                              double delta_roll, double delta_pitch,
                              double z_calib, double roll_fused,
                              double pitch_fused, double yaw_fused,
                              float rms_prim, float rms_sec,
                              double confidence,
                              const std::string& timestamp) const
    {
        std::ofstream f(calib_output_file_);
        if (!f.is_open()) {
            ROS_ERROR("[SecondaryRpCali] Failed to open file: %s", calib_output_file_.c_str());
            return false;
        }
        f << std::fixed << std::setprecision(6);

        const lidar_rp_cali::InstallationParams& ip = install_params_;

        f << "schema_version: 1\n";
        f << "calib_type: lidar_secondary_rp\n";
        f << "sensor_name: " << ip.sensor_name << "\n";
        f << "generated_at: " << timestamp << "\n";
        f << "method: dual_lidar_clicked_ground_plane\n";
        f << "coordinate_system: FLU\n";
        f << "installation:\n";
        f << "  x: " << ip.x << "\n";
        f << "  y: " << ip.y << "\n";
        f << "  z: " << ip.z << "\n";
        f << "  roll_deg: " << ip.roll_deg << "\n";
        f << "  pitch_deg: " << ip.pitch_deg << "\n";
        f << "  yaw_deg: " << ip.yaw_deg << "\n";
        f << "  z_offset: " << ip.z_offset << "\n";
        f << "  lidar_to_ground_height: " << (ip.z + ip.z_offset) << "\n";
        f << "calibrated:\n";
        f << "  x: " << ip.x << "\n";
        f << "  y: " << ip.y << "\n";
        f << "  z: " << z_calib << "\n";
        f << "  roll_deg: " << roll_fused << "\n";
        f << "  pitch_deg: " << pitch_fused << "\n";
        f << "  yaw_deg: " << yaw_fused << "\n";
        f << "delta:\n";
        f << "  delta_roll_deg: " << delta_roll << "\n";
        f << "  delta_pitch_deg: " << delta_pitch << "\n";
        f << "  delta_z: " << (z_gnd_prim - z_gnd_sec) << "\n";
        f << "ground_measurement:\n";
        f << "  z_ground_primary: " << z_gnd_prim << "\n";
        f << "  z_ground_secondary: " << z_gnd_sec << "\n";
        f << "  z_ground_expected: " << -ip.z_offset << "\n";
        f << "quality:\n";
        f << "  confidence: " << confidence << "\n";
        f << "  primary_plane_rms_m: " << rms_prim << "\n";
        f << "  secondary_plane_rms_m: " << rms_sec << "\n";
        f << "  n_primary_points: " << n_prim << "\n";
        f << "  n_secondary_points: " << n_sec << "\n";

        f.close();
        return true;
    }

    // -------------------------------------------------------------------------
    void publishPrimaryCloud(const std_msgs::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(primary_pts_));
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = header;
        pub_primary_cloud_.publish(msg);
    }

    void publishSecondaryCloud(const std_msgs::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(secondary_pts_));
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = header;
        pub_secondary_cloud_.publish(msg);
    }

    void publishResult(const lidar_rp_cali::CalibrationResult& r, const ros::Time& stamp)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = secondary_frame_;
        msg.vector.x = r.roll_deg;
        msg.vector.y = r.pitch_deg;
        msg.vector.z = r.confidence;
        pub_result_.publish(msg);
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::String msg;
        msg.data = s;
        pub_status_.publish(msg);
        ROS_INFO("[SecondaryRpCali] %s", s.c_str());
    }

    // -------------------------------------------------------------------------
    static std::string getCurrentTimestamp()
    {
        std::time_t t = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
        return std::string(buf);
    }

    // -------------------------------------------------------------------------
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_clicked_;
    ros::Publisher  pub_primary_cloud_, pub_secondary_cloud_, pub_result_, pub_status_;
    ros::ServiceServer srv_confirm_primary_, srv_reset_, srv_save_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string primary_frame_;
    std::string secondary_frame_;
    std::string base_frame_;
    std::string calib_output_file_;
    int min_clicked_points_;

    lidar_rp_cali::InstallationParams    install_params_;
    lidar_rp_cali::PlaneFitter           plane_fitter_;
    lidar_rp_cali::CalibrationCalculator cali_calc_;

    Phase phase_;

    // 主雷达阶段收集的点击点和拟合结果
    pcl::PointCloud<pcl::PointXYZI>    primary_pts_;
    lidar_rp_cali::PlaneModel          primary_plane_;
    lidar_rp_cali::CalibrationResult   primary_result_;

    // 辅雷达阶段收集的点击点
    pcl::PointCloud<pcl::PointXYZI>    secondary_pts_;
};

} // namespace lidar_secondary_rp_cali

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_secondary_rp_cali");
    ros::NodeHandle nh, pnh("~");
    lidar_secondary_rp_cali::LidarSecondaryRpCaliNode node(nh, pnh);
    ros::spin();
    return 0;
}
