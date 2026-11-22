/**
 * lidar_rp_cali_clicked_node.cpp
 *
 * 基于 RViz Publish Point 的 LiDAR Roll/Pitch/Z 手动标定节点
 *
 * 用法：
 *   1. 在 RViz 中将 Fixed Frame 设置为点云所在帧（通常为 base_link / VCS）
 *   2. 加载并显示 LiDAR 点云话题
 *   3. 使用 RViz 工具栏 "Publish Point" 点击地面上的若干点（建议 >=6 个）
 *   4. 节点自动收集点、实时拟合平面、计算 roll/pitch/Z
 *   5. 调用 /lidar_rp_cali_clicked/save_calibration 服务保存结果
 *
 * 订阅：
 *   /clicked_point  (geometry_msgs/PointStamped)  — RViz 发布的点击点
 *
 * 发布：
 *   ~clicked_cloud  (sensor_msgs/PointCloud2)     — 已收集的点击点（绿色标记）
 *   ~result         (geometry_msgs/Vector3Stamped) — roll_deg, pitch_deg, confidence
 *   ~status         (std_msgs/String)              — 文字状态信息
 *
 * 服务：
 *   ~trigger_calibration  — 打印/返回当前标定结果
 *   ~reset_calibration    — 清空已收集的点击点，重置标定
 *   ~save_calibration     — 将结果写入 yaml 文件
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
#include <deque>
#include <vector>
#include <cmath>
#include <numeric>
#include <sstream>

#include "lidar_rp_cali/plane_fitter.h"
#include "lidar_rp_cali/calibration_calculator.h"
#include "lidar_rp_cali/types.h"
#include "lidar_shm_param.h"

namespace lidar_rp_cali {

class LidarRpCaliClickedNode {
public:
    LidarRpCaliClickedNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), min_clicked_points_(4), z_window_size_(10), min_confidence_(0.3)
    {
        loadParams();

        sub_clicked_ = nh_.subscribe(
            "/clicked_point", 10,
            &LidarRpCaliClickedNode::clickedPointCallback, this);

        pub_clicked_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("clicked_cloud", 1, true);
        pub_result_        = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_        = pnh_.advertise<std_msgs::String>("status", 10);

        srv_trigger_ = pnh_.advertiseService(
            "trigger_calibration", &LidarRpCaliClickedNode::triggerCalibration, this);
        srv_reset_ = pnh_.advertiseService(
            "reset_calibration", &LidarRpCaliClickedNode::resetCalibration, this);
        srv_save_ = pnh_.advertiseService(
            "save_calibration", &LidarRpCaliClickedNode::saveCalibrationSrv, this);

        ROS_INFO("[LidarRpCaliClicked] 初始化完成，等待 /clicked_point 点击点");
        ROS_INFO("[LidarRpCaliClicked] 安装参数: z=%.3f roll=%.2f° pitch=%.2f° yaw=%.2f° z_offset=%.3f",
                 install_params_.z, install_params_.roll_deg,
                 install_params_.pitch_deg, install_params_.yaw_deg,
                 install_params_.z_offset);
        ROS_INFO("[LidarRpCaliClicked] 最少需要 %d 个点才开始计算，输出文件: %s",
                 min_clicked_points_, calib_output_file_.c_str());
        ROS_INFO("[LidarRpCaliClicked] 请在 RViz 中将 Fixed Frame 设为点云所在帧（通常为 base_link），");
        ROS_INFO("[LidarRpCaliClicked] 然后用 'Publish Point' 工具点击地面点。");

        publishStatus("等待点击地面点（已收集: 0 / 需要: " +
                      std::to_string(min_clicked_points_) + "）");
    }

private:
    // -----------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
        pnh_.param<std::string>("base_frame",  base_frame_,  "base_link");
        pnh_.param<std::string>("calibration_output_file", calib_output_file_,
                                std::string("/tmp/lidar_calibration_clicked.yaml"));
        pnh_.param("min_clicked_points", min_clicked_points_, 4);

        // 安装参数
        pnh_.param<std::string>("installation/sensor_name",
                                install_params_.sensor_name, std::string("lidar"));
        pnh_.param("installation/x",         install_params_.x,         0.0);
        pnh_.param("installation/y",         install_params_.y,         0.0);
        pnh_.param("installation/z",         install_params_.z,         1.8);
        pnh_.param("installation/roll_deg",  install_params_.roll_deg,  0.0);
        pnh_.param("installation/pitch_deg", install_params_.pitch_deg, 0.0);
        pnh_.param("installation/yaw_deg",   install_params_.yaw_deg,   0.0);
        pnh_.param("installation/z_offset",  install_params_.z_offset,  0.37);

        // 平面拟合
        PlaneFitter::Params pf_params;
        std::string method_str;
        pnh_.param<std::string>("plane/method", method_str, "LEAST_SQUARES");
        if (method_str == "RANSAC")
            pf_params.method = FitMethod::RANSAC;
        else if (method_str == "PCA")
            pf_params.method = FitMethod::PCA;
        else
            pf_params.method = FitMethod::LEAST_SQUARES;
        pnh_.param("plane/ransac_distance_threshold",
                   pf_params.ransac_distance_threshold, 0.05f);
        pnh_.param("plane/ransac_max_iterations",
                   pf_params.ransac_max_iterations, 200);
        plane_fitter_.setParams(pf_params);

        // 标定计算
        CalibrationCalculator::Params cc_params;
        pnh_.param("cali/max_roll_deg",   cc_params.max_roll_deg,   15.0);
        pnh_.param("cali/max_pitch_deg",  cc_params.max_pitch_deg,  15.0);
        pnh_.param("cali/window_size",    cc_params.window_size,    50);  // 手动模式窗口放大
        pnh_.param("cali/min_confidence", cc_params.min_confidence, 0.0); // 手动模式不过滤
        pnh_.param("cali/rms_weight",     cc_params.rms_weight,     0.5f);
        cali_calc_.setParams(cc_params);

        z_window_size_  = 50;
        min_confidence_ = cc_params.min_confidence;
    }

    // -----------------------------------------------------------------------
    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
    {
        // 记录点击点（转为 PCL 格式存储，intensity 用点序号标记）
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(msg->point.x);
        pt.y = static_cast<float>(msg->point.y);
        pt.z = static_cast<float>(msg->point.z);
        pt.intensity = static_cast<float>(clicked_points_.size() + 1);
        clicked_points_.push_back(pt);

        int n = static_cast<int>(clicked_points_.size());
        ROS_INFO("[LidarRpCaliClicked] 收到第 %d 个点 [%.3f, %.3f, %.3f] frame=%s",
                 n, pt.x, pt.y, pt.z, msg->header.frame_id.c_str());

        // 发布已收集点云（用于 RViz 可视化）
        publishClickedCloud(msg->header);

        if (n < min_clicked_points_) {
            publishStatus("已收集 " + std::to_string(n) + " / " +
                          std::to_string(min_clicked_points_) + " 个点，继续点击地面...");
            return;
        }

        // 用当前所有点拟合平面并计算标定
        computeAndUpdate(msg->header.stamp);
    }

    // -----------------------------------------------------------------------
    // 将所有收集点拟合平面 → 更新 Z 窗口 → 计算标定结果
    void computeAndUpdate(const ros::Time& stamp)
    {
        // 将收集的点转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(clicked_points_));

        // 平面拟合
        PlaneModel plane;
        if (!plane_fitter_.fitPlane(cloud, plane)) {
            publishStatus("平面拟合失败（共 " + std::to_string(cloud->size()) + " 个点），请继续添加点");
            return;
        }

        // 更新 Z 窗口
        updateZGroundWindow(plane);

        // 标定计算（roll/pitch 残余）
        CalibrationResult result = cali_calc_.calculate(plane);
        if (!result.valid) {
            publishStatus("标定计算失败: " + result.message);
            return;
        }

        // 跟踪 roll/pitch 窗口
        roll_window_deg_.push_back(result.roll_deg);
        pitch_window_deg_.push_back(result.pitch_deg);
        if (static_cast<int>(roll_window_deg_.size()) > z_window_size_) {
            roll_window_deg_.pop_front();
            pitch_window_deg_.pop_front();
        }

        // 发布结果 & TF
        publishResult(result, stamp);
        broadcastTF(result, stamp);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "OK | 共 " << clicked_points_.size() << " 点"
            << " | roll=" << result.roll_deg << "°"
            << " pitch=" << result.pitch_deg << "°"
            << " z_gnd=" << getAveragedZGround() << "m"
            << " rms=" << plane.rms_error << "m"
            << " conf=" << result.confidence;
        publishStatus(oss.str());
    }

    // -----------------------------------------------------------------------
    // 发布已收集的点击点云
    void publishClickedCloud(const std_msgs::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(clicked_points_));
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = header;
        pub_clicked_cloud_.publish(msg);
    }

    // -----------------------------------------------------------------------
    // 从平面拟合结果更新地面高度窗口
    void updateZGroundWindow(const PlaneModel& plane)
    {
        if (!plane.valid || plane.normal.z() < 0.5f) return;
        double z_ground = -static_cast<double>(plane.coefficients[3]) /
                           static_cast<double>(plane.normal.z());
        z_ground_window_.push_back(z_ground);
        if (static_cast<int>(z_ground_window_.size()) > z_window_size_) {
            z_ground_window_.pop_front();
        }
    }

    double getAveragedZGround() const
    {
        if (z_ground_window_.empty()) return -install_params_.z_offset;
        double sum = 0.0;
        for (double v : z_ground_window_) sum += v;
        return sum / static_cast<double>(z_ground_window_.size());
    }

    // -----------------------------------------------------------------------
    static void computeMeanStd(const std::deque<double>& data,
                               double& mean, double& std_dev)
    {
        if (data.empty()) { mean = 0; std_dev = 0; return; }
        mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        double sq_sum = 0.0;
        for (double v : data) sq_sum += (v - mean) * (v - mean);
        std_dev = std::sqrt(sq_sum / data.size());
    }

    // -----------------------------------------------------------------------
    static std::string gradeCalibration(double confidence,
                                        double std_roll, double std_pitch,
                                        double std_z, int n_points)
    {
        // 手动模式：以点数代替帧数阈值；std 阈值放宽（每次+点都会导致轻微波动）
        if (confidence >= 0.80 &&
            std_roll   <= 0.15 && std_pitch <= 0.15 &&
            std_z      <= 0.008 && n_points >= 8)
            return "EXCELLENT";

        if (confidence >= 0.60 &&
            std_roll   <= 0.30 && std_pitch <= 0.30 &&
            std_z      <= 0.015 && n_points >= 5)
            return "GOOD";

        if (confidence >= 0.40 &&
            std_roll   <= 0.70 && std_pitch <= 0.70 &&
            std_z      <= 0.030 && n_points >= 3)
            return "ACCEPTABLE";

        return "POOR";
    }

    // -----------------------------------------------------------------------
    // 四元数合成融合欧拉角（与 lidar_rp_cali_node 相同逻辑）
    static void computeFusedEuler(const InstallationParams& ip,
                                  double delta_roll_deg, double delta_pitch_deg,
                                  double& roll_fused, double& pitch_fused,
                                  double& yaw_fused)
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
                                  std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
        roll_fused  = std::atan2(R(2, 1), R(2, 2));
        yaw_fused   = std::atan2(R(1, 0), R(0, 0));

        roll_fused  *= R2D;
        pitch_fused *= R2D;
        yaw_fused   *= R2D;
    }

    // -----------------------------------------------------------------------
    double computeCalibratedZ(double z_ground_measured) const
    {
        return install_params_.z - (z_ground_measured + install_params_.z_offset);
    }

    // -----------------------------------------------------------------------
    void publishResult(const CalibrationResult& r, const ros::Time& stamp)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = lidar_frame_;
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
        ROS_INFO("[LidarRpCaliClicked] %s", s.c_str());
    }

    void broadcastTF(const CalibrationResult& r, const ros::Time& stamp)
    {
        if (!r.valid) return;
        geometry_msgs::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = base_frame_;
        ts.child_frame_id  = lidar_frame_ + "_calibrated";
        ts.transform.translation.x = 0;
        ts.transform.translation.y = 0;
        ts.transform.translation.z = 0;
        ts.transform.rotation.x = r.quaternion.x();
        ts.transform.rotation.y = r.quaternion.y();
        ts.transform.rotation.z = r.quaternion.z();
        ts.transform.rotation.w = r.quaternion.w();
        tf_broadcaster_.sendTransform(ts);
    }

    // -----------------------------------------------------------------------
    bool triggerCalibration(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        if (static_cast<int>(clicked_points_.size()) < min_clicked_points_) {
            res.success = false;
            res.message = "点击点数量不足（" + std::to_string(clicked_points_.size()) +
                          " / " + std::to_string(min_clicked_points_) + "），请继续点击地面";
            return true;
        }
        CalibrationResult avg = cali_calc_.getAveraged();
        if (!avg.valid) {
            res.success = false;
            res.message = "暂无有效标定数据";
            return true;
        }
        double z_ground = getAveragedZGround();
        double z_calib  = computeCalibratedZ(z_ground);
        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(install_params_, avg.roll_deg, avg.pitch_deg,
                          roll_f, pitch_f, yaw_f);
        res.success = true;
        res.message =
            "点数=" + std::to_string(clicked_points_.size()) +
            " Z=" + std::to_string(z_calib) + "m"
            " Roll=" + std::to_string(roll_f) + "deg"
            " Pitch=" + std::to_string(pitch_f) + "deg"
            " conf=" + std::to_string(avg.confidence);
        ROS_INFO("[LidarRpCaliClicked] trigger: %s", res.message.c_str());
        return true;
    }

    bool resetCalibration(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& res)
    {
        clicked_points_.clear();
        cali_calc_.reset();
        z_ground_window_.clear();
        roll_window_deg_.clear();
        pitch_window_deg_.clear();
        res.success = true;
        res.message = "已清空点击点和标定数据";
        ROS_INFO("[LidarRpCaliClicked] %s", res.message.c_str());
        publishStatus("已重置，等待重新点击地面点（需要 ≥ " +
                      std::to_string(min_clicked_points_) + " 个）");
        return true;
    }

    // -----------------------------------------------------------------------
    bool saveCalibrationSrv(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        int n = static_cast<int>(clicked_points_.size());
        if (n < min_clicked_points_ || z_ground_window_.empty()) {
            res.success = false;
            res.message = n < min_clicked_points_ ?
                          "点击点不足（" + std::to_string(n) + " / " +
                          std::to_string(min_clicked_points_) + "）" :
                          "暂无 z_ground 数据";
            return true;
        }

        // 用最新的所有点重拟合一次，作为最终结果
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>(clicked_points_));
        PlaneModel final_plane;
        if (!plane_fitter_.fitPlane(cloud, final_plane)) {
            res.success = false;
            res.message = "最终平面拟合失败";
            return true;
        }
        CalibrationResult final_result = cali_calc_.calculate(final_plane);
        if (!final_result.valid) {
            res.success = false;
            res.message = "最终标定计算失败: " + final_result.message;
            return true;
        }

        CalibrationOutput out;
        out.installation      = install_params_;
        out.timestamp         = getCurrentTimestamp();

        out.delta_roll_deg    = final_result.roll_deg;
        out.delta_pitch_deg   = final_result.pitch_deg;
        out.z_ground_measured = getAveragedZGround();
        out.z_ground_expected = -install_params_.z_offset;
        out.delta_z           = -(out.z_ground_measured + install_params_.z_offset);

        out.z_calib = computeCalibratedZ(out.z_ground_measured);
        computeFusedEuler(install_params_,
                          out.delta_roll_deg, out.delta_pitch_deg,
                          out.roll_calib_deg, out.pitch_calib_deg,
                          out.yaw_fused_deg);

        out.confidence  = final_result.confidence;
        out.frames_used = n;  // 手动模式下 frames_used = 点击点数量
        double mean_r, mean_p, mean_z;
        computeMeanStd(roll_window_deg_,   mean_r, out.std_roll_deg);
        computeMeanStd(pitch_window_deg_,  mean_p, out.std_pitch_deg);
        computeMeanStd(z_ground_window_,   mean_z, out.std_z_m);
        out.quality_grade = gradeCalibration(out.confidence,
                                             out.std_roll_deg, out.std_pitch_deg,
                                             out.std_z_m, n);

        if (!writeCalibrationFile(out, n, final_plane.rms_error)) {
            res.success = false;
            res.message = "写文件失败: " + calib_output_file_;
            return true;
        }

        res.success = true;
        res.message = "[" + out.quality_grade + "] 已保存至 " + calib_output_file_ +
                      " | z=" + std::to_string(out.z_calib) +
                      " roll=" + std::to_string(out.roll_calib_deg) + "deg" +
                      " pitch=" + std::to_string(out.pitch_calib_deg) + "deg" +
                      " | 点数=" + std::to_string(n) +
                      " rms=" + std::to_string(final_plane.rms_error) + "m";
        ROS_INFO("[LidarRpCaliClicked] %s", res.message.c_str());

        LidarShmParam::SetModuleParam_roll_pitch_z(install_params_.sensor_name, out.z_calib, out.roll_calib_deg, out.pitch_calib_deg);
        return true;
    }

    // -----------------------------------------------------------------------
    bool writeCalibrationFile(const CalibrationOutput& out,
                              int n_clicked, float plane_rms) const
    {
        std::ofstream f(calib_output_file_);
        if (!f.is_open()) {
            ROS_ERROR("[LidarRpCaliClicked] 无法打开文件: %s", calib_output_file_.c_str());
            return false;
        }

        const InstallationParams& ip = out.installation;
        f << std::fixed << std::setprecision(6);

        f << "schema_version: 1\n";
        f << "calib_type: lidar_rp\n";
        f << "sensor_name: " << ip.sensor_name << "\n";
        f << "generated_at: " << out.timestamp << "\n";
        f << "method: rviz_clicked_ground_plane\n";
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
        f << "  z: " << out.z_calib << "\n";
        f << "  roll_deg: " << out.roll_calib_deg << "\n";
        f << "  pitch_deg: " << out.pitch_calib_deg << "\n";
        f << "  yaw_deg: " << out.yaw_fused_deg << "\n";
        f << "delta:\n";
        f << "  delta_z: " << out.delta_z << "\n";
        f << "  delta_roll_deg: " << out.delta_roll_deg << "\n";
        f << "  delta_pitch_deg: " << out.delta_pitch_deg << "\n";
        f << "quality:\n";
        f << "  quality_grade: " << out.quality_grade << "\n";
        f << "  confidence: " << out.confidence << "\n";
        f << "  plane_rms_m: " << plane_rms << "\n";
        f << "  n_clicked_points: " << n_clicked << "\n";
        f << "  std_roll_deg: " << out.std_roll_deg << "\n";
        f << "  std_pitch_deg: " << out.std_pitch_deg << "\n";
        f << "  std_z_m: " << out.std_z_m << "\n";
        f << "  z_ground_measured: " << out.z_ground_measured << "\n";
        f << "  z_ground_expected: " << out.z_ground_expected << "\n";

        f.close();
        return true;
    }

    // -----------------------------------------------------------------------
    static std::string getCurrentTimestamp()
    {
        std::time_t t = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
        return std::string(buf);
    }

    // -----------------------------------------------------------------------
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_clicked_;
    ros::Publisher  pub_clicked_cloud_, pub_result_, pub_status_;
    ros::ServiceServer srv_trigger_, srv_reset_, srv_save_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string lidar_frame_;
    std::string base_frame_;
    std::string calib_output_file_;
    int min_clicked_points_;

    InstallationParams    install_params_;
    PlaneFitter           plane_fitter_;
    CalibrationCalculator cali_calc_;

    // 已收集的点击点
    pcl::PointCloud<pcl::PointXYZI> clicked_points_;

    // 滑动窗口（每加一个新点重拟合一次，用于稳定性评估）
    std::deque<double> z_ground_window_;
    std::deque<double> roll_window_deg_;
    std::deque<double> pitch_window_deg_;
    int    z_window_size_;
    double min_confidence_;
};

} // namespace lidar_rp_cali

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_rp_cali_clicked");
    ros::NodeHandle nh, pnh("~");
    lidar_rp_cali::LidarRpCaliClickedNode node(nh, pnh);
    ros::spin();
    return 0;
}
