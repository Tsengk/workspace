/**
 * radar_cali_node.cpp
 *
 * 毫米波雷达外参标定 ROS 节点
 * 订阅雷达点云（sensor_msgs/PointCloud2），支持两种标定模式：
 *   1. 地面平面拟合标定（估计 roll/pitch，z 由安装参数给定）
 *   2. 目标物标定（若给定参考点云话题，使用 SVD 求解 R,t）
 *
 * 服务：
 *   trigger_calibration — 触发标定
 *   reset_calibration   — 清空已累积的点云/数据
 *   save_calibration    — 标定并保存结果
 *
 * 话题（发布）：
 *   calibrated_cloud (sensor_msgs/PointCloud2) — 坐标变换后的雷达点云（VCS）
 *   result (geometry_msgs/Vector3Stamped)      — (roll_deg, pitch_deg, yaw_deg)
 *   status (std_msgs/String)                   — 当前状态信息
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <fstream>
#include <iomanip>
#include <ctime>
#include <deque>
#include <mutex>
#include <cmath>

#include "radar_cali/types.h"
#include "radar_cali/radar_calibrator.h"

namespace radar_cali {

// ============================================================
// RadarCaliNode
// ============================================================
class RadarCaliNode {
public:
    RadarCaliNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), calibrating_(false), frame_count_(0)
    {
        loadParams();
        setupSubscribersAndPublishers();

        ROS_INFO("[RadarCaliNode] init OK");
        ROS_INFO("[RadarCaliNode] radar_topic=%s", radar_topic_.c_str());
        ROS_INFO("[RadarCaliNode] install: name=%s x=%.2f y=%.2f z=%.2f rpy=[%.1f,%.1f,%.1f]deg",
                 install_params_.radar_name.c_str(),
                 install_params_.x, install_params_.y, install_params_.z,
                 install_params_.roll_deg, install_params_.pitch_deg, install_params_.yaw_deg);
        ROS_INFO("[RadarCaliNode] save_path=%s", save_path_.c_str());
    }

private:
    // ---- ROS handles ----
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    // ---- 标定器 ----
    RadarCalibrator calibrator_;

    // ---- 数据缓存 ----
    std::vector<Eigen::Vector3d> accumulated_pts_;
    std::mutex pts_mutex_;
    int frame_count_;
    bool calibrating_;

    // ---- 已标定结果 ----
    RadarCalibResult last_result_;
    bool has_result_{false};

    // ---- 发布器 ----
    ros::Publisher pub_calibrated_cloud_;
    ros::Publisher pub_result_;
    ros::Publisher pub_status_;

    // ---- 服务 ----
    ros::ServiceServer srv_trigger_;
    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_save_;

    // ---- 订阅器 ----
    ros::Subscriber sub_radar_;

    // ---- 参数 ----
    std::string  radar_topic_;
    std::string  save_path_;
    int          accumulate_frames_;   // 累积多少帧再触发自动标定
    int          min_frames_to_cali_;  // 最少几帧才能标定
    InstallParams install_params_;

    // ------------------------------------------------------------------
    // loadParams
    // ------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("radar_topic", radar_topic_, "/radar_points");
        pnh_.param<std::string>("save_path",   save_path_,   "/tmp/radar_cali_result.yaml");
        pnh_.param<int>("accumulate_frames",   accumulate_frames_,   30);
        pnh_.param<int>("min_frames_to_cali",  min_frames_to_cali_,  10);

        // 安装参数
        pnh_.param<std::string>("installation/radar_name",  install_params_.radar_name,  "radar");
        pnh_.param<double>("installation/x",         install_params_.x,         4.0);
        pnh_.param<double>("installation/y",         install_params_.y,         0.0);
        pnh_.param<double>("installation/z",         install_params_.z,         0.5);
        pnh_.param<double>("installation/roll_deg",  install_params_.roll_deg,  0.0);
        pnh_.param<double>("installation/pitch_deg", install_params_.pitch_deg, 0.0);
        pnh_.param<double>("installation/yaw_deg",   install_params_.yaw_deg,   0.0);

        // 标定器参数
        RadarCalibrator::Params cp;
        pnh_.param<double>("calibrator/max_range",          cp.max_range,          100.0);
        pnh_.param<double>("calibrator/min_range",          cp.min_range,          1.0);
        pnh_.param<double>("calibrator/roi_x_min",          cp.roi_x_min,          -20.0);
        pnh_.param<double>("calibrator/roi_x_max",          cp.roi_x_max,          20.0);
        pnh_.param<double>("calibrator/roi_y_min",          cp.roi_y_min,          -10.0);
        pnh_.param<double>("calibrator/roi_y_max",          cp.roi_y_max,          10.0);
        pnh_.param<double>("calibrator/roi_z_min",          cp.roi_z_min,          -3.0);
        pnh_.param<double>("calibrator/roi_z_max",          cp.roi_z_max,          5.0);
        pnh_.param<double>("calibrator/ground_height",      cp.ground_height,      -0.37);
        pnh_.param<double>("calibrator/ground_tolerance",   cp.ground_tolerance,   0.1);
        pnh_.param<int>("calibrator/min_points",            cp.min_points,         50);
        pnh_.param<double>("calibrator/max_iterations",     cp.max_iterations,     1000);
        pnh_.param<double>("calibrator/distance_threshold", cp.distance_threshold, 0.1);
        calibrator_.setParams(cp);
    }

    // ------------------------------------------------------------------
    // setupSubscribersAndPublishers
    // ------------------------------------------------------------------
    void setupSubscribersAndPublishers()
    {
        sub_radar_ = nh_.subscribe(radar_topic_, 1,
                                    &RadarCaliNode::radarCallback, this);

        pub_calibrated_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("calibrated_cloud", 1);
        pub_result_           = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10, true);
        pub_status_           = pnh_.advertise<std_msgs::String>("status", 10);

        srv_trigger_ = pnh_.advertiseService("trigger_calibration",
                                              &RadarCaliNode::triggerCalibration, this);
        srv_reset_   = pnh_.advertiseService("reset_calibration",
                                              &RadarCaliNode::resetCalibration, this);
        srv_save_    = pnh_.advertiseService("save_calibration",
                                              &RadarCaliNode::saveCalibration, this);
    }

    // ------------------------------------------------------------------
    // radarCallback：把 PointCloud2 转换为 Eigen 点并累积
    // ------------------------------------------------------------------
    void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        {
            std::lock_guard<std::mutex> lk(pts_mutex_);
            for (const auto& pt : cloud->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                    continue;
                accumulated_pts_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
            }
            ++frame_count_;
        }

        // 发布变换后的点云（若已有标定结果）
        if (has_result_) {
            publishCalibratedCloud(cloud, msg->header);
        }

        // 状态更新
        if (frame_count_ % 5 == 0) {
            std::string s = "[RadarCaliNode] frames=" + std::to_string(frame_count_)
                            + " pts=" + std::to_string(accumulated_pts_.size());
            publishStatus(s);
        }
    }

    // ------------------------------------------------------------------
    // triggerCalibration 服务：地面平面标定
    // ------------------------------------------------------------------
    bool triggerCalibration(std_srvs::Trigger::Request&  /*req*/,
                             std_srvs::Trigger::Response& res)
    {
        std::vector<Eigen::Vector3d> pts;
        int frames;
        {
            std::lock_guard<std::mutex> lk(pts_mutex_);
            pts    = accumulated_pts_;
            frames = frame_count_;
        }

        if (frames < min_frames_to_cali_) {
            res.success = false;
            res.message = "Need >= " + std::to_string(min_frames_to_cali_)
                          + " frames, have " + std::to_string(frames);
            publishStatus(res.message);
            return true;
        }

        if ((int)pts.size() < 50) {
            res.success = false;
            res.message = "Insufficient points: " + std::to_string(pts.size());
            publishStatus(res.message);
            return true;
        }

        RadarCalibResult result;
        bool ok = calibrator_.calibrateGroundPlane(pts, install_params_, result);

        if (!ok || !result.valid) {
            res.success = false;
            res.message = "Calibration failed: " + result.message;
            publishStatus(res.message);
            return true;
        }

        last_result_ = result;
        has_result_  = true;

        // 发布结果（用 Vector3Stamped 传递 roll/pitch/yaw 修正量，度）
        geometry_msgs::Vector3Stamped out;
        out.header.stamp    = ros::Time::now();
        out.header.frame_id = "radar_vcs";
        out.vector.x = result.roll_deg;
        out.vector.y = result.pitch_deg;
        out.vector.z = result.yaw_deg;
        pub_result_.publish(out);

        res.success = true;
        res.message = "OK Grade=" + result.quality_grade
                      + " roll=" + std::to_string(result.roll_deg) + "deg"
                      + " pitch=" + std::to_string(result.pitch_deg) + "deg"
                      + " yaw=" + std::to_string(result.yaw_deg) + "deg";
        publishStatus(res.message);
        ROS_INFO("[RadarCaliNode] %s", res.message.c_str());

        // 自动保存
        CalibOutput cOut;
        cOut.installation = install_params_;
        cOut.result       = result;
        cOut.save_path    = save_path_;
        writeCalibFile(cOut);

        return true;
    }

    // ------------------------------------------------------------------
    // resetCalibration 服务
    // ------------------------------------------------------------------
    bool resetCalibration(std_srvs::Trigger::Request&  /*req*/,
                           std_srvs::Trigger::Response& res)
    {
        {
            std::lock_guard<std::mutex> lk(pts_mutex_);
            accumulated_pts_.clear();
            frame_count_ = 0;
        }
        calibrating_ = false;
        has_result_  = false;
        res.success  = true;
        res.message  = "Reset: accumulated points cleared.";
        publishStatus(res.message);
        ROS_INFO("[RadarCaliNode] %s", res.message.c_str());
        return true;
    }

    // ------------------------------------------------------------------
    // saveCalibration 服务：（重新）标定并保存
    // ------------------------------------------------------------------
    bool saveCalibration(std_srvs::Trigger::Request&  /*req*/,
                          std_srvs::Trigger::Response& res)
    {
        std::vector<Eigen::Vector3d> pts;
        {
            std::lock_guard<std::mutex> lk(pts_mutex_);
            pts = accumulated_pts_;
        }

        if (pts.empty()) {
            res.success = false;
            res.message = "No data accumulated.";
            return true;
        }

        RadarCalibResult result;
        bool ok = calibrator_.calibrateGroundPlane(pts, install_params_, result);
        if (!ok || !result.valid) {
            res.success = false;
            res.message = "Calibration failed: " + result.message;
            return true;
        }

        last_result_ = result;
        has_result_  = true;

        CalibOutput cOut;
        cOut.installation = install_params_;
        cOut.result       = result;
        cOut.save_path    = save_path_;
        bool saved = writeCalibFile(cOut);
        res.success = saved;
        res.message = saved ? "Saved to " + save_path_ : "Failed to open output file.";
        publishStatus(res.message);
        return true;
    }

    // ------------------------------------------------------------------
    // publishCalibratedCloud：将雷达点云变换到 VCS 并发布
    // ------------------------------------------------------------------
    void publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                const std_msgs::Header& header)
    {
        if (pub_calibrated_cloud_.getNumSubscribers() == 0) return;

        std::vector<Eigen::Vector3d> radar_pts;
        for (const auto& pt : cloud->points) {
            if (!std::isfinite(pt.x)) continue;
            radar_pts.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }

        std::vector<Eigen::Vector3d> vcs_pts;
        calibrator_.transformToVCS(radar_pts, last_result_, vcs_pts);

        pcl::PointCloud<pcl::PointXYZ> out_cloud;
        for (const auto& p : vcs_pts)
            out_cloud.points.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        out_cloud.width  = out_cloud.points.size();
        out_cloud.height = 1;

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(out_cloud, cloud_msg);
        cloud_msg.header        = header;
        cloud_msg.header.frame_id = "base_link";
        pub_calibrated_cloud_.publish(cloud_msg);
    }

    // ------------------------------------------------------------------
    // writeCalibFile：写 YAML
    // ------------------------------------------------------------------
    bool writeCalibFile(const CalibOutput& out)
    {
        std::ofstream ofs(out.save_path);
        if (!ofs.is_open()) {
            ROS_ERROR("[RadarCaliNode] Cannot open: %s", out.save_path.c_str());
            return false;
        }

        std::time_t t = std::time(nullptr);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        const RadarCalibResult& r = out.result;
        ofs << std::fixed << std::setprecision(6);
        ofs << "# Radar calibration result\n";
        ofs << "# Generated: " << buf << "\n";
        ofs << "# Quality: " << r.quality_grade << "\n\n";

        ofs << "radar_name: " << out.installation.radar_name << "\n\n";

        ofs << "installation:\n";
        ofs << "  x:     " << out.installation.x         << "\n";
        ofs << "  y:     " << out.installation.y         << "\n";
        ofs << "  z:     " << out.installation.z         << "\n";
        ofs << "  roll:  " << out.installation.roll_deg  << "\n";
        ofs << "  pitch: " << out.installation.pitch_deg << "\n";
        ofs << "  yaw:   " << out.installation.yaw_deg   << "\n\n";

        ofs << "# T_radar_to_vcs: P_vcs = R * P_radar + t\n";
        ofs << "rotation_matrix:\n";
        for (int i = 0; i < 3; ++i) {
            ofs << "  - [";
            for (int j = 0; j < 3; ++j) {
                ofs << r.R(i, j);
                if (j < 2) ofs << ", ";
            }
            ofs << "]\n";
        }
        ofs << "translation: [" << r.x_m << ", " << r.y_m << ", " << r.z_m << "]\n\n";

        ofs << "euler_zyx_deg:\n";
        ofs << "  roll:  " << r.roll_deg  << "\n";
        ofs << "  pitch: " << r.pitch_deg << "\n";
        ofs << "  yaw:   " << r.yaw_deg   << "\n\n";

        ofs << "accuracy:\n";
        ofs << "  quality_grade: " << r.quality_grade << "\n";
        ofs << "  confidence:    " << r.confidence    << "\n";

        ofs.close();
        ROS_INFO("[RadarCaliNode] Saved to: %s", out.save_path.c_str());
        return true;
    }

    // ------------------------------------------------------------------
    // publishStatus
    // ------------------------------------------------------------------
    void publishStatus(const std::string& msg_str)
    {
        std_msgs::String s;
        s.data = msg_str;
        pub_status_.publish(s);
    }
};

} // namespace radar_cali

// ============================================================
// main
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_cali_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    radar_cali::RadarCaliNode node(nh, pnh);
    ros::spin();
    return 0;
}
