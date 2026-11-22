/**
 * @file lidar_gnss_yaw_node.cpp
 * @brief LiDAR-GNSS 联合 yaw 标定 ROS 节点
 *
 * 使用方式：
 *   1. 摆好锥桶（4-8 个，间距 1-2m，成一条直线）
 *   2. 启动节点，调用 rosservice call /lidar_gnss_yaw/start "{}"
 *   3. 缓慢驾车绕过/经过锥桶（建议不同方向 2-3 次）
 *   4. 调用 rosservice call /lidar_gnss_yaw/stop "{}" → 自动计算结果
 *   5. 查看 /lidar_gnss_yaw/status 或 /lidar_gnss_yaw/result
 *   6. 调用 rosservice call /lidar_gnss_yaw/save "{}" → 保存到文件
 *
 * 订阅：
 *   ~lidar_topic  sensor_msgs/PointCloud2   LiDAR 原始点云
 *   ~gnss_topic   nav_msgs/Odometry         GNSS 位姿（ENU 坐标系）
 *
 * GNSS 航向提取（nav_msgs/Odometry，ENU 四元数）：
 *   yaw_enu = atan2(2(wz+xy), 1-2(y²+z²))     [东=0, CCW+]
 *   heading = 90° - yaw_enu_deg               [北=0, 顺时针+]
 *
 * 发布：
 *   ~cone_cloud   sensor_msgs/PointCloud2   可视化锥桶点云
 *   ~result       geometry_msgs/Vector3Stamped  (x=yaw_cal, y=delta, z=rmse_m)
 *   ~status       std_msgs/String            状态文本
 *
 * 服务：
 *   ~start   std_srvs/Trigger   开始采集
 *   ~stop    std_srvs/Trigger   停止并计算
 *   ~save    std_srvs/Trigger   保存结果
 *   ~reset   std_srvs/Trigger   重置
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/utils.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

#include "lidar_gnss_yaw/types.h"
#include "lidar_gnss_yaw/cone_detector.h"
#include "lidar_gnss_yaw/yaw_calibrator.h"

namespace lidar_gnss_yaw {

enum class State { IDLE, COLLECTING, COMPUTING, DONE };

// ==========================================================================
class LidarGnssYawNode {
public:
    LidarGnssYawNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        loadParams(pnh);

        cone_detector_ = std::make_shared<ConeDetector>(params_);
        calibrator_    = std::make_shared<YawCalibrator>(params_);

        // ---- 订阅 ----
        lidar_sub_ = nh.subscribe(lidar_topic_, 1,
                                  &LidarGnssYawNode::lidarCb, this);
        gnss_sub_  = nh.subscribe(gnss_topic_, 20,
                                  &LidarGnssYawNode::gnssCb, this);

        // ---- 发布 ----
        cone_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("cone_cloud", 1);
        result_pub_     = pnh.advertise<geometry_msgs::Vector3Stamped>("result", 1, true);
        status_pub_     = pnh.advertise<std_msgs::String>("status", 1, true);

        // ---- 服务 ----
        start_srv_ = pnh.advertiseService("start", &LidarGnssYawNode::onStart, this);
        stop_srv_  = pnh.advertiseService("stop",  &LidarGnssYawNode::onStop,  this);
        save_srv_  = pnh.advertiseService("save",  &LidarGnssYawNode::onSave,  this);
        reset_srv_ = pnh.advertiseService("reset", &LidarGnssYawNode::onReset, this);

        pubStatus("IDLE: call ~start 开始采集");
        ROS_INFO("[LidarGnssYaw] 节点启动。lidar=%s, gnss=%s",
                 lidar_topic_.c_str(), gnss_topic_.c_str());
        ROS_INFO("[LidarGnssYaw] 先验安装 yaw=%.3f°, LiDAR 在车体(%.2f, %.2f)m",
                 params_.yaw_install, params_.x_lb, params_.y_lb);
    }

private:
    // ---- 参数 ----
    CalibParams params_;
    std::string lidar_topic_;
    std::string gnss_topic_;
    std::string save_path_;

    // ---- ROS ----
    ros::Subscriber lidar_sub_, gnss_sub_;
    ros::Publisher  cone_cloud_pub_, result_pub_, status_pub_;
    ros::ServiceServer start_srv_, stop_srv_, save_srv_, reset_srv_;

    // ---- 运行状态 ----
    State       state_{State::IDLE};
    GnssPose    latest_gnss_;
    ros::Time   last_snap_time_{0};
    CalibResult last_result_;

    std::shared_ptr<ConeDetector>  cone_detector_;
    std::shared_ptr<YawCalibrator> calibrator_;

    // ------------------------------------------------------------------
    void loadParams(ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("lidar_topic", lidar_topic_, "/rslidar_points");
        pnh.param<std::string>("gnss_topic",  gnss_topic_,  "/sensorgps");
        pnh.param<std::string>("save_path",   save_path_,
                               "/tmp/lidar_gnss_yaw_result.yaml");

        pnh.param("installation/x_offset",    params_.x_lb,         0.0);
        pnh.param("installation/y_offset",    params_.y_lb,         0.0);
        pnh.param("installation/z_offset",    params_.z_lb,         1.8);
        pnh.param("installation/axis_offset", params_.z_offset,     0.37);
        pnh.param("installation/yaw_install", params_.yaw_install,  0.0);

        pnh.param("cone/height_min",         params_.cone_height_min,   0.15);
        pnh.param("cone/height_max",         params_.cone_height_max,   0.90);
        pnh.param("cone/radius_max",         params_.cone_radius_max,   0.35);
        pnh.param("cone/max_range",          params_.cone_max_range,    25.0);
        pnh.param("cone/min_range",          params_.cone_min_range,    1.0);
        pnh.param("cone/voxel_size",         params_.voxel_size,        0.05);
        pnh.param("cone/cluster_tolerance",  params_.cluster_tolerance, 0.25);
        pnh.param("cone/min_cluster_points", params_.min_cluster_points, 5);
        pnh.param("cone/max_cluster_points", params_.max_cluster_points, 500);

        pnh.param("collection/min_snapshots",         params_.min_snapshots,          6);
        pnh.param("collection/max_snapshots",         params_.max_snapshots,         60);
        pnh.param("collection/min_cones_per_snapshot",params_.min_cones_per_snapshot,  3);
        pnh.param("collection/min_heading_spread",    params_.min_heading_spread,     3.0);
        pnh.param("collection/snapshot_interval",     params_.snapshot_interval,      0.5);

        pnh.param("calibration/search_range", params_.search_range,  12.0);
        pnh.param("calibration/coarse_step",  params_.coarse_step,    0.05);
        pnh.param("calibration/fine_tol",     params_.fine_tol,       0.001);
    }

    // ------------------------------------------------------------------
    // GNSS 回调：从 nav_msgs/Odometry 提取位置与航向
    //
    // 坐标系：xg = 北（Northing，UTM），yg = 东（Easting，UTM）
    //   position.x = xg（北向，m）
    //   position.y = yg（东向，m）
    //
    // 航向提取（全局帧为 xg=北，yg=东，z=上 的左手系，等效于右手 NED 的水平投影）：
    //   yaw_formula = atan2(2(wz + xy), 1 - 2(y² + z²))
    //   在 NE（x=北, y=东）坐标系中，正 yaw（绕 +z）= 从北向东旋转
    //                                               = 导航顺时针方向
    //   因此：heading（导航）= yaw_formula_deg（直接，无需偏置）
    // ------------------------------------------------------------------
    void gnssCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        const auto& q = msg->pose.pose.orientation;
        // 四元数提取 yaw（弧度）
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        latest_gnss_.xg       = msg->pose.pose.position.x;   // 北（m）
        latest_gnss_.yg       = msg->pose.pose.position.y;   // 东（m）
        latest_gnss_.heading  = yaw * 180.0 / M_PI;          // 导航航向（°）
        latest_gnss_.timestamp = msg->header.stamp.toSec();
        latest_gnss_.valid    = true;
    }

    // ------------------------------------------------------------------
    void lidarCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (state_ != State::COLLECTING) return;
        if (!latest_gnss_.valid) {
            ROS_WARN_THROTTLE(5.0, "[LidarGnssYaw] 等待 GNSS 数据...");
            return;
        }

        // 限制采样频率
        ros::Time now = msg->header.stamp;
        if ((now - last_snap_time_).toSec() < params_.snapshot_interval) return;

        // 转换点云
        PointCloudPtr cloud(new PointCloud());
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) return;

        // 锥桶检测
        std::vector<Eigen::Vector2d> cones;
        int n_cones = cone_detector_->detect(cloud, cones);

        // 发布锥桶点云（可视化）
        auto cone_cloud = cone_detector_->lastConeCloud();
        if (!cone_cloud->empty()) {
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*cone_cloud, out);
            out.header = msg->header;
            cone_cloud_pub_.publish(out);
        }

        // 锥桶数量不足，跳过
        if (n_cones < params_.min_cones_per_snapshot) {
            ROS_DEBUG_THROTTLE(1.0, "[LidarGnssYaw] 检测到 %d 个锥桶，需要 %d 个",
                               n_cones, params_.min_cones_per_snapshot);
            return;
        }

        // 记录快照
        Snapshot snap;
        snap.gnss      = latest_gnss_;
        snap.cones_vcs = cones;   // VCS 坐标（驱动已完成安装变换）
        calibrator_->addSnapshot(snap);
        last_snap_time_ = now;

        int total = calibrator_->numSnapshots();
        ROS_INFO("[LidarGnssYaw] 快照 #%d: %d 锥桶, heading=%.2f°, 跨度=%.2f°",
                 total, n_cones, latest_gnss_.heading,
                 calibrator_->headingSpread());

        std::ostringstream oss;
        oss << "COLLECTING: " << total << "/" << params_.min_snapshots
            << " 快照, 跨度=" << std::fixed << std::setprecision(1)
            << calibrator_->headingSpread() << "°";
        pubStatus(oss.str());

        // 达到最大快照数自动停止
        if (total >= params_.max_snapshots) {
            ROS_INFO("[LidarGnssYaw] 达到最大快照数 %d，自动停止并计算", params_.max_snapshots);
            runCalibration();
        }
    }

    // ------------------------------------------------------------------
    bool onStart(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        if (state_ == State::COLLECTING) {
            res.success = false;
            res.message = "已在采集中";
            return true;
        }
        calibrator_->reset();
        last_snap_time_ = ros::Time(0);
        latest_gnss_.valid = false;
        state_ = State::COLLECTING;
        pubStatus("COLLECTING: 驾车缓慢经过锥桶...");
        ROS_INFO("[LidarGnssYaw] 开始采集。请缓驾驶经过锥桶区域，建议从 2-3 个方向经过。");
        res.success = true;
        res.message = "采集已启动";
        return true;
    }

    bool onStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        if (state_ != State::COLLECTING) {
            res.success = false;
            res.message = "未在采集状态";
            return true;
        }
        runCalibration();
        res.success = last_result_.success;
        res.message = last_result_.message;
        return true;
    }

    bool onSave(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        if (!last_result_.success) {
            res.success = false;
            res.message = "无有效标定结果，请先完成标定";
            return true;
        }
        bool ok = saveResult(last_result_);
        res.success = ok;
        res.message = ok ? ("已保存到 " + save_path_) : ("保存失败: " + save_path_);
        return true;
    }

    bool onReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        calibrator_->reset();
        state_       = State::IDLE;
        last_result_ = CalibResult{};
        pubStatus("IDLE: call ~start 开始采集");
        res.success = true;
        res.message = "已重置";
        return true;
    }

    // ------------------------------------------------------------------
    void runCalibration()
    {
        state_ = State::COMPUTING;
        pubStatus("COMPUTING: 正在计算...");
        ROS_INFO("[LidarGnssYaw] 开始标定，共 %d 帧快照...",
                 calibrator_->numSnapshots());

        last_result_ = calibrator_->calibrate();

        if (last_result_.success) {
            state_ = State::DONE;

            ROS_INFO("[LidarGnssYaw] ============ 标定结果 ============");
            ROS_INFO("  先验安装 yaw:     %.4f °", last_result_.yaw_install);
            ROS_INFO("  标定后 yaw:       %.4f °", last_result_.yaw_calibrated);
            ROS_INFO("  修正量 delta_yaw: %.4f °", last_result_.delta_yaw);
            ROS_INFO("  直线拟合 RMSE:    %.4f m (%.2f cm)",
                     last_result_.rmse_m, last_result_.rmse_m * 100.0);
            ROS_INFO("  航向角跨度:       %.2f °", last_result_.heading_spread);
            ROS_INFO("  有效快照:         %d",   last_result_.num_snapshots);
            ROS_INFO("  锥桶检测总数:     %d",   last_result_.num_cones_total);
            ROS_INFO("=====================================");

            // 发布结果（x=yaw_calibrated, y=delta_yaw, z=rmse_m）
            geometry_msgs::Vector3Stamped r;
            r.header.stamp    = ros::Time::now();
            r.header.frame_id = "calibration";
            r.vector.x = last_result_.yaw_calibrated;
            r.vector.y = last_result_.delta_yaw;
            r.vector.z = last_result_.rmse_m;
            result_pub_.publish(r);

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3);
            oss << "DONE: yaw_cal=" << last_result_.yaw_calibrated
                << "° delta=" << last_result_.delta_yaw
                << "° RMSE=" << last_result_.rmse_m * 100.0 << "cm"
                << " (call ~save 保存)";
            pubStatus(oss.str());
        } else {
            state_ = State::IDLE;
            pubStatus("FAILED: " + last_result_.message);
            ROS_WARN("[LidarGnssYaw] 标定失败: %s", last_result_.message.c_str());
        }
    }

    // ------------------------------------------------------------------
    bool saveResult(const CalibResult& res)
    {
        std::ofstream f(save_path_);
        if (!f.is_open()) {
            ROS_ERROR("[LidarGnssYaw] 无法写入: %s", save_path_.c_str());
            return false;
        }

        // 时间戳
        std::time_t t = std::time(nullptr);
        char tbuf[64];
        std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        f << std::fixed << std::setprecision(6);
        f << "# LiDAR-GNSS 联合 yaw 标定结果\n";
        f << "# 生成时间: " << tbuf << "\n";
        f << "# 标定精度要求: < 0.2°\n\n";

        f << "installation:\n";
        f << "  yaw_install:    " << res.yaw_install    << "  # 先验安装值（°）\n";
        f << "  yaw_calibrated: " << res.yaw_calibrated << "  # 标定后 yaw（°），CCW正，车体坐标系\n";
        f << "  delta_yaw:      " << res.delta_yaw      << "  # 修正量 = calibrated - install（°）\n\n";

        f << "quality:\n";
        f << "  rmse_m:          " << res.rmse_m         << "  # 直线拟合 RMSE（m）\n";
        f << "  rmse_cm:         " << res.rmse_m * 100.0 << "  # 直线拟合 RMSE（cm）\n";
        f << "  heading_spread:  " << res.heading_spread  << "  # 航向角跨度（°）\n";
        f << "  num_snapshots:   " << res.num_snapshots   << "\n";
        f << "  num_cones_total: " << res.num_cones_total << "\n";

        f.close();
        ROS_INFO("[LidarGnssYaw] 结果已保存: %s", save_path_.c_str());
        return true;
    }

    void pubStatus(const std::string& msg)
    {
        std_msgs::String s;
        s.data = msg;
        status_pub_.publish(s);
    }
};

}  // namespace lidar_gnss_yaw

// ==========================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_gnss_yaw_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    lidar_gnss_yaw::LidarGnssYawNode node(nh, pnh);
    ros::spin();
    return 0;
}
