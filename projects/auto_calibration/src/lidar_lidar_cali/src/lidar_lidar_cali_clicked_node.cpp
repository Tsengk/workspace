/**
 * lidar_lidar_cali_clicked_node.cpp
 *
 * 基于 RViz Publish Point 的双激光雷达 x/y/yaw 手动标定节点
 *
 * 工作原理：
 *   1. 实时订阅主/辅两路 LiDAR 点云，各保存最新一帧到缓冲
 *   2. 用服务 select_primary / select_secondary 切换当前活跃雷达
 *   3. 用户在 RViz 中用 "Publish Point" 点击锥形桶
 *   4. 节点在点击位置周围 search_radius（默认 0.5m）内搜索活跃雷达点云
 *   5. 取该邻域最高点 Z（锥桶顶）及 XY 质心，存入对应雷达锥桶列表
 *   6. 当主/辅各收集的锥桶数量相同（>= min_cones）时，自动触发 2D SVD 标定
 *      - 节点会自动搜索主/辅锥桶的最佳对应关系，不依赖点击顺序
 *      - 至少 2 个锥桶可算 x/y，至少 3 个非共线锥桶可稳定求 yaw
 *
 * 订阅：
 *   primary_lidar/topic      (sensor_msgs/PointCloud2)    — 主雷达点云（VCS）
 *   secondary_lidar/topic    (sensor_msgs/PointCloud2)    — 辅雷达点云
 *   /clicked_point           (geometry_msgs/PointStamped) — RViz 点击点
 *
 * 发布：
 *   ~primary_cones           (sensor_msgs/PointCloud2)    — 主雷达已提取锥桶点云
 *   ~secondary_cones         (sensor_msgs/PointCloud2)    — 辅雷达已提取锥桶点云
 *   ~result                  (geometry_msgs/Vector3Stamped) — Δx, Δy, Δyaw_deg
 *   ~status                  (std_msgs/String)
 *   ~cone_markers            (visualization_msgs/MarkerArray)
 *
 * 服务：
 *   ~select_primary       — 将 /clicked_point 点击事件路由到主雷达
 *   ~select_secondary     — 将 /clicked_point 点击事件路由到辅雷达
 *   ~trigger_calibration  — 主动触发标定（无需等待数量相等）
 *   ~reset_calibration    — 清空主/辅锥桶列表，重置
 *   ~save_calibration     — 将结果写入 yaml 文件
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "lidar_lidar_cali/types.h"
#include "lidar_shm_param.h"

namespace lidar_lidar_cali {

// ===========================================================================
// 2D SVD 点集配准（Procrustes，求 Rz(yaw) + t）
// 输入: primary_pts, secondary_q_pts（均已去除安装偏置）
// 输出: yaw_deg（旋转角）, tx, ty（平移）
// ===========================================================================
static bool svd2dCalibrate(
    const std::vector<Eigen::Vector2d>& primary,   // 主雷达锥桶 XY（VCS）
    const std::vector<Eigen::Vector2d>& secondary_q, // 辅雷达锥桶 XY（已去除安装 x/y/yaw）
    double& yaw_deg, double& tx, double& ty, double& rmse)
{
    if (primary.size() != secondary_q.size() || primary.size() < 2) return false;
    size_t N = primary.size();

    // 计算质心
    Eigen::Vector2d cp = Eigen::Vector2d::Zero();
    Eigen::Vector2d cs = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < N; ++i) {
        cp += primary[i];
        cs += secondary_q[i];
    }
    cp /= static_cast<double>(N);
    cs /= static_cast<double>(N);

    // 构建 2×2 协方差矩阵 H
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < N; ++i)
        H += (secondary_q[i] - cs) * (primary[i] - cp).transpose();

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

    // 确保正定旋转（行列式应为 +1）
    if (R.determinant() < 0) {
        Eigen::Matrix2d V = svd.matrixV();
        V.col(1) *= -1.0;
        R = V * svd.matrixU().transpose();
    }

    Eigen::Vector2d t = cp - R * cs;

    // 计算 RMSE
    double sum_sq = 0.0;
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector2d diff = primary[i] - (R * secondary_q[i] + t);
        sum_sq += diff.squaredNorm();
    }
    rmse = std::sqrt(sum_sq / static_cast<double>(N));

    yaw_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
    tx      = t.x();
    ty      = t.y();
    return true;
}

// ===========================================================================
class LidarLidarCaliClickedNode {
public:
    LidarLidarCaliClickedNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh),
          min_cones_(2), search_radius_(0.5f), min_cluster_pts_(3),
          active_lidar_("primary"),
          calibrated_(false),
          only_use_top_point_(false)
    {
        loadParams();
        setupROS();

        ROS_INFO("[lidar_lidar_cali_clicked] 初始化完成");
        ROS_INFO("[lidar_lidar_cali_clicked] 主雷达: %s  辅雷达: %s",
                 primary_topic_.c_str(), secondary_topic_.c_str());
        ROS_INFO("[lidar_lidar_cali_clicked] 安装标称: x=%.3fm y=%.3fm yaw=%.3f°",
                 install_params_.x_install, install_params_.y_install,
                 install_params_.yaw_install_deg);
        ROS_INFO("[lidar_lidar_cali_clicked] 搜索半径=%.2fm 最少锥桶=%d only_use_top_point=%d",
                 search_radius_, min_cones_, only_use_top_point_);
        ROS_INFO("[lidar_lidar_cali_clicked] 当前活跃雷达: %s", active_lidar_.c_str());
        ROS_INFO("[lidar_lidar_cali_clicked] 用 select_primary / select_secondary 服务切换");
        publishStatus("活跃雷达=primary | 主锥桶=0 辅锥桶=0 | 点击锥桶...");
    }

private:
    // -----------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("primary_lidar/topic",   primary_topic_,   "/lidar_master/points");
        pnh_.param<std::string>("secondary_lidar/topic", secondary_topic_,  "/lidar_sub/points");
        pnh_.param<std::string>("output/file",           output_file_,
                                "/tmp/lidar_lidar_calib_clicked.yaml");
        pnh_.param("min_cones",       min_cones_,       2);
        pnh_.param("search_radius",   search_radius_,   0.5f);
        pnh_.param("min_cluster_pts", min_cluster_pts_, 3);
        pnh_.param<bool>("only_use_top_point", only_use_top_point_, false);

        // 安装参数
        pnh_.param<std::string>("secondary_lidar/sensor_name",
                                install_params_.sensor_name, "lidar_sub");
        pnh_.param("secondary_lidar/roll_calib_deg",  install_params_.roll_calib_deg,  0.0);
        pnh_.param("secondary_lidar/pitch_calib_deg", install_params_.pitch_calib_deg, 0.0);
        pnh_.param("secondary_lidar/z_calib",         install_params_.z_calib,         1.43);
        pnh_.param("secondary_lidar/z_offset",        install_params_.z_offset,        0.37);
        pnh_.param("secondary_lidar/x_install",       install_params_.x_install,       0.0);
        pnh_.param("secondary_lidar/y_install",       install_params_.y_install,       0.0);
        pnh_.param("secondary_lidar/yaw_install_deg", install_params_.yaw_install_deg, 0.0);
    }

    void setupROS()
    {
        sub_primary_ = nh_.subscribe(
            primary_topic_, 1, &LidarLidarCaliClickedNode::primaryCallback, this);
        sub_secondary_ = nh_.subscribe(
            secondary_topic_, 1, &LidarLidarCaliClickedNode::secondaryCallback, this);
        sub_clicked_ = nh_.subscribe(
            "/clicked_point", 10, &LidarLidarCaliClickedNode::clickedPointCallback, this);

        pub_primary_cloud_   = pnh_.advertise<sensor_msgs::PointCloud2>("primary_cones", 1, true);
        pub_secondary_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("secondary_cones", 1, true);
        pub_result_          = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_          = pnh_.advertise<std_msgs::String>("status", 10);
        pub_markers_         = pnh_.advertise<visualization_msgs::MarkerArray>("cone_markers", 1, true);

        srv_select_primary_ = pnh_.advertiseService(
            "select_primary",
            &LidarLidarCaliClickedNode::selectPrimary, this);
        srv_select_secondary_ = pnh_.advertiseService(
            "select_secondary",
            &LidarLidarCaliClickedNode::selectSecondary, this);
        srv_trigger_ = pnh_.advertiseService(
            "trigger_calibration",
            &LidarLidarCaliClickedNode::triggerCalibration, this);
        srv_reset_ = pnh_.advertiseService(
            "reset_calibration",
            &LidarLidarCaliClickedNode::resetCalibration, this);
        srv_save_ = pnh_.advertiseService(
            "save_calibration",
            &LidarLidarCaliClickedNode::saveCalibration, this);
    }

    // -----------------------------------------------------------------------
    void primaryCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(cloud_mutex_);
        latest_primary_ = msg;
        ROS_INFO_THROTTLE(5.0, "[lidar_lidar_cali_clicked] received primary cloud frame=%s stamp=%.3f",
                          msg->header.frame_id.c_str(), msg->header.stamp.toSec());
    }

    void secondaryCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(cloud_mutex_);
        latest_secondary_ = msg;
        ROS_INFO_THROTTLE(5.0, "[lidar_lidar_cali_clicked] received secondary cloud frame=%s stamp=%.3f",
                          msg->header.frame_id.c_str(), msg->header.stamp.toSec());
    }

    // -----------------------------------------------------------------------
    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
    {
        ROS_INFO("[lidar_lidar_cali_clicked] received clicked point frame=%s stamp=%.3f active=%s xyz=[%.3f, %.3f, %.3f]",
                 msg->header.frame_id.c_str(), msg->header.stamp.toSec(),
                 active_lidar_.c_str(), msg->point.x, msg->point.y, msg->point.z);

        // 取活跃雷达点云快照
        sensor_msgs::PointCloud2ConstPtr cloud_snap;
        std::string active;
        {
            std::lock_guard<std::mutex> lk(cloud_mutex_);
            active = active_lidar_;
            cloud_snap = (active == "primary") ? latest_primary_ : latest_secondary_;
        }

        if (!cloud_snap) {
            const std::string& active_topic = (active == "primary") ? primary_topic_ : secondary_topic_;
            ROS_WARN("[lidar_lidar_cali_clicked] click received but no cached cloud for active lidar [%s], active_topic=%s, primary_topic=%s, secondary_topic=%s, primary_publishers=%u, secondary_publishers=%u",
                     active.c_str(), active_topic.c_str(), primary_topic_.c_str(), secondary_topic_.c_str(),
                     sub_primary_.getNumPublishers(), sub_secondary_.getNumPublishers());
            publishStatus("活跃雷达 [" + active + "] 尚未收到点云，请等待...");
            return;
        }

        ROS_INFO("[lidar_lidar_cali_clicked] using %s cloud frame=%s stamp=%.3f to process click",
                 active.c_str(), cloud_snap->header.frame_id.c_str(),
                 cloud_snap->header.stamp.toSec());

        // 解析点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_snap, *cloud);

        // 搜索邻域（2D 水平距离 <= search_radius，且高于地面）
        float cx = static_cast<float>(msg->point.x);
        float cy = static_cast<float>(msg->point.y);
        float ground_z = -static_cast<float>(install_params_.z_offset);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& pt : cloud->points) {
            float dx = pt.x - cx;
            float dy = pt.y - cy;
            if (std::sqrt(dx * dx + dy * dy) <= search_radius_ &&
                pt.z > ground_z + 0.05f)
                cluster->push_back(pt);
        }

        ROS_INFO("[lidar_lidar_cali_clicked] click neighborhood search done active=%s cloud_pts=%zu cluster_pts=%zu radius=%.3f ground_z=%.3f",
                 active.c_str(), cloud->size(), cluster->size(), search_radius_, ground_z);

        if (static_cast<int>(cluster->size()) < min_cluster_pts_) {
            ROS_WARN("[lidar_lidar_cali_clicked] [%s] insufficient cluster points near click: cluster=%zu min=%d click=[%.3f, %.3f]",
                     active.c_str(), cluster->size(), min_cluster_pts_, cx, cy);
            std::ostringstream oss;
            oss << "[" << active << "] 点击点附近只有 " << cluster->size()
                << " 个点（< " << min_cluster_pts_ << "），请点击更靠近锥桶的位置";
            publishStatus(oss.str());
            return;
        }

        // 计算 XY 质心 + 最高 Z
        float sum_x = 0, sum_y = 0, max_z = -1e9f;
        float top_x = -1e9f, top_y = -1e9f;
        for (const auto& pt : cluster->points) {
            sum_x += pt.x;
            sum_y += pt.y;
            if (pt.z > max_z)
            {
                max_z = pt.z;
                top_x = pt.x;
                top_y = pt.y;
            }
        }
        float cen_x = sum_x / static_cast<float>(cluster->size());
        float cen_y = sum_y / static_cast<float>(cluster->size());
        if(only_use_top_point_)
        {
            ROS_INFO("[lidar_lidar_cali_clicked] only_use_top_point=1 center=[%.5f, %.5f] top=[%.5f, %.5f, %.5f]",
                     cen_x, cen_y, top_x, top_y, max_z);
            cen_x = top_x;
            cen_y = top_y;
        }
        Eigen::Vector3d cone(cen_x, cen_y, max_z);

        if (active == "primary") {
            primary_cones_.push_back(cone);
            primary_cloud_agg_.operator+=(pcl::PointCloud<pcl::PointXYZI>(*cluster));
            ROS_INFO("[lidar_lidar_cali_clicked] 主雷达锥桶 #%zu: [%.3f, %.3f, %.3f]",
                     primary_cones_.size(), cen_x, cen_y, max_z);
        } else {
            secondary_cones_.push_back(cone);
            secondary_cloud_agg_.operator+=(pcl::PointCloud<pcl::PointXYZI>(*cluster));
            ROS_INFO("[lidar_lidar_cali_clicked] 辅雷达锥桶 #%zu: [%.3f, %.3f, %.3f]",
                     secondary_cones_.size(), cen_x, cen_y, max_z);
        }

        publishConeClouds(cloud_snap->header);
        publishConeMarkers(cloud_snap->header);

        // 数量相等且满足最小要求时自动标定
        if (primary_cones_.size() == secondary_cones_.size() &&
            static_cast<int>(primary_cones_.size()) >= min_cones_)
        {
            ROS_INFO("[lidar_lidar_cali_clicked] auto calibration condition met: primary=%zu secondary=%zu min_cones=%d",
                     primary_cones_.size(), secondary_cones_.size(), min_cones_);
            runCalibration(ros::Time::now());
        } else {
            std::ostringstream oss;
            oss << "活跃=" << active
                << " | 主锥桶=" << primary_cones_.size()
                << " 辅锥桶=" << secondary_cones_.size()
                << " | 继续点击...";
            publishStatus(oss.str());
        }
    }

    // -----------------------------------------------------------------------
    // 执行 2D SVD 标定
    void runCalibration(const ros::Time& stamp)
    {
        size_t N = primary_cones_.size(); // == secondary_cones_.size()

        // 逆变换辅雷达锥桶（去除安装标称值），还原到"水平辅雷达帧"
        double ci = std::cos(-install_params_.yaw_install_deg * M_PI / 180.0);
        double si = std::sin(-install_params_.yaw_install_deg * M_PI / 180.0);
        double xi = install_params_.x_install;
        double yi = install_params_.y_install;

        std::vector<size_t> perm(N);
        for (size_t i = 0; i < N; ++i) perm[i] = i;

        std::vector<size_t> best_perm;
        double best_yaw = 0.0, best_tx = 0.0, best_ty = 0.0, best_rmse = std::numeric_limits<double>::infinity();
        bool found = false;

        do {
            std::vector<Eigen::Vector2d> pri_xy(N), sec_q(N);
            for (size_t i = 0; i < N; ++i) {
                const size_t j = perm[i];
                pri_xy[i] = primary_cones_[i].head<2>();
                double dx = secondary_cones_[j].x() - xi;
                double dy = secondary_cones_[j].y() - yi;
                sec_q[i].x() = ci * dx - si * dy;
                sec_q[i].y() = si * dx + ci * dy;
            }

            double yaw_calib_deg, x_calib, y_calib, rmse;
            if (!svd2dCalibrate(pri_xy, sec_q, yaw_calib_deg, x_calib, y_calib, rmse)) {
                continue;
            }
            if (!found || rmse < best_rmse) {
                found = true;
                best_rmse = rmse;
                best_yaw = yaw_calib_deg;
                best_tx = x_calib;
                best_ty = y_calib;
                best_perm = perm;
            }
        } while (std::next_permutation(perm.begin(), perm.end()));

        if (!found) {
            publishStatus("SVD 标定失败（锥桶数不足或共线）");
            return;
        }

        double yaw_calib_deg = best_yaw;
        double x_calib = best_tx;
        double y_calib = best_ty;
        double rmse = best_rmse;

        std::ostringstream perm_oss;
        perm_oss << "[";
        for (size_t i = 0; i < best_perm.size(); ++i) {
            if (i) perm_oss << ",";
            perm_oss << best_perm[i];
        }
        perm_oss << "]";
        ROS_INFO("[lidar_lidar_cali_clicked] best cone permutation chosen: rmse=%.4fm perm=%s",
                 rmse, perm_oss.str().c_str());

        last_result_.x_calib       = x_calib;
        last_result_.y_calib       = y_calib;
        last_result_.yaw_calib_deg = yaw_calib_deg;
        last_result_.delta_x       = x_calib - install_params_.x_install;
        last_result_.delta_y       = y_calib - install_params_.y_install;
        last_result_.delta_yaw_deg = yaw_calib_deg - install_params_.yaw_install_deg;
        last_result_.rmse_m        = rmse;
        last_result_.num_scenes    = static_cast<int>(N);
        last_result_.success       = true;
        last_result_.timestamp     = getCurrentTimestamp();

        // 质量分级
        if      (rmse < 0.03 && N >= 4) last_result_.quality_grade = "EXCELLENT";
        else if (rmse < 0.05 && N >= 3) last_result_.quality_grade = "GOOD";
        else if (rmse < 0.10 && N >= 2) last_result_.quality_grade = "ACCEPTABLE";
        else                            last_result_.quality_grade = "POOR";

        calibrated_ = true;

        // 发布结果
        geometry_msgs::Vector3Stamped vm;
        vm.header.stamp    = stamp;
        vm.header.frame_id = "base_link";
        vm.vector.x = last_result_.delta_x;
        vm.vector.y = last_result_.delta_y;
        vm.vector.z = last_result_.delta_yaw_deg;
        pub_result_.publish(vm);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "[" << last_result_.quality_grade << "] 锥桶=" << N
            << " | x_calib=" << x_calib << "m"
            << " y_calib=" << y_calib << "m"
            << " yaw_calib=" << yaw_calib_deg << "°"
            << " | Δx=" << last_result_.delta_x
            << " Δy=" << last_result_.delta_y
            << " Δyaw=" << last_result_.delta_yaw_deg << "°"
            << " RMSE=" << rmse << "m";
        publishStatus(oss.str());
    }

    // -----------------------------------------------------------------------
    void publishConeClouds(const std_msgs::Header& header)
    {
        sensor_msgs::PointCloud2 pm, sm;
        pcl::toROSMsg(primary_cloud_agg_,   pm);
        pcl::toROSMsg(secondary_cloud_agg_, sm);
        pm.header = sm.header = header;
        pub_primary_cloud_.publish(pm);
        pub_secondary_cloud_.publish(sm);
        ROS_INFO("[lidar_lidar_cali_clicked] published cone clouds: primary_pts=%zu secondary_pts=%zu frame=%s",
                 primary_cloud_agg_.size(), secondary_cloud_agg_.size(), header.frame_id.c_str());
    }

    void publishConeMarkers(const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker del;
        del.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(del);

        int id = 0;
        // 主雷达：绿色
        addConeMarkers(ma, primary_cones_, header, "primary_cones",
                       0.1f, 0.9f, 0.1f, "M", id);
        // 辅雷达：蓝色
        addConeMarkers(ma, secondary_cones_, header, "secondary_cones",
                       0.1f, 0.4f, 1.0f, "S", id);

        pub_markers_.publish(ma);
        ROS_INFO("[lidar_lidar_cali_clicked] published cone markers: primary=%zu secondary=%zu markers=%zu frame=%s",
                 primary_cones_.size(), secondary_cones_.size(), ma.markers.size(), header.frame_id.c_str());
    }

    void addConeMarkers(visualization_msgs::MarkerArray& ma,
                        const std::vector<Eigen::Vector3d>& cones,
                        const std_msgs::Header& header,
                        const std::string& ns,
                        float r, float g, float b,
                        const std::string& prefix, int& id)
    {
        for (size_t i = 0; i < cones.size(); ++i) {
            visualization_msgs::Marker m;
            m.header = header;
            m.ns     = ns;
            m.id     = id++;
            m.type   = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = cones[i].x();
            m.pose.position.y = cones[i].y();
            m.pose.position.z = cones[i].z();
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.30;
            m.scale.y = 0.30;
            m.scale.z = 0.60;
            m.color.r = r;
            m.color.g = g;
            m.color.b = b;
            m.color.a = 0.8f;
            ma.markers.push_back(m);

            visualization_msgs::Marker txt;
            txt.header = header;
            txt.ns     = ns + "_label";
            txt.id     = id++;
            txt.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::Marker::ADD;
            txt.pose.position.x = cones[i].x();
            txt.pose.position.y = cones[i].y();
            txt.pose.position.z = cones[i].z() + 0.4;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.25;
            txt.color.r = r;
            txt.color.g = g;
            txt.color.b = b;
            txt.color.a = 1.0f;
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%s%zu", prefix.c_str(), i + 1);
            txt.text = buf;
            ma.markers.push_back(txt);
        }
    }

    // -----------------------------------------------------------------------
    // 服务：切换活跃雷达
    bool selectPrimary(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& res)
    {
        {
            std::lock_guard<std::mutex> lk(cloud_mutex_);
            active_lidar_ = "primary";
        }
        res.success = true;
        res.message = "已切换到主雷达模式，后续点击点击事件将添加到主雷达锥桶列表"
                      "（当前 " + std::to_string(primary_cones_.size()) + " 个）";
        ROS_INFO("[lidar_lidar_cali_clicked] %s", res.message.c_str());
        publishStatus("活跃=primary | 主锥桶=" + std::to_string(primary_cones_.size()) +
                      " 辅锥桶=" + std::to_string(secondary_cones_.size()));
        return true;
    }

    bool selectSecondary(std_srvs::Trigger::Request&,
                         std_srvs::Trigger::Response& res)
    {
        {
            std::lock_guard<std::mutex> lk(cloud_mutex_);
            active_lidar_ = "secondary";
        }
        res.success = true;
        res.message = "已切换到辅雷达模式，后续点击点击事件将添加到辅雷达锥桶列表"
                      "（当前 " + std::to_string(secondary_cones_.size()) + " 个）";
        ROS_INFO("[lidar_lidar_cali_clicked] %s", res.message.c_str());
        publishStatus("活跃=secondary | 主锥桶=" + std::to_string(primary_cones_.size()) +
                      " 辅锥桶=" + std::to_string(secondary_cones_.size()));
        return true;
    }

    // -----------------------------------------------------------------------
    bool triggerCalibration(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        size_t np = primary_cones_.size();
        size_t ns = secondary_cones_.size();
        size_t N  = std::min(np, ns);

        if (static_cast<int>(N) < min_cones_) {
            res.success = false;
            res.message = "锥桶数量不足：主=" + std::to_string(np) +
                          " 辅=" + std::to_string(ns) +
                          " 需要各 >= " + std::to_string(min_cones_);
            return true;
        }

        // 截断到 min(np, ns)
        std::vector<Eigen::Vector3d> pri(primary_cones_.begin(),
                                          primary_cones_.begin() + N);
        std::vector<Eigen::Vector3d> sec(secondary_cones_.begin(),
                                          secondary_cones_.begin() + N);
        // 临时赋值再调 runCalibration 内部逻辑
        auto save_pri = primary_cones_;
        auto save_sec = secondary_cones_;
        primary_cones_   = pri;
        secondary_cones_ = sec;
        runCalibration(ros::Time::now());
        primary_cones_   = save_pri;
        secondary_cones_ = save_sec;

        if (!calibrated_) {
            res.success = false;
            res.message = "标定计算失败";
            return true;
        }
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "[%s] x=%.4fm y=%.4fm yaw=%.4f° RMSE=%.4fm 锥桶=%d",
                      last_result_.quality_grade.c_str(),
                      last_result_.x_calib, last_result_.y_calib,
                      last_result_.yaw_calib_deg, last_result_.rmse_m,
                      last_result_.num_scenes);
        res.success = true;
        res.message = buf;
        return true;
    }

    bool resetCalibration(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& res)
    {
        primary_cones_.clear();
        secondary_cones_.clear();
        primary_cloud_agg_.clear();
        secondary_cloud_agg_.clear();
        calibrated_ = false;
        last_result_ = CalibResult{};
        {
            std::lock_guard<std::mutex> lk(cloud_mutex_);
            active_lidar_ = "primary";
        }
        res.success = true;
        res.message = "已清空主/辅锥桶列表，活跃雷达重置为 primary";
        ROS_INFO("[lidar_lidar_cali_clicked] %s", res.message.c_str());
        publishStatus("已重置，活跃=primary | 主锥桶=0 辅锥桶=0");
        return true;
    }

    // -----------------------------------------------------------------------
    bool saveCalibration(std_srvs::Trigger::Request&,
                         std_srvs::Trigger::Response& res)
    {
        if (!calibrated_) {
            res.success = false;
            res.message = "尚无有效标定结果，请先确保主/辅锥桶各 >= " +
                          std::to_string(min_cones_) + " 个";
            return true;
        }

        if (!writeCalibrationFile(last_result_)) {
            res.success = false;
            res.message = "写文件失败: " + output_file_;
            return true;
        }

        char buf[256];
        std::snprintf(buf, sizeof(buf), "[%s] 已保存到 %s",
                      last_result_.quality_grade.c_str(), output_file_.c_str());
        res.success = true;
        res.message = buf;
        ROS_INFO("[lidar_lidar_cali_clicked] %s", buf);
        LidarShmParam::SetModuleParam_x_y_yaw(install_params_.sensor_name, last_result_.x_calib, last_result_.y_calib, last_result_.yaw_calib_deg);
        return true;
    }

    // -----------------------------------------------------------------------
    bool writeCalibrationFile(const CalibResult& r) const
    {
        std::ofstream f(output_file_);
        if (!f.is_open()) return false;

        f << std::fixed << std::setprecision(6);
        f << "schema_version: 1\n";
        f << "calib_type: lidar_lidar\n";
        f << "sensor_name: " << install_params_.sensor_name << "\n";
        f << "generated_at: " << r.timestamp << "\n";
        f << "method: rviz_clicked_cones_2d_procrustes\n";
        f << "coordinate_system: FLU\n";
        f << "installation:\n";
        f << "  x: " << install_params_.x_install << "\n";
        f << "  y: " << install_params_.y_install << "\n";
        f << "  z: " << install_params_.z_calib << "\n";
        f << "  roll_deg: " << install_params_.roll_calib_deg << "\n";
        f << "  pitch_deg: " << install_params_.pitch_calib_deg << "\n";
        f << "  yaw_deg: " << install_params_.yaw_install_deg << "\n";
        f << "  z_offset: " << install_params_.z_offset << "\n";
        f << "calibrated:\n";
        f << "  x: " << r.x_calib << "\n";
        f << "  y: " << r.y_calib << "\n";
        f << "  z: " << install_params_.z_calib << "\n";
        f << "  roll_deg: " << install_params_.roll_calib_deg << "\n";
        f << "  pitch_deg: " << install_params_.pitch_calib_deg << "\n";
        f << "  yaw_deg: " << r.yaw_calib_deg << "\n";
        f << "delta:\n";
        f << "  delta_x_m: " << r.delta_x << "\n";
        f << "  delta_y_m: " << r.delta_y << "\n";
        f << "  delta_yaw_deg: " << r.delta_yaw_deg << "\n";
        f << "quality:\n";
        f << "  quality_grade: " << r.quality_grade << "\n";
        f << "  rmse_m: " << r.rmse_m << "\n";
        f << "  n_cones: " << r.num_scenes << "\n";

        f.close();
        return true;
    }

    // -----------------------------------------------------------------------
    void publishStatus(const std::string& s)
    {
        std_msgs::String msg;
        msg.data = s;
        pub_status_.publish(msg);
        ROS_INFO("[lidar_lidar_cali_clicked] %s", s.c_str());
    }

    static std::string getCurrentTimestamp()
    {
        std::time_t t = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
        return std::string(buf);
    }

    // -----------------------------------------------------------------------
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_primary_, sub_secondary_, sub_clicked_;
    ros::Publisher  pub_primary_cloud_, pub_secondary_cloud_;
    ros::Publisher  pub_result_, pub_status_, pub_markers_;
    ros::ServiceServer srv_select_primary_, srv_select_secondary_;
    ros::ServiceServer srv_trigger_, srv_reset_, srv_save_;

    std::string primary_topic_, secondary_topic_, output_file_;
    int   min_cones_;
    float search_radius_;
    int   min_cluster_pts_;

    InstallParams install_params_;

    // 点云缓冲（加锁）
    std::mutex cloud_mutex_;
    std::string active_lidar_;
    sensor_msgs::PointCloud2ConstPtr latest_primary_, latest_secondary_;

    // 已收集的锥桶
    std::vector<Eigen::Vector3d>      primary_cones_;
    std::vector<Eigen::Vector3d>      secondary_cones_;
    pcl::PointCloud<pcl::PointXYZI>   primary_cloud_agg_;
    pcl::PointCloud<pcl::PointXYZI>   secondary_cloud_agg_;

    bool        calibrated_;
    CalibResult last_result_;
    bool        only_use_top_point_;
};

} // namespace lidar_lidar_cali

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_lidar_cali_clicked");
    ros::NodeHandle nh, pnh("~");
    lidar_lidar_cali::LidarLidarCaliClickedNode node(nh, pnh);
    ros::spin();
    return 0;
}
