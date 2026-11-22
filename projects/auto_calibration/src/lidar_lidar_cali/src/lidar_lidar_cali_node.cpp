/**
 * lidar_lidar_cali_node.cpp
 *
 * 双激光雷达联合 x/y/yaw 标定节点（FAST-LIVO2 标定板版本）
 *
 * 原理：
 *   主/辅雷达同时观测 FAST-LIVO2 标定板，各检测 4 个圆孔中心（3D），
 *   通过 SVD 3D-3D 配准求解辅雷达精确外参 (x_calib, y_calib, yaw_calib)。
 *
 * 流程：
 *   1. 每帧点云：LidarProcessor 检测 4 个圆孔中心
 *   2. 时间戳对齐（容差 100ms）：主+辅同时检测成功 → 累积 SceneData
 *   3. trigger_calibration → SVD 3D-3D 标定，输出结果
 *   4. save_calibration → 写 YAML 文件
 *
 * 支持雷达类型：
 *   solid — 固态/无ring (pcl::PointXYZI)，BoundaryEstimation + 欧氏聚类
 *   mech  — 机械多线/有ring (PointXYZIR)，ring 邻距跳变边缘
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <sstream>
#include <string>
#include <mutex>

#include "image_lidar_calibration/types.h"
#include "image_lidar_calibration/lidar_processor.h"
#include "lidar_lidar_cali/types.h"
#include "lidar_lidar_cali/lidar_cali.h"
#include "lidar_shm_param.h"

namespace lidar_lidar_cali {

static constexpr double kSyncToleranceSec = 0.10;

struct DetectionResult {
    std::vector<Eigen::Vector3d> centers;  // 4 圆孔中心（雷达坐标系）
    ros::Time stamp;
    bool valid = false;
};

// ===========================================================================
class LidarLidarCaliNode {
public:
    LidarLidarCaliNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), calibrated_(false)
    {
        loadParams();
        setupROS();

        ROS_INFO("[lidar_lidar_cali] 初始化完成 (FAST-LIVO2 标定板)");
        ROS_INFO("[lidar_lidar_cali] 传感器: %s", install_params_.sensor_name.c_str());
        ROS_INFO("[lidar_lidar_cali] 主雷达: %s (%s)", primary_topic_.c_str(), primary_lidar_type_.c_str());
        ROS_INFO("[lidar_lidar_cali] 辅雷达: %s (%s)", secondary_topic_.c_str(), secondary_lidar_type_.c_str());
        ROS_INFO("[lidar_lidar_cali] 安装标称值: x=%.3fm y=%.3fm yaw=%.3f°",
                 install_params_.x_install, install_params_.y_install,
                 install_params_.yaw_install_deg);
        ROS_INFO("[lidar_lidar_cali] 最少场景数: %d, 结果输出: %s",
                 calib_params_.min_scenes, output_file_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("primary_lidar/topic",   primary_topic_,   "/lidar_master/points");
        pnh_.param<std::string>("primary_lidar/lidar_type", primary_lidar_type_, "solid");
        pnh_.param<std::string>("secondary_lidar/topic", secondary_topic_,  "/lidar_sub/points");
        pnh_.param<std::string>("secondary_lidar/lidar_type", secondary_lidar_type_, "solid");
        pnh_.param<std::string>("output/file",           output_file_,
                                "/tmp/lidar_lidar_calib_result.yaml");

        // 安装参数
        pnh_.param<std::string>("secondary_lidar/sensor_name",
                                install_params_.sensor_name, "lidar_sub");
        pnh_.param("secondary_lidar/roll_calib_deg",   install_params_.roll_calib_deg,  0.0);
        pnh_.param("secondary_lidar/pitch_calib_deg",  install_params_.pitch_calib_deg, 0.0);
        pnh_.param("secondary_lidar/z_calib",          install_params_.z_calib,         1.43);
        pnh_.param("secondary_lidar/z_offset",         install_params_.z_offset,        0.37);
        pnh_.param("secondary_lidar/x_install",        install_params_.x_install,       0.0);
        pnh_.param("secondary_lidar/y_install",        install_params_.y_install,       0.0);
        pnh_.param("secondary_lidar/yaw_install_deg",  install_params_.yaw_install_deg, 0.0);

        // 主雷达 LidarProcessor 参数
        image_lidar_calibration::LidarProcessor::Params pp;
        pnh_.param("primary_lidar/roi_x_min", pp.roi_x_min, 1.5);
        pnh_.param("primary_lidar/roi_x_max", pp.roi_x_max, 5.0);
        pnh_.param("primary_lidar/roi_y_min", pp.roi_y_min, -1.5);
        pnh_.param("primary_lidar/roi_y_max", pp.roi_y_max,  2.0);
        pnh_.param("primary_lidar/roi_z_min", pp.roi_z_min, -0.5);
        pnh_.param("primary_lidar/roi_z_max", pp.roi_z_max,  2.0);
        pnh_.param("target/ransac_threshold",     pp.ransac_threshold,    0.01);
        pnh_.param("target/max_iterations",       pp.max_iterations,      1000);
        pnh_.param("target/circle_radius",        pp.circle_radius,       0.12);
        pnh_.param("target/delta_width_circles",  pp.delta_width_circles, 0.50);
        pnh_.param("target/delta_height_circles", pp.delta_height_circles, 0.40);
        primary_proc_.setParams(pp);

        // 辅雷达 LidarProcessor 参数（ROI 可独立配置）
        image_lidar_calibration::LidarProcessor::Params sp = pp;
        pnh_.param("secondary_lidar/roi_x_min", sp.roi_x_min, 1.5);
        pnh_.param("secondary_lidar/roi_x_max", sp.roi_x_max, 5.0);
        pnh_.param("secondary_lidar/roi_y_min", sp.roi_y_min, -1.5);
        pnh_.param("secondary_lidar/roi_y_max", sp.roi_y_max,  2.0);
        pnh_.param("secondary_lidar/roi_z_min", sp.roi_z_min, -0.5);
        pnh_.param("secondary_lidar/roi_z_max", sp.roi_z_max,  2.0);
        secondary_proc_.setParams(sp);

        // 标定算法参数
        calib_params_.x_install       = install_params_.x_install;
        calib_params_.y_install       = install_params_.y_install;
        calib_params_.yaw_install_deg = install_params_.yaw_install_deg;
        pnh_.param("calibration/min_scenes", calib_params_.min_scenes, 3);

        calibrator_.reset(new LidarCali(calib_params_));
    }

    // -----------------------------------------------------------------------
    void setupROS()
    {
        sub_primary_ = nh_.subscribe(
            primary_topic_, 1, &LidarLidarCaliNode::primaryCallback, this);
        sub_secondary_ = nh_.subscribe(
            secondary_topic_, 1, &LidarLidarCaliNode::secondaryCallback, this);

        pub_primary_cloud_   = pnh_.advertise<sensor_msgs::PointCloud2>("primary_circles", 1);
        pub_secondary_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("secondary_circles", 1);
        pub_result_          = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_          = pnh_.advertise<std_msgs::String>("status", 10);
        pub_markers_         = pnh_.advertise<visualization_msgs::MarkerArray>("circle_markers", 1);

        srv_trigger_ = pnh_.advertiseService(
            "trigger_calibration", &LidarLidarCaliNode::triggerCalibration, this);
        srv_reset_   = pnh_.advertiseService(
            "reset_calibration",   &LidarLidarCaliNode::resetCalibration,   this);
        srv_save_    = pnh_.advertiseService(
            "save_calibration",    &LidarLidarCaliNode::saveCalibration,    this);
    }

    // -----------------------------------------------------------------------
    // 主雷达回调
    // -----------------------------------------------------------------------
    void primaryCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        DetectionResult det;
        det.stamp = msg->header.stamp;

        if (primary_lidar_type_ == "mech") {
            pcl::PointCloud<image_lidar_calibration::PointXYZIR>::Ptr cloud(new pcl::PointCloud<image_lidar_calibration::PointXYZIR>());
            pcl::fromROSMsg(*msg, *cloud);
            det.valid = primary_proc_.detectCircleCentersMech(cloud, det.centers);
        } else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*msg, *cloud);
            det.valid = primary_proc_.detectCircleCenters(cloud, det.centers);
        }

        {
            std::lock_guard<std::mutex> lk(mutex_);
            latest_primary_ = det;
            // 主雷达检测成功时，尝试与最新辅雷达结果配对
            if (det.valid && latest_secondary_.valid) {
                double dt = std::abs(
                    (det.stamp - latest_secondary_.stamp).toSec());
                if (dt < kSyncToleranceSec) {
                    tryAddScene(det, latest_secondary_, msg->header);
                }
            }
        }

        publishMarkers(msg->header);
        publishStatusThrottle();
    }

    // -----------------------------------------------------------------------
    // 辅雷达回调
    // -----------------------------------------------------------------------
    void secondaryCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        DetectionResult det;
        det.stamp = msg->header.stamp;

        if (secondary_lidar_type_ == "mech") {
            pcl::PointCloud<image_lidar_calibration::PointXYZIR>::Ptr cloud(new pcl::PointCloud<image_lidar_calibration::PointXYZIR>());
            pcl::fromROSMsg(*msg, *cloud);
            det.valid = secondary_proc_.detectCircleCentersMech(cloud, det.centers);
        } else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*msg, *cloud);
            det.valid = secondary_proc_.detectCircleCenters(cloud, det.centers);
        }

        {
            std::lock_guard<std::mutex> lk(mutex_);
            latest_secondary_ = det;
            // 辅雷达检测成功时，尝试与最新主雷达结果配对
            if (det.valid && latest_primary_.valid) {
                double dt = std::abs(
                    (det.stamp - latest_primary_.stamp).toSec());
                if (dt < kSyncToleranceSec) {
                    tryAddScene(latest_primary_, det, msg->header);
                }
            }
        }

        publishMarkers(msg->header);
        publishStatusThrottle();
    }

    // -----------------------------------------------------------------------
    // 尝试添加场景（必须持有 mutex_）
    // 限速：每次成功配对后至少等 1 秒才再次添加，避免连续相同帧重复入库
    // -----------------------------------------------------------------------
    void tryAddScene(const DetectionResult& primary,
                     const DetectionResult& secondary,
                     const std_msgs::Header& header)
    {
        // 场景间隔限速：避免同一位置重复采集
        ros::Time now = ros::Time::now();
        if (!last_scene_time_.isZero() &&
            (now - last_scene_time_).toSec() < 1.0)
            return;

        SceneData scene;
        scene.primary_centers   = primary.centers;
        scene.secondary_centers = secondary.centers;
        scene.valid             = true;
        calibrator_->addScene(scene);
        last_scene_time_ = now;

        int n = calibrator_->numScenes();
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "场景已采集 %d/%d，主RMSE待标定",
                      n, calib_params_.min_scenes);
        ROS_INFO("[lidar_lidar_cali] %s", buf);
        publishStatus(buf);

        (void)header;
    }

    // -----------------------------------------------------------------------
    // trigger：触发 SVD 标定
    // -----------------------------------------------------------------------
    bool triggerCalibration(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        if (!calibrator_->isReady()) {
            res.success = false;
            res.message = "场景不足: " + std::to_string(calibrator_->numScenes())
                          + " < " + std::to_string(calib_params_.min_scenes);
            return true;
        }

        ROS_INFO("[lidar_lidar_cali] 触发 SVD 3D-3D 标定 ...");
        CalibResult r = calibrator_->calibrate();
        r.timestamp = getCurrentTimestamp();

        if (r.success) {
            last_result_ = r;
            calibrated_  = true;
            publishResult();
            ROS_INFO("[lidar_lidar_cali] %s", r.message.c_str());
        } else {
            ROS_WARN("[lidar_lidar_cali] 标定失败: %s", r.message.c_str());
        }

        res.success = r.success;
        res.message = r.message;
        publishStatus(r.message);
        return true;
    }

    // -----------------------------------------------------------------------
    bool resetCalibration(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& res)
    {
        calibrator_->reset();
        calibrated_ = false;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            latest_primary_   = DetectionResult{};
            latest_secondary_ = DetectionResult{};
            last_scene_time_  = ros::Time();
        }
        res.success = true;
        res.message = "标定数据已重置";
        ROS_INFO("[lidar_lidar_cali] %s", res.message.c_str());
        return true;
    }

    // -----------------------------------------------------------------------
    bool saveCalibration(std_srvs::Trigger::Request&,
                         std_srvs::Trigger::Response& res)
    {
        if (!calibrated_) {
            res.success = false;
            res.message = "尚未标定，请先调用 trigger_calibration";
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
        ROS_INFO("[lidar_lidar_cali] %s", buf);
        LidarShmParam::SetModuleParam_x_y_yaw(install_params_.sensor_name, last_result_.x_calib, last_result_.y_calib, last_result_.yaw_calib_deg);
        return true;
    }

    // -----------------------------------------------------------------------
    void publishResult()
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = "base_link";
        msg.vector.x = last_result_.delta_x;
        msg.vector.y = last_result_.delta_y;
        msg.vector.z = last_result_.delta_yaw_deg;
        pub_result_.publish(msg);
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::String msg;
        msg.data = s;
        pub_status_.publish(msg);
    }

    void publishStatusThrottle()
    {
        std::ostringstream oss;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            oss << "主=" << (latest_primary_.valid ? "OK" : "--")
                << " 辅=" << (latest_secondary_.valid ? "OK" : "--")
                << " 场景=" << calibrator_->numScenes()
                << "/" << calib_params_.min_scenes;
        }
        if (calibrated_) {
            oss << " | [标定完成] Δx=" << std::fixed << std::setprecision(3)
                << last_result_.delta_x << "m Δy=" << last_result_.delta_y
                << "m Δyaw=" << last_result_.delta_yaw_deg << "°"
                << " RMSE=" << last_result_.rmse_m << "m";
        }
        ROS_INFO_THROTTLE(2.0, "[lidar_lidar_cali] %s", oss.str().c_str());
        publishStatus(oss.str());
    }

    // -----------------------------------------------------------------------
    // 可视化 Marker：主雷达圆孔中心（绿色球），辅雷达（蓝色球）
    // -----------------------------------------------------------------------
    void publishMarkers(const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray ma;

        visualization_msgs::Marker del;
        del.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(del);

        int id = 0;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            addCircleMarkers(ma, latest_primary_.centers, header,
                             "primary_circles",   0.0f, 0.8f, 0.2f, id);
            addCircleMarkers(ma, latest_secondary_.centers, header,
                             "secondary_circles", 0.1f, 0.4f, 1.0f, id);
        }

        pub_markers_.publish(ma);
    }

    void addCircleMarkers(visualization_msgs::MarkerArray& ma,
                          const std::vector<Eigen::Vector3d>& centers,
                          const std_msgs::Header& header,
                          const std::string& ns,
                          float r, float g, float b, int& id)
    {
        for (size_t i = 0; i < centers.size(); ++i) {
            visualization_msgs::Marker m;
            m.header   = header;
            m.ns       = ns;
            m.id       = id++;
            m.type     = visualization_msgs::Marker::SPHERE;
            m.action   = visualization_msgs::Marker::ADD;
            m.pose.position.x  = centers[i].x();
            m.pose.position.y  = centers[i].y();
            m.pose.position.z  = centers[i].z();
            m.pose.orientation.w = 1.0;
            m.scale.x  = 0.15;
            m.scale.y  = 0.15;
            m.scale.z  = 0.15;
            m.color.r  = r;
            m.color.g  = g;
            m.color.b  = b;
            m.color.a  = 0.9f;
            m.lifetime = ros::Duration(0.5);
            ma.markers.push_back(m);

            visualization_msgs::Marker txt;
            txt.header   = header;
            txt.ns       = ns + "_label";
            txt.id       = id++;
            txt.type     = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action   = visualization_msgs::Marker::ADD;
            txt.pose.position.x = centers[i].x();
            txt.pose.position.y = centers[i].y();
            txt.pose.position.z = centers[i].z() + 0.15;
            txt.pose.orientation.w = 1.0;
            txt.scale.z  = 0.15;
            txt.color.r  = r;
            txt.color.g  = g;
            txt.color.b  = b;
            txt.color.a  = 1.0f;
            txt.lifetime = ros::Duration(0.5);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "C%zu(%.2f,%.2f)",
                          i, centers[i].x(), centers[i].y());
            txt.text = buf;
            ma.markers.push_back(txt);
        }
    }

    // -----------------------------------------------------------------------
    bool writeCalibrationFile(const CalibResult& r) const
    {
        std::ofstream f(output_file_);
        if (!f.is_open()) {
            ROS_ERROR("[lidar_lidar_cali] 无法打开输出文件: %s", output_file_.c_str());
            return false;
        }

        f << std::fixed << std::setprecision(6);
        f << "schema_version: 1\n";
        f << "calib_type: lidar_lidar\n";
        f << "sensor_name: " << install_params_.sensor_name << "\n";
        f << "generated_at: " << r.timestamp << "\n";
        f << "method: board_holes_svd_3d3d\n";
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
        f << "  num_scenes: " << r.num_scenes << "\n";

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

    ros::Subscriber sub_primary_, sub_secondary_;
    ros::Publisher  pub_primary_cloud_, pub_secondary_cloud_;
    ros::Publisher  pub_result_, pub_status_, pub_markers_;
    ros::ServiceServer srv_trigger_, srv_reset_, srv_save_;

    std::string primary_topic_;
    std::string secondary_topic_;
    std::string primary_lidar_type_;
    std::string secondary_lidar_type_;
    std::string output_file_;

    InstallParams install_params_;

    image_lidar_calibration::LidarProcessor primary_proc_;
    image_lidar_calibration::LidarProcessor secondary_proc_;

    LidarCali::Params calib_params_;
    std::unique_ptr<LidarCali> calibrator_;

    // 最新检测结果（加锁保护）
    std::mutex      mutex_;
    DetectionResult latest_primary_;
    DetectionResult latest_secondary_;
    ros::Time       last_scene_time_;

    bool        calibrated_;
    CalibResult last_result_;
};

} // namespace lidar_lidar_cali

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_lidar_cali");
    ros::NodeHandle nh, pnh("~");
    lidar_lidar_cali::LidarLidarCaliNode node(nh, pnh);
    ros::spin();
    return 0;
}
