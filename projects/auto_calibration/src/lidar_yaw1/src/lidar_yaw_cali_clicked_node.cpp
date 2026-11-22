/**
 * lidar_yaw_cali_clicked_node.cpp
 *
 * 基于 RViz Publish Point 的 LiDAR Yaw 手动标定节点
 *
 * 工作原理：
 *   1. 实时订阅 LiDAR 点云，保存最新一帧到缓冲
 *   2. 用户在 RViz 中用 "Publish Point" 工具点击锥形桶附近
 *   3. 节点在点击位置周围 search_radius（默认 0.5m）内搜索点云
 *   4. 取该邻域内的最高点 Z 值（锥桶顶部）及 XY 质心，构造 ConeCluster
 *   5. 积累到 >= min_cones 个锥桶后，调用 YawCalculator 计算 yaw
 *
 * 订阅：
 *   input_topic              (sensor_msgs/PointCloud2)    — LiDAR 点云（已变换到 VCS）
 *   /clicked_point           (geometry_msgs/PointStamped) — RViz 发布的点击点
 *
 * 发布：
 *   ~cone_cloud              (sensor_msgs/PointCloud2)    — 已提取的锥桶点云
 *   ~cone_markers            (visualization_msgs/MarkerArray) — 锥桶位置球体 Marker
 *   ~result                  (geometry_msgs/Vector3Stamped)   — yaw_delta_deg, n_cones, confidence
 *   ~status                  (std_msgs/String)                — 文字状态
 *
 * 服务：
 *   ~trigger_calibration  — 查询当前标定结果
 *   ~reset_calibration    — 清空锥桶列表，重置标定
 *   ~save_calibration     — 将结果写入 yaml 文件
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <deque>
#include <mutex>
#include <sstream>
#include <string>
#include <numeric>

#include "lidar_yaw1/cone_detector.h"
#include "lidar_yaw1/yaw_calculator.h"
#include "lidar_yaw1/types.h"
#include "lidar_shm_param.h"

namespace lidar_yaw1 {

class LidarYawCaliClickedNode {
public:
    LidarYawCaliClickedNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh),
          min_cones_(2), search_radius_(0.5f), min_cluster_pts_(3)
    {
        loadParams();

        sub_cloud_ = nh_.subscribe(
            input_topic_, 1, &LidarYawCaliClickedNode::cloudCallback, this);
        sub_clicked_ = nh_.subscribe(
            "/clicked_point", 10, &LidarYawCaliClickedNode::clickedPointCallback, this);

        pub_cone_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("cone_cloud", 1, true);
        pub_result_     = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_     = pnh_.advertise<std_msgs::String>("status", 10);
        pub_markers_    = pnh_.advertise<visualization_msgs::MarkerArray>("cone_markers", 1, true);

        srv_trigger_ = pnh_.advertiseService(
            "trigger_calibration", &LidarYawCaliClickedNode::triggerCalibration, this);
        srv_reset_ = pnh_.advertiseService(
            "reset_calibration", &LidarYawCaliClickedNode::resetCalibration, this);
        srv_save_ = pnh_.advertiseService(
            "save_calibration", &LidarYawCaliClickedNode::saveCalibrationSrv, this);

        ROS_INFO("[LidarYawCaliClicked] 初始化完成，等待点云和 /clicked_point 点击");
        ROS_INFO("[LidarYawCaliClicked] 安装参数: yaw=%.2f° roll=%.2f° pitch=%.2f° z=%.3fm",
                 install_params_.yaw_deg, install_params_.roll_deg,
                 install_params_.pitch_deg, install_params_.z);
        ROS_INFO("[LidarYawCaliClicked] 搜索半径=%.2fm 最少锥桶=%d 输出: %s",
                 search_radius_, min_cones_, calib_output_file_.c_str());
        ROS_INFO("[LidarYawCaliClicked] 请在 RViz 中将 Fixed Frame 设为点云帧，"
                 "然后用 'Publish Point' 工具点击锥桶附近。");
        publishStatus("等待点云缓冲... 然后点击锥桶（需要 ≥ " +
                      std::to_string(min_cones_) + " 个）");
    }

private:
    // -----------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("input_topic", input_topic_, "/rslidar_points");
        pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
        pnh_.param<std::string>("base_frame",  base_frame_,  "base_link");
        pnh_.param<std::string>("calibration_output_file", calib_output_file_,
                                "/tmp/lidar_yaw_front_main_lidar.yaml");
        pnh_.param("min_cones",     min_cones_,     2);
        pnh_.param("search_radius", search_radius_, 0.5f);
        pnh_.param("min_cluster_pts", min_cluster_pts_, 3);

        // 安装参数
        pnh_.param<std::string>("installation/sensor_name",
                                install_params_.sensor_name, "lidar");
        pnh_.param("installation/x",         install_params_.x,         0.0);
        pnh_.param("installation/y",         install_params_.y,         0.0);
        pnh_.param("installation/z",         install_params_.z,         1.8);
        pnh_.param("installation/roll_deg",  install_params_.roll_deg,  0.0);
        pnh_.param("installation/pitch_deg", install_params_.pitch_deg, 0.0);
        pnh_.param("installation/yaw_deg",   install_params_.yaw_deg,   0.0);
        pnh_.param("installation/z_offset",  install_params_.z_offset,  0.37);

        // Yaw 计算参数
        YawCalculator::Params yc_params;
        pnh_.param("yaw_cali/max_yaw_deg",   yc_params.max_yaw_deg,   30.0);
        pnh_.param("yaw_cali/window_size",   yc_params.window_size,   50);  // 手动模式放大
        pnh_.param("yaw_cali/min_confidence", yc_params.min_confidence, 0.0); // 不过滤
        yaw_calc_.setParams(yc_params);
        window_size_ = yc_params.window_size;
    }

    // -----------------------------------------------------------------------
    // 缓存最新点云
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(cloud_mutex_);
        latest_cloud_ = msg;
    }

    // -----------------------------------------------------------------------
    // 处理 RViz 点击事件
    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
    {
        sensor_msgs::PointCloud2ConstPtr cloud_snap;
        {
            std::lock_guard<std::mutex> lk(cloud_mutex_);
            cloud_snap = latest_cloud_;
        }
        if (!cloud_snap) {
            publishStatus("尚未收到点云，请等待 LiDAR 数据...");
            return;
        }

        // 解析点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_snap, *cloud);

        // 在点击位置附近（2D 水平距离 <= search_radius）搜索点
        float cx = static_cast<float>(msg->point.x);
        float cy = static_cast<float>(msg->point.y);
        float ground_z = -static_cast<float>(install_params_.z_offset);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& pt : cloud->points) {
            float dx = pt.x - cx;
            float dy = pt.y - cy;
            float horiz_dist = std::sqrt(dx * dx + dy * dy);
            // 水平距离 <= search_radius，且高于地面 5cm 以上（排除地面点）
            if (horiz_dist <= search_radius_ && pt.z > ground_z + 0.05f)
                cluster->push_back(pt);
        }

        if (static_cast<int>(cluster->size()) < min_cluster_pts_) {
            std::ostringstream oss;
            oss << "点击位置附近只找到 " << cluster->size()
                << " 个点（< " << min_cluster_pts_ << "），请点击更靠近锥桶的位置";
            publishStatus(oss.str());
            return;
        }

        // 计算 XY 质心 和 最高点 Z
        float sum_x = 0, sum_y = 0, max_z = -1e9f;
        for (const auto& pt : cluster->points) {
            sum_x += pt.x;
            sum_y += pt.y;
            if (pt.z > max_z) max_z = pt.z;
        }
        float cen_x = sum_x / static_cast<float>(cluster->size());
        float cen_y = sum_y / static_cast<float>(cluster->size());

        // 构造 ConeCluster
        ConeCluster cone;
        cone.center     = Eigen::Vector3f(cen_x, cen_y, max_z);
        cone.radius_xy  = search_radius_;
        cone.point_count = static_cast<int>(cluster->size());
        cone.valid       = true;

        cones_.push_back(cone);
        cone_clusters_cloud_.operator+=(pcl::PointCloud<pcl::PointXYZI>(*cluster));

        int n = static_cast<int>(cones_.size());
        ROS_INFO("[LidarYawCaliClicked] 锥桶 #%d: center=[%.3f, %.3f, %.3f] points=%d",
                 n, cen_x, cen_y, max_z, cone.point_count);

        // 发布点云和 Marker
        publishConeCloud(cloud_snap->header);
        publishConeMarkers(cloud_snap->header);

        if (n < min_cones_) {
            publishStatus("已收集 " + std::to_string(n) + " / " +
                          std::to_string(min_cones_) + " 个锥桶，继续点击...");
            return;
        }

        // 有足够的锥桶，计算 Yaw
        computeAndUpdate(cloud_snap->header.stamp);
    }

    // -----------------------------------------------------------------------
    // 用当前所有 ConeCluster 计算 Yaw，更新窗口
    void computeAndUpdate(const ros::Time& stamp)
    {
        YawResult result = yaw_calc_.calculate(cones_);
        if (!result.valid) {
            publishStatus("Yaw 计算失败: " + result.message);
            return;
        }

        // 跟踪窗口（用于稳定性统计）
        yaw_window_deg_.push_back(result.yaw_delta_deg);
        if (static_cast<int>(yaw_window_deg_.size()) > window_size_)
            yaw_window_deg_.pop_front();

        // 保存最新单次结果（用于 save 时使用全量锥桶的精确结果）
        final_result_ = result;
        final_result_valid_ = true;

        publishResult(result, stamp);
        broadcastTF(result, stamp);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "OK | 锥桶=" << cones_.size()
            << " yaw_delta=" << result.yaw_delta_deg << "°"
            << " rms=" << result.line_rms << "m"
            << " conf=" << result.confidence;
        publishStatus(oss.str());
    }

    // -----------------------------------------------------------------------
    // 发布已收集的锥桶点云
    void publishConeCloud(const std_msgs::Header& header)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cone_clusters_cloud_, msg);
        msg.header = header;
        pub_cone_cloud_.publish(msg);
    }

    // -----------------------------------------------------------------------
    void publishConeMarkers(const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray ma;

        visualization_msgs::Marker del;
        del.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(del);

        for (int i = 0; i < static_cast<int>(cones_.size()); ++i) {
            // 锥桶球体
            visualization_msgs::Marker m;
            m.header      = header;
            m.ns          = "cones";
            m.id          = i;
            m.type        = visualization_msgs::Marker::CYLINDER;
            m.action      = visualization_msgs::Marker::ADD;
            m.pose.position.x = cones_[i].center.x();
            m.pose.position.y = cones_[i].center.y();
            m.pose.position.z = cones_[i].center.z();
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.30;
            m.scale.y = 0.30;
            m.scale.z = 0.60;
            m.color.r = 1.0f;
            m.color.g = 0.5f;
            m.color.b = 0.0f;
            m.color.a = 0.8f;
            ma.markers.push_back(m);

            // 编号标签
            visualization_msgs::Marker txt;
            txt.header = header;
            txt.ns     = "cone_labels";
            txt.id     = i + 100;
            txt.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::Marker::ADD;
            txt.pose.position.x = cones_[i].center.x();
            txt.pose.position.y = cones_[i].center.y();
            txt.pose.position.z = cones_[i].center.z() + 0.4f;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.25f;
            txt.color.r = 1.0f;
            txt.color.g = 1.0f;
            txt.color.b = 0.0f;
            txt.color.a = 1.0f;
            char buf[16];
            std::snprintf(buf, sizeof(buf), "C%d", i + 1);
            txt.text = buf;
            ma.markers.push_back(txt);
        }

        pub_markers_.publish(ma);
    }

    // -----------------------------------------------------------------------
    // 四元数合成融合欧拉角
    void computeFusedEuler(double delta_yaw_rad,
                           double& roll_f, double& pitch_f, double& yaw_f) const
    {
        static constexpr double D2R = M_PI / 180.0;
        static constexpr double R2D = 180.0 / M_PI;

        const InstallationParams& ip = install_params_;

        Eigen::Quaterniond q_install =
            Eigen::AngleAxisd(ip.yaw_deg   * D2R, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ip.pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ip.roll_deg  * D2R, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q_delta =
            Eigen::Quaterniond(Eigen::AngleAxisd(delta_yaw_rad, Eigen::Vector3d::UnitZ()));

        Eigen::Matrix3d R = (q_delta * q_install).normalized().toRotationMatrix();

        pitch_f = std::atan2(-R(2, 0),
                              std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
        roll_f  = std::atan2(R(2, 1), R(2, 2));
        yaw_f   = std::atan2(R(1, 0), R(0, 0));

        roll_f  *= R2D;
        pitch_f *= R2D;
        yaw_f   *= R2D;
    }

    // -----------------------------------------------------------------------
    static std::string gradeCalibration(double confidence, double std_yaw, int n_cones)
    {
        if (confidence >= 0.80 && std_yaw <= 0.15 && n_cones >= 4) return "EXCELLENT";
        if (confidence >= 0.60 && std_yaw <= 0.30 && n_cones >= 3) return "GOOD";
        if (confidence >= 0.40 && std_yaw <= 0.70 && n_cones >= 2) return "ACCEPTABLE";
        return "POOR";
    }

    static void computeMeanStd(const std::deque<double>& data,
                               double& mean, double& std_dev)
    {
        if (data.empty()) { mean = 0; std_dev = 0; return; }
        mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        double sq = 0.0;
        for (double v : data) sq += (v - mean) * (v - mean);
        std_dev = std::sqrt(sq / data.size());
    }

    // -----------------------------------------------------------------------
    void publishResult(const YawResult& r, const ros::Time& stamp)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = lidar_frame_;
        msg.vector.x = r.yaw_delta_deg;
        msg.vector.y = static_cast<double>(r.n_cones);
        msg.vector.z = r.confidence;
        pub_result_.publish(msg);
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::String msg;
        msg.data = s;
        pub_status_.publish(msg);
        ROS_INFO("[LidarYawCaliClicked] %s", s.c_str());
    }

    void broadcastTF(const YawResult& r, const ros::Time& stamp)
    {
        if (!r.valid) return;
        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(r.yaw_delta_rad, roll_f, pitch_f, yaw_f);

        static constexpr double D2R = M_PI / 180.0;
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(yaw_f   * D2R, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_f * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_f  * D2R, Eigen::Vector3d::UnitX());
        q.normalize();

        geometry_msgs::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = base_frame_;
        ts.child_frame_id  = lidar_frame_ + "_yaw_calibrated";
        ts.transform.translation.x = install_params_.x;
        ts.transform.translation.y = install_params_.y;
        ts.transform.translation.z = install_params_.z;
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(ts);
    }

    // -----------------------------------------------------------------------
    bool triggerCalibration(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        if (!final_result_valid_) {
            res.success = false;
            res.message = "锥桶数量不足（已收集 " + std::to_string(cones_.size()) +
                          " / 需要 " + std::to_string(min_cones_) + "）";
            return true;
        }
        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(final_result_.yaw_delta_rad, roll_f, pitch_f, yaw_f);
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "锥桶=%zu yaw_delta=%.4f° yaw_calib=%.4f° conf=%.3f rms=%.3fm",
                      cones_.size(), final_result_.yaw_delta_deg,
                      yaw_f, final_result_.confidence, final_result_.line_rms);
        res.success = true;
        res.message = buf;
        ROS_INFO("[LidarYawCaliClicked] trigger: %s", buf);
        return true;
    }

    bool resetCalibration(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& res)
    {
        cones_.clear();
        cone_clusters_cloud_.clear();
        yaw_calc_.reset();
        yaw_window_deg_.clear();
        final_result_       = YawResult{};
        final_result_valid_ = false;
        res.success = true;
        res.message = "已清空锥桶列表和标定数据";
        ROS_INFO("[LidarYawCaliClicked] %s", res.message.c_str());
        publishStatus("已重置，请重新点击锥桶（需要 ≥ " +
                      std::to_string(min_cones_) + " 个）");
        return true;
    }

    // -----------------------------------------------------------------------
    bool saveCalibrationSrv(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        if (!final_result_valid_) {
            res.success = false;
            res.message = "暂无有效标定结果，请先点击足够数量的锥桶";
            return true;
        }

        // 用全量锥桶重新计算一次，作为最终结果
        YawCalculator calc_final;
        YawCalculator::Params fp;
        fp.window_size = 1;
        fp.min_confidence = 0.0;
        fp.max_yaw_deg = yaw_calc_.getParams().max_yaw_deg;
        calc_final.setParams(fp);
        YawResult final = calc_final.calculate(cones_);
        if (!final.valid) {
            res.success = false;
            res.message = "最终 Yaw 计算失败: " + final.message;
            return true;
        }

        YawCalibOutput out;
        out.installation = install_params_;
        out.timestamp    = getCurrentTimestamp();
        out.delta_yaw_deg = final.yaw_delta_deg;

        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(final.yaw_delta_rad, roll_f, pitch_f, yaw_f);
        out.x_calib         = install_params_.x;
        out.y_calib         = install_params_.y;
        out.z_calib         = install_params_.z;
        out.roll_calib_deg  = roll_f;
        out.pitch_calib_deg = pitch_f;
        out.yaw_calib_deg   = yaw_f;

        out.confidence  = final.confidence;
        out.frames_used = static_cast<int>(cones_.size()); // 点数当帧数
        double mean_y;
        computeMeanStd(yaw_window_deg_, mean_y, out.std_yaw_deg);
        out.quality_grade = gradeCalibration(
            out.confidence, out.std_yaw_deg, static_cast<int>(cones_.size()));

        if (!writeCalibrationFile(out, final)) {
            res.success = false;
            res.message = "写文件失败: " + calib_output_file_;
            return true;
        }

        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "[%s] 已保存 %s | yaw_calib=%.4f° (delta=%.4f°) 锥桶=%zu rms=%.3fm",
                      out.quality_grade.c_str(), calib_output_file_.c_str(),
                      out.yaw_calib_deg, out.delta_yaw_deg,
                      cones_.size(), final.line_rms);
        res.success = true;
        res.message = buf;
        ROS_INFO("[LidarYawCaliClicked] %s", buf);
        LidarShmParam::SetModuleParam_yaw(install_params_.sensor_name, out.yaw_calib_deg);
        return true;
    }

    // -----------------------------------------------------------------------
    bool writeCalibrationFile(const YawCalibOutput& out, const YawResult& final) const
    {
        std::ofstream f(calib_output_file_);
        if (!f.is_open()) return false;

        const InstallationParams& ip = out.installation;
        f << std::fixed << std::setprecision(6);
        f << "schema_version: 1\n";
        f << "calib_type: lidar_yaw\n";
        f << "sensor_name: " << ip.sensor_name << "\n";
        f << "generated_at: " << out.timestamp << "\n";
        f << "method: rviz_clicked_cones\n";
        f << "coordinate_system: FLU\n";
        f << "installation:\n";
        f << "  x: " << ip.x << "\n";
        f << "  y: " << ip.y << "\n";
        f << "  z: " << ip.z << "\n";
        f << "  roll_deg: " << ip.roll_deg << "\n";
        f << "  pitch_deg: " << ip.pitch_deg << "\n";
        f << "  yaw_deg: " << ip.yaw_deg << "\n";
        f << "  z_offset: " << ip.z_offset << "\n";
        f << "calibrated:\n";
        f << "  x: " << out.x_calib << "\n";
        f << "  y: " << out.y_calib << "\n";
        f << "  z: " << out.z_calib << "\n";
        f << "  roll_deg: " << out.roll_calib_deg << "\n";
        f << "  pitch_deg: " << out.pitch_calib_deg << "\n";
        f << "  yaw_deg: " << out.yaw_calib_deg << "\n";
        f << "delta:\n";
        f << "  delta_yaw_deg: " << out.delta_yaw_deg << "\n";
        f << "quality:\n";
        f << "  quality_grade: " << out.quality_grade << "\n";
        f << "  confidence: " << out.confidence << "\n";
        f << "  line_rms_m: " << final.line_rms << "\n";
        f << "  n_cones: " << cones_.size() << "\n";
        f << "  std_yaw_deg: " << out.std_yaw_deg << "\n";

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
    ros::Subscriber sub_cloud_, sub_clicked_;
    ros::Publisher  pub_cone_cloud_, pub_result_, pub_status_, pub_markers_;
    ros::ServiceServer srv_trigger_, srv_reset_, srv_save_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string input_topic_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string calib_output_file_;

    int   min_cones_;
    float search_radius_;
    int   min_cluster_pts_;

    InstallationParams install_params_;
    YawCalculator      yaw_calc_;

    // 最新点云缓冲
    std::mutex cloud_mutex_;
    sensor_msgs::PointCloud2ConstPtr latest_cloud_;

    // 已收集的锥桶
    std::vector<ConeCluster>          cones_;
    pcl::PointCloud<pcl::PointXYZI>   cone_clusters_cloud_;

    // 结果追踪
    YawResult final_result_;
    bool      final_result_valid_{false};
    std::deque<double> yaw_window_deg_;
    int window_size_{50};
};

} // namespace lidar_yaw1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_yaw_cali_clicked");
    ros::NodeHandle nh, pnh("~");
    lidar_yaw1::LidarYawCaliClickedNode node(nh, pnh);
    ros::spin();
    return 0;
}
