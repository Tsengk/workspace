#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
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
#include <numeric>
#include <string>

#include "lidar_yaw1/cone_detector.h"
#include "lidar_yaw1/yaw_calculator.h"
#include "lidar_yaw1/types.h"
#include "lidar_shm_param.h"

namespace lidar_yaw1 {

class LidarYawCaliNode {
public:
    LidarYawCaliNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        loadParams();

        sub_cloud_ = nh_.subscribe(
            input_topic_, 1, &LidarYawCaliNode::cloudCallback, this);

        pub_cone_cloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("cone_cloud", 1);
        pub_result_     = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_     = pnh_.advertise<std_msgs::String>("status", 10);
        pub_markers_    = pnh_.advertise<visualization_msgs::MarkerArray>("cone_markers", 1);

        srv_trigger_ = pnh_.advertiseService(
            "trigger_calibration", &LidarYawCaliNode::triggerCalibration, this);
        srv_reset_ = pnh_.advertiseService(
            "reset_calibration", &LidarYawCaliNode::resetCalibration, this);
        srv_save_ = pnh_.advertiseService(
            "save_calibration", &LidarYawCaliNode::saveCalibrationSrv, this);

        ROS_INFO("[LidarYawCali] init OK, listening: %s", input_topic_.c_str());
        ROS_INFO("[LidarYawCali] install: yaw=%.2f° roll=%.2f° pitch=%.2f°"
                 " z=%.3fm z_offset=%.3fm",
                 install_params_.yaw_deg, install_params_.roll_deg,
                 install_params_.pitch_deg, install_params_.z,
                 install_params_.z_offset);
        ROS_INFO("[LidarYawCali] output: %s", calib_output_file_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("input_topic", input_topic_, "/rslidar_points");
        pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
        pnh_.param<std::string>("base_frame",  base_frame_,  "base_link");
        pnh_.param<std::string>("calibration_output_file", calib_output_file_,
                                std::string("/tmp/lidar_yaw_calibration.yaml"));

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

        // 锥桶检测参数
        ConeDetector::Params cd_params;
        cd_params.z_offset = static_cast<float>(install_params_.z_offset);

        pnh_.param("cone/x_min", cd_params.x_min,  2.0f);
        pnh_.param("cone/x_max", cd_params.x_max, 40.0f);
        pnh_.param("cone/y_min", cd_params.y_min, -10.0f);
        pnh_.param("cone/y_max", cd_params.y_max,  10.0f);
        pnh_.param("cone/height_above_ground_min",
                   cd_params.height_above_ground_min, 0.08f);
        pnh_.param("cone/height_above_ground_max",
                   cd_params.height_above_ground_max, 0.80f);
        pnh_.param("cone/voxel_size",            cd_params.voxel_size,            0.05f);
        pnh_.param("cone/cluster_tolerance",     cd_params.cluster_tolerance,     0.30f);
        pnh_.param("cone/min_cluster_points",    cd_params.min_cluster_points,    5);
        pnh_.param("cone/max_cluster_points",    cd_params.max_cluster_points,    500);
        pnh_.param("cone/max_cluster_radius_xy", cd_params.max_cluster_radius_xy, 0.50f);
        pnh_.param("cone/min_cones",             cd_params.min_cones,             2);
        cone_detector_.setParams(cd_params);

        float z_cone_min = -cd_params.z_offset + cd_params.height_above_ground_min;
        float z_cone_max = -cd_params.z_offset + cd_params.height_above_ground_max;
        ROS_INFO("[LidarYawCali] cone z range in VCS: [%.2f, %.2f]",
                 z_cone_min, z_cone_max);

        // Yaw 计算参数
        YawCalculator::Params yc_params;
        pnh_.param("yaw_cali/max_yaw_deg",    yc_params.max_yaw_deg,    30.0);
        pnh_.param("yaw_cali/window_size",     yc_params.window_size,    10);
        pnh_.param("yaw_cali/min_confidence",  yc_params.min_confidence, 0.30);
        yaw_calc_.setParams(yc_params);
        window_size_ = yc_params.window_size;
    }

    // -----------------------------------------------------------------------
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *raw);

        // 1. 锥桶检测
        std::vector<ConeCluster> cones;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cone_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (!cone_detector_.detectCones(raw, cones, cone_cloud)) {
            publishStatus("cone detection failed: insufficient cones");
            return;
        }

        // 2. 发布锥桶点云
        sensor_msgs::PointCloud2 cone_msg;
        pcl::toROSMsg(*cone_cloud, cone_msg);
        cone_msg.header = msg->header;
        pub_cone_cloud_.publish(cone_msg);

        // 3. 发布锥桶 Marker
        publishConeMarkers(cones, msg->header);

        // 4. Yaw 计算
        YawResult result = yaw_calc_.calculate(cones);
        if (!result.valid) {
            publishStatus("yaw calc failed: " + result.message);
            return;
        }

        // 5. 跟踪 yaw_delta 进入独立质量窗口
        if (result.confidence >= yaw_calc_.getParams().min_confidence) {
            yaw_window_deg_.push_back(result.yaw_delta_deg);
            if (static_cast<int>(yaw_window_deg_.size()) > window_size_)
                yaw_window_deg_.pop_front();
        }

        // 6. 发布结果 / TF
        publishResult(result, msg->header.stamp);
        broadcastTF(yaw_calc_.getAveraged(), msg->header.stamp);

        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "OK | cones=%d yaw_delta=%.3f° conf=%.2f rms=%.3fm",
                      result.n_cones, result.yaw_delta_deg,
                      result.confidence, result.line_rms);
        publishStatus(buf);
    }

    // -----------------------------------------------------------------------
    // 计算融合后的 ZYX 欧拉角
    //
    // 原理：
    //   R_install = Rz(yaw_install) * Ry(pitch_install) * Rx(roll_install)
    //   R_delta   = Rz(-theta)    （-theta 为 yaw 修正量）
    //   R_calib   = R_delta * R_install
    //
    // 从 R_calib 提取 ZYX 欧拉角：
    //   yaw   = atan2(R(1,0), R(0,0))
    //   pitch = atan2(-R(2,0), sqrt(R(0,0)^2 + R(1,0)^2))
    //   roll  = atan2(R(2,1), R(2,2))
    // -----------------------------------------------------------------------
    void computeFusedEuler(double delta_yaw_rad,
                           double& roll_f, double& pitch_f, double& yaw_f) const
    {
        static constexpr double D2R = M_PI / 180.0;
        static constexpr double R2D = 180.0 / M_PI;

        const InstallationParams& ip = install_params_;

        // R_install = Rz(yaw_install) * Ry(pitch_install) * Rx(roll_install)
        Eigen::Quaterniond q_install =
            Eigen::AngleAxisd(ip.yaw_deg   * D2R, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ip.pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ip.roll_deg  * D2R, Eigen::Vector3d::UnitX());

        // R_delta = Rz(delta_yaw)
        Eigen::Quaterniond q_delta =
            Eigen::Quaterniond(Eigen::AngleAxisd(delta_yaw_rad, Eigen::Vector3d::UnitZ()));

        // R_calib = R_delta * R_install
        Eigen::Matrix3d R = (q_delta * q_install).normalized().toRotationMatrix();

        // ZYX 欧拉角提取
        pitch_f = std::atan2(-R(2, 0),
                              std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
        roll_f  = std::atan2(R(2, 1), R(2, 2));
        yaw_f   = std::atan2(R(1, 0), R(0, 0));

        roll_f  *= R2D;
        pitch_f *= R2D;
        yaw_f   *= R2D;
    }

    // -----------------------------------------------------------------------
    // 标定质量等级
    // -----------------------------------------------------------------------
    static std::string gradeCalibration(double confidence,
                                        double std_yaw, int frames)
    {
        if (confidence >= 0.80 && std_yaw <= 0.10 && frames >= 8)
            return "EXCELLENT";
        if (confidence >= 0.60 && std_yaw <= 0.20 && frames >= 5)
            return "GOOD";
        if (confidence >= 0.40 && std_yaw <= 0.50 && frames >= 3)
            return "ACCEPTABLE";
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
        msg.vector.x = r.yaw_delta_deg;   // Yaw 修正量（°）
        msg.vector.y = static_cast<double>(r.n_cones);
        msg.vector.z = r.confidence;
        pub_result_.publish(msg);
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::String msg;
        msg.data = s;
        pub_status_.publish(msg);
        ROS_INFO_THROTTLE(1.0, "[LidarYawCali] %s", s.c_str());
    }

    void broadcastTF(const YawResult& avg, const ros::Time& stamp)
    {
        if (!avg.valid) return;

        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(avg.yaw_delta_rad, roll_f, pitch_f, yaw_f);

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
    // 可视化：锥桶 Marker（球体）
    // -----------------------------------------------------------------------
    void publishConeMarkers(const std::vector<ConeCluster>& cones,
                            const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray ma;

        // 先发一个删除所有旧 marker 的 action
        visualization_msgs::Marker del;
        del.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(del);

        for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
            visualization_msgs::Marker m;
            m.header      = header;
            m.ns          = "cones";
            m.id          = i;
            m.type        = visualization_msgs::Marker::CYLINDER;
            m.action      = visualization_msgs::Marker::ADD;
            m.pose.position.x = cones[i].center.x();
            m.pose.position.y = cones[i].center.y();
            m.pose.position.z = cones[i].center.z();
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.30;  // 锥桶直径
            m.scale.y = 0.30;
            m.scale.z = 0.60;  // 锥桶高度（近似）
            m.color.r = 1.0f;
            m.color.g = 0.5f;
            m.color.b = 0.0f;
            m.color.a = 0.8f;
            m.lifetime = ros::Duration(0.5);
            ma.markers.push_back(m);
        }

        pub_markers_.publish(ma);
    }

    // -----------------------------------------------------------------------
    bool triggerCalibration(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        YawResult avg = yaw_calc_.getAveraged();
        if (!avg.valid) {
            res.success = false;
            res.message = "no valid calibration data yet";
            return true;
        }

        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(avg.yaw_delta_rad, roll_f, pitch_f, yaw_f);

        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "yaw_delta=%.4f° yaw_calib=%.4f° roll=%.4f° pitch=%.4f° conf=%.3f",
                      avg.yaw_delta_deg, yaw_f, roll_f, pitch_f, avg.confidence);
        res.success = true;
        res.message = buf;
        ROS_INFO("[LidarYawCali] trigger: %s", res.message.c_str());
        return true;
    }

    bool resetCalibration(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& res)
    {
        yaw_calc_.reset();
        yaw_window_deg_.clear();
        res.success = true;
        res.message = "calibration data reset";
        ROS_INFO("[LidarYawCali] %s", res.message.c_str());
        return true;
    }

    // -----------------------------------------------------------------------
    bool saveCalibrationSrv(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        YawResult avg = yaw_calc_.getAveraged();
        if (!avg.valid) {
            res.success = false;
            res.message = "no valid calibration data yet";
            return true;
        }

        YawCalibOutput out;
        out.installation = install_params_;
        out.timestamp    = getCurrentTimestamp();

        // 标定增量
        out.delta_yaw_deg = avg.yaw_delta_deg;

        // 融合后 ZYX 欧拉角（四元数合成）
        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(avg.yaw_delta_rad, roll_f, pitch_f, yaw_f);
        out.x_calib         = install_params_.x;
        out.y_calib         = install_params_.y;
        out.z_calib         = install_params_.z;
        out.roll_calib_deg  = roll_f;
        out.pitch_calib_deg = pitch_f;
        out.yaw_calib_deg   = yaw_f;

        // 质量统计
        out.confidence  = avg.confidence;
        out.frames_used = static_cast<int>(yaw_window_deg_.size());
        double mean_y;
        computeMeanStd(yaw_window_deg_, mean_y, out.std_yaw_deg);
        out.quality_grade = gradeCalibration(
            out.confidence, out.std_yaw_deg, out.frames_used);

        if (!writeCalibrationFile(out)) {
            res.success = false;
            res.message = "failed to write: " + calib_output_file_;
            return true;
        }

        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "[%s] saved to %s | yaw_calib=%.4f° (delta=%.4f°) conf=%.3f",
                      out.quality_grade.c_str(), calib_output_file_.c_str(),
                      out.yaw_calib_deg, out.delta_yaw_deg, out.confidence);
        res.success = true;
        res.message = buf;
        ROS_INFO("[LidarYawCali] %s", res.message.c_str());
        LidarShmParam::SetModuleParam_yaw(install_params_.sensor_name, out.yaw_calib_deg);
        return true;
    }

    // -----------------------------------------------------------------------
    bool writeCalibrationFile(const YawCalibOutput& out) const
    {
        std::ofstream f(calib_output_file_);
        if (!f.is_open()) {
            ROS_ERROR("[LidarYawCali] cannot open: %s", calib_output_file_.c_str());
            return false;
        }

        const InstallationParams& ip = out.installation;
        f << std::fixed << std::setprecision(6);

        f << "# LiDAR Yaw 标定文件\n";
        f << "# 生成时间: " << out.timestamp << "\n";
        f << "# 坐标系: FLU (X=前,Y=左,Z=上)，VCS 原点为后轴中心投影到地面\n";
        f << "# 融合方法: R_calib = Rz(delta_yaw) * R_install  (四元数合成)\n";
        f << "# 标定场景: 车辆正前方 4~5 个锥形桶，沿 X 轴排列\n";
        f << "\n";
        f << ip.sensor_name << ":\n\n";

        // 原始安装参数
        f << "  # 原始安装参数（驱动使用的标称值，roll/pitch 已完成前一步标定）\n";
        f << "  installation:\n";
        f << "    x:         " << ip.x         << "    # m\n";
        f << "    y:         " << ip.y         << "    # m\n";
        f << "    z:         " << ip.z         << "    # m\n";
        f << "    roll_deg:  " << ip.roll_deg  << "    # 度（已标定值，本步骤未改变）\n";
        f << "    pitch_deg: " << ip.pitch_deg << "    # 度（已标定值，本步骤未改变）\n";
        f << "    yaw_deg:   " << ip.yaw_deg   << "    # 度（标定前估计值）\n";
        f << "    z_offset:  " << ip.z_offset  << "    # 后轴到地面高度 (m)\n";
        f << "\n";

        // 标定后参数
        f << "  # 标定后参数（融合：R_calib = Rz(delta_yaw) * R_install）\n";
        f << "  # yaw_calib  = yaw_install + delta_yaw（四元数合成后 ZYX 提取）\n";
        f << "  # roll/pitch 在 yaw 修正后保持不变（小角度近似），\n";
        f << "  #            四元数合成下有微小差异，以下为精确值。\n";
        f << "  calibrated:\n";
        f << "    x:         " << out.x_calib         << "    # m     (未变)\n";
        f << "    y:         " << out.y_calib         << "    # m     (未变)\n";
        f << "    z:         " << out.z_calib         << "    # m     (未变)\n";
        f << "    roll_deg:  " << out.roll_calib_deg  << "    # 度    (精确合成值)\n";
        f << "    pitch_deg: " << out.pitch_calib_deg << "    # 度    (精确合成值)\n";
        f << "    yaw_deg:   " << out.yaw_calib_deg   << "    # 度    (已更新)\n";
        f << "    z_offset:  " << ip.z_offset         << "    # m     (未变)\n";
        f << "\n";

        // 标定增量
        f << "  # 标定增量\n";
        f << "  # delta_yaw = -θ，θ 为锥桶连线相对 VCS X 轴的偏角\n";
        f << "  delta:\n";
        f << "    delta_yaw_deg: " << out.delta_yaw_deg << "    # 度（yaw 修正量）\n";
        f << "\n";

        // 质量评估
        f << "  # 标定质量\n";
        f << "  # EXCELLENT : confidence>=0.80, std_yaw<=0.10°, frames>=8\n";
        f << "  # GOOD      : confidence>=0.60, std_yaw<=0.20°, frames>=5\n";
        f << "  # ACCEPTABLE: confidence>=0.40, std_yaw<=0.50°, frames>=3\n";
        f << "  # POOR      : 未达上述条件，建议检查锥桶摆放或增加采集帧数\n";
        f << "  quality:\n";
        f << "    quality_grade:   " << out.quality_grade << "\n";
        f << "    confidence:      " << out.confidence
          << "    # 平均置信度（锥桶数量+共线性，越高越好）\n";
        f << "    std_yaw_deg:     " << out.std_yaw_deg
          << "    # yaw 测量标准差（°，越小越稳定）\n";
        f << "    frames_used:     " << out.frames_used << "\n";
        f << "    calibration_time: " << out.timestamp << "\n";

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
    ros::Subscriber sub_cloud_;
    ros::Publisher  pub_cone_cloud_, pub_result_, pub_status_, pub_markers_;
    ros::ServiceServer srv_trigger_, srv_reset_, srv_save_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string input_topic_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string calib_output_file_;

    InstallationParams install_params_;
    ConeDetector       cone_detector_;
    YawCalculator      yaw_calc_;

    // 质量统计窗口（与 yaw_calc_ 同步，用于 std 计算）
    std::deque<double> yaw_window_deg_;
    int window_size_{10};
};

} // namespace lidar_yaw1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_yaw_cali");
    ros::NodeHandle nh, pnh("~");
    lidar_yaw1::LidarYawCaliNode node(nh, pnh);
    ros::spin();
    return 0;
}
