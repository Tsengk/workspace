#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
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
#include <cmath>
#include <numeric>

#include "lidar_rp_cali/ground_detector.h"
#include "lidar_rp_cali/plane_fitter.h"
#include "lidar_rp_cali/calibration_calculator.h"
#include "lidar_rp_cali/types.h"
// #include "devkit_shm_param/include/lidar_shm_param.h"
#include "lidar_shm_param.h"

namespace lidar_rp_cali {

class LidarRpCaliNode {
public:
    LidarRpCaliNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), z_window_size_(10), min_confidence_(0.3)
    {
        loadParams();

        sub_cloud_ = nh_.subscribe(
            input_topic_, 1,
            &LidarRpCaliNode::cloudCallback, this);

        pub_ground_ = pnh_.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1);
        pub_result_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("result", 10);
        pub_status_ = pnh_.advertise<std_msgs::String>("status", 10);

        srv_trigger_ = pnh_.advertiseService(
            "trigger_calibration", &LidarRpCaliNode::triggerCalibration, this);
        srv_reset_ = pnh_.advertiseService(
            "reset_calibration", &LidarRpCaliNode::resetCalibration, this);
        srv_save_ = pnh_.advertiseService(
            "save_calibration", &LidarRpCaliNode::saveCalibrationSrv, this);

        ROS_INFO("[LidarRpCaliNode] init OK, listening: %s", input_topic_.c_str());
        ROS_INFO("[LidarRpCaliNode] install: z=%.3f roll=%.2f° pitch=%.2f° yaw=%.2f° z_offset=%.3f",
                 install_params_.z, install_params_.roll_deg,
                 install_params_.pitch_deg, install_params_.yaw_deg,
                 install_params_.z_offset);
        ROS_INFO("[LidarRpCaliNode] LiDAR to ground: %.3f m  output: %s",
                 install_params_.z + install_params_.z_offset,
                 calib_output_file_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    void loadParams()
    {
        pnh_.param<std::string>("input_topic", input_topic_, "/rslidar_points");
        pnh_.param<std::string>("lidar_frame", lidar_frame_, "lidar");
        pnh_.param<std::string>("base_frame",  base_frame_,  "base_link");
        pnh_.param<std::string>("calibration_output_file", calib_output_file_,
                                std::string("/tmp/lidar_calibration.yaml"));

        // 安装参数（驱动标称值）
        pnh_.param<std::string>("installation/sensor_name",
                                install_params_.sensor_name, std::string("lidar"));
        pnh_.param("installation/x",         install_params_.x,         0.0);
        pnh_.param("installation/y",         install_params_.y,         0.0);
        pnh_.param("installation/z",         install_params_.z,         1.8);
        pnh_.param("installation/roll_deg",  install_params_.roll_deg,  0.0);
        pnh_.param("installation/pitch_deg", install_params_.pitch_deg, 0.0);
        pnh_.param("installation/yaw_deg",   install_params_.yaw_deg,   0.0);
        pnh_.param("installation/z_offset",  install_params_.z_offset,  0.37);

        // 地面检测：以 -z_offset 为期望地面高度，自动计算默认 z 范围
        float expected_ground_z = -static_cast<float>(install_params_.z_offset);
        GroundDetector::Params gd_params;
        pnh_.param("ground/min_z",       gd_params.min_z,       expected_ground_z - 0.5f);
        pnh_.param("ground/max_z",        gd_params.max_z,        expected_ground_z + 0.3f);
        pnh_.param("ground/min_distance", gd_params.min_distance, 2.0f);
        pnh_.param("ground/max_distance", gd_params.max_distance, 30.0f);
        pnh_.param("ground/voxel_size",   gd_params.voxel_size,   0.2f);
        pnh_.param("ground/sor_mean_k",   gd_params.sor_mean_k,   20);
        pnh_.param("ground/sor_std_thresh", gd_params.sor_std_thresh, 1.5f);
        pnh_.param("ground/cluster_tolerance",  gd_params.cluster_tolerance, 0.5f);
        pnh_.param("ground/min_cluster_points", gd_params.min_cluster_points, 200);
        ground_detector_.setParams(gd_params);
        ROS_INFO("[LidarRpCaliNode] ground z range: [%.2f, %.2f]",
                 gd_params.min_z, gd_params.max_z);

        // 平面拟合
        PlaneFitter::Params pf_params;
        std::string method_str;
        pnh_.param<std::string>("plane/method", method_str, "RANSAC");
        if (method_str == "LEAST_SQUARES")
            pf_params.method = FitMethod::LEAST_SQUARES;
        else if (method_str == "PCA")
            pf_params.method = FitMethod::PCA;
        else
            pf_params.method = FitMethod::RANSAC;
        pnh_.param("plane/ransac_distance_threshold",
                   pf_params.ransac_distance_threshold, 0.05f);
        pnh_.param("plane/ransac_max_iterations",
                   pf_params.ransac_max_iterations, 200);
        plane_fitter_.setParams(pf_params);

        // 标定计算
        CalibrationCalculator::Params cc_params;
        pnh_.param("cali/max_roll_deg",   cc_params.max_roll_deg,   15.0);
        pnh_.param("cali/max_pitch_deg",  cc_params.max_pitch_deg,  15.0);
        pnh_.param("cali/window_size",    cc_params.window_size,    10);
        pnh_.param("cali/min_confidence", cc_params.min_confidence, 0.3);
        pnh_.param("cali/rms_weight",     cc_params.rms_weight,     0.5f);
        cali_calc_.setParams(cc_params);

        z_window_size_   = cc_params.window_size;
        min_confidence_  = cc_params.min_confidence;
    }

    // -----------------------------------------------------------------------
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *raw);

        // 1. 地面检测
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>());
        if (!ground_detector_.detectGround(raw, ground)) {
            publishStatus("ground detection failed");
            return;
        }
        sensor_msgs::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground, ground_msg);
        ground_msg.header = msg->header;
        pub_ground_.publish(ground_msg);

        // 2. 平面拟合
        PlaneModel plane;
        if (!plane_fitter_.fitPlane(ground, plane)) {
            publishStatus("plane fitting failed");
            return;
        }

        // 3. 地面高度更新（VCS 中 z 截距）
        updateZGroundWindow(plane);

        // 4. 标定计算（roll/pitch 残余）
        CalibrationResult result = cali_calc_.calculate(plane);
        if (!result.valid) {
            publishStatus("calibration failed: " + result.message);
            return;
        }

        // 5. 同步跟踪 roll/pitch 进入质量窗口
        //    只接受与 CalibrationCalculator 相同条件的帧（置信度过滤）
        if (result.confidence >= min_confidence_) {
            roll_window_deg_.push_back(result.roll_deg);
            pitch_window_deg_.push_back(result.pitch_deg);
            if (static_cast<int>(roll_window_deg_.size()) > z_window_size_) {
                roll_window_deg_.pop_front();
                pitch_window_deg_.pop_front();
            }
        }

        publishResult(result, msg->header.stamp);
        broadcastTF(cali_calc_.getAveraged(), msg->header.stamp);

        publishStatus("OK | roll=" + std::to_string(result.roll_deg) +
                      "° pitch=" + std::to_string(result.pitch_deg) +
                      "° z_gnd=" + std::to_string(getAveragedZGround()) +
                      "m conf=" + std::to_string(result.confidence));
    }

    // -----------------------------------------------------------------------
    // 从平面拟合结果更新地面高度窗口
    // z_ground = -d / n.z  （归一化平面方程：n·p + d = 0）
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
    // 统计工具：计算均值和标准差
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
    // 质量等级判定
    //
    // 综合三个维度：
    //   1. confidence   — 平面拟合质量（RMS误差+曲率）
    //   2. std_*        — 窗口内测量稳定性
    //   3. frames_used  — 样本数量充足性
    //
    // 等级 | confidence | std_roll/pitch | std_z   | frames
    // -----+------------+----------------+---------+-------
    // EXCELLENT | >=0.80 | <=0.10°       | <=0.005m | >=8
    // GOOD      | >=0.60 | <=0.20°       | <=0.010m | >=5
    // ACCEPTABLE| >=0.40 | <=0.50°       | <=0.020m | >=3
    // POOR      | otherwise
    static std::string gradeCalibration(double confidence,
                                        double std_roll, double std_pitch,
                                        double std_z, int frames)
    {
        if (confidence >= 0.80 &&
            std_roll   <= 0.10 && std_pitch <= 0.10 &&
            std_z      <= 0.005 && frames >= 8)
            return "EXCELLENT";

        if (confidence >= 0.60 &&
            std_roll   <= 0.20 && std_pitch <= 0.20 &&
            std_z      <= 0.010 && frames >= 5)
            return "GOOD";

        if (confidence >= 0.40 &&
            std_roll   <= 0.50 && std_pitch <= 0.50 &&
            std_z      <= 0.020 && frames >= 3)
            return "ACCEPTABLE";

        return "POOR";
    }

    // -----------------------------------------------------------------------
    // 计算融合后的 roll/pitch/yaw（四元数合成）
    //
    // 问题背景：当安装参数中 yaw ≠ 0（如侧向或后向雷达）时，
    // 标定在安装后坐标系中测得的 delta_roll/delta_pitch 对应的 VCS 修正
    // 并不是简单地在 VCS 欧拉角上相加。
    //
    // 正确做法：
    //   R_install = Rz(yaw) * Ry(pitch) * Rx(roll)
    //   R_delta   = Ry(delta_pitch) * Rx(delta_roll)  （在安装后坐标系测量）
    //   R_fused   = R_delta * R_install
    //
    // 然后从 R_fused 提取 ZYX 欧拉角：
    //   yaw   = atan2(R(1,0), R(0,0))
    //   pitch = atan2(-R(2,0), sqrt(R(0,0)^2 + R(1,0)^2))
    //   roll  = atan2(R(2,1), R(2,2))
    //
    // 示例说明简单相加的错误：
    //   后向雷达 yaw=180°，测得 delta_roll=+2°
    //   → 简单相加：roll_fused = 0 + 2 = +2°  ← 错误
    //   → 四元数合成：roll_fused ≈ -2°         ← 正确（X轴方向被 yaw 翻转）
    static void computeFusedEuler(const InstallationParams& ip,
                                  double delta_roll_deg, double delta_pitch_deg,
                                  double& roll_fused, double& pitch_fused,
                                  double& yaw_fused)
    {
        static constexpr double D2R = M_PI / 180.0;
        static constexpr double R2D = 180.0 / M_PI;

        // R_install = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::Quaterniond q_install =
            Eigen::AngleAxisd(ip.yaw_deg   * D2R, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ip.pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(ip.roll_deg  * D2R, Eigen::Vector3d::UnitX());

        // R_delta = Ry(delta_pitch) * Rx(delta_roll)
        Eigen::Quaterniond q_delta =
            Eigen::AngleAxisd(delta_pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(delta_roll_deg  * D2R, Eigen::Vector3d::UnitX());

        // R_fused = R_delta * R_install
        Eigen::Matrix3d R = (q_delta * q_install).normalized().toRotationMatrix();

        // ZYX 欧拉角提取（与 CalibrationCalculator 约定一致）
        pitch_fused = std::atan2(-R(2, 0),
                                  std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
        roll_fused  = std::atan2(R(2, 1), R(2, 2));
        yaw_fused   = std::atan2(R(1, 0), R(0, 0));

        roll_fused  *= R2D;
        pitch_fused *= R2D;
        yaw_fused   *= R2D;
    }

    // -----------------------------------------------------------------------
    // z 标定公式推导：
    //   驱动使用 z_install 做 LiDAR→VCS 变换。
    //   若真实安装高度为 z_true = z_install + Δz，
    //   则 VCS 中地面高度：z_meas = -Δz - z_offset
    //   → Δz = -(z_meas + z_offset)
    //   → z_calib = z_install + Δz = z_install - (z_meas + z_offset)
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
        ROS_INFO_THROTTLE(1.0, "[LidarRpCali] %s", s.c_str());
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
        CalibrationResult avg = cali_calc_.getAveraged();
        if (!avg.valid) {
            res.success = false;
            res.message = "no valid calibration data yet";
            return true;
        }
        double z_ground = getAveragedZGround();
        double z_calib  = computeCalibratedZ(z_ground);
        double roll_f, pitch_f, yaw_f;
        computeFusedEuler(install_params_, avg.roll_deg, avg.pitch_deg,
                          roll_f, pitch_f, yaw_f);
        res.success = true;
        res.message =
            "Z=" + std::to_string(z_calib) + "m"
            " Roll=" + std::to_string(roll_f) + "deg"
            " Pitch=" + std::to_string(pitch_f) + "deg"
            " conf=" + std::to_string(avg.confidence);
        ROS_INFO("[LidarRpCali] trigger: %s", res.message.c_str());
        return true;
    }

    bool resetCalibration(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& res)
    {
        cali_calc_.reset();
        z_ground_window_.clear();
        roll_window_deg_.clear();
        pitch_window_deg_.clear();
        res.success = true;
        res.message = "calibration data reset";
        ROS_INFO("[LidarRpCali] %s", res.message.c_str());
        return true;
    }

    // -----------------------------------------------------------------------
    bool saveCalibrationSrv(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res)
    {
        CalibrationResult avg = cali_calc_.getAveraged();
        if (!avg.valid || z_ground_window_.empty()) {
            res.success = false;
            res.message = avg.valid ? "no z_ground data yet" : "no valid calibration yet";
            return true;
        }

        CalibrationOutput out;
        out.installation      = install_params_;
        out.timestamp         = getCurrentTimestamp();

        // 增量（在安装后坐标系中测量）
        out.delta_roll_deg    = avg.roll_deg;
        out.delta_pitch_deg   = avg.pitch_deg;
        out.z_ground_measured = getAveragedZGround();
        out.z_ground_expected = -install_params_.z_offset;
        out.delta_z           = -(out.z_ground_measured + install_params_.z_offset);

        // 融合后参数（四元数合成）
        out.z_calib = computeCalibratedZ(out.z_ground_measured);
        computeFusedEuler(install_params_,
                          out.delta_roll_deg, out.delta_pitch_deg,
                          out.roll_calib_deg, out.pitch_calib_deg,
                          out.yaw_fused_deg);

        // 质量统计
        out.confidence  = avg.confidence;
        out.frames_used = static_cast<int>(z_ground_window_.size());
        double mean_r, mean_p, mean_z;
        computeMeanStd(roll_window_deg_,   mean_r, out.std_roll_deg);
        computeMeanStd(pitch_window_deg_,  mean_p, out.std_pitch_deg);
        computeMeanStd(z_ground_window_,   mean_z, out.std_z_m);
        out.quality_grade = gradeCalibration(out.confidence,
                                             out.std_roll_deg, out.std_pitch_deg,
                                             out.std_z_m, out.frames_used);

        if (!writeCalibrationFile(out)) {
            res.success = false;
            res.message = "failed to write: " + calib_output_file_;
            return true;
        }

        res.success = true;
        res.message = "[" + out.quality_grade + "] saved to " + calib_output_file_ +
                      " | z=" + std::to_string(out.z_calib) +
                      " roll=" + std::to_string(out.roll_calib_deg) + "deg" +
                      " pitch=" + std::to_string(out.pitch_calib_deg) + "deg";
        ROS_INFO("[LidarRpCali] %s", res.message.c_str());
        LidarShmParam::SetModuleParam_roll_pitch_z(install_params_.sensor_name, out.z_calib, out.roll_calib_deg, out.pitch_calib_deg);
        return true;
    }

    // -----------------------------------------------------------------------
    bool writeCalibrationFile(const CalibrationOutput& out) const
    {
        std::ofstream f(calib_output_file_);
        if (!f.is_open()) {
            ROS_ERROR("[LidarRpCali] cannot open file: %s", calib_output_file_.c_str());
            return false;
        }

        const InstallationParams& ip = out.installation;
        f << std::fixed << std::setprecision(6);

        f << "schema_version: 1\n";
        f << "calib_type: lidar_rp\n";
        f << "sensor_name: " << ip.sensor_name << "\n";
        f << "generated_at: " << out.timestamp << "\n";
        f << "method: ground_plane_auto\n";
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
        f << "  std_roll_deg: " << out.std_roll_deg << "\n";
        f << "  std_pitch_deg: " << out.std_pitch_deg << "\n";
        f << "  std_z_m: " << out.std_z_m << "\n";
        f << "  z_ground_measured: " << out.z_ground_measured << "\n";
        f << "  z_ground_expected: " << out.z_ground_expected << "\n";
        f << "  frames_used: " << out.frames_used << "\n";

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
    ros::Publisher  pub_ground_, pub_result_, pub_status_;
    ros::ServiceServer srv_trigger_, srv_reset_, srv_save_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string input_topic_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string calib_output_file_;

    InstallationParams    install_params_;
    GroundDetector        ground_detector_;
    PlaneFitter           plane_fitter_;
    CalibrationCalculator cali_calc_;

    // 滑动窗口（与 cali_calc_ 同步，用于质量统计）
    std::deque<double> z_ground_window_;
    std::deque<double> roll_window_deg_;
    std::deque<double> pitch_window_deg_;
    int    z_window_size_;
    double min_confidence_;
};

} // namespace lidar_rp_cali

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_rp_cali");
    ros::NodeHandle nh, pnh("~");
    lidar_rp_cali::LidarRpCaliNode node(nh, pnh);
    ros::spin();
    return 0;
}
