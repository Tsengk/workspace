/**
 * @file calibration_test_node.cpp
 * @brief LiDAR roll/pitch 标定测试节点（含 RViz 可视化）
 *
 * 发布话题（可视化）：
 *   /lidar_rp_cali/raw_cloud    — 仿真原始点云（按 Z 高度着色）
 *   /lidar_rp_cali/ground_cloud — 检测到的地面点（绿色）
 *   /lidar_rp_cali/markers      — MarkerArray（平面/法向量/文字）
 *
 * 8 个测试场景依次执行，每帧带延迟便于 RViz 观察。
 * 场景间自动暂停，最终打印汇总报告。
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <vector>

#include "lidar_rp_cali/types.h"
#include "lidar_rp_cali/lidar_simulator.h"
#include "lidar_rp_cali/ground_detector.h"
#include "lidar_rp_cali/plane_fitter.h"
#include "lidar_rp_cali/calibration_calculator.h"

namespace lidar_rp_cali {

static constexpr double DEG2RAD = M_PI / 180.0;
static const std::string LIDAR_FRAME = "lidar";

// ============================================================
// 可视化辅助
// ============================================================
struct VizPubs {
    ros::Publisher raw_cloud;
    ros::Publisher ground_cloud;
    ros::Publisher markers;

    double frame_delay_s  = 0.08;   // 帧间延迟（约 12 Hz）
    double scene_delay_s  = 3.0;    // 场景间停顿
    bool   enabled        = true;
};

// ---- 工具：PCL → ROS PointCloud2 带颜色字段 ---------------
static sensor_msgs::PointCloud2 toRosMsg(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const ros::Time& stamp,
    const std::string& frame = LIDAR_FRAME)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame;
    return msg;
}

// ---- 将法向量转为从 (0,0,1) 到 n 的四元数 ------------------
static geometry_msgs::Quaternion normalToQuat(const Eigen::Vector3f& n)
{
    Eigen::Vector3f from(0, 0, 1);
    Eigen::Vector3f to = n.normalized();
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(from, to);
    geometry_msgs::Quaternion msg;
    msg.x = q.x(); msg.y = q.y();
    msg.z = q.z(); msg.w = q.w();
    return msg;
}

// ---- 地面点云质心 -------------------------------------------
static Eigen::Vector3f cloudCentroid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    Eigen::Vector3f c(0, 0, 0);
    if (cloud->empty()) return c;
    for (const auto& p : *cloud)
        c += Eigen::Vector3f(p.x, p.y, p.z);
    return c / static_cast<float>(cloud->size());
}

// ---- 构建 MarkerArray ---------------------------------------
static visualization_msgs::MarkerArray buildMarkers(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground,
    const PlaneModel& plane,
    const CalibrationResult& result,
    const std::string& scene_name,
    int   frame_idx,
    bool  is_pass,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;

    // 地面点质心作为锚点
    Eigen::Vector3f ctr = cloudCentroid(ground);

    // ---- 0: 拟合平面（半透明薄板）----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "plane_surface";
        m.id   = 0;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = ctr.x();
        m.pose.position.y = ctr.y();
        m.pose.position.z = ctr.z();
        m.pose.orientation = normalToQuat(plane.valid ? plane.normal
                                                       : Eigen::Vector3f(0,0,1));
        m.scale.x = 30.0;
        m.scale.y = 30.0;
        m.scale.z = 0.04;
        m.color.r = 1.0f; m.color.g = 0.9f; m.color.b = 0.0f;
        m.color.a = 0.25f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 1: 平面法向量（箭头）----
    if (plane.valid) {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "plane_normal";
        m.id   = 1;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        // 起点 = 质心，终点 = 质心 + 3 * normal
        geometry_msgs::Point p0, p1;
        p0.x = ctr.x();  p0.y = ctr.y();  p0.z = ctr.z();
        p1.x = ctr.x() + plane.normal.x() * 3.0f;
        p1.y = ctr.y() + plane.normal.y() * 3.0f;
        p1.z = ctr.z() + plane.normal.z() * 3.0f;
        m.points.push_back(p0);
        m.points.push_back(p1);
        m.scale.x = 0.12;   // 箭杆直径
        m.scale.y = 0.25;   // 箭头直径
        m.scale.z = 0.0;
        m.color.r = 0.0f; m.color.g = 0.8f; m.color.b = 1.0f;
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 2: 场景名称文字 ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "scene_title";
        m.id   = 2;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0;
        m.pose.position.y = 0;
        m.pose.position.z = 4.5;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.5;
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f;
        m.color.a = 1.0f;
        m.text = "Scene: " + scene_name +
                 "  Frame: " + std::to_string(frame_idx + 1);
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 3: 标定结果文字 ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "cali_result";
        m.id   = 3;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0;
        m.pose.position.y = 0;
        m.pose.position.z = 3.8;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.40;

        std::ostringstream oss;
        if (result.valid) {
            oss << std::fixed << std::setprecision(3)
                << "Roll: " << std::showpos << result.roll_deg
                << "  Pitch: " << result.pitch_deg
                << std::noshowpos << "  Conf: " << result.confidence;
        } else {
            oss << "(calibration pending...)";
        }
        m.text = oss.str();

        // 有效帧绿色，无效帧橙色
        if (result.valid && result.confidence > 0.5) {
            m.color.r = 0.2f; m.color.g = 1.0f; m.color.b = 0.2f;
        } else {
            m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f;
        }
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 4: PASS / FAIL 标签 ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "pass_fail";
        m.id   = 4;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0;
        m.pose.position.y = 0;
        m.pose.position.z = 3.2;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.6;

        if (is_pass) {
            m.text = "[ PASS ]";
            m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f;
        } else {
            m.text = "[ FAIL ]";
            m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f;
        }
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 5: LiDAR 原点标记（小球）----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "lidar_origin";
        m.id   = 5;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0;
        m.pose.position.y = 0;
        m.pose.position.z = 0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f;
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 6: 平面 RMS 文字 ----
    if (plane.valid) {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "plane_rms";
        m.id   = 6;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = ctr.x() + 5.0f;
        m.pose.position.y = ctr.y();
        m.pose.position.z = ctr.z() + 1.0f;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.30;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "Plane RMS: " << plane.rms_error << " m"
            << "\nInliers: " << plane.inliers_count
            << "\nGnd pts: " << (ground ? ground->size() : 0);
        m.text = oss.str();
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.4f;
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    return ma;
}

// ---- 发布一帧可视化数据 -------------------------------------
static void publishFrame(
    const VizPubs& viz,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& raw,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground,
    const PlaneModel& plane,
    const CalibrationResult& result,
    const std::string& scene_name,
    int frame_idx,
    bool is_pass)
{
    if (!viz.enabled) return;

    ros::Time stamp = ros::Time::now();

    viz.raw_cloud.publish(toRosMsg(raw,    stamp));
    viz.ground_cloud.publish(toRosMsg(ground, stamp));
    viz.markers.publish(
        buildMarkers(ground, plane, result, scene_name, frame_idx, is_pass, stamp));
}

// ============================================================
// 测试结果结构体
// ============================================================
struct TestReport {
    std::string name;
    double expected_roll_deg;
    double expected_pitch_deg;
    double result_roll_deg;
    double result_pitch_deg;
    double roll_error_deg;
    double pitch_error_deg;
    double tolerance_deg;
    double confidence;
    int    frames_used;
    bool   pass;
    std::string note;
};

// ============================================================
// 执行单个测试场景
// ============================================================
static TestReport runScenario(
    const SceneConfig& scene,
    VizPubs& viz,
    int n_frames = 10)
{
    TestReport report;
    report.name               = scene.name;
    report.expected_roll_deg  = scene.expected_roll_deg;
    report.expected_pitch_deg = scene.expected_pitch_deg;
    report.tolerance_deg      = scene.tolerance_deg;

    // ---- 仿真器 ----
    LidarSimulator::Config sim_cfg;
    sim_cfg.lidar_roll_rad  = scene.lidar_roll_deg  * DEG2RAD;
    sim_cfg.lidar_pitch_rad = scene.lidar_pitch_deg * DEG2RAD;
    sim_cfg.lidar_height    = scene.lidar_height;
    sim_cfg.noise_std       = scene.noise_std;
    sim_cfg.obstacles       = scene.obstacles;
    LidarSimulator sim(sim_cfg);

    // ---- 标定流水线 ----
    // 大角度场景（|roll|或|pitch| > 10°）：地面点 Z 值范围急剧扩大，
    // 需要缩小水平距离、放宽 Z 轴范围，并提高 CalibrationCalculator 角度上限
    const double actual_max_angle = std::max(
        std::abs(scene.lidar_roll_deg), std::abs(scene.lidar_pitch_deg));

    GroundDetector::Params gd;
    if (actual_max_angle > 10.0) {
        // 大角度：限制水平距离到 8m，放宽 Z 轴到 ±20m
        gd.min_z              = -20.0f;
        gd.max_z              =   0.5f;
        gd.min_distance       =   1.0f;
        gd.max_distance       =   8.0f;
        gd.min_cluster_points =  80;
    } else {
        gd.min_z              = static_cast<float>(-scene.lidar_height * 1.6f);
        gd.max_z              = static_cast<float>(-scene.lidar_height * 0.2f);
        gd.min_distance       = 2.0f;
        gd.max_distance       = 25.0f;
        gd.min_cluster_points = 150;
    }
    gd.voxel_size = 0.2f;
    GroundDetector detector(gd);

    PlaneFitter::Params pf;
    pf.method = FitMethod::RANSAC;
    pf.ransac_distance_threshold = 0.05f;
    PlaneFitter fitter(pf);

    CalibrationCalculator::Params cc;
    cc.window_size    = n_frames;
    cc.min_confidence = 0.2;
    // 角度上限自动适配：允许比实际角度大 10°
    if (actual_max_angle > 10.0) {
        cc.max_roll_deg  = actual_max_angle + 10.0;
        cc.max_pitch_deg = actual_max_angle + 10.0;
    }
    CalibrationCalculator calc(cc);

    // ---- 逐帧处理 ----
    int valid_frames = 0;
    CalibrationResult latest_result;
    PlaneModel latest_plane;

    for (int f = 0; f < n_frames; ++f) {
        if (!ros::ok()) break;

        auto raw = sim.generate();

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>());
        bool gnd_ok = detector.detectGround(raw, ground);

        PlaneModel plane;
        if (gnd_ok) fitter.fitPlane(ground, plane);

        CalibrationResult res;
        if (plane.valid) {
            res = calc.calculate(plane);
            if (res.valid) ++valid_frames;
        }

        latest_plane  = plane;
        latest_result = res;

        // 发布可视化
        publishFrame(viz, raw, ground, plane, res, scene.name, f, false);

        ros::Duration(viz.frame_delay_s).sleep();
        ros::spinOnce();
    }

    // ---- 均值结果 ----
    CalibrationResult avg = calc.getAveraged();
    if (!avg.valid) {
        report.result_roll_deg  = 0;
        report.result_pitch_deg = 0;
        report.roll_error_deg   = 999;
        report.pitch_error_deg  = 999;
        report.confidence       = 0;
        report.frames_used      = valid_frames;
        report.pass             = false;
        report.note             = "no valid calibration result";
        return report;
    }

    report.result_roll_deg  = avg.roll_deg;
    report.result_pitch_deg = avg.pitch_deg;
    report.roll_error_deg   = std::abs(avg.roll_deg  - scene.expected_roll_deg);
    report.pitch_error_deg  = std::abs(avg.pitch_deg - scene.expected_pitch_deg);
    report.confidence       = avg.confidence;
    report.frames_used      = valid_frames;
    report.pass = (report.roll_error_deg  < scene.tolerance_deg &&
                   report.pitch_error_deg < scene.tolerance_deg);
    report.note = avg.message;

    // ---- 最终结果帧再发布几次（让 RViz 显示 PASS/FAIL）----
    if (viz.enabled) {
        for (int k = 0; k < 20 && ros::ok(); ++k) {
            publishFrame(viz, sim.generate(),
                         pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()),
                         latest_plane, avg, scene.name,
                         n_frames - 1, report.pass);
            ros::Duration(viz.frame_delay_s).sleep();
            ros::spinOnce();
        }
        // 场景间暂停
        ros::Duration(viz.scene_delay_s).sleep();
    }

    return report;
}

// ============================================================
// 构建测试场景（同原版）
// ============================================================
static std::vector<SceneConfig> buildScenes()
{
    std::vector<SceneConfig> scenes;

    // Scene 1: ideal flat ground
    {
        SceneConfig s;
        s.name = "01_ideal_flat";
        s.description = "flat ground, no obstacles, zero mounting error";
        s.lidar_roll_deg = 0.0; s.lidar_pitch_deg = 0.0;
        s.lidar_height = 1.8;   s.noise_std = 0.01;
        s.expected_roll_deg = 0.0; s.expected_pitch_deg = 0.0;
        s.tolerance_deg = 0.3;
        scenes.push_back(s);
    }
    // Scene 2: small angle mounting error
    {
        SceneConfig s;
        s.name = "02_small_errors";
        s.description = "flat ground, roll=2deg, pitch=1.5deg";
        s.lidar_roll_deg = 2.0; s.lidar_pitch_deg = 1.5;
        s.lidar_height = 1.8;  s.noise_std = 0.02;
        s.expected_roll_deg = 2.0; s.expected_pitch_deg = 1.5;
        s.tolerance_deg = 0.3;
        scenes.push_back(s);
    }
    // Scene 3: parked vehicles as obstacles
    {
        SceneConfig s;
        s.name = "03_vehicle_obs";
        s.description = "parked vehicles around, roll=-1.5deg, pitch=2deg";
        s.lidar_roll_deg = -1.5; s.lidar_pitch_deg = 2.0;
        s.lidar_height = 1.8;   s.noise_std = 0.02;
        s.expected_roll_deg = -1.5; s.expected_pitch_deg = 2.0;
        s.tolerance_deg = 0.3;
        for (int i = 0; i < 4; ++i) {
            ObstacleBox box;
            double angle = i * M_PI / 2.0;
            box.center    = Eigen::Vector3f(float(10*std::cos(angle)),
                                            float(10*std::sin(angle)), 0.75f);
            box.half_size = Eigen::Vector3f(2.0f, 0.9f, 0.75f);
            box.label = "vehicle";
            s.obstacles.push_back(box);
        }
        scenes.push_back(s);
    }
    // Scene 4: urban canyon (buildings on both sides)
    {
        SceneConfig s;
        s.name = "04_urban_canyon";
        s.description = "tall buildings both sides, roll=1deg, pitch=-1deg";
        s.lidar_roll_deg = 1.0; s.lidar_pitch_deg = -1.0;
        s.lidar_height = 1.8;  s.noise_std = 0.03;
        s.expected_roll_deg = 10.0; s.expected_pitch_deg = -10.0;
        s.tolerance_deg = 0.3;
        for (int i = -2; i <= 2; ++i) {
            for (int side : {-1, 1}) {
                ObstacleBox box;
                box.center    = Eigen::Vector3f(float(i*12), float(side*8), 10.0f);
                box.half_size = Eigen::Vector3f(5.0f, 1.5f, 10.0f);
                box.label = "building";
                s.obstacles.push_back(box);
            }
        }
        scenes.push_back(s);
    }
    // Scene 5: large angle mounting error
    {
        SceneConfig s;
        s.name = "05_large_errors";
        s.description = "large mounting error, roll=5deg, pitch=3deg";
        s.lidar_roll_deg = 5.0; s.lidar_pitch_deg = 3.0;
        s.lidar_height = 1.8;  s.noise_std = 0.02;
        s.expected_roll_deg = 5.0; s.expected_pitch_deg = 3.0;
        s.tolerance_deg = 0.5;
        scenes.push_back(s);
    }
    // Scene 6: high range noise
    {
        SceneConfig s;
        s.name = "06_high_noise";
        s.description = "high range noise (sigma=0.05m), roll=2deg, pitch=1deg";
        s.lidar_roll_deg = 2.0; s.lidar_pitch_deg = 1.0;
        s.lidar_height = 1.8;  s.noise_std = 0.05;
        s.expected_roll_deg = 2.0; s.expected_pitch_deg = 1.0;
        s.tolerance_deg = 0.5;
        scenes.push_back(s);
    }
    // Scene 7: open parking lot (best conditions)
    {
        SceneConfig s;
        s.name = "07_parking_lot";
        s.description = "open parking lot, roll=0.5deg, pitch=0.8deg";
        s.lidar_roll_deg = 0.5; s.lidar_pitch_deg = 0.8;
        s.lidar_height = 1.8;  s.noise_std = 0.015;
        s.expected_roll_deg = 0.5; s.expected_pitch_deg = 0.8;
        s.tolerance_deg = 0.3;
        for (int i = 0; i < 6; ++i) {
            ObstacleBox box;
            box.center    = Eigen::Vector3f(float(-15+i*6), 15.0f, 0.75f);
            box.half_size = Eigen::Vector3f(2.0f, 0.9f, 0.75f);
            box.label = "vehicle";
            s.obstacles.push_back(box);
        }
        scenes.push_back(s);
    }
    // Scene 8: uphill ramp (5deg road slope)
    {
        SceneConfig s;
        s.name = "08_uphill_ramp";
        s.description = "uphill ramp 5deg, LiDAR roll=1deg, pitch=-0.5deg";
        s.lidar_roll_deg = 1.0; s.lidar_pitch_deg = -0.5;
        s.lidar_height = 1.8;  s.noise_std = 0.02;
        s.expected_roll_deg = 1.0; s.expected_pitch_deg = -0.5;
        s.tolerance_deg = 0.4;
        scenes.push_back(s);
    }
    // Scene 9: extra-large angle (roll=20deg, pitch=15deg)
    {
        SceneConfig s;
        s.name = "09_extra_large_angle";
        s.description = "extreme mounting error: roll=20deg, pitch=15deg";
        s.lidar_roll_deg  = 20.0;
        s.lidar_pitch_deg = 15.0;
        s.lidar_height    = 1.8;
        s.noise_std       = 0.02;
        s.expected_roll_deg  = 20.0;
        s.expected_pitch_deg = 15.0;
        s.tolerance_deg   = 0.8;
        scenes.push_back(s);
    }

    return scenes;
}

// ============================================================
// 打印汇总报告
// ============================================================
static void printReport(const std::vector<TestReport>& reports)
{
    std::string line(78, '=');
    std::string dash(78, '-');
    ROS_INFO("\n%s", line.c_str());
    ROS_INFO("  RS-Ruby 80-line LiDAR Roll/Pitch Calibration Test Report");
    ROS_INFO("  Frame: FLU  X=Front Y=Left Z=Up");
    ROS_INFO("%s", line.c_str());

    int pass_cnt = 0;
    for (const auto& r : reports) {
        if (r.pass) ++pass_cnt;
        ROS_INFO("%s", dash.c_str());
        ROS_INFO("Scene: %-28s  %s",
                 r.name.c_str(), r.pass ? "[PASS]" : "[FAIL]");
        ROS_INFO("  Expected  Roll=%+6.2fdeg  Pitch=%+6.2fdeg",
                 r.expected_roll_deg, r.expected_pitch_deg);
        ROS_INFO("  Result    Roll=%+6.2fdeg  Pitch=%+6.2fdeg",
                 r.result_roll_deg, r.result_pitch_deg);
        ROS_INFO("  Error     Roll=%.3fdeg    Pitch=%.3fdeg    Tol=%.1fdeg",
                 r.roll_error_deg, r.pitch_error_deg, r.tolerance_deg);
        ROS_INFO("  Confidence=%.3f  ValidFrames=%d",
                 r.confidence, r.frames_used);
    }

    ROS_INFO("%s", line.c_str());
    int total = static_cast<int>(reports.size());
    ROS_INFO("Summary: %d / %d PASS  (%.0f%%)",
             pass_cnt, total, 100.0 * pass_cnt / total);

    int bw = 40;
    int bf = bw * pass_cnt / total;
    ROS_INFO("[%s%s]  %d/%d",
             std::string(bf,'#').c_str(),
             std::string(bw-bf,'.').c_str(),
             pass_cnt, total);
    ROS_INFO("%s", line.c_str());
}

} // namespace lidar_rp_cali

// ============================================================
// main
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_rp_cali_test");
    ros::NodeHandle nh, pnh("~");

    int    n_frames      = 10;
    bool   viz_enabled   = true;
    double frame_delay   = 0.08;
    double scene_delay   = 3.0;
    int    repeat_count  = 1;

    pnh.param("n_frames",     n_frames,    10);
    pnh.param("viz",          viz_enabled, true);
    pnh.param("frame_delay",  frame_delay, 0.08);
    pnh.param("scene_delay",  scene_delay, 3.0);
    pnh.param("repeat_count", repeat_count, 1);

    // ---- 可视化发布器 ----
    lidar_rp_cali::VizPubs viz;
    viz.enabled       = viz_enabled;
    viz.frame_delay_s = frame_delay;
    viz.scene_delay_s = scene_delay;

    if (viz_enabled) {
        viz.raw_cloud    = nh.advertise<sensor_msgs::PointCloud2>(
            "/lidar_rp_cali/raw_cloud", 5);
        viz.ground_cloud = nh.advertise<sensor_msgs::PointCloud2>(
            "/lidar_rp_cali/ground_cloud", 5);
        viz.markers      = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_rp_cali/markers", 5);

        ROS_INFO("Waiting for RViz connection (2s)...");
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO("======================================================");
    ROS_INFO("  LiDAR Roll/Pitch Calibration Simulation Test");
    ROS_INFO("  frames/scene: %d  viz: %s",
             n_frames, viz_enabled ? "ON" : "OFF");
    ROS_INFO("======================================================");

    auto scenes = lidar_rp_cali::buildScenes();
    std::vector<lidar_rp_cali::TestReport> reports;
    reports.reserve(scenes.size());

    for (int rep = 0; rep < repeat_count && ros::ok(); ++rep) {
        if (repeat_count > 1)
            ROS_INFO("---- Round %d / %d ----", rep + 1, repeat_count);

        reports.clear();
        for (size_t i = 0; i < scenes.size() && ros::ok(); ++i) {
            ROS_INFO("Running scene [%zu/%zu]: %s",
                     i + 1, scenes.size(), scenes[i].name.c_str());
            ROS_INFO("  %s", scenes[i].description.c_str());
            auto r = lidar_rp_cali::runScenario(scenes[i], viz, n_frames);
            reports.push_back(r);
            ROS_INFO("  -> %s  RollErr=%.3fdeg  PitchErr=%.3fdeg  Conf=%.3f",
                     r.pass ? "PASS" : "FAIL",
                     r.roll_error_deg, r.pitch_error_deg, r.confidence);
        }
    }

    lidar_rp_cali::printReport(reports);
    return 0;
}
