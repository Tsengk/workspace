/**
 * @file gnss_yaw_test_node.cpp
 * @brief LiDAR-GNSS 联合 Yaw 标定算法测试节点（含 RViz 可视化）
 *
 * 直接使用合成快照（GNSS 位姿 + VCS 锥桶坐标）测试 YawCalibrator，
 * 无需真实硬件，4 个测试场景验证不同 yaw 误差量级下的标定精度。
 *
 * 标定原理摘要：
 *   - 5 个锥桶固定在全局 UTM 坐标系中（East 方向，yg=10~18m）
 *   - 车辆以 8 种不同航向（0°~315°）观测锥桶
 *   - 每次观测生成一个快照：VCS 坐标（驱动用 yaw_install=0 变换后含残差 ε）
 *   - YawCalibrator 搜索最优 ε* 使全局投影点集最集中
 *
 * 发布话题（RViz 可视化）：
 *   /gnss_yaw_test/global_cones     visualization_msgs/MarkerArray  全局锥桶
 *   /gnss_yaw_test/vehicle_poses    visualization_msgs/MarkerArray  车辆位姿箭头
 *   /gnss_yaw_test/projected_cones  visualization_msgs/MarkerArray  以最优 ε 投影的锥桶
 *   /gnss_yaw_test/result_text      visualization_msgs/MarkerArray  标定结果文字
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include "lidar_gnss_yaw/types.h"
#include "lidar_gnss_yaw/yaw_calibrator.h"

namespace gnss_yaw_test {

static constexpr double DEG2RAD = M_PI / 180.0;
static const std::string MAP_FRAME = "map";

// ===========================================================================
// 测试场景
// ===========================================================================
struct SceneConfig {
    std::string name;
    std::string description;
    double true_yaw_err_deg;  // 真实 yaw 误差 ε（待标定量，°）
    double tolerance_deg;     // 验收阈值（°）
};

// ===========================================================================
// 测试结果
// ===========================================================================
struct TestReport {
    std::string name;
    double expected_delta_deg;
    double result_delta_deg;
    double error_deg;
    double tolerance_deg;
    double rmse_m;
    int    num_snapshots;
    double heading_spread;
    bool   pass;
};

// ===========================================================================
// 可视化发布器
// ===========================================================================
struct VizPubs {
    ros::Publisher global_cones;
    ros::Publisher vehicle_poses;
    ros::Publisher projected_cones;
    ros::Publisher result_text;
    bool   enabled       = true;
    double scene_delay_s = 3.0;
};

// ===========================================================================
// R_bg 矩阵（VCS → UTM，heading=h° 时）
//   R_bg(h) = [[cos(h), sin(h)], [sin(h), -cos(h)]]
//   其中 h 为 GNSS 航向角（正北=0°，顺时针+）
// ===========================================================================
static Eigen::Matrix2d Rbg(double heading_deg)
{
    double h = heading_deg * DEG2RAD;
    Eigen::Matrix2d m;
    m(0, 0) =  std::cos(h); m(0, 1) =  std::sin(h);
    m(1, 0) =  std::sin(h); m(1, 1) = -std::cos(h);
    return m;
}

// ===========================================================================
// 生成一个测试快照
//   cones_utm:  全局锥桶 UTM 坐标 (xg=北, yg=东)，单位 m
//   veh_xg/yg:  车辆 UTM 位置 (m)
//   heading:    GNSS 航向 (°)
//   true_yaw_err: 真实 yaw 误差 ε（°），yaw_install=0° 时的残差
// ===========================================================================
static lidar_gnss_yaw::Snapshot generateSnapshot(
    const std::vector<Eigen::Vector2d>& cones_utm,
    double veh_xg, double veh_yg,
    double heading_deg,
    double true_yaw_err_deg)
{
    double eps = true_yaw_err_deg * DEG2RAD;
    Eigen::Matrix2d Rbg_h = Rbg(heading_deg);
    Eigen::Vector2d veh_pos(veh_xg, veh_yg);

    // R(-ε) 旋转矩阵
    Eigen::Matrix2d R_neg_eps;
    R_neg_eps(0, 0) =  std::cos(eps); R_neg_eps(0, 1) = std::sin(eps);
    R_neg_eps(1, 0) = -std::sin(eps); R_neg_eps(1, 1) = std::cos(eps);

    lidar_gnss_yaw::Snapshot snap;
    snap.gnss.xg        = veh_xg;
    snap.gnss.yg        = veh_yg;
    snap.gnss.heading   = heading_deg;
    snap.gnss.timestamp = 0.0;
    snap.gnss.valid     = true;

    for (const auto& p_utm : cones_utm) {
        // 真实 VCS 坐标
        Eigen::Vector2d p_vcs = Rbg_h * (p_utm - veh_pos);
        // 驱动输出（含 yaw 残差 ε）：q = R(-ε) * p_vcs（yaw_install=0, t_lb=0）
        Eigen::Vector2d q = R_neg_eps * p_vcs;
        snap.cones_vcs.push_back(q);
    }

    return snap;
}

// ===========================================================================
// 构建全局锥桶 Marker（蓝色球体）
// ===========================================================================
static visualization_msgs::MarkerArray buildGlobalConeMarkers(
    const std::vector<Eigen::Vector2d>& cones_utm,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(del);

    for (int i = 0; i < static_cast<int>(cones_utm.size()); ++i) {
        // 球体
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = MAP_FRAME;
        m.ns     = "global_cones";
        m.id     = i;
        m.type   = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;
        // UTM: x=北(xg), y=东(yg) → RViz x=东, y=北 调换
        m.pose.position.x = cones_utm[i][1];   // yg = 东 → RViz x
        m.pose.position.y = cones_utm[i][0];   // xg = 北 → RViz y
        m.pose.position.z = 0;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.35; m.scale.y = 0.35; m.scale.z = 0.70;
        m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 1.0f; m.color.a = 0.90f;
        m.lifetime = ros::Duration(0);  // 持久显示
        ma.markers.push_back(m);

        // 编号
        visualization_msgs::Marker txt;
        txt.header    = m.header;
        txt.ns        = "cone_labels";
        txt.id        = 100 + i;
        txt.type      = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action    = visualization_msgs::Marker::ADD;
        txt.pose      = m.pose;
        txt.pose.position.z += 0.8;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 0.30;
        txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
        char buf[8];
        std::snprintf(buf, sizeof(buf), "C%d", i + 1);
        txt.text    = buf;
        txt.lifetime = ros::Duration(0);
        ma.markers.push_back(txt);
    }
    return ma;
}

// ===========================================================================
// 构建车辆位姿 Marker（航向箭头）
// ===========================================================================
static visualization_msgs::MarkerArray buildVehiclePoseMarkers(
    const std::vector<lidar_gnss_yaw::Snapshot>& snaps,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(del);

    for (int i = 0; i < static_cast<int>(snaps.size()); ++i) {
        const auto& g = snaps[i].gnss;
        visualization_msgs::Marker arrow;
        arrow.header.stamp    = stamp;
        arrow.header.frame_id = MAP_FRAME;
        arrow.ns     = "vehicle_arrows";
        arrow.id     = i;
        arrow.type   = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        // 箭头起点
        geometry_msgs::Point p0;
        p0.x = g.yg; p0.y = g.xg; p0.z = 0.1;  // UTM(xg=N,yg=E) → RViz(x=E,y=N)

        // 箭头终点（沿航向 2m）
        double h = g.heading * DEG2RAD;
        geometry_msgs::Point p1;
        p1.x = g.yg + 2.0 * std::sin(h);   // 东分量
        p1.y = g.xg + 2.0 * std::cos(h);   // 北分量
        p1.z = 0.1;

        arrow.points.push_back(p0);
        arrow.points.push_back(p1);
        arrow.scale.x = 0.15;
        arrow.scale.y = 0.30;
        arrow.scale.z = 0.0;
        arrow.color.r = 1.0f; arrow.color.g = 0.8f; arrow.color.b = 0.0f; arrow.color.a = 0.90f;
        arrow.lifetime = ros::Duration(0);
        ma.markers.push_back(arrow);

        // 编号
        visualization_msgs::Marker txt;
        txt.header    = arrow.header;
        txt.ns        = "vehicle_labels";
        txt.id        = 100 + i;
        txt.type      = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action    = visualization_msgs::Marker::ADD;
        txt.pose.position.x = p0.x;
        txt.pose.position.y = p0.y;
        txt.pose.position.z = 0.6;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 0.35;
        txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 0.5f; txt.color.a = 1.0f;
        char buf[8];
        std::snprintf(buf, sizeof(buf), "V%d", i + 1);
        txt.text    = buf;
        txt.lifetime = ros::Duration(0);
        ma.markers.push_back(txt);
    }
    return ma;
}

// ===========================================================================
// 构建标定结果文字 Marker
// ===========================================================================
static visualization_msgs::MarkerArray buildResultMarkers(
    const lidar_gnss_yaw::CalibResult& result,
    const std::string& scene_name,
    double expected_delta_deg,
    bool is_pass,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;

    auto addText = [&](int id, const std::string& ns,
                       double x, double y, double z,
                       double scale, const std::string& text,
                       float r, float g, float b)
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = MAP_FRAME;
        m.ns   = ns; m.id = id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
        m.pose.orientation.w = 1.0;
        m.scale.z = scale;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
        m.text    = text;
        m.lifetime = ros::Duration(0);
        ma.markers.push_back(m);
    };

    // 在地图坐标系内居中显示（相对于锥桶群右侧）
    double tx = 25, ty = 5;

    addText(0, "scene_title", tx, ty, 4.0, 0.55,
            "Scene: " + scene_name,
            1.0f, 1.0f, 1.0f);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    if (result.success) {
        double err = std::abs(result.delta_yaw - expected_delta_deg);
        oss << "True ε: " << std::showpos << expected_delta_deg << "deg"
            << "  Got delta_yaw: " << result.delta_yaw << "deg"
            << std::noshowpos
            << "\nRMSE: " << result.rmse_m * 100.0 << "cm"
            << "  Heading spread: " << result.heading_spread << "deg"
            << "  Snapshots: " << result.num_snapshots
            << "\nError: " << err << "deg";
    } else {
        oss << "Calibration FAILED: " << result.message;
    }
    addText(1, "cali_result", tx, ty, 3.2, 0.38, oss.str(),
            result.success ? 0.2f : 1.0f,
            result.success ? 1.0f : 0.3f,
            0.2f);

    addText(2, "pass_fail", tx, ty, 2.3, 0.60,
            is_pass ? "[ PASS ]" : "[ FAIL ]",
            is_pass ? 0.0f : 1.0f,
            is_pass ? 1.0f : 0.0f,
            0.0f);

    return ma;
}

// ===========================================================================
// 车辆状态（UTM 坐标 + 航向）
// ===========================================================================
struct VehicleState {
    double xg;       // 北向 (m)
    double yg;       // 东向 (m)
    double heading;  // 航向 (°)，正北=0，顺时针+
};

// ===========================================================================
// 执行单个测试场景
// ===========================================================================
static TestReport runScenario(
    const SceneConfig& scene,
    VizPubs& viz)
{
    TestReport report;
    report.name               = scene.name;
    report.expected_delta_deg = scene.true_yaw_err_deg;
    report.tolerance_deg      = scene.tolerance_deg;

    // 全局 UTM 锥桶坐标（沿 East 方向，xg=0，yg=10~18m）
    std::vector<Eigen::Vector2d> cones_utm;
    for (int i = 0; i < 5; ++i) {
        cones_utm.emplace_back(0.0, 10.0 + i * 2.0);
    }

    // 算法要求车辆从不同「位置」（而非同一位置的不同航向）环绕锥桶群观测，
    // 否则目标函数对所有 δ 均退化为 0（无法收敛）。
    // 数学原因：R_bg(h)·R(δ)·R_bg(h) = R(-δ) 与 h 无关，
    // 同一位置多角度等价于只旋转锥桶，投影仍共线。
    //
    // 车辆在锥桶群中心 (xg=0, yg=14) 半径 20m 处环绕行驶，
    // 航向均朝向锥桶群中心：heading = atan2(Δyg, Δxg)（顺时针，北=0°）
    const std::vector<VehicleState> veh_states = {
        { 20.0,  14.0, 180.0},   // 北侧，朝南
        { 14.0,  28.0, 225.0},   // 东北，朝西南
        {  0.0,  34.0, 270.0},   // 东侧，朝西
        {-14.0,  28.0, 315.0},   // 东南，朝西北
        {-20.0,  14.0,   0.0},   // 南侧，朝北
        {-14.0,   0.0,  45.0},   // 西南，朝东北
        {  0.0,  -6.0,  90.0},   // 西侧，朝东
        { 14.0,   0.0, 135.0},   // 西北，朝东南
    };

    // 标定参数（yaw_install=0°，搜索 ±15°，精度 0.001°）
    lidar_gnss_yaw::CalibParams params;
    params.yaw_install         = 0.0;
    params.x_lb                = 0.0;
    params.y_lb                = 0.0;
    params.min_snapshots       = static_cast<int>(veh_states.size());
    params.max_snapshots       = static_cast<int>(veh_states.size()) + 10;
    params.min_cones_per_snapshot = 3;
    params.min_heading_spread  = 3.0;
    params.search_range        = 15.0;
    params.coarse_step         = 0.05;
    params.fine_tol            = 0.001;

    lidar_gnss_yaw::YawCalibrator calibrator(params);

    // 生成并添加快照
    std::vector<lidar_gnss_yaw::Snapshot> snaps;
    for (const auto& vs : veh_states) {
        auto snap = generateSnapshot(cones_utm, vs.xg, vs.yg, vs.heading,
                                     scene.true_yaw_err_deg);
        calibrator.addSnapshot(snap);
        snaps.push_back(snap);
    }

    // 运行标定
    auto result = calibrator.calibrate();

    report.result_delta_deg = result.delta_yaw;
    report.error_deg        = std::abs(result.delta_yaw - scene.true_yaw_err_deg);
    report.rmse_m           = result.rmse_m;
    report.num_snapshots    = result.num_snapshots;
    report.heading_spread   = result.heading_spread;
    report.pass             = result.success &&
                              (report.error_deg < scene.tolerance_deg);

    // 发布可视化
    if (viz.enabled && ros::ok()) {
        ros::Time stamp = ros::Time::now();
        viz.global_cones.publish(buildGlobalConeMarkers(cones_utm, stamp));
        viz.vehicle_poses.publish(buildVehiclePoseMarkers(snaps, stamp));
        viz.result_text.publish(
            buildResultMarkers(result, scene.name,
                               scene.true_yaw_err_deg, report.pass, stamp));
        ros::spinOnce();
        ros::Duration(viz.scene_delay_s).sleep();
    }

    return report;
}

// ===========================================================================
// 构建测试场景
// ===========================================================================
static std::vector<SceneConfig> buildScenes()
{
    return {
        {"01_zero_error",
         "Zero yaw error (perfect installation), expected delta=0deg",
         0.0, 0.05},

        {"02_pos_2deg",
         "Positive yaw error +2deg (sensor rotated CCW by 2deg)",
         2.0, 0.10},

        {"03_neg_3deg",
         "Negative yaw error -3deg (sensor rotated CW by 3deg)",
         -3.0, 0.10},

        {"04_pos_5deg",
         "Medium yaw error +5deg",
         5.0, 0.15},
    };
}

// ===========================================================================
// 打印汇总报告
// ===========================================================================
static void printReport(const std::vector<TestReport>& reports)
{
    std::string line(76, '=');
    std::string dash(76, '-');

    ROS_INFO("\n%s", line.c_str());
    ROS_INFO("  LiDAR-GNSS Joint Yaw Calibration (lidar_gnss_yaw) Test Report");
    ROS_INFO("  Algorithm: Global projection consistency (min eigenvalue)");
    ROS_INFO("%s", line.c_str());

    int pass_cnt = 0;
    for (const auto& r : reports) {
        if (r.pass) ++pass_cnt;
        ROS_INFO("%s", dash.c_str());
        ROS_INFO("Scene: %-34s  %s",
                 r.name.c_str(), r.pass ? "[PASS]" : "[FAIL]");
        ROS_INFO("  True ε:   %+7.3fdeg  Got delta_yaw: %+7.3fdeg",
                 r.expected_delta_deg, r.result_delta_deg);
        ROS_INFO("  Error: %.4fdeg  Tol: %.3fdeg  RMSE: %.2fcm  Snaps: %d  Spread: %.1fdeg",
                 r.error_deg, r.tolerance_deg,
                 r.rmse_m * 100.0, r.num_snapshots, r.heading_spread);
    }

    ROS_INFO("%s", line.c_str());
    int total = static_cast<int>(reports.size());
    ROS_INFO("Summary: %d / %d PASS  (%.0f%%)",
             pass_cnt, total, 100.0 * pass_cnt / total);
    int bw = 40, bf = (total > 0) ? bw * pass_cnt / total : 0;
    ROS_INFO("[%s%s]  %d/%d",
             std::string(bf, '#').c_str(),
             std::string(bw - bf, '.').c_str(),
             pass_cnt, total);
    ROS_INFO("%s", line.c_str());
}

} // namespace gnss_yaw_test

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_yaw_test_node");
    ros::NodeHandle nh, pnh("~");

    bool   viz_enabled = true;
    double scene_delay = 4.0;

    pnh.param("viz",         viz_enabled, true);
    pnh.param("scene_delay", scene_delay, 4.0);

    gnss_yaw_test::VizPubs viz;
    viz.enabled       = viz_enabled;
    viz.scene_delay_s = scene_delay;

    if (viz_enabled) {
        viz.global_cones    = nh.advertise<visualization_msgs::MarkerArray>(
            "/gnss_yaw_test/global_cones", 5);
        viz.vehicle_poses   = nh.advertise<visualization_msgs::MarkerArray>(
            "/gnss_yaw_test/vehicle_poses", 5);
        viz.projected_cones = nh.advertise<visualization_msgs::MarkerArray>(
            "/gnss_yaw_test/projected_cones", 5);
        viz.result_text     = nh.advertise<visualization_msgs::MarkerArray>(
            "/gnss_yaw_test/result_text", 5);

        ROS_INFO("[gnss_yaw_test] Waiting for RViz connection (2s)...");
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO("=======================================================");
    ROS_INFO("  LiDAR-GNSS Yaw Calibration Test (lidar_gnss_yaw)");
    ROS_INFO("  viz: %s", viz_enabled ? "ON" : "OFF");
    ROS_INFO("=======================================================");

    auto scenes = gnss_yaw_test::buildScenes();
    std::vector<gnss_yaw_test::TestReport> reports;
    reports.reserve(scenes.size());

    for (size_t i = 0; i < scenes.size() && ros::ok(); ++i) {
        ROS_INFO("Running scene [%zu/%zu]: %s",
                 i + 1, scenes.size(), scenes[i].name.c_str());
        ROS_INFO("  %s", scenes[i].description.c_str());

        auto r = gnss_yaw_test::runScenario(scenes[i], viz);
        reports.push_back(r);

        ROS_INFO("  -> %s  DeltaYawErr=%.4fdeg  RMSE=%.2fcm",
                 r.pass ? "PASS" : "FAIL",
                 r.error_deg, r.rmse_m * 100.0);
    }

    gnss_yaw_test::printReport(reports);
    return 0;
}
