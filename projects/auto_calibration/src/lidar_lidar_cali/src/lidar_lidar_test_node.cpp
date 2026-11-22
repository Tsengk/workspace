/**
 * lidar_lidar_test_node.cpp
 *
 * lidar_lidar_cali 测试节点（FAST-LIVO2 标定板版本）
 *
 * 使用合成 SceneData（4 个圆孔中心）测试 LidarCali SVD 3D-3D 标定，
 * 无需真实硬件。5 个测试场景覆盖 Δx、Δy、Δyaw 及组合偏差。
 *
 * 生成原理：
 *   已知主雷达圆孔中心 P（VCS），真实变换参数 (x_true, y_true, yaw_true)，
 *   辅雷达驱动输出 Q（以安装标称值 x_install=x_true, y_install=y_true,
 *   yaw_install=yaw_true 为前提，则 q_rp = P → 标定后 T_calib = identity）。
 *
 *   更实用的测试场景：安装标称值全为零（x=y=yaw=0），真实偏差已嵌入坐标中：
 *     P_secondary_approxVCS = R(-yaw_true) · (P_primary - [x_true, y_true, 0])
 *   标定器应恢复 x_calib≈x_true, y_calib≈y_true, yaw_calib≈yaw_true。
 *
 * 可视化话题：
 *   /lidar_lidar_test/primary_markers    主雷达圆孔（绿色球）
 *   /lidar_lidar_test/secondary_markers  辅雷达圆孔（蓝色球）
 *   /lidar_lidar_test/aligned_markers    校正后辅雷达圆孔（品红球）
 *   /lidar_lidar_test/result_markers     标定结果文字
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include "lidar_lidar_cali/types.h"
#include "lidar_lidar_cali/lidar_cali.h"

namespace lidar_lidar_test {

static constexpr double DEG2RAD = M_PI / 180.0;
static const std::string VCS_FRAME = "base_link";

// ===========================================================================
struct SceneConfig {
    std::string name;
    std::string description;
    double x_true;        // 真实 x 外参（m）
    double y_true;        // 真实 y 外参（m）
    double yaw_true_deg;  // 真实 yaw 外参（°）
    double tol_xy_m;
    double tol_yaw_deg;
};

struct TestReport {
    std::string name;
    double expected_x,  expected_y,  expected_yaw;
    double result_x,    result_y,    result_yaw;
    double error_x,     error_y,     error_yaw;
    double tol_xy_m,    tol_yaw_deg;
    double rmse_m;
    int    num_scenes;
    bool   pass;
};

struct VizPubs {
    ros::Publisher primary_markers;
    ros::Publisher secondary_markers;
    ros::Publisher aligned_markers;
    ros::Publisher result_markers;
    bool   enabled       = true;
    double scene_delay_s = 3.5;
};

// ===========================================================================
// 3D 旋转矩阵 Rz(θ)
// ===========================================================================
static Eigen::Matrix3d Rz3(double theta_rad)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double c = std::cos(theta_rad), s = std::sin(theta_rad);
    R(0,0) =  c; R(0,1) = -s;
    R(1,0) =  s; R(1,1) =  c;
    return R;
}

// ===========================================================================
// 生成 4 个圆孔中心（板坐标系，矩形排列）
// ===========================================================================
static std::vector<Eigen::Vector3d> makeBoardCenters(
    double cx, double cy, double cz,
    double half_w = 0.25, double half_h = 0.20)
{
    // 顺序：TL, TR, BR, BL（不影响标定，sortPatternCenters 内部会排序）
    return {
        {cx - half_w, cy + half_h, cz},
        {cx + half_w, cy + half_h, cz},
        {cx + half_w, cy - half_h, cz},
        {cx - half_w, cy - half_h, cz},
    };
}

// ===========================================================================
// 从主雷达圆孔中心 + 已知真实外参生成辅雷达的"近似VCS"坐标
//   p_secondary_approxVCS = R(-yaw_true) · (p_primary_VCS − [x_true, y_true, 0])
// 模拟：辅雷达驱动安装参数标称值为 0，真实外参已通过点云变换引入
// ===========================================================================
static std::vector<Eigen::Vector3d> computeSecondaryCenters(
    const std::vector<Eigen::Vector3d>& primary,
    double x_true, double y_true, double yaw_true_deg,
    double noise_m = 0.0)
{
    Eigen::Matrix3d R_inv = Rz3(-yaw_true_deg * DEG2RAD);
    Eigen::Vector3d t(x_true, y_true, 0.0);

    std::vector<Eigen::Vector3d> sec;
    for (const auto& p : primary) {
        Eigen::Vector3d q = R_inv * (p - t);
        if (noise_m > 0.0) {
            q.x() += noise_m * (2.0 * (std::rand() / (double)RAND_MAX) - 1.0);
            q.y() += noise_m * (2.0 * (std::rand() / (double)RAND_MAX) - 1.0);
        }
        sec.push_back(q);
    }
    return sec;
}

// ===========================================================================
// Marker 工具
// ===========================================================================
static void addSphereMarkers(
    visualization_msgs::MarkerArray& ma,
    const std::vector<Eigen::Vector3d>& centers,
    const std::string& ns,
    float r, float g, float b,
    const ros::Time& stamp, int id_offset = 0)
{
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    del.ns = ns;
    ma.markers.push_back(del);

    for (int i = 0; i < (int)centers.size(); ++i) {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = VCS_FRAME;
        m.ns     = ns;
        m.id     = id_offset + i;
        m.type   = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = centers[i].x();
        m.pose.position.y = centers[i].y();
        m.pose.position.z = centers[i].z();
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.12; m.scale.y = 0.12; m.scale.z = 0.12;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);

        visualization_msgs::Marker txt;
        txt.header    = m.header;
        txt.ns        = ns + "_lbl";
        txt.id        = id_offset + 100 + i;
        txt.type      = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action    = visualization_msgs::Marker::ADD;
        txt.pose      = m.pose;
        txt.pose.position.z += 0.14;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 0.14;
        txt.color.r = r; txt.color.g = g; txt.color.b = b; txt.color.a = 1.0f;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "C%d(%.1f,%.1f)", i,
                      centers[i].x(), centers[i].y());
        txt.text = buf;
        txt.lifetime = ros::Duration(0.5);
        ma.markers.push_back(txt);
    }
}

static visualization_msgs::MarkerArray buildAlignedMarkers(
    const std::vector<Eigen::Vector3d>& secondary,
    const lidar_lidar_cali::CalibResult& result,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;
    if (!result.success || result.yaw_calib_deg == 0.0) return ma;

    Eigen::Matrix3d R = Rz3(result.yaw_calib_deg * DEG2RAD);
    Eigen::Vector3d t(result.x_calib, result.y_calib, 0.0);

    std::vector<Eigen::Vector3d> aligned;
    for (const auto& q : secondary) {
        Eigen::Vector3d p = R * q + t;
        p.z() = q.z() + 0.04;  // 稍微抬高避免重叠
        aligned.push_back(p);
    }

    addSphereMarkers(ma, aligned, "aligned_circles", 1.0f, 0.0f, 1.0f, stamp, 200);
    return ma;
}

static visualization_msgs::MarkerArray buildResultMarkers(
    const lidar_lidar_cali::CalibResult& result,
    const SceneConfig& scene,
    bool pass,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;
    float tx = 10.0f, ty = 5.0f, z_text = 4.0f;

    auto addText = [&](int id, const std::string& ns,
                       float x, float y, float z, float sz,
                       const std::string& text, float r, float g, float b) {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = VCS_FRAME;
        m.ns = ns; m.id = id;
        m.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
        m.pose.orientation.w = 1.0;
        m.scale.z = sz;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
        m.text = text;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    };

    addText(0, "scene_title", tx, ty, z_text, 0.45f,
            "Scene: " + scene.name, 1.0f, 1.0f, 1.0f);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    if (result.success) {
        oss << "True: x=" << std::showpos << scene.x_true
            << "m y=" << scene.y_true
            << "m yaw=" << scene.yaw_true_deg << "deg\n"
            << std::noshowpos
            << "Got:  x=" << std::showpos << result.x_calib
            << "m y=" << result.y_calib
            << "m yaw=" << result.yaw_calib_deg << "deg\n"
            << std::noshowpos
            << "Err:  |Δx|=" << std::abs(result.x_calib - scene.x_true)
            << "m |Δy|=" << std::abs(result.y_calib - scene.y_true)
            << "m |Δyaw|=" << std::abs(result.yaw_calib_deg - scene.yaw_true_deg) << "deg\n"
            << "RMSE=" << result.rmse_m * 100.0 << "cm"
            << " scenes=" << result.num_scenes
            << " grade=" << result.quality_grade;
    } else {
        oss << "FAILED: " << result.message;
    }
    addText(1, "cali_result", tx, ty, z_text - 0.55f, 0.30f, oss.str(),
            result.success ? 0.2f : 1.0f,
            result.success ? 1.0f : 0.3f, 0.2f);

    addText(2, "pass_fail", tx, ty, z_text - 1.55f, 0.55f,
            pass ? "[ PASS ]" : "[ FAIL ]",
            pass ? 0.0f : 1.0f, pass ? 1.0f : 0.0f, 0.0f);

    addText(10, "legend_p", tx, ty, z_text - 2.20f, 0.28f,
            "Green   = Primary LiDAR circles",   0.0f, 0.8f, 0.2f);
    addText(11, "legend_s", tx, ty, z_text - 2.55f, 0.28f,
            "Blue    = Secondary LiDAR circles", 0.1f, 0.4f, 1.0f);
    addText(12, "legend_a", tx, ty, z_text - 2.90f, 0.28f,
            "Magenta = Aligned secondary",       1.0f, 0.0f, 1.0f);

    return ma;
}

// ===========================================================================
// 执行单个测试场景
// ===========================================================================
static TestReport runScenario(
    const SceneConfig& scene, VizPubs& viz, int n_scenes)
{
    TestReport report;
    report.name          = scene.name;
    report.expected_x    = scene.x_true;
    report.expected_y    = scene.y_true;
    report.expected_yaw  = scene.yaw_true_deg;
    report.tol_xy_m      = scene.tol_xy_m;
    report.tol_yaw_deg   = scene.tol_yaw_deg;

    // 4 组不同位置的标定板（每组4个圆孔）
    // 每组放在不同距离 / 横向位置，保证 SVD 几何多样性
    std::vector<std::array<double,3>> board_positions = {
        {3.0,  0.0, 1.0},
        {3.5, -0.5, 1.0},
        {4.0,  0.5, 1.0},
        {4.5,  0.0, 1.0},
    };

    // 标定算法参数（安装标称值全为零）
    lidar_lidar_cali::LidarCali::Params cp;
    cp.x_install       = 0.0;
    cp.y_install       = 0.0;
    cp.yaw_install_deg = 0.0;
    cp.min_scenes      = n_scenes;

    lidar_lidar_cali::LidarCali calibrator(cp);

    std::vector<Eigen::Vector3d> last_primary, last_secondary;

    for (int s = 0; s < n_scenes; ++s) {
        const auto& bp = board_positions[s % board_positions.size()];
        auto primary = makeBoardCenters(bp[0], bp[1], bp[2]);

        // 辅雷达坐标：逆变换到辅雷达近似VCS（安装标称=0），
        // 嵌入真实外参偏差
        auto secondary = computeSecondaryCenters(
            primary, scene.x_true, scene.y_true, scene.yaw_true_deg);

        lidar_lidar_cali::SceneData sd;
        sd.primary_centers   = primary;
        sd.secondary_centers = secondary;
        sd.valid             = true;
        calibrator.addScene(sd);

        last_primary   = primary;
        last_secondary = secondary;
    }

    lidar_lidar_cali::CalibResult result = calibrator.calibrate();

    report.result_x   = result.x_calib;
    report.result_y   = result.y_calib;
    report.result_yaw = result.yaw_calib_deg;
    report.error_x    = std::abs(result.x_calib   - scene.x_true);
    report.error_y    = std::abs(result.y_calib   - scene.y_true);
    report.error_yaw  = std::abs(result.yaw_calib_deg - scene.yaw_true_deg);
    report.rmse_m     = result.rmse_m;
    report.num_scenes = result.num_scenes;
    report.pass       = result.success &&
                        (report.error_x   < scene.tol_xy_m) &&
                        (report.error_y   < scene.tol_xy_m) &&
                        (report.error_yaw < scene.tol_yaw_deg);

    // 可视化
    if (viz.enabled && ros::ok()) {
        for (int k = 0; k < 20 && ros::ok(); ++k) {
            ros::Time ts = ros::Time::now();

            visualization_msgs::MarkerArray ma_p, ma_s;
            addSphereMarkers(ma_p, last_primary,   "primary_circles",
                             0.0f, 0.8f, 0.2f, ts);
            addSphereMarkers(ma_s, last_secondary, "secondary_circles",
                             0.1f, 0.4f, 1.0f, ts, 100);
            auto ma_a = buildAlignedMarkers(last_secondary, result, ts);
            auto ma_r = buildResultMarkers(result, scene, report.pass, ts);

            viz.primary_markers.publish(ma_p);
            viz.secondary_markers.publish(ma_s);
            viz.aligned_markers.publish(ma_a);
            viz.result_markers.publish(ma_r);

            ros::Duration(0.10).sleep();
            ros::spinOnce();
        }
        ros::Duration(viz.scene_delay_s).sleep();
    }

    return report;
}

// ===========================================================================
static std::vector<SceneConfig> buildScenes()
{
    return {
        {"01_zero_offset",
         "Perfect alignment (convergence check)",
         0.00, 0.00, 0.0, 0.005, 0.10},

        {"02_dx_only",
         "X-axis offset only: x_true=+0.10m",
         0.10, 0.00, 0.0, 0.005, 0.10},

        {"03_dy_only",
         "Y-axis offset only: y_true=+0.08m",
         0.00, 0.08, 0.0, 0.005, 0.10},

        {"04_yaw_only",
         "Yaw offset only: yaw_true=+3.0deg",
         0.00, 0.00, 3.0, 0.010, 0.20},

        {"05_combined",
         "Combined: x=+0.05m, y=-0.04m, yaw=+2.0deg",
         0.05, -0.04, 2.0, 0.010, 0.20},
    };
}

static void printReport(const std::vector<TestReport>& reports)
{
    const std::string line(80, '=');
    const std::string dash(80, '-');

    ROS_INFO("\n%s", line.c_str());
    ROS_INFO("  LiDAR-LiDAR Calibration Test (FAST-LIVO2 Board, SVD 3D-3D)");
    ROS_INFO("%s", line.c_str());

    int pass_cnt = 0;
    for (const auto& r : reports) {
        if (r.pass) ++pass_cnt;
        ROS_INFO("%s", dash.c_str());
        ROS_INFO("Scene: %-36s  %s",
                 r.name.c_str(), r.pass ? "[PASS]" : "[FAIL]");
        ROS_INFO("  True:   x=%+.4fm  y=%+.4fm  yaw=%+.4fdeg",
                 r.expected_x, r.expected_y, r.expected_yaw);
        ROS_INFO("  Got:    x=%+.4fm  y=%+.4fm  yaw=%+.4fdeg",
                 r.result_x, r.result_y, r.result_yaw);
        ROS_INFO("  Err:    |Δx|=%.4fm  |Δy|=%.4fm  |Δyaw|=%.4fdeg  RMSE=%.3fcm  scenes=%d",
                 r.error_x, r.error_y, r.error_yaw,
                 r.rmse_m * 100.0, r.num_scenes);
    }

    ROS_INFO("%s", line.c_str());
    int total = (int)reports.size();
    ROS_INFO("Summary: %d / %d PASS  (%.0f%%)",
             pass_cnt, total, 100.0 * pass_cnt / total);
    int bw = 40, bf = (total > 0) ? bw * pass_cnt / total : 0;
    ROS_INFO("[%s%s]  %d/%d",
             std::string(bf, '#').c_str(),
             std::string(bw - bf, '.').c_str(),
             pass_cnt, total);
    ROS_INFO("%s", line.c_str());
}

} // namespace lidar_lidar_test

// ===========================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_lidar_test_node");
    ros::NodeHandle nh, pnh("~");

    int    n_scenes     = 4;
    bool   viz_enabled  = true;
    double scene_delay  = 3.5;

    pnh.param("n_scenes",    n_scenes,    4);
    pnh.param("viz",         viz_enabled, true);
    pnh.param("scene_delay", scene_delay, 3.5);

    lidar_lidar_test::VizPubs viz;
    viz.enabled       = viz_enabled;
    viz.scene_delay_s = scene_delay;

    if (viz_enabled) {
        viz.primary_markers   = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_lidar_test/primary_markers", 5);
        viz.secondary_markers = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_lidar_test/secondary_markers", 5);
        viz.aligned_markers   = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_lidar_test/aligned_markers", 5);
        viz.result_markers    = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_lidar_test/result_markers", 5);

        ROS_INFO("[lidar_lidar_test] Waiting for RViz (2s)...");
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO("===========================================================");
    ROS_INFO("  LiDAR-LiDAR Calibration Test (FAST-LIVO2 Board)");
    ROS_INFO("  n_scenes: %d  viz: %s", n_scenes, viz_enabled ? "ON" : "OFF");
    ROS_INFO("===========================================================");

    auto scenes = lidar_lidar_test::buildScenes();
    std::vector<lidar_lidar_test::TestReport> reports;
    reports.reserve(scenes.size());

    for (size_t i = 0; i < scenes.size() && ros::ok(); ++i) {
        ROS_INFO("Running [%zu/%zu]: %s — %s",
                 i + 1, scenes.size(),
                 scenes[i].name.c_str(), scenes[i].description.c_str());

        auto r = lidar_lidar_test::runScenario(scenes[i], viz, n_scenes);
        reports.push_back(r);

        ROS_INFO("  -> %s  ErrX=%.4fm  ErrY=%.4fm  ErrYaw=%.4fdeg  RMSE=%.3fcm",
                 r.pass ? "PASS" : "FAIL",
                 r.error_x, r.error_y, r.error_yaw,
                 r.rmse_m * 100.0);
    }

    lidar_lidar_test::printReport(reports);
    return 0;
}
