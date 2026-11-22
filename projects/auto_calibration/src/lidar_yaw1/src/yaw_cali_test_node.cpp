/**
 * @file yaw_cali_test_node.cpp
 * @brief LiDAR Yaw 标定算法测试节点（含 RViz 可视化）
 *
 * 直接使用合成锥桶数据测试 YawCalculator，无需真实硬件。
 * 8 个测试场景依次执行，最终输出汇总报告。
 *
 * 坐标系：FLU（X=前，Y=左，Z=上），锥桶沿 X 轴方向排列，
 * yaw_error > 0 表示锥桶连线相对 X 轴 CCW 旋转。
 *
 * 发布话题（RViz 可视化）：
 *   /lidar_yaw1_test/cone_cloud    sensor_msgs/PointCloud2   合成锥桶点云
 *   /lidar_yaw1_test/cone_markers  visualization_msgs/MarkerArray  锥桶圆柱体
 *   /lidar_yaw1_test/fit_markers   visualization_msgs/MarkerArray  拟合线 + 结果文字
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <random>

#include "lidar_yaw1/types.h"
#include "lidar_yaw1/yaw_calculator.h"

namespace lidar_yaw1_test {

static constexpr double DEG2RAD = M_PI / 180.0;
static const std::string LIDAR_FRAME = "lidar";

// ===========================================================================
// 测试场景配置
// ===========================================================================
struct SceneConfig {
    std::string name;
    std::string description;
    double true_yaw_deg;      // 真实 yaw 误差（锥桶连线角度，°）
    double tolerance_deg;     // 验收阈值（°）
    int    n_cones;           // 锥桶数量
    double noise_m;           // 位置噪声标准差（m）
    double cone_spacing_m;    // 锥桶间距（m）
    double start_x_m;         // 第一个锥桶距离（m）
};

// ===========================================================================
// 测试结果
// ===========================================================================
struct TestReport {
    std::string name;
    double expected_yaw_deg;
    double result_yaw_deg;
    double error_deg;
    double tolerance_deg;
    double confidence;
    int    n_cones;
    bool   pass;
};

// ===========================================================================
// 可视化发布器
// ===========================================================================
struct VizPubs {
    ros::Publisher cone_cloud;
    ros::Publisher cone_markers;
    ros::Publisher fit_markers;
    bool   enabled       = true;
    double frame_delay_s = 0.12;
    double scene_delay_s = 3.0;
};

// ===========================================================================
// 生成锥桶中心坐标（VCS 坐标系，沿偏角 yaw_deg 的直线排列）
// ===========================================================================
static std::vector<Eigen::Vector2f> generateConePositions(
    double yaw_deg, int n_cones, double spacing_m,
    double start_x, double noise_m, unsigned int seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise_dist(0.0f, static_cast<float>(noise_m));

    double theta   = yaw_deg * DEG2RAD;
    double cos_t   = std::cos(theta);
    double sin_t   = std::sin(theta);

    std::vector<Eigen::Vector2f> pos;
    for (int i = 0; i < n_cones; ++i) {
        double d  = start_x + i * spacing_m;
        float  nx = (noise_m > 0.0) ? noise_dist(rng) : 0.0f;
        float  ny = (noise_m > 0.0) ? noise_dist(rng) : 0.0f;
        pos.emplace_back(static_cast<float>(d * cos_t) + nx,
                         static_cast<float>(d * sin_t) + ny);
    }
    return pos;
}

// ===========================================================================
// 从坐标列表构建 ConeCluster 对象（直接传给 YawCalculator）
// ===========================================================================
static std::vector<lidar_yaw1::ConeCluster> buildConeClusters(
    const std::vector<Eigen::Vector2f>& positions,
    float z_offset = 0.37f)
{
    std::vector<lidar_yaw1::ConeCluster> cones;
    float cone_z = -z_offset + 0.44f;  // 锥桶高度中点

    for (const auto& pos : positions) {
        lidar_yaw1::ConeCluster c;
        c.center      = Eigen::Vector3f(pos[0], pos[1], cone_z);
        c.radius_xy   = 0.18f;
        c.point_count = 22;
        c.valid       = true;
        cones.push_back(c);
    }
    return cones;
}

// ===========================================================================
// 生成合成锥桶点云（用于 RViz 可视化）
// ===========================================================================
static pcl::PointCloud<pcl::PointXYZI>::Ptr generateConeCloud(
    const std::vector<Eigen::Vector2f>& positions,
    float z_offset = 0.37f, int pts_per_cone = 22,
    float noise_std = 0.035f, unsigned int seed = 42)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::mt19937 rng(seed);
    std::normal_distribution<float> nd(0.0f, noise_std);

    float z_min = -z_offset + 0.08f;
    float z_max = -z_offset + 0.80f;

    for (const auto& pos : positions) {
        for (int i = 0; i < pts_per_cone; ++i) {
            pcl::PointXYZI pt;
            pt.x = pos[0] + nd(rng);
            pt.y = pos[1] + nd(rng);
            float t = static_cast<float>(i) / pts_per_cone;
            pt.z = z_min + t * (z_max - z_min) + nd(rng) * 0.08f;
            pt.intensity = 120.0f;
            cloud->push_back(pt);
        }
    }
    cloud->width   = cloud->size();
    cloud->height  = 1;
    cloud->is_dense = false;
    return cloud;
}

// ===========================================================================
// 构建锥桶圆柱体 Marker（橙色）
// ===========================================================================
static visualization_msgs::MarkerArray buildConeMarkers(
    const std::vector<lidar_yaw1::ConeCluster>& cones,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(del);

    for (int i = 0; i < static_cast<int>(cones.size()); ++i) {
        // 锥桶圆柱
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns     = "cones";
        m.id     = i;
        m.type   = visualization_msgs::Marker::CYLINDER;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = cones[i].center.x();
        m.pose.position.y = cones[i].center.y();
        m.pose.position.z = cones[i].center.z();
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.28;
        m.scale.y = 0.28;
        m.scale.z = 0.60;
        m.color.r = 1.0f; m.color.g = 0.45f; m.color.b = 0.0f; m.color.a = 0.88f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);

        // 编号文字
        visualization_msgs::Marker txt;
        txt.header    = m.header;
        txt.ns        = "cone_labels";
        txt.id        = 100 + i;
        txt.type      = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action    = visualization_msgs::Marker::ADD;
        txt.pose      = m.pose;
        txt.pose.position.z += 0.55;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 0.22;
        txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
        char buf[8];
        std::snprintf(buf, sizeof(buf), "C%d", i + 1);
        txt.text    = buf;
        txt.lifetime = ros::Duration(0.5);
        ma.markers.push_back(txt);
    }
    return ma;
}

// ===========================================================================
// 构建拟合直线 + 结果文字 Marker
// ===========================================================================
static visualization_msgs::MarkerArray buildFitMarkers(
    const std::vector<lidar_yaw1::ConeCluster>& cones,
    const lidar_yaw1::YawResult& result,
    const std::string& scene_name,
    double expected_yaw_deg,
    bool is_pass,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray ma;
    float z_text = 4.5f;

    // ---- 拟合直线（若有结果） ----
    if (result.valid && !cones.empty()) {
        double theta = result.yaw_delta_deg * DEG2RAD;
        float  z_line = cones[0].center.z();

        visualization_msgs::Marker line;
        line.header.stamp    = stamp;
        line.header.frame_id = LIDAR_FRAME;
        line.ns   = "fit_line";
        line.id   = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point p0, p1;
        p0.x = -3.0 * std::cos(theta); p0.y = -3.0 * std::sin(theta); p0.z = z_line;
        p1.x = 45.0 * std::cos(theta); p1.y = 45.0 * std::sin(theta); p1.z = z_line;
        line.points.push_back(p0);
        line.points.push_back(p1);
        line.scale.x = 0.05f;
        line.color.r = 0.0f; line.color.g = 0.9f; line.color.b = 1.0f; line.color.a = 0.9f;
        line.lifetime = ros::Duration(0.5);
        ma.markers.push_back(line);
    }

    // ---- X 轴参考线（黄色虚线）----
    {
        visualization_msgs::Marker ref;
        ref.header.stamp    = stamp;
        ref.header.frame_id = LIDAR_FRAME;
        ref.ns   = "x_axis_ref";
        ref.id   = 1;
        ref.type = visualization_msgs::Marker::LINE_STRIP;
        ref.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point pa, pb;
        pa.x = 0; pa.y = 0; pa.z = (!cones.empty() ? cones[0].center.z() : 0.0f);
        pb.x = 45; pb.y = 0; pb.z = pa.z;
        ref.points.push_back(pa);
        ref.points.push_back(pb);
        ref.scale.x = 0.03f;
        ref.color.r = 0.9f; ref.color.g = 0.9f; ref.color.b = 0.0f; ref.color.a = 0.5f;
        ref.lifetime = ros::Duration(0.5);
        ma.markers.push_back(ref);
    }

    // ---- 场景标题 ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "scene_title";
        m.id   = 2;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 8; m.pose.position.y = 0; m.pose.position.z = z_text;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.48f;
        m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f;
        m.text     = "Scene: " + scene_name;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- 标定结果数值 ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "cali_result";
        m.id   = 3;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 8; m.pose.position.y = 0; m.pose.position.z = z_text - 0.65f;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.36f;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        if (result.valid) {
            double err = std::abs(result.yaw_delta_deg - expected_yaw_deg);
            oss << "Expected: " << std::showpos << expected_yaw_deg << "deg"
                << "  Got: "    << result.yaw_delta_deg << "deg"
                << std::noshowpos
                << "  Err: " << err << "deg"
                << "  Conf: " << result.confidence
                << "  RMS: " << result.line_rms << "m";
        } else {
            oss << "(no valid result, cones=" << cones.size() << ")";
        }
        m.text    = oss.str();
        m.color.r = result.valid ? 0.2f : 1.0f;
        m.color.g = result.valid ? 1.0f : 0.5f;
        m.color.b = 0.2f;
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- PASS / FAIL ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "pass_fail";
        m.id   = 4;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 8; m.pose.position.y = 0; m.pose.position.z = z_text - 1.3f;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.58f;
        m.text    = is_pass ? "[ PASS ]" : "[ FAIL ]";
        m.color.r = is_pass ? 0.0f : 1.0f;
        m.color.g = is_pass ? 1.0f : 0.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    // ---- LiDAR 原点小球 ----
    {
        visualization_msgs::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = LIDAR_FRAME;
        m.ns   = "lidar_origin";
        m.id   = 5;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = 0; m.pose.position.y = 0; m.pose.position.z = 0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.30;
        m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 1.0f;
        m.lifetime = ros::Duration(0.5);
        ma.markers.push_back(m);
    }

    return ma;
}

// ===========================================================================
// 执行单个测试场景
// ===========================================================================
static TestReport runScenario(
    const SceneConfig& scene, VizPubs& viz, int n_frames)
{
    TestReport report;
    report.name             = scene.name;
    // scene.true_yaw_deg = 锥桶连线偏角（物理安装误差）
    // 算法输出 yaw_delta_deg = -theta = 修正量，符号与偏角相反
    report.expected_yaw_deg = -scene.true_yaw_deg;
    report.tolerance_deg    = scene.tolerance_deg;
    report.n_cones          = scene.n_cones;

    lidar_yaw1::YawCalculator::Params yc;
    yc.window_size    = n_frames;
    yc.min_confidence = 0.20;
    yc.max_yaw_deg    = 35.0;
    lidar_yaw1::YawCalculator calc(yc);

    for (int f = 0; f < n_frames && ros::ok(); ++f) {
        auto positions = generateConePositions(
            scene.true_yaw_deg, scene.n_cones,
            scene.cone_spacing_m, scene.start_x_m,
            scene.noise_m, 42u + static_cast<unsigned>(f));

        auto cones = buildConeClusters(positions);
        lidar_yaw1::YawResult result = calc.calculate(cones);

        if (viz.enabled && ros::ok()) {
            auto cloud = generateConeCloud(positions, 0.37f, 22, 0.035f,
                                           42u + static_cast<unsigned>(f));
            ros::Time stamp = ros::Time::now();

            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.stamp    = stamp;
            cloud_msg.header.frame_id = LIDAR_FRAME;
            viz.cone_cloud.publish(cloud_msg);

            viz.cone_markers.publish(buildConeMarkers(cones, stamp));
            viz.fit_markers.publish(
                buildFitMarkers(cones, result, scene.name,
                                -scene.true_yaw_deg, false, stamp));

            ros::Duration(viz.frame_delay_s).sleep();
            ros::spinOnce();
        }
    }

    lidar_yaw1::YawResult avg = calc.getAveraged();
    if (!avg.valid) {
        report.result_yaw_deg = 0.0;
        report.error_deg      = 999.0;
        report.confidence     = 0.0;
        report.pass           = false;
        return report;
    }

    report.result_yaw_deg = avg.yaw_delta_deg;
    report.error_deg      = std::abs(avg.yaw_delta_deg - report.expected_yaw_deg);
    report.confidence     = avg.confidence;
    report.pass           = (report.error_deg < scene.tolerance_deg);

    // 展示最终帧（含 PASS/FAIL 标签）
    if (viz.enabled) {
        auto positions = generateConePositions(
            scene.true_yaw_deg, scene.n_cones,
            scene.cone_spacing_m, scene.start_x_m, 0.0, 42u);
        auto cones = buildConeClusters(positions);
        auto cloud = generateConeCloud(positions);

        for (int k = 0; k < 18 && ros::ok(); ++k) {
            ros::Time stamp = ros::Time::now();
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.stamp    = stamp;
            cloud_msg.header.frame_id = LIDAR_FRAME;
            viz.cone_cloud.publish(cloud_msg);
            viz.cone_markers.publish(buildConeMarkers(cones, stamp));
            viz.fit_markers.publish(
                buildFitMarkers(cones, avg, scene.name,
                                -scene.true_yaw_deg, report.pass, stamp));
            ros::Duration(viz.frame_delay_s).sleep();
            ros::spinOnce();
        }
        ros::Duration(viz.scene_delay_s).sleep();
    }

    return report;
}

// ===========================================================================
// 构建测试场景列表
// ===========================================================================
static std::vector<SceneConfig> buildScenes()
{
    std::vector<SceneConfig> sc;

    // 1. 完美对齐（零误差基线）
    sc.push_back({"01_zero_yaw",
                  "Perfect alignment, zero yaw error, 5 cones on X-axis",
                  0.0, 0.30, 5, 0.00, 2.0, 5.0});

    // 2. 小角度正偏差 +2°
    sc.push_back({"02_small_yaw_pos",
                  "Small yaw error +2deg, 5 cones",
                  2.0, 0.30, 5, 0.00, 2.0, 5.0});

    // 3. 小角度负偏差 -2°
    sc.push_back({"03_small_yaw_neg",
                  "Small yaw error -2deg, 5 cones",
                  -2.0, 0.30, 5, 0.00, 2.0, 5.0});

    // 4. 中等角度 +5°
    sc.push_back({"04_medium_yaw",
                  "Medium yaw error +5deg, 5 cones",
                  5.0, 0.50, 5, 0.00, 2.0, 5.0});

    // 5. 大角度 +10°
    sc.push_back({"05_large_yaw",
                  "Large yaw error +10deg, 5 cones",
                  10.0, 0.50, 5, 0.00, 2.0, 5.0});

    // 6. 最少锥桶（2 个）
    sc.push_back({"06_min_cones",
                  "Minimum 2 cones, yaw error 3deg",
                  3.0, 0.50, 2, 0.00, 3.0, 5.0});

    // 7. 多锥桶（8 个）
    sc.push_back({"07_many_cones",
                  "8 cones, yaw error 4deg (best precision)",
                  4.0, 0.30, 8, 0.00, 1.5, 4.0});

    // 8. 带位置噪声（σ=0.10m）
    sc.push_back({"08_noisy_cones",
                  "5 cones with position noise sigma=0.10m, yaw 2deg",
                  2.0, 0.55, 5, 0.10, 2.0, 5.0});

    return sc;
}

// ===========================================================================
// 打印汇总报告
// ===========================================================================
static void printReport(const std::vector<TestReport>& reports)
{
    std::string line(74, '=');
    std::string dash(74, '-');

    ROS_INFO("\n%s", line.c_str());
    ROS_INFO("  LiDAR Yaw Calibration (lidar_yaw1) Test Report");
    ROS_INFO("  Frame: FLU (X=Front, Y=Left)  Algorithm: PCA Line Fitting");
    ROS_INFO("%s", line.c_str());

    int pass_cnt = 0;
    for (const auto& r : reports) {
        if (r.pass) ++pass_cnt;
        ROS_INFO("%s", dash.c_str());
        ROS_INFO("Scene: %-34s  %s",
                 r.name.c_str(), r.pass ? "[PASS]" : "[FAIL]");
        ROS_INFO("  Expected Yaw: %+7.3fdeg  Got: %+7.3fdeg",
                 r.expected_yaw_deg, r.result_yaw_deg);
        ROS_INFO("  Error: %.4fdeg  Tol: %.2fdeg  Conf: %.3f  Cones: %d",
                 r.error_deg, r.tolerance_deg, r.confidence, r.n_cones);
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

} // namespace lidar_yaw1_test

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_yaw1_test");
    ros::NodeHandle nh, pnh("~");

    int    n_frames    = 10;
    bool   viz_enabled = true;
    double frame_delay = 0.12;
    double scene_delay = 3.0;

    pnh.param("n_frames",    n_frames,    10);
    pnh.param("viz",         viz_enabled, true);
    pnh.param("frame_delay", frame_delay, 0.12);
    pnh.param("scene_delay", scene_delay, 3.0);

    lidar_yaw1_test::VizPubs viz;
    viz.enabled       = viz_enabled;
    viz.frame_delay_s = frame_delay;
    viz.scene_delay_s = scene_delay;

    if (viz_enabled) {
        viz.cone_cloud   = nh.advertise<sensor_msgs::PointCloud2>(
            "/lidar_yaw1_test/cone_cloud", 5);
        viz.cone_markers = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_yaw1_test/cone_markers", 5);
        viz.fit_markers  = nh.advertise<visualization_msgs::MarkerArray>(
            "/lidar_yaw1_test/fit_markers", 5);

        ROS_INFO("[lidar_yaw1_test] Waiting for RViz connection (2s)...");
        ros::Duration(2.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO("=======================================================");
    ROS_INFO("  LiDAR Yaw Calibration Test (lidar_yaw1)");
    ROS_INFO("  frames/scene: %d  viz: %s",
             n_frames, viz_enabled ? "ON" : "OFF");
    ROS_INFO("=======================================================");

    auto scenes = lidar_yaw1_test::buildScenes();
    std::vector<lidar_yaw1_test::TestReport> reports;
    reports.reserve(scenes.size());

    for (size_t i = 0; i < scenes.size() && ros::ok(); ++i) {
        ROS_INFO("Running scene [%zu/%zu]: %s",
                 i + 1, scenes.size(), scenes[i].name.c_str());
        ROS_INFO("  %s", scenes[i].description.c_str());

        auto r = lidar_yaw1_test::runScenario(scenes[i], viz, n_frames);
        reports.push_back(r);

        ROS_INFO("  -> %s  YawErr=%.4fdeg  Conf=%.3f  Cones=%d",
                 r.pass ? "PASS" : "FAIL",
                 r.error_deg, r.confidence, r.n_cones);
    }

    lidar_yaw1_test::printReport(reports);
    return 0;
}
