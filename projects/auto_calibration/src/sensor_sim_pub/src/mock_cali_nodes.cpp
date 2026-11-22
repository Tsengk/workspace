/**
 * mock_cali_nodes.cpp
 *
 * 模拟所有 8 个标定后端节点，供 calibration_gui 可视化验证。
 *
 * 每个标定节点对应一个 ROS 命名空间，对外发布：
 *   - 点云/图像话题（GUI 可视化）
 *   - result 话题（geometry_msgs/Vector3Stamped）
 *   - status 话题（std_msgs/String）
 *   - 标定服务（std_srvs/Trigger）
 *
 * 节点命名空间与 GUI 完全一致：
 *   /lidar_rp_cali          Tab1
 *   /lidar_yaw_cali         Tab2
 *   /lidar_gnss_yaw         Tab3
 *   /lidar_lidar_cali       Tab4
 *   /image_lidar_calib_node Tab5
 *   /camera_cali_node       Tab6
 *   /bev_surround_view_node Tab7
 *   /front_view_stitch_node Tab8
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <string>
#include <functional>
#include <vector>
#include <memory>
#include <random>

// =============================================================
// 点云生成工具
// =============================================================
namespace sim {

static std::default_random_engine g_rng(42);
static std::normal_distribution<float> g_noise(0.f, 0.02f);

// 生成平坦地面点云（32线，模拟激光雷达扫描地面）
sensor_msgs::PointCloud2 makeGroundCloud(const std::string& frame_id,
                                          const ros::Time& stamp,
                                          float height = 1.8f,
                                          float roll_bias = 0.2f,
                                          float pitch_bias = -0.3f)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    const int n_rings = 32, pts_per_ring = 720;
    const float el_min = -25.f * M_PI / 180.f;
    const float el_max =  5.f  * M_PI / 180.f;
    const float h_step = 2.f * M_PI / pts_per_ring;

    float cr = std::cos(roll_bias  * M_PI / 180.f);
    float sr = std::sin(roll_bias  * M_PI / 180.f);
    float cp = std::cos(pitch_bias * M_PI / 180.f);
    float sp = std::sin(pitch_bias * M_PI / 180.f);

    for (int ring = 0; ring < n_rings; ++ring) {
        float el = el_min + (el_max - el_min) * ring / (n_rings - 1);
        for (int j = 0; j < pts_per_ring; ++j) {
            float az = j * h_step;
            float dx = std::cos(el) * std::cos(az);
            float dy = std::cos(el) * std::sin(az);
            float dz = std::sin(el);
            if (dz >= -1e-4f) continue;
            float t = -height / dz;
            if (t < 1.f || t > 45.f) continue;

            float px = dx * t + g_noise(g_rng);
            float py = dy * t + g_noise(g_rng);
            float pz = dz * t + g_noise(g_rng);

            // 应用微小 roll/pitch 偏置（模拟安装误差）
            float rx = px;
            float ry = cp * py - sp * pz;
            float rz = sp * py + cp * pz;
            float fx = cr * rx + sr * rz;
            float fy = ry;
            float fz = -sr * rx + cr * rz;

            pcl::PointXYZI pt;
            pt.x = fx; pt.y = fy; pt.z = fz;
            pt.intensity = 60.f + g_noise(g_rng) * 10.f;
            cloud->push_back(pt);
        }
    }
    cloud->width = (uint32_t)cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    return msg;
}

// 生成锥桶点云（沿 X 轴排列，可设偏移）
sensor_msgs::PointCloud2 makeConeCloud(const std::string& frame_id,
                                        const ros::Time& stamp,
                                        float height = 1.8f,
                                        int n_cones = 5,
                                        float spacing = 1.5f,
                                        float start = 3.f,
                                        float lateral = 0.f)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    const float cone_r = 0.15f, cone_h = 0.70f;
    for (int i = 0; i < n_cones; ++i) {
        float cx = start + i * spacing;
        float cy = lateral;
        for (int ia = 0; ia < 24; ++ia) {
            float az = ia * 2.f * M_PI / 24;
            for (int ih = 0; ih < 12; ++ih) {
                float h = ih * cone_h / 12;
                float r = cone_r * (1.f - h / cone_h);
                pcl::PointXYZI pt;
                pt.x = cx + r * std::cos(az) + g_noise(g_rng) * 0.005f;
                pt.y = cy + r * std::sin(az) + g_noise(g_rng) * 0.005f;
                pt.z = -height + h;
                pt.intensity = 200.f;
                cloud->push_back(pt);
            }
        }
    }
    cloud->width = (uint32_t)cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    return msg;
}

// 生成棋盘格合成图像
sensor_msgs::Image::Ptr makeCheckerImage(const std::string& /*frame_id*/,
                                          const ros::Time& /*stamp*/,
                                          int w, int h,
                                          int hue,   // HSV色调 0-179
                                          double t_sec,
                                          const std::string& label)
{
    cv::Mat img(h, w, CV_8UC3);
    // 彩色背景
    cv::Mat hsv(h, w, CV_8UC3);
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            hsv.at<cv::Vec3b>(r, c) = cv::Vec3b(
                (uchar)hue,
                (uchar)(70 + 80 * r / h),
                (uchar)(50 + 80 * c / w));
    cv::cvtColor(hsv, img, cv::COLOR_HSV2BGR);

    // 棋盘格
    int cell = std::min(w, h) / 10;
    int bw = 8 * cell, bh = 6 * cell;
    int ox = (w - bw) / 2, oy = (h - bh) / 2;
    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 8; ++col) {
            uchar c_val = ((row + col) % 2 == 0) ? 255 : 0;
            cv::rectangle(img,
                cv::Rect(ox + col * cell, oy + row * cell, cell, cell),
                cv::Scalar(c_val, c_val, c_val), cv::FILLED);
        }

    // 时间戳 + 标签
    char buf[128];
    snprintf(buf, sizeof(buf), "[%s]  t=%.1fs", label.c_str(), t_sec);
    cv::putText(img, buf, cv::Point(8, h - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

    // 滚动竖线（表示数据在动）
    int xline = (int)(t_sec * 60) % w;
    cv::line(img, cv::Point(xline, 0), cv::Point(xline, h),
             cv::Scalar(0, 255, 255), 2);

    return cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
}

// 模拟 BEV 鸟瞰图：拼接4个不同色调的扇形图像
sensor_msgs::Image::Ptr makeBevImage(const ros::Time& stamp, double t_sec)
{
    const int W = 600, H = 600;
    cv::Mat bev(H, W, CV_8UC3, cv::Scalar(20, 20, 20));
    // 绘制车辆轮廓
    cv::rectangle(bev, cv::Rect(W/2-25, H/2-50, 50, 100),
                  cv::Scalar(80, 200, 80), 2);
    // 四个方向扇形（模拟相机视野）
    struct Sect { int hue; double start_ang; std::string label; };
    std::vector<Sect> sects = {
        {30,  -45, "F"},   // 前：橙色
        {120, 135, "R"},   // 后：绿色
        {90,  225, "L"},   // 左：青色
        {0,   45,  "RR"},  // 右：红色
    };
    for (auto& s : sects) {
        cv::ellipse(bev, cv::Point(W/2, H/2),
                    cv::Size(240, 240),
                    s.start_ang, 0, 90,
                    cv::Scalar(s.hue * 2, 180, 100), cv::FILLED);
    }
    // 覆盖车辆轮廓（最终显示在顶层）
    cv::rectangle(bev, cv::Rect(W/2-22, H/2-48, 44, 96),
                  cv::Scalar(20, 20, 20), cv::FILLED);
    cv::rectangle(bev, cv::Rect(W/2-22, H/2-48, 44, 96),
                  cv::Scalar(80, 200, 80), 2);
    // 动态元素：行驶方向指示箭头
    double ang = t_sec * 30.0 * M_PI / 180.0;
    cv::arrowedLine(bev,
        cv::Point(W/2, H/2),
        cv::Point(W/2 + (int)(40 * std::cos(ang)),
                  H/2 - (int)(40 * std::sin(ang))),
        cv::Scalar(255, 255, 0), 3);

    char buf[32];
    snprintf(buf, sizeof(buf), "BEV  t=%.1fs", t_sec);
    cv::putText(bev, buf, cv::Point(8, H - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);

    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bev).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = "bev";
    return msg;
}

// 模拟拼接图像：三个色块横向排列
sensor_msgs::Image::Ptr makeStitchedImage(const ros::Time& stamp, double t_sec)
{
    const int W = 1200, H = 400;
    cv::Mat img(H, W, CV_8UC3);
    // 左相机区域 - 蓝色调
    img(cv::Rect(0, 0, W/3, H)) = cv::Scalar(200, 100, 50);
    // 前相机区域 - 绿色调
    img(cv::Rect(W/3, 0, W/3, H)) = cv::Scalar(50, 200, 100);
    // 右相机区域 - 红色调
    img(cv::Rect(2*W/3, 0, W/3, H)) = cv::Scalar(50, 100, 200);

    // 拼接线
    cv::line(img, cv::Point(W/3, 0), cv::Point(W/3, H), cv::Scalar(255,255,255), 2);
    cv::line(img, cv::Point(2*W/3, 0), cv::Point(2*W/3, H), cv::Scalar(255,255,255), 2);

    // 棋盘格叠加（验证对齐）
    int cell = 40;
    for (int r = 0; r < H/cell; ++r)
        for (int c = 0; c < W/cell; ++c)
            if ((r + c) % 2 == 0)
                cv::rectangle(img,
                    cv::Rect(c * cell, r * cell, cell, cell),
                    cv::Scalar(255, 255, 255), 1);

    // 动态水平线
    int yline = (int)(t_sec * 30) % H;
    cv::line(img, cv::Point(0, yline), cv::Point(W, yline),
             cv::Scalar(0, 255, 255), 2);

    char buf[64];
    snprintf(buf, sizeof(buf), "Stitched  t=%.1fs", t_sec);
    cv::putText(img, buf, cv::Point(8, H - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);

    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = "stitch";
    return msg;
}

// 通用 Trigger 服务回调工厂
bool triggerSrv(std_srvs::Trigger::Request&,
                std_srvs::Trigger::Response& res,
                const std::string& action_name)
{
    ROS_INFO("[mock] Service called: %s", action_name.c_str());
    res.success = true;
    res.message = action_name + " OK (mock)";
    return true;
}

} // namespace sim

// =============================================================
// 单个模拟标定节点封装
// =============================================================
struct MockNode {
    std::string ns;
    std::string name;

    ros::Publisher pub_status;
    ros::Publisher pub_result;

    // 点云发布器（可有多个）
    std::vector<ros::Publisher> pub_clouds;
    std::vector<std::string>   cloud_keys;   // 对应话题名

    // 图像发布器（可有多个）
    std::vector<ros::Publisher> pub_images;
    std::vector<std::string>   image_keys;

    // 服务列表
    std::vector<ros::ServiceServer> srvs;

    void advertiseCloud(ros::NodeHandle& nh, const std::string& key)
    {
        pub_clouds.push_back(nh.advertise<sensor_msgs::PointCloud2>(ns + "/" + key, 5));
        cloud_keys.push_back(key);
    }
    void advertiseImage(ros::NodeHandle& nh, const std::string& key)
    {
        pub_images.push_back(nh.advertise<sensor_msgs::Image>(ns + "/" + key, 5));
        image_keys.push_back(key);
    }
    void advertiseService(ros::NodeHandle& nh, const std::string& key)
    {
        std::string sname = ns + "/" + key;
        srvs.push_back(nh.advertiseService<std_srvs::Trigger::Request,
                                            std_srvs::Trigger::Response>(
            sname,
            [sname](std_srvs::Trigger::Request& req,
                    std_srvs::Trigger::Response& res) {
                return sim::triggerSrv(req, res, sname);
            }));
    }

    void publishStatus(const std::string& msg, const ros::Time& stamp)
    {
        std_msgs::String s;
        s.data = msg;
        pub_status.publish(s);
        (void)stamp;
    }

    void publishResult(double x, double y, double z, const ros::Time& stamp)
    {
        geometry_msgs::Vector3Stamped res;
        res.header.stamp = stamp;
        res.vector.x = x;
        res.vector.y = y;
        res.vector.z = z;
        pub_result.publish(res);
    }

    void publishCloud(size_t idx, const sensor_msgs::PointCloud2& msg)
    {
        if (idx < pub_clouds.size()) pub_clouds[idx].publish(msg);
    }
    void publishImage(size_t idx, const sensor_msgs::Image::Ptr& msg)
    {
        if (idx < pub_images.size()) pub_images[idx].publish(*msg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_cali_nodes");
    ros::NodeHandle nh;

    double rate_hz = 10.0;
    nh.param("mock_rate_hz", rate_hz, 10.0);
    ros::Rate rate(rate_hz);

    // =============================================================
    // 初始化 8 个模拟节点
    // =============================================================

    // --- Tab1: LiDAR RP ---
    MockNode rp;
    rp.ns   = "/lidar_rp_cali";
    rp.name = "lidar_rp_cali";
    rp.pub_status = nh.advertise<std_msgs::String>(rp.ns + "/status", 5, true);
    rp.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(rp.ns + "/result", 5);
    rp.advertiseCloud(nh, "ground_cloud");
    rp.advertiseService(nh, "trigger_calibration");
    rp.advertiseService(nh, "reset_calibration");
    rp.advertiseService(nh, "save_calibration");

    // --- Tab2: LiDAR Yaw ---
    MockNode yaw;
    yaw.ns   = "/lidar_yaw_cali";
    yaw.name = "lidar_yaw_cali";
    yaw.pub_status = nh.advertise<std_msgs::String>(yaw.ns + "/status", 5, true);
    yaw.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(yaw.ns + "/result", 5);
    yaw.advertiseCloud(nh, "cone_cloud");
    yaw.advertiseService(nh, "trigger_calibration");
    yaw.advertiseService(nh, "reset_calibration");
    yaw.advertiseService(nh, "save_calibration");

    // --- Tab3: GNSS+LiDAR Yaw ---
    MockNode gnss;
    gnss.ns   = "/lidar_gnss_yaw";
    gnss.name = "lidar_gnss_yaw";
    gnss.pub_status = nh.advertise<std_msgs::String>(gnss.ns + "/status", 5, true);
    gnss.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(gnss.ns + "/result", 5);
    gnss.advertiseCloud(nh, "cone_cloud");
    gnss.advertiseService(nh, "start");
    gnss.advertiseService(nh, "stop");
    gnss.advertiseService(nh, "save");
    gnss.advertiseService(nh, "reset");

    // --- Tab4: LiDAR-LiDAR ---
    MockNode ll;
    ll.ns   = "/lidar_lidar_cali";
    ll.name = "lidar_lidar_cali";
    ll.pub_status = nh.advertise<std_msgs::String>(ll.ns + "/status", 5, true);
    ll.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(ll.ns + "/result", 5);
    ll.advertiseCloud(nh, "primary_cones");
    ll.advertiseCloud(nh, "secondary_cones");
    ll.advertiseService(nh, "trigger_calibration");
    ll.advertiseService(nh, "reset_calibration");
    ll.advertiseService(nh, "save_calibration");

    // --- Tab5: Image-LiDAR ---
    MockNode il;
    il.ns   = "/image_lidar_calib_node";
    il.name = "image_lidar_calib_node";
    il.pub_status = nh.advertise<std_msgs::String>(il.ns + "/status", 5, true);
    il.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(il.ns + "/result", 5);
    il.advertiseImage(nh, "debug_image");
    il.advertiseService(nh, "trigger_calibration");
    il.advertiseService(nh, "reset_calibration");
    il.advertiseService(nh, "save_calibration");

    // --- Tab6: Camera-Camera ---
    MockNode cc;
    cc.ns   = "/camera_cali_node";
    cc.name = "camera_cali_node";
    cc.pub_status = nh.advertise<std_msgs::String>(cc.ns + "/status", 5, true);
    cc.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(cc.ns + "/result", 5);
    cc.advertiseImage(nh, "debug_ref");
    cc.advertiseImage(nh, "debug_target");
    cc.advertiseService(nh, "trigger_calibration");
    cc.advertiseService(nh, "reset_calibration");
    cc.advertiseService(nh, "save_calibration");

    // --- Tab7: BEV ---
    MockNode bev;
    bev.ns   = "/bev_surround_view_node";
    bev.name = "bev_surround_view_node";
    bev.pub_status = nh.advertise<std_msgs::String>(bev.ns + "/status", 5, true);
    bev.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(bev.ns + "/result", 5);
    bev.advertiseImage(nh, "bev_image");
    bev.advertiseService(nh, "trigger_calibration");
    bev.advertiseService(nh, "reset_calibration");
    bev.advertiseService(nh, "save_calibration");

    // --- Tab8: Front Stitch ---
    MockNode fs;
    fs.ns   = "/front_view_stitch_node";
    fs.name = "front_view_stitch_node";
    fs.pub_status = nh.advertise<std_msgs::String>(fs.ns + "/status", 5, true);
    fs.pub_result = nh.advertise<geometry_msgs::Vector3Stamped>(fs.ns + "/result", 5);
    fs.advertiseImage(nh, "stitched_image");
    fs.advertiseService(nh, "trigger_left_calibration");
    fs.advertiseService(nh, "trigger_right_calibration");
    fs.advertiseService(nh, "reset_calibration");
    fs.advertiseService(nh, "save_calibration");

    ROS_INFO("[mock_cali_nodes] All 8 mock calibration nodes ready at %.0f Hz", rate_hz);
    ROS_INFO("  Subscribing GUI to:");
    ROS_INFO("  /lidar_rp_cali/{ground_cloud,result,status,trigger*,reset*,save*}");
    ROS_INFO("  /lidar_yaw_cali/{cone_cloud,result,status,...}");
    ROS_INFO("  /lidar_gnss_yaw/{cone_cloud,result,status,start,stop,save,reset}");
    ROS_INFO("  /lidar_lidar_cali/{primary_cones,secondary_cones,result,status,...}");
    ROS_INFO("  /image_lidar_calib_node/{debug_image,result,status,...}");
    ROS_INFO("  /camera_cali_node/{debug_ref,debug_target,result,status,...}");
    ROS_INFO("  /bev_surround_view_node/{bev_image,result,status,...}");
    ROS_INFO("  /front_view_stitch_node/{stitched_image,result,status,...}");

    // =============================================================
    // 主循环
    // =============================================================
    double t = 0.0;
    const double dt = 1.0 / rate_hz;
    uint32_t frame = 0;

    // 模拟标定结果随时间缓慢收敛
    auto convergeFn = [](double t, double target, double speed) {
        return target * (1.0 - std::exp(-speed * t));
    };

    while (ros::ok()) {
        ros::Time stamp = ros::Time::now();
        ++frame;

        // --------- Tab1: LiDAR RP ---------
        {
            // ground_cloud：带微小 roll/pitch 偏置的地面点云
            float rb = 0.5f * (1.f - std::exp(-0.1f * (float)t));  // 缓慢收敛
            float pb = -0.3f * (1.f - std::exp(-0.1f * (float)t));
            rp.publishCloud(0, sim::makeGroundCloud("lidar_rp_cali", stamp, 1.8f, rb, pb));

            double conf = std::min(1.0, t / 10.0);
            rp.publishResult(
                0.5 * (1.0 - convergeFn(t, 1.0, 0.1)),   // roll 偏差
               -0.3 * (1.0 - convergeFn(t, 1.0, 0.1)),   // pitch 偏差
                conf,                                       // 置信度
                stamp);
            int n_frames = (int)(t * rate_hz);
            rp.publishStatus(
                n_frames < 8
                    ? "采集中... " + std::to_string(n_frames) + "/8帧"
                    : "OK  已采集" + std::to_string(n_frames) + "帧",
                stamp);
        }

        // --------- Tab2: LiDAR Yaw ---------
        {
            yaw.publishCloud(0, sim::makeConeCloud("lidar_yaw_cali", stamp, 1.8f, 5, 1.5f, 3.f, 0.f));
            double yaw_delta = 2.5 * std::exp(-0.08 * t);  // 从2.5°收敛到0
            yaw.publishResult(yaw_delta, 5.0, std::min(1.0, t / 8.0), stamp);
            yaw.publishStatus("已检测 5 个锥桶  yaw_delta=" +
                               std::to_string((int)(yaw_delta * 100) / 100.0) + "°", stamp);
        }

        // --------- Tab3: GNSS+LiDAR Yaw ---------
        {
            gnss.publishCloud(0, sim::makeConeCloud("lidar_gnss_yaw", stamp, 1.8f, 4, 2.f, 4.f, 0.5f));
            double rmse = 0.05 * std::exp(-0.05 * t) + 0.005;
            gnss.publishResult(
                1.234,      // yaw_calibrated (deg)
                0.012,      // delta_yaw
                rmse,       // rmse_m
                stamp);
            gnss.publishStatus("采集: " + std::to_string((int)(t * rate_hz)) +
                                " 帧  RMSE=" + std::to_string(rmse).substr(0, 6), stamp);
        }

        // --------- Tab4: LiDAR-LiDAR ---------
        {
            // 主雷达锥桶（正前方）
            ll.publishCloud(0, sim::makeConeCloud("lidar_top", stamp, 1.82f, 5, 1.5f, 3.f, 0.f));
            // 副雷达锥桶（稍有偏移，模拟未标定状态）
            double dx_err = 0.15 * std::exp(-0.05 * t);
            double dy_err = 0.08 * std::exp(-0.05 * t);
            ll.publishCloud(1, sim::makeConeCloud("lidar_rear", stamp, 0.55f, 5, 1.5f, 3.f, dy_err));
            ll.publishResult(dx_err, dy_err, 0.5 * std::exp(-0.05 * t), stamp);
            ll.publishStatus("Δx=" + std::to_string(dx_err).substr(0,6) +
                              "m  Δy=" + std::to_string(dy_err).substr(0,6) + "m", stamp);
        }

        // --------- Tab5: Image-LiDAR ---------
        {
            auto img = sim::makeCheckerImage(
                "image_lidar_calib_node", stamp, 1280, 720, 30, t, "cam_front+lidar");
            img->header.stamp = stamp;
            il.publishImage(0, img);
            il.publishResult(0.02 * std::exp(-0.1*t), -0.01 * std::exp(-0.1*t),
                             0.005 * std::exp(-0.1*t), stamp);
            il.publishStatus("角点检测: OK  重投影误差=" +
                              std::to_string(0.5 * std::exp(-0.1*t)).substr(0,4) + "px", stamp);
        }

        // --------- Tab6: Camera-Camera ---------
        {
            auto img_ref = sim::makeCheckerImage(
                "camera_cali_node", stamp, 640, 480, 60, t, "cam_ref");
            img_ref->header.stamp = stamp;
            cc.publishImage(0, img_ref);

            auto img_tgt = sim::makeCheckerImage(
                "camera_cali_node", stamp, 640, 480, 120, t + 0.5, "cam_target");
            img_tgt->header.stamp = stamp;
            cc.publishImage(1, img_tgt);

            cc.publishResult(0.1 * std::exp(-0.08*t), 0.05 * std::exp(-0.08*t),
                             0.3 * std::exp(-0.08*t), stamp);
            cc.publishStatus("双目标定中...  误差=" +
                              std::to_string(0.3 * std::exp(-0.08*t)).substr(0,4) + "px", stamp);
        }

        // --------- Tab7: BEV ---------
        {
            auto bev_img = sim::makeBevImage(stamp, t);
            bev.publishImage(0, bev_img);
            bev.publishStatus("BEV拼接正常  4路相机就绪", stamp);
        }

        // --------- Tab8: Front Stitch ---------
        {
            auto stitch_img = sim::makeStitchedImage(stamp, t);
            fs.publishImage(0, stitch_img);
            fs.publishStatus("左前标定: OK  前右标定: 等待触发", stamp);
        }

        t += dt;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
