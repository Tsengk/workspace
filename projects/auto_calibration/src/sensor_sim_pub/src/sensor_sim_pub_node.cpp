/**
 * sensor_sim_pub_node.cpp
 *
 * 合成传感器数据发布节点，用于标定 GUI 可视化测试。
 *
 * 读取 vehicle_sample.yaml，对每个传感器持续发布：
 *   - LiDAR  : PointCloud2（地面平面 + 锥桶群）
 *   - Camera : sensor_msgs/Image（合成彩色图）+ CameraInfo
 *   - GNSS   : nav_msgs/Odometry（匀速直线轨迹）
 *
 * 参数（rosparam，均有默认值）：
 *   ~vehicle_config   yaml 路径，默认读 ROS param "vehicle_config"
 *   ~rate_hz          发布频率，默认 10.0
 *   ~lidar_noise      LiDAR 测距噪声 σ (m)，默认 0.02
 *   ~cone_rows        前方锥桶列数，默认 5
 *   ~cone_spacing     锥桶间距 (m)，默认 1.5
 *   ~image_width      合成图宽（未配内参时），默认 640
 *   ~image_height     合成图高，默认 480
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vehicle_config/config_loader.h>

#include <random>
#include <cmath>
#include <string>
#include <vector>
#include <memory>

// ============================================================
// 辅助工具
// ============================================================
namespace {

static constexpr double DEG2RAD = M_PI / 180.0;

// 欧拉角 ZYX → 旋转矩阵
Eigen::Matrix3d euler2rot(double roll_deg, double pitch_deg, double yaw_deg)
{
    return (Eigen::AngleAxisd(yaw_deg   * DEG2RAD, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_deg * DEG2RAD, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_deg  * DEG2RAD, Eigen::Vector3d::UnitX()))
               .toRotationMatrix();
}

// ============================================================
// LiDAR 合成点云生成器
//   生成：360° 地面平面扫描 + 5×2 矩阵锥桶（车辆正前方）
// ============================================================
class LidarSim {
public:
    struct Cfg {
        double height_m   = 1.8;    // 安装高度 (m)
        double roll_deg   = 0.0;
        double pitch_deg  = 0.0;
        double noise_std  = 0.02;   // 测距噪声 σ (m)
        int    cone_rows  = 5;      // 前方锥桶数
        double cone_spacing = 1.5;  // 锥桶纵向间距 (m)
        double cone_dist_start = 3.0; // 第一个锥桶距离 (m)
    };

    explicit LidarSim(const Cfg& cfg) : cfg_(cfg), rng_(42)
    {
        noise_ = std::normal_distribution<float>(0.f,
                    static_cast<float>(cfg_.noise_std));
    }

    // 生成一帧点云（在传感器坐标系下，FLU）
    pcl::PointCloud<pcl::PointXYZI>::Ptr generate()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 旋转矩阵：将水平坐标系点变换到当前安装姿态下
        Eigen::Matrix3d R = euler2rot(cfg_.roll_deg, cfg_.pitch_deg, 0.0);
        // 地面平面在水平坐标系中 z = -height
        // 在传感器坐标系中：p_s = R * p_h

        // ---- 地面扫描（80线仿真，每线 360°/1440点）----
        const int   n_rings     = 32;
        const int   pts_per_ring = 720;
        const float v_fov_min   = -25.0f * static_cast<float>(DEG2RAD);
        const float v_fov_max   =  15.0f * static_cast<float>(DEG2RAD);
        const float h_step      = 2.0f * static_cast<float>(M_PI) / pts_per_ring;

        for (int ring = 0; ring < n_rings; ++ring) {
            float el = v_fov_min + (v_fov_max - v_fov_min) *
                       ring / std::max(1, n_rings - 1);
            for (int j = 0; j < pts_per_ring; ++j) {
                float az = j * h_step;
                // 单位方向向量（水平坐标系，FLU）
                float dx = std::cos(el) * std::cos(az);
                float dy = std::cos(el) * std::sin(az);
                float dz = std::sin(el);

                // 与地面（z = -height）求交
                if (dz >= -1e-4f) continue;   // 射向上方，不会打到地面
                float t_hit = static_cast<float>(-cfg_.height_m) / dz;
                if (t_hit < 1.0f || t_hit > 50.0f) continue;

                float px = dx * t_hit + noise_(rng_);
                float py = dy * t_hit + noise_(rng_);
                float pz = dz * t_hit + noise_(rng_);

                // 应用安装倾角
                Eigen::Vector3d p(px, py, pz);
                p = R * p;

                pcl::PointXYZI pt;
                pt.x = static_cast<float>(p.x());
                pt.y = static_cast<float>(p.y());
                pt.z = static_cast<float>(p.z());
                pt.intensity = 50.f + noise_(rng_) * 10.f;
                cloud->push_back(pt);
            }
        }

        // ---- 锥桶（圆柱点云，正前方 X 轴）----
        addCones(cloud, R);

        cloud->width    = static_cast<uint32_t>(cloud->size());
        cloud->height   = 1;
        cloud->is_dense = false;
        return cloud;
    }

private:
    void addCones(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                  const Eigen::Matrix3d& R)
    {
        const float cone_r = 0.15f;  // 锥桶底半径
        const float cone_h = 0.70f;  // 锥桶高

        for (int i = 0; i < cfg_.cone_rows; ++i) {
            float cx = static_cast<float>(
                cfg_.cone_dist_start + i * cfg_.cone_spacing);
            float cy = 0.0f;    // 沿 X 轴排列，Y=0

            // 圆柱面采样
            const int n_az = 30, n_h = 15;
            for (int ia = 0; ia < n_az; ++ia) {
                float az = ia * 2.0f * static_cast<float>(M_PI) / n_az;
                for (int ih = 0; ih < n_h; ++ih) {
                    float h = ih * cone_h / n_h;
                    // 随着高度锥形收窄
                    float r = cone_r * (1.0f - h / cone_h);

                    float px = cx + r * std::cos(az);
                    float py = cy + r * std::sin(az);
                    float pz = static_cast<float>(-cfg_.height_m) + h;

                    Eigen::Vector3d p(px, py, pz);
                    p = R * p;

                    pcl::PointXYZI pt;
                    pt.x = static_cast<float>(p.x());
                    pt.y = static_cast<float>(p.y());
                    pt.z = static_cast<float>(p.z());
                    pt.intensity = 200.f; // 高反射率，便于识别
                    cloud->push_back(pt);
                }
            }
        }
    }

    Cfg cfg_;
    std::default_random_engine rng_;
    std::normal_distribution<float> noise_;
};

// ============================================================
// 相机合成图像生成器
//   生成带有：棋盘格靶标 + 随机背景纹理 + 时间戳水印
// ============================================================
class CamSim {
public:
    struct Cfg {
        int    width  = 640;
        int    height = 480;
        double fx = 820.0, fy = 820.0;
        double cx = 320.0, cy = 240.0;
        // D: k1,k2,p1,p2,k3
        std::vector<double> D = {0.0, 0.0, 0.0, 0.0, 0.0};
        std::string name;
        int    idx   = 0;    // 相机编号，用于颜色区分
    };

    explicit CamSim(const Cfg& cfg) : cfg_(cfg) {}

    // 生成合成图
    cv::Mat generate(double t_sec)
    {
        const int W = (cfg_.width  > 0) ? cfg_.width  : 640;
        const int H = (cfg_.height > 0) ? cfg_.height : 480;

        cv::Mat img(H, W, CV_8UC3, cv::Scalar(30, 30, 30));

        // ---- 背景：渐变色块（按相机编号区分色调）----
        int hue = (cfg_.idx * 45) % 180;
        cv::Mat hsv(H, W, CV_8UC3);
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                hsv.at<cv::Vec3b>(r, c) = cv::Vec3b(
                    static_cast<uchar>(hue),
                    static_cast<uchar>(80 + 80 * r / H),
                    static_cast<uchar>(60 + 80 * c / W));
            }
        }
        cv::cvtColor(hsv, img, cv::COLOR_HSV2BGR);

        // ---- 棋盘格（8×6，格子大小 = min(W,H)/10）----
        int cell = std::min(W, H) / 10;
        int board_w = 8 * cell, board_h = 6 * cell;
        int ox = (W - board_w) / 2;
        int oy = (H - board_h) / 2;
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 8; ++col) {
                cv::Rect rect(ox + col * cell, oy + row * cell, cell, cell);
                uchar color = ((row + col) % 2 == 0) ? 255 : 0;
                cv::rectangle(img, rect, cv::Scalar(color, color, color),
                              cv::FILLED);
            }
        }

        // ---- 时间戳 + 相机名 ----
        char buf[64];
        snprintf(buf, sizeof(buf), "t=%.2fs  %s", t_sec, cfg_.name.c_str());
        cv::putText(img, buf,
                    cv::Point(8, H - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 0), 1);

        // ---- 模拟运动模糊（微弱）----
        double phase = std::fmod(t_sec * 0.5, 2.0 * M_PI);
        int    shift = static_cast<int>(3.0 * std::sin(phase));
        if (shift != 0) {
            cv::Mat M = (cv::Mat_<double>(2,3) <<
                         1, 0, shift,
                         0, 1, 0);
            cv::warpAffine(img, img, M, cv::Size(W, H));
        }

        return img;
    }

    // 构建 CameraInfo 消息
    sensor_msgs::CameraInfo makeCameraInfo(const std::string& frame_id,
                                            const ros::Time& stamp) const
    {
        sensor_msgs::CameraInfo ci;
        ci.header.stamp    = stamp;
        ci.header.frame_id = frame_id;
        ci.width  = static_cast<uint32_t>(cfg_.width  > 0 ? cfg_.width  : 640);
        ci.height = static_cast<uint32_t>(cfg_.height > 0 ? cfg_.height : 480);
        ci.distortion_model = "plumb_bob";

        // K: [fx 0 cx; 0 fy cy; 0 0 1]
        ci.K = {cfg_.fx, 0, cfg_.cx,
                0, cfg_.fy, cfg_.cy,
                0, 0, 1};
        // D
        ci.D = cfg_.D;
        // R = identity
        ci.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        // P = [fx 0 cx 0; 0 fy cy 0; 0 0 1 0]
        ci.P = {cfg_.fx, 0, cfg_.cx, 0,
                0, cfg_.fy, cfg_.cy, 0,
                0, 0, 1, 0};
        return ci;
    }

private:
    Cfg cfg_;
};

// ============================================================
// GNSS 轨迹生成器（ENU，绕圆行驶）
// ============================================================
class GnssSim {
public:
    nav_msgs::Odometry generate(double t_sec,
                                 const std::string& frame_id,
                                 const std::string& child_frame,
                                 const ros::Time& stamp)
    {
        // 绕半径 20m 的圆，线速度 2m/s
        const double R   = 20.0;
        const double v   = 2.0;
        const double omega = v / R;
        double theta = omega * t_sec;

        nav_msgs::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = frame_id;
        odom.child_frame_id  = child_frame;

        // 位置（ENU）
        odom.pose.pose.position.x = R * std::cos(theta);
        odom.pose.pose.position.y = R * std::sin(theta);
        odom.pose.pose.position.z = 0.0;

        // 朝向：切线方向（ENU yaw = θ + π/2）
        double yaw_enu = theta + M_PI / 2.0;
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_enu, Eigen::Vector3d::UnitZ()));
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度
        odom.twist.twist.linear.x  =  v * std::cos(yaw_enu);
        odom.twist.twist.linear.y  =  v * std::sin(yaw_enu);
        odom.twist.twist.angular.z =  omega;

        return odom;
    }
};

} // anonymous namespace

// ============================================================
// 主节点
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_sim_pub");
    ros::NodeHandle nh, pnh("~");

    // ---- 参数 ----
    std::string config_path;
    pnh.param<std::string>("vehicle_config", config_path, "");
    if (config_path.empty())
        nh.param<std::string>("vehicle_config", config_path, "");
    if (config_path.empty()) {
        ROS_FATAL("[sensor_sim_pub] ~vehicle_config not set. "
                  "Use: roslaunch ... vehicle_config:=<path>");
        return 1;
    }

    double rate_hz    = 10.0;
    double lidar_noise = 0.02;
    int    cone_rows  = 5;
    double cone_spacing = 1.5;
    int    img_w = 640, img_h = 480;

    pnh.param("rate_hz",     rate_hz,      10.0);
    pnh.param("lidar_noise", lidar_noise,  0.02);
    pnh.param("cone_rows",   cone_rows,    5);
    pnh.param("cone_spacing",cone_spacing, 1.5);
    pnh.param("image_width", img_w,        640);
    pnh.param("image_height",img_h,        480);

    // ---- 加载 vehicle config ----
    vehicle_config::VehicleConfig vcfg;
    if (!vehicle_config::ConfigLoader::loadFromFile(config_path, vcfg)) {
        ROS_FATAL("[sensor_sim_pub] Failed to load config: %s", config_path.c_str());
        return 1;
    }
    ROS_INFO("[sensor_sim_pub] Loaded config: %s  lidars=%zu  cameras=%zu",
             vcfg.vehicle.id.c_str(),
             vcfg.lidars.size(), vcfg.cameras.size());

    // ================================================================
    // 为每个 LiDAR 建立发布器 + 仿真器
    // ================================================================
    struct LidarPub {
        ros::Publisher pub;
        std::shared_ptr<LidarSim> sim;
        std::string frame_id;
        std::string topic;
    };
    std::vector<LidarPub> lidar_pubs;

    for (const auto& li : vcfg.lidars) {
        LidarSim::Cfg scfg;
        scfg.height_m    = (li.t.size() >= 3) ? li.t[2] : 1.8;
        scfg.roll_deg    = (li.R.size() >= 1) ? li.R[0] : 0.0;
        scfg.pitch_deg   = (li.R.size() >= 2) ? li.R[1] : 0.0;
        scfg.noise_std   = lidar_noise;
        scfg.cone_rows   = cone_rows;
        scfg.cone_spacing = cone_spacing;

        LidarPub lp;
        lp.sim      = std::make_shared<LidarSim>(scfg);
        lp.frame_id = li.name;
        lp.topic    = li.topic;
        lp.pub      = nh.advertise<sensor_msgs::PointCloud2>(li.topic, 5);

        lidar_pubs.push_back(std::move(lp));
        ROS_INFO("  LiDAR '%s' → %s", li.name.c_str(), li.topic.c_str());
    }

    // ================================================================
    // 为每个相机建立发布器 + 仿真器
    // ================================================================
    struct CamPub {
        ros::Publisher img_pub;
        ros::Publisher info_pub;
        std::shared_ptr<CamSim> sim;
        std::string frame_id;
        std::string topic;
    };
    std::vector<CamPub> cam_pubs;

    for (int idx = 0; idx < static_cast<int>(vcfg.cameras.size()); ++idx) {
        const auto& ci = vcfg.cameras[idx];

        CamSim::Cfg ccfg;
        // 优先使用配置内参
        if (ci.intrinsics.Size.size() >= 2) {
            ccfg.width  = ci.intrinsics.Size[0];
            ccfg.height = ci.intrinsics.Size[1];
        } else {
            ccfg.width  = img_w;
            ccfg.height = img_h;
        }
        // 内参矩阵 K: [fx,0,cx, 0,fy,cy, 0,0,1]
        if (ci.intrinsics.K.size() >= 9) {
            ccfg.fx = ci.intrinsics.K[0];
            ccfg.fy = ci.intrinsics.K[4];
            ccfg.cx = ci.intrinsics.K[2];
            ccfg.cy = ci.intrinsics.K[5];
        } else {
            ccfg.fx = ccfg.fy = 820.0;
            ccfg.cx = ccfg.width  / 2.0;
            ccfg.cy = ccfg.height / 2.0;
        }
        if (!ci.intrinsics.D.empty())
            ccfg.D = ci.intrinsics.D;
        ccfg.name = ci.name;
        ccfg.idx  = idx;

        CamPub cp;
        cp.sim      = std::make_shared<CamSim>(ccfg);
        cp.frame_id = ci.name;
        cp.topic    = ci.topic;
        cp.img_pub  = nh.advertise<sensor_msgs::Image>(
                        ci.topic, 5);
        // CameraInfo 发布在 image_raw 同级目录
        std::string base = ci.topic;
        // 去掉末尾的 /image_raw 或 /image_color 等，加 /camera_info
        auto slash_pos = base.rfind('/');
        std::string info_topic = (slash_pos != std::string::npos)
            ? base.substr(0, slash_pos) + "/camera_info"
            : base + "_info";
        cp.info_pub = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 5);

        cam_pubs.push_back(std::move(cp));
        ROS_INFO("  Camera '%s' → %s  (%dx%d)",
                 ci.name.c_str(), ci.topic.c_str(), ccfg.width, ccfg.height);
    }

    // ================================================================
    // GNSS 发布器
    // ================================================================
    GnssSim gnss_sim;
    ros::Publisher gnss_pub;
    std::string gnss_frame = "base_link";
    if (!vcfg.gnss.topic.empty()) {
        gnss_pub = nh.advertise<nav_msgs::Odometry>(vcfg.gnss.topic, 5);
        ROS_INFO("  GNSS '%s' → %s", vcfg.gnss.name.c_str(),
                 vcfg.gnss.topic.c_str());
    }

    // ================================================================
    // TF：发布各传感器的静态 TF（base_link → sensor_frame）
    // ================================================================
    tf2_ros::TransformBroadcaster tf_bcast;

    auto makeTF = [](const std::string& parent, const std::string& child,
                     const std::vector<double>& t, const std::vector<double>& R_euler,
                     const ros::Time& stamp) -> geometry_msgs::TransformStamped
    {
        geometry_msgs::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = parent;
        ts.child_frame_id  = child;
        if (t.size() >= 3) {
            ts.transform.translation.x = t[0];
            ts.transform.translation.y = t[1];
            ts.transform.translation.z = t[2];
        }
        double roll  = (R_euler.size() >= 1) ? R_euler[0] : 0.0;
        double pitch = (R_euler.size() >= 2) ? R_euler[1] : 0.0;
        double yaw   = (R_euler.size() >= 3) ? R_euler[2] : 0.0;
        Eigen::Quaterniond q(euler2rot(roll, pitch, yaw));
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();
        return ts;
    };

    // ================================================================
    // 主循环
    // ================================================================
    ros::Rate rate(rate_hz);
    double t_sec = 0.0;
    const double dt = 1.0 / rate_hz;

    ROS_INFO("[sensor_sim_pub] Publishing at %.1f Hz. Press Ctrl-C to stop.", rate_hz);

    while (ros::ok()) {
        ros::Time stamp = ros::Time::now();

        // ---- 发布 TF ----
        std::vector<geometry_msgs::TransformStamped> tfs;
        for (const auto& li : vcfg.lidars)
            tfs.push_back(makeTF("base_link", li.name, li.t, li.R, stamp));
        for (const auto& ci : vcfg.cameras)
            tfs.push_back(makeTF("base_link", ci.name,
                                  ci.extrinsics.t, ci.extrinsics.R_euler, stamp));
        if (!tfs.empty())
            tf_bcast.sendTransform(tfs);

        // ---- 发布 LiDAR 点云 ----
        for (auto& lp : lidar_pubs) {
            auto cloud = lp.sim->generate();
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.stamp    = stamp;
            msg.header.frame_id = lp.frame_id;
            lp.pub.publish(msg);
        }

        // ---- 发布相机图像 + CameraInfo ----
        for (auto& cp : cam_pubs) {
            cv::Mat img = cp.sim->generate(t_sec);
            // Image
            auto img_msg = cv_bridge::CvImage(
                std_msgs::Header(), "bgr8", img).toImageMsg();
            img_msg->header.stamp    = stamp;
            img_msg->header.frame_id = cp.frame_id;
            cp.img_pub.publish(img_msg);
            // CameraInfo
            auto ci_msg = cp.sim->makeCameraInfo(cp.frame_id, stamp);
            cp.info_pub.publish(ci_msg);
        }

        // ---- 发布 GNSS Odometry ----
        if (gnss_pub) {
            auto odom = gnss_sim.generate(t_sec, "world", gnss_frame, stamp);
            gnss_pub.publish(odom);
        }

        t_sec += dt;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
