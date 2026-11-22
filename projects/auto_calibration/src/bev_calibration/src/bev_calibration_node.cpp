/**
 * bev_calibration_node.cpp
 *
 * 环视BEV标定节点，分3步完成外参标定：
 *   Step 1 - detect:       对4路鱼眼相机做棋盘格角点检测并排序
 *   Step 2 - calib_rpyz:   联合优化 roll/pitch/yaw/Z，使角点投影共面且间距等于 square_size
 *   Step 3 - calib_xy:     以前相机为基准，通过共视区域对齐其他相机的XY
 *
 * 内外参格式与 bev_surround_view 完全一致。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#if AUTO_CALIB_BOOST_NEW_PLACEHOLDERS
#include <boost/bind/bind.hpp>
#else
#include <boost/bind.hpp>
#endif
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Dense>
#include "camera_model/mei.h"

#include "CornerDetAC.h"
#include "ChessboradStruct.h"

// ============================================================
// 数据结构
// ============================================================

struct CameraConfig {
    std::string role;          // "front" / "back" / "left" / "right"
    std::string camera_name;
    std::string image_topic;

    // 内参
    cv::Mat K;
    cv::Mat D;
    double epsilon = 0.0;
    int width = 0;
    int height = 0;

    // 外参（可标定量）
    double tx = 0.0;   // t[0] = install_y
    double ty = 0.0;   // t[1] = install_z + vcs_to_ground_height
    double tz = 0.0;   // t[2] = -install_x
    double roll  = 0.0;  // rad
    double pitch = 0.0;  // rad
    double yaw   = 0.0;  // rad

    double vcs_to_ground_height = 0.37;
    std::array<int, 3> rotation_order = {{2, 1, 0}};

    bool intrinsics_ready = false;
};

struct BoardSpec {
    int rows = 4;
    int cols = 4;
    std::string label;
};

struct BoardDetection {
    BoardSpec spec;
    std::vector<cv::Point2f> corners;
};

struct DetectionResult {
    bool valid = false;
    std::vector<cv::Point2f> corners;        // 所有板子角点拼接（用于 RPY/XY 优化）
    std::vector<BoardDetection> boards;      // 每块板子的独立检测结果（用于 Z 优化）
    int corner_rows = 0;                     // 第一块有效板的行数
    int corner_cols = 0;                     // 第一块有效板的列数
    double square_size = 0.2;               // m
    cv::Mat debug_image;
};

// ============================================================
// 工具函数
// ============================================================

static camera::Mei buildMei(const CameraConfig& cfg)
{
    camera::Mei m(cfg.rotation_order);
    m.setIntrinsics(cfg.epsilon, cfg.K, cfg.D, cfg.width, cfg.height);
    m.setGroundHeight(cfg.vcs_to_ground_height);
    Eigen::Vector3d t(cfg.tx, cfg.ty, cfg.tz);
    Eigen::Vector3d r(cfg.roll, cfg.pitch, cfg.yaw);
    m.setExtrinsics(t, r);
    return m;
}

// 将像素坐标投影到地面平面（VCS z = 0），返回 VCS XY
// 使用射线-平面求交：ray_world = R^T * ray_cam，cam_center = -R^T * t
static bool projectPixelToGround(const camera::Mei& mei,
                                  const cv::Point2d& px,
                                  Eigen::Vector3d& pt_world)
{
    // 取相机坐标系中 z=1 处的点，得到射线方向
    Eigen::Vector3d ray_cam;
    cv::Point2d dummy;
    // projectCamera2dToCamera3d 给出 camera_z=1 时的相机坐标点
    if (!mei.projectCamera2dToCamera3d(px, 1.0, ray_cam)) {
        return false;
    }
    // 转到世界坐标系
    Eigen::Vector3d ray_world;
    Eigen::Vector3d origin_cam(0, 0, 0);
    Eigen::Vector3d origin_world;
    if (!mei.camera3dToWorld3d(origin_cam, origin_world)) {
        return false;
    }
    Eigen::Vector3d pt_cam_unit;
    if (!mei.camera3dToWorld3d(ray_cam, pt_cam_unit)) {
        return false;
    }
    ray_world = pt_cam_unit - origin_world;

    if (std::abs(ray_world.z()) < 1e-9) {
        return false;
    }
    // 地面平面 z_world = 0
    double lambda = -origin_world.z() / ray_world.z();
    if (!std::isfinite(lambda)) {
        return false;
    }
    pt_world = origin_world + lambda * ray_world;
    return std::isfinite(pt_world.x()) && std::isfinite(pt_world.y()) && std::isfinite(pt_world.z());
}

// 规范化角点方向：保留 ChessboradStruct 的网格拓扑，只翻转行/列
// 使 row0 在图像上方（y 较小），col0 在图像左侧（x 较小）
// 对鱼眼畸变比像素坐标全局排序更鲁棒
static void normalizeCornerOrientation(std::vector<cv::Point2f>& corners, int rows, int cols)
{
    if ((int)corners.size() != rows * cols) return;

    // 第一行 vs 最后一行的平均 y
    float y0 = 0, yN = 0;
    for (int c = 0; c < cols; ++c) {
        y0 += corners[c].y;
        yN += corners[(rows - 1) * cols + c].y;
    }
    if (y0 > yN) {  // row0 在下方，翻转行
        for (int r = 0; r < rows / 2; ++r)
            for (int c = 0; c < cols; ++c)
                std::swap(corners[r * cols + c], corners[(rows - 1 - r) * cols + c]);
    }

    // 第一列 vs 最后一列的平均 x
    float x0 = 0, xN = 0;
    for (int r = 0; r < rows; ++r) {
        x0 += corners[r * cols].x;
        xN += corners[r * cols + cols - 1].x;
    }
    if (x0 > xN) {  // col0 在右侧，翻转列
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols / 2; ++c)
                std::swap(corners[r * cols + c], corners[r * cols + cols - 1 - c]);
    }
}

// 计算一组地面投影点的 z 方差（用于 RPY 优化目标）
static double groundZVariance(const camera::Mei& mei,
                               const std::vector<cv::Point2f>& pixels)
{
    std::vector<double> zvals;
    for (const auto& px : pixels) {
        Eigen::Vector3d pw;
        if (projectPixelToGround(mei, cv::Point2d(px.x, px.y), pw)) {
            zvals.push_back(pw.z());
        }
    }
    if (zvals.size() < 2) return 1e9;
    double mean = std::accumulate(zvals.begin(), zvals.end(), 0.0) / zvals.size();
    double var = 0.0;
    for (double z : zvals) var += (z - mean) * (z - mean);
    return var / zvals.size();
}

// 计算相邻角点在地面投影后的平均间距（用于 Z 标定）
static double meanGroundCornerSpacing(const camera::Mei& mei,
                                       const std::vector<cv::Point2f>& pixels,
                                       int rows, int cols)
{
    std::vector<double> dists;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols - 1; ++c) {
            Eigen::Vector3d p0, p1;
            const cv::Point2f& px0 = pixels[r * cols + c];
            const cv::Point2f& px1 = pixels[r * cols + c + 1];
            if (projectPixelToGround(mei, cv::Point2d(px0.x, px0.y), p0) &&
                projectPixelToGround(mei, cv::Point2d(px1.x, px1.y), p1)) {
                dists.push_back((p1 - p0).norm());
            }
        }
    }
    for (int r = 0; r < rows - 1; ++r) {
        for (int c = 0; c < cols; ++c) {
            Eigen::Vector3d p0, p1;
            const cv::Point2f& px0 = pixels[r * cols + c];
            const cv::Point2f& px1 = pixels[(r + 1) * cols + c];
            if (projectPixelToGround(mei, cv::Point2d(px0.x, px0.y), p0) &&
                projectPixelToGround(mei, cv::Point2d(px1.x, px1.y), p1)) {
                dists.push_back((p1 - p0).norm());
            }
        }
    }
    if (dists.empty()) return 0.0;
    return std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
}

// 简单数值梯度下降（有限差分），最小化 func(params)
static void gradientDescent(std::vector<double>& params,
                             std::function<double(const std::vector<double>&)> func,
                             double step, double tol, int max_iter)
{
    int n = params.size();
    for (int iter = 0; iter < max_iter; ++iter) {
        double f0 = func(params);
        if (f0 < tol) break;
        std::vector<double> grad(n, 0.0);
        for (int i = 0; i < n; ++i) {
            std::vector<double> p = params;
            p[i] += step;
            grad[i] = (func(p) - f0) / step;
        }
        double gnorm = 0.0;
        for (double g : grad) gnorm += g * g;
        gnorm = std::sqrt(gnorm);
        if (gnorm < 1e-12) break;
        for (int i = 0; i < n; ++i) {
            params[i] -= step * grad[i] / gnorm;
        }
    }
}

// 带边界约束的梯度下降，每步更新后 clamp 到 [lb, ub]
static void gradientDescentBounded(std::vector<double>& params,
                                    std::function<double(const std::vector<double>&)> func,
                                    double step, double tol, int max_iter,
                                    const std::vector<double>& lb,
                                    const std::vector<double>& ub)
{
    int n = params.size();
    for (int iter = 0; iter < max_iter; ++iter) {
        double f0 = func(params);
        if (f0 < tol) break;
        std::vector<double> grad(n, 0.0);
        for (int i = 0; i < n; ++i) {
            std::vector<double> p = params;
            p[i] += step;
            grad[i] = (func(p) - f0) / step;
        }
        double gnorm = 0.0;
        for (double g : grad) gnorm += g * g;
        gnorm = std::sqrt(gnorm);
        if (gnorm < 1e-12) break;
        for (int i = 0; i < n; ++i) {
            params[i] -= step * grad[i] / gnorm;
            params[i] = std::max(lb[i], std::min(ub[i], params[i]));
        }
    }
}

// ============================================================
// 主节点类
// ============================================================

class BevCalibrationNode {
public:
    BevCalibrationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh)
    {
        loadParams();
        setupIo();
        ROS_INFO("[BevCalibrationNode] init OK");
    }

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    std::array<CameraConfig, 4> cameras_;
    std::array<DetectionResult, 4> detections_;
    std::array<cv::Mat, 4> latest_images_;
    std::array<bool, 4> has_image_ = {{false, false, false, false}};
    std::mutex img_mutex_;

    ros::Subscriber subs_[4];
    image_transport::Publisher pub_debug_[4];
    ros::Publisher pub_status_;

    ros::ServiceServer srv_detect_;
    ros::ServiceServer srv_calib_rpyz_;
    ros::ServiceServer srv_calib_xy_;
    ros::ServiceServer srv_save_;

    std::vector<BoardSpec> board_specs_;
    double square_size_ = 0.2;   // m
    std::string output_dir_;

    // --------------------------------------------------------
    // 参数加载
    // --------------------------------------------------------
    void loadParams()
    {
        pnh_.param<double>("chessboard/square_size", square_size_, 0.2);

        // 加载多组棋盘格规格
        // ROS param server 不直接支持 YAML 列表，用 XmlRpc 读取
        if (pnh_.hasParam("chessboard/boards")) {
            XmlRpc::XmlRpcValue boards_param;
            pnh_.getParam("chessboard/boards", boards_param);
            if (boards_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int b = 0; b < boards_param.size(); ++b) {
                    BoardSpec spec;
                    spec.rows  = static_cast<int>(boards_param[b]["rows"]);
                    spec.cols  = static_cast<int>(boards_param[b]["cols"]);
                    if (boards_param[b].hasMember("label"))
                        spec.label = static_cast<std::string>(boards_param[b]["label"]);
                    board_specs_.push_back(spec);
                }
            }
        }
        if (board_specs_.empty()) {
            // 兼容旧配置：单组棋盘格
            BoardSpec spec;
            pnh_.param<int>("chessboard/corner_rows", spec.rows, 4);
            pnh_.param<int>("chessboard/corner_cols", spec.cols, 6);
            spec.label = "default";
            board_specs_.push_back(spec);
        }
        ROS_INFO("[BevCalibrationNode] loaded %zu board spec(s)", board_specs_.size());
        pnh_.param<std::string>("output_dir", output_dir_, "/tmp/bev_calibration");

        const std::string roles[4]  = {"front", "back", "left", "right"};
        const std::string dnames[4] = {"front_fish_eye", "back_fish_eye",
                                       "left_fish_eye",  "right_fish_eye"};
        const std::string dtopics[4] = {
            "/camera_front/compressed",
            "/camera_rear/compressed",
            "/camera_left/compressed",
            "/camera_right/compressed"
        };

        for (int i = 0; i < 4; ++i) {
            cameras_[i].role = roles[i];
            pnh_.param<std::string>("cameras/" + roles[i] + "/name",
                                    cameras_[i].camera_name, dnames[i]);
            pnh_.param<std::string>("cameras/" + roles[i] + "/topic",
                                    cameras_[i].image_topic, dtopics[i]);

            std::string intr_file;
            pnh_.param<std::string>(roles[i] + "_intrinsics_file", intr_file, "");
            std::string err;
            if (!loadIntrinsics(intr_file, cameras_[i], err)) {
                ROS_FATAL("[BevCalibrationNode] %s", err.c_str());
                throw std::runtime_error(err);
            }

            double ground_h = 0.37;
            pnh_.param<double>("vcs/z_offset", ground_h, 0.37);
            cameras_[i].vcs_to_ground_height = ground_h;

            loadInstallation(roles[i], cameras_[i]);
        }
    }

    bool loadIntrinsics(const std::string& path, CameraConfig& cam, std::string& err)
    {
        if (path.empty()) { err = "intrinsics file empty for " + cam.role; return false; }
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) { err = "cannot open " + path; return false; }

        int w = 0, h = 0;
        double xi = 0, g1 = 0, g2 = 0, u0 = 0, v0 = 0;
        double k1 = 0, k2 = 0, p1 = 0, p2 = 0;
        fs["image_width"]  >> w;
        fs["image_height"] >> h;
        fs["mirror_parameters"]["xi"]       >> xi;
        fs["projection_parameters"]["gamma1"] >> g1;
        fs["projection_parameters"]["gamma2"] >> g2;
        fs["projection_parameters"]["u0"]     >> u0;
        fs["projection_parameters"]["v0"]     >> v0;
        fs["distortion_parameters"]["k1"]     >> k1;
        fs["distortion_parameters"]["k2"]     >> k2;
        fs["distortion_parameters"]["p1"]     >> p1;
        fs["distortion_parameters"]["p2"]     >> p2;

        if (w <= 0 || h <= 0) { err = "invalid image size in " + path; return false; }

        cam.K = cv::Mat::eye(3, 3, CV_64F);
        cam.K.at<double>(0,0) = g1; cam.K.at<double>(1,1) = g2;
        cam.K.at<double>(0,2) = u0; cam.K.at<double>(1,2) = v0;
        cam.D = cv::Mat::zeros(1, 4, CV_64F);
        cam.D.at<double>(0,0) = k1; cam.D.at<double>(0,1) = k2;
        cam.D.at<double>(0,2) = p1; cam.D.at<double>(0,3) = p2;
        cam.epsilon = xi;
        cam.width = w; cam.height = h;
        cam.intrinsics_ready = true;
        return true;
    }

    void loadInstallation(const std::string& role, CameraConfig& cam)
    {
        const std::string pfx = "installation/" + role + "/";
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        pnh_.param<double>(pfx + "x",     x,     0.0);
        pnh_.param<double>(pfx + "y",     y,     0.0);
        pnh_.param<double>(pfx + "z",     z,     0.0);
        pnh_.param<double>(pfx + "roll",  roll,  0.0);
        pnh_.param<double>(pfx + "pitch", pitch, 0.0);
        pnh_.param<double>(pfx + "yaw",   yaw,   0.0);

        // 与 bev_surround_view 保持一致的坐标映射
        cam.tx = y;
        cam.ty = z + cam.vcs_to_ground_height;
        cam.tz = -x;
        cam.roll  = roll  * M_PI / 180.0;
        cam.pitch = pitch * M_PI / 180.0;
        cam.yaw   = yaw   * M_PI / 180.0;
    }

    // --------------------------------------------------------
    // IO 设置
    // --------------------------------------------------------
    void setupIo()
    {
        const std::string roles[4] = {"front", "back", "left", "right"};
        for (int i = 0; i < 4; ++i) {
            subs_[i] = nh_.subscribe<sensor_msgs::CompressedImage>(
                cameras_[i].image_topic, 1,
#if AUTO_CALIB_BOOST_NEW_PLACEHOLDERS
                boost::bind(&BevCalibrationNode::imageCallback, this, boost::placeholders::_1, i));
#else
                boost::bind(&BevCalibrationNode::imageCallback, this, _1, i));
#endif
            pub_debug_[i] = it_.advertise(
                "/bev_calibration/debug/" + roles[i], 1, true);
        }
        pub_status_ = pnh_.advertise<std_msgs::String>("status", 10, true);

        srv_detect_     = pnh_.advertiseService("detect",     &BevCalibrationNode::srvDetect,    this);
        srv_calib_rpyz_ = pnh_.advertiseService("calib_rpyz",&BevCalibrationNode::srvCalibRpyZ, this);
        srv_calib_xy_   = pnh_.advertiseService("calib_xy",  &BevCalibrationNode::srvCalibXY,   this);
        srv_save_       = pnh_.advertiseService("save",      &BevCalibrationNode::srvSave,       this);
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg, int idx)
    {
        try {
            cv::Mat img = cv::imdecode(
                cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (img.empty()) return;
            {
                std::lock_guard<std::mutex> lk(img_mutex_);
                latest_images_[idx] = img;
                has_image_[idx] = true;
            }
            // 每帧都跑检测并发布调试图，方便实时观察角点检测效果
            detectChessboard(img, cameras_[idx].role, idx);
        } catch (...) {}
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::String msg;
        msg.data = s;
        pub_status_.publish(msg);
        ROS_INFO("[BevCalibrationNode] %s", s.c_str());
    }

    // --------------------------------------------------------
    // Step 1: 棋盘格检测
    // --------------------------------------------------------
    bool srvDetect(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        std::array<cv::Mat, 4> imgs;
        {
            std::lock_guard<std::mutex> lk(img_mutex_);
            for (int i = 0; i < 4; ++i) {
                if (!has_image_[i]) {
                    res.success = false;
                    res.message = "no image for camera " + cameras_[i].role;
                    return true;
                }
                imgs[i] = latest_images_[i].clone();
            }
        }

        bool all_ok = true;
        for (int i = 0; i < 4; ++i) {
            detections_[i] = detectChessboard(imgs[i], cameras_[i].role, i);
            if (!detections_[i].valid) {
                all_ok = false;
                ROS_WARN("[BevCalibrationNode] detect failed: %s", cameras_[i].role.c_str());
            }
        }

        res.success = all_ok;
        res.message = all_ok ? "detection OK for all cameras"
                             : "detection failed for one or more cameras";
        publishStatus(res.message);
        return true;
    }

    DetectionResult detectChessboard(const cv::Mat& img,
                                      const std::string& role,
                                      int cam_idx)
    {
        DetectionResult result;
        result.square_size = square_size_;
        result.debug_image = img.clone();

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // 用 CornerDetAC + ChessboradStruct 同时检测所有棋盘格
        std::vector<cv::Point> corners_p;
        Corners corners_s;
        std::vector<cv::Mat> chessboards;
        {
            cv::Mat gray_copy = gray.clone();
            CornerDetAC corner_detector(gray_copy);
            corner_detector.detectCorners(gray_copy, corners_p, corners_s, 0.01);
            ChessboradStruct chessboard_struct;
            chessboard_struct.chessboardsFromCorners(corners_s, chessboards, 0.5f);
        }

        // 将 cv::Mat 索引表转为角点坐标列表，同时记录重心 cx/cy 用于排序
        struct RawBoard {
            std::vector<cv::Point2f> corners;
            float cx, cy;   // 重心，比 min_x/min_y 对畸变更鲁棒
        };
        std::vector<RawBoard> raw_boards;
        for (const auto& cb : chessboards) {
            RawBoard rb;
            rb.cx = 0.f; rb.cy = 0.f;
            int cnt = 0;
            for (int r = 0; r < cb.rows; ++r) {
                for (int c = 0; c < cb.cols; ++c) {
                    int idx = cb.at<int>(r, c);
                    if (idx >= 0 && idx < (int)corners_s.p.size()) {
                        cv::Point2f pt = corners_s.p[idx];
                        rb.corners.push_back(pt);
                        rb.cx += pt.x;
                        rb.cy += pt.y;
                        ++cnt;
                    }
                }
            }
            if (cnt > 0) {
                rb.cx /= cnt;
                rb.cy /= cnt;
                raw_boards.push_back(rb);
            }
        }

        size_t n_boards = board_specs_.size();

        // 打印每块检测到的板子角点数，方便诊断匹配问题
        {
            std::string sz_str;
            for (size_t i = 0; i < raw_boards.size(); ++i) {
                sz_str += std::to_string(raw_boards[i].corners.size());
                if (i + 1 < raw_boards.size()) sz_str += ",";
            }
            ROS_INFO("[BevCalibrationNode] %s: detected %zu raw boards (need %zu) sizes=[%s]",
                     role.c_str(), raw_boards.size(), n_boards, sz_str.c_str());
        }

        // 按角点数量匹配 BoardSpec，对同尺寸的板子再按 cx 升序区分左右
        // 这比纯空间排序对鱼眼畸变更鲁棒（中间板 4×6=24 角点，左右板 4×4=16 角点）
        std::vector<RawBoard*> assigned(n_boards, nullptr);
        for (size_t bi = 0; bi < n_boards; ++bi) {
            int expected = board_specs_[bi].rows * board_specs_[bi].cols;
            // 收集与该 spec 角点数匹配、且尚未被分配的候选板
            std::vector<RawBoard*> candidates;
            for (auto& rb : raw_boards) {
                if ((int)rb.corners.size() == expected) {
                    // 检查是否已被前面的 spec 占用
                    bool used = false;
                    for (size_t k = 0; k < bi; ++k)
                        if (assigned[k] == &rb) { used = true; break; }
                    if (!used) candidates.push_back(&rb);
                }
            }
            if (candidates.empty()) continue;
            // candidates 已排除已分配的板，按 cx 升序取第一个即可
            std::sort(candidates.begin(), candidates.end(),
                      [](const RawBoard* a, const RawBoard* b){ return a->cx < b->cx; });
            assigned[bi] = candidates[0];
        }

        for (size_t bi = 0; bi < n_boards; ++bi) {
            const BoardSpec& spec = board_specs_[bi];
            int expected = spec.rows * spec.cols;
            int draw_y   = 50 + static_cast<int>(bi) * 40;

            if (!assigned[bi]) {
                ROS_WARN("[BevCalibrationNode] %s board '%s': not detected",
                         role.c_str(), spec.label.c_str());
                cv::putText(result.debug_image, spec.label + ": FAILED",
                            cv::Point(20, draw_y),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                continue;
            }

            std::vector<cv::Point2f> corners = assigned[bi]->corners;

            if ((int)corners.size() != expected) {
                ROS_WARN("[BevCalibrationNode] %s board '%s': corners=%zu expected=%d",
                         role.c_str(), spec.label.c_str(), corners.size(), expected);
                cv::putText(result.debug_image, spec.label + ": FAILED",
                            cv::Point(20, draw_y),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                continue;
            }

            // 亚像素精化
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                              30, 0.01));

            normalizeCornerOrientation(corners, spec.rows, spec.cols);

            cv::drawChessboardCorners(result.debug_image, cv::Size(spec.cols, spec.rows),
                                      corners, true);
            cv::putText(result.debug_image, spec.label + ": OK",
                        cv::Point(20, draw_y),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            BoardDetection bd;
            bd.spec    = spec;
            bd.corners = corners;
            result.boards.push_back(bd);

            result.corners.insert(result.corners.end(), corners.begin(), corners.end());

            if (result.corner_rows == 0) {
                result.corner_rows = spec.rows;
                result.corner_cols = spec.cols;
            }
        }

        result.valid = !result.boards.empty();
        if (!result.valid) {
            cv::putText(result.debug_image, "ALL BOARDS FAILED",
                        cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX,
                        1.5, cv::Scalar(0, 0, 255), 3);
        }

        // 无论成功失败都发布调试图
        sensor_msgs::Image dbg_msg;
        dbg_msg.header.stamp = ros::Time::now();
        dbg_msg.encoding = "bgr8";
        dbg_msg.height   = result.debug_image.rows;
        dbg_msg.width    = result.debug_image.cols;
        dbg_msg.step     = result.debug_image.cols * 3;
        dbg_msg.data.assign(result.debug_image.data,
                            result.debug_image.data +
                            result.debug_image.total() * result.debug_image.elemSize());
        pub_debug_[cam_idx].publish(dbg_msg);

        return result;
    }

    // --------------------------------------------------------
    // Step 2: RPY + Z 联合标定
    // 目标：同时优化 roll/pitch/yaw/ty，使角点投影共面（z 方差最小）
    //       且投影后棋盘格间距等于 square_size
    // 搜索范围：roll/pitch/yaw ±10°，ty ±0.1m
    // --------------------------------------------------------
    bool srvCalibRpyZ(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        for (int i = 0; i < 4; ++i) {
            if (!detections_[i].valid) {
                res.success = false;
                res.message = "no valid detection for " + cameras_[i].role + ", run detect first";
                return true;
            }
        }

        for (int i = 0; i < 4; ++i) {
            calibrateRpyZ(cameras_[i], detections_[i]);
        }

        res.success = true;
        res.message = "RPY+Z calibration done";
        publishStatus(res.message);
        return true;
    }

    void calibrateRpyZ(CameraConfig& cam, const DetectionResult& det)
    {
        const double kRpyRange = 10.0 * M_PI / 180.0;  // ±10°
        const double kTyRange  = 0.1;                   // ±0.1m

        // 优化参数: [roll, pitch, yaw, ty]
        std::vector<double> params = {cam.roll, cam.pitch, cam.yaw, cam.ty};
        std::vector<double> lb = {cam.roll  - kRpyRange,
                                  cam.pitch - kRpyRange,
                                  cam.yaw   - kRpyRange,
                                  cam.ty    - kTyRange};
        std::vector<double> ub = {cam.roll  + kRpyRange,
                                  cam.pitch + kRpyRange,
                                  cam.yaw   + kRpyRange,
                                  cam.ty    + kTyRange};

        // 联合代价：z 方差（共面性）+ 间距误差²（尺度正确性），单位均为 m²
        auto cost = [&](const std::vector<double>& p) -> double {
            CameraConfig tmp = cam;
            tmp.roll  = p[0];
            tmp.pitch = p[1];
            tmp.yaw   = p[2];
            tmp.ty    = p[3];
            camera::Mei mei = buildMei(tmp);
            double z_var = groundZVariance(mei, det.corners);
            double spacing = meanGroundCornerSpacing(
                mei, det.corners, det.corner_rows, det.corner_cols);
            if (spacing < 1e-6) return 1e9;
            double spacing_err = spacing - det.square_size;
            return z_var + spacing_err * spacing_err;
        };

        // 三阶段：粗 → 中 → 细（精度 ~0.03°）
        gradientDescentBounded(params, cost, 5e-2, 1e-3, 500, lb, ub);
        gradientDescentBounded(params, cost, 5e-3, 1e-5, 300, lb, ub);
        gradientDescentBounded(params, cost, 5e-4, 1e-8, 200, lb, ub);

        cam.roll  = params[0];
        cam.pitch = params[1];
        cam.yaw   = params[2];
        cam.ty    = params[3];

        // 打印结果
        camera::Mei mei_final = buildMei(cam);
        double z_var_final = groundZVariance(mei_final, det.corners);
        double spacing_final = meanGroundCornerSpacing(
            mei_final, det.corners, det.corner_rows, det.corner_cols);
        ROS_INFO("[BevCalibrationNode] %s RPY+Z calib done: "
                 "roll=%.4f pitch=%.4f yaw=%.4f (deg), ty=%.4f | "
                 "z_var=%.6f spacing=%.4f (target=%.4f)",
                 cam.role.c_str(),
                 cam.roll  * 180.0 / M_PI,
                 cam.pitch * 180.0 / M_PI,
                 cam.yaw   * 180.0 / M_PI,
                 cam.ty,
                 z_var_final, spacing_final, det.square_size);
    }

    // --------------------------------------------------------
    // Step 4: XY 对齐
    // 以前相机为基准，通过共视区域最小化角点投影误差对齐其他相机的 tx/tz
    // 共视区域定义：前-左、前-右、后-左、后-右 四个角落
    // --------------------------------------------------------
    bool srvCalibXY(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        for (int i = 0; i < 4; ++i) {
            if (!detections_[i].valid) {
                res.success = false;
                res.message = "no valid detection for " + cameras_[i].role + ", run detect first";
                return true;
            }
        }

        // 先计算前相机的地面投影作为参考
        camera::Mei front_mei = buildMei(cameras_[0]);
        std::vector<Eigen::Vector3d> front_pts = projectToGround(front_mei, detections_[0].corners);

        // 对 back(1), left(2), right(3) 分别优化 tx/tz
        for (int i = 1; i < 4; ++i) {
            calibrateXY(cameras_[i], detections_[i], front_pts);
            ROS_INFO("[BevCalibrationNode] %s XY after calib: tx=%.4f tz=%.4f",
                     cameras_[i].role.c_str(), cameras_[i].tx, cameras_[i].tz);
        }

        res.success = true;
        res.message = "XY calibration done";
        publishStatus(res.message);
        return true;
    }

    std::vector<Eigen::Vector3d> projectToGround(const camera::Mei& mei,
                                                   const std::vector<cv::Point2f>& pixels)
    {
        std::vector<Eigen::Vector3d> pts;
        for (const auto& px : pixels) {
            Eigen::Vector3d pw;
            if (projectPixelToGround(mei, cv::Point2d(px.x, px.y), pw)) {
                pts.push_back(pw);
            } else {
                pts.push_back(Eigen::Vector3d(0, 0, 0));
            }
        }
        return pts;
    }

    // 计算两组地面点的 XY 均值偏差（用于 XY 对齐目标函数）
    void calibrateXY(CameraConfig& cam,
                     const DetectionResult& det,
                     const std::vector<Eigen::Vector3d>& ref_pts)
    {
        // 优化参数: [tx, tz]
        std::vector<double> params = {cam.tx, cam.tz};

        auto cost = [&](const std::vector<double>& p) -> double {
            CameraConfig tmp = cam;
            tmp.tx = p[0];
            tmp.tz = p[1];
            camera::Mei mei = buildMei(tmp);
            std::vector<Eigen::Vector3d> cur_pts = projectToGround(mei, det.corners);

            // 只使用两组点中都有效的点对
            size_t n = std::min(cur_pts.size(), ref_pts.size());
            if (n == 0) return 1e9;

            double total = 0.0;
            int count = 0;
            for (size_t k = 0; k < n; ++k) {
                double dx = cur_pts[k].x() - ref_pts[k].x();
                double dy = cur_pts[k].y() - ref_pts[k].y();
                total += dx * dx + dy * dy;
                ++count;
            }
            return count > 0 ? total / count : 1e9;
        };

        gradientDescent(params, cost, 1e-3, 1e-10, 500);
        gradientDescent(params, cost, 1e-5, 1e-12, 200);

        cam.tx = params[0];
        cam.tz = params[1];
    }

    // --------------------------------------------------------
    // 保存标定结果（与 bev_surround_view 外参 YAML 格式一致）
    // --------------------------------------------------------
    bool srvSave(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        // 确保输出目录存在
        if (system(("mkdir -p " + output_dir_).c_str()) != 0) {
            res.success = false;
            res.message = "failed to create output dir: " + output_dir_;
            return true;
        }

        const std::string roles[4] = {"front", "back", "left", "right"};
        for (int i = 0; i < 4; ++i) {
            std::string path = output_dir_ + "/" + roles[i] + "_extrinsic.yaml";
            if (!saveExtrinsicYaml(cameras_[i], path)) {
                res.success = false;
                res.message = "failed to save " + path;
                return true;
            }
            ROS_INFO("[BevCalibrationNode] saved %s", path.c_str());
        }

        res.success = true;
        res.message = "saved calibration to " + output_dir_;
        publishStatus(res.message);
        return true;
    }

    bool saveExtrinsicYaml(const CameraConfig& cam, const std::string& path)
    {
        // 将内部弧度制转回度，tx/tz 还原为 VCS install_x/y/z
        // cam.tx = install_y, cam.tz = -install_x, cam.ty = install_z + ground_h
        double install_x = -cam.tz;
        double install_y =  cam.tx;
        double install_z =  cam.ty - cam.vcs_to_ground_height;
        double roll_deg  =  cam.roll  * 180.0 / M_PI;
        double pitch_deg =  cam.pitch * 180.0 / M_PI;
        double yaw_deg   =  cam.yaw   * 180.0 / M_PI;

        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        if (!fs.isOpened()) return false;

        fs << "camera_name" << cam.camera_name;
        fs << "role"        << cam.role;
        fs << "installation";
        fs << "{";
        fs << "x"     << install_x;
        fs << "y"     << install_y;
        fs << "z"     << install_z;
        fs << "roll"  << roll_deg;
        fs << "pitch" << pitch_deg;
        fs << "yaw"   << yaw_deg;
        fs << "}";
        fs << "t" << std::vector<double>{cam.tx, cam.ty, cam.tz};
        fs << "r_euler_rad" << std::vector<double>{cam.roll, cam.pitch, cam.yaw};
        fs << "vcs_to_ground_height" << cam.vcs_to_ground_height;
        fs.release();
        return true;
    }
};  // class BevCalibrationNode

// ============================================================
// main
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bev_calibration_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try {
        BevCalibrationNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[bev_calibration_node] fatal: %s", e.what());
        return 1;
    }
    return 0;
}
