#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Trigger.h>

#include <fstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>

#include "camera_model/mei.h"

namespace {

bool endsWith(const std::string& value, const std::string& suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isFinitePoint(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x()) &&
           std::isfinite(point.y()) &&
           std::isfinite(point.z());
}

cv::Scalar distanceToRainbowBgr(double distance_m)
{
    int i = static_cast<int>(distance_m * 5.0);
    if (i < 0) i = 0;
    if (i > 255) i = 255;
    const int idx = i * 5;

    if (idx < 256) return cv::Scalar(0, idx, 255);
    if (idx < 512) return cv::Scalar(0, 255, 511 - idx);
    if (idx < 768) return cv::Scalar(idx - 512, 255, 0);
    if (idx < 1024) return cv::Scalar(255, 1023 - idx, 0);
    return cv::Scalar(255, 0, idx - 1024);
}

Eigen::Vector3d toRadians(const Eigen::Vector3d& degrees)
{
    return degrees * M_PI / 180.0;
}

void ensureDir(const std::string& dir)
{
    struct stat st;
    if (stat(dir.c_str(), &st) != 0) {
        mkdir(dir.c_str(), 0755);
    }
}

}  // namespace

class LidarCameraProjectionNode {
public:
    LidarCameraProjectionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh)
    {
        loadParams();
        initCamera();
        setupPubSub();
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                                 &LidarCameraProjectionNode::onTimer,
                                 this);
        param_timer_ = nh_.createTimer(ros::Duration(0.1),
                                       &LidarCameraProjectionNode::onParamTimer,
                                       this);
        ROS_INFO("[LidarCameraProjection] init OK");
    }

private:
    struct ProjectedPoint {
        cv::Point2f uv;
        float depth = 0.0f;
        cv::Scalar color;
        int pixel_x = 0;
        int pixel_y = 0;
        int pixel_index = 0;
    };

    void loadParams()
    {
        pnh_.param<std::string>("calib_file", calib_file_, "");
        pnh_.param<std::string>("image_topic", image_topic_, "/driver/gmsl_camera/front_fish_eye/compressed");
        pnh_.param<std::string>("lidar_topic", lidar_topic_, "/lidar_points");
        pnh_.param<std::string>("output_topic", output_topic_, "/lidar_camera_projection/image");
        pnh_.param<std::string>("output_dir", output_dir_, "/tmp/lidar_camera_projection");
        pnh_.param<std::string>("output/yaml_path", yaml_path_, "");
        pnh_.param<double>("publish_rate_hz", publish_rate_hz_, 10.0);
        pnh_.param<double>("sync_tolerance_sec", sync_tolerance_sec_, 0.10);
        pnh_.param<double>("min_depth_m", min_depth_m_, 0.3);
        pnh_.param<double>("max_depth_m", max_depth_m_, 30.0);
        pnh_.param<int>("display_width", display_width_, 960);

        int rot0 = 2;
        int rot1 = 1;
        int rot2 = 0;
        pnh_.param<int>("rotation_order_0", rot0, 2);
        pnh_.param<int>("rotation_order_1", rot1, 1);
        pnh_.param<int>("rotation_order_2", rot2, 0);
        rotation_order_[0] = rot0;
        rotation_order_[1] = rot1;
        rotation_order_[2] = rot2;

        double x,y,z,roll,pitch,yaw;
        pnh_.param<double>("x", x, 0.0);
        pnh_.param<double>("y", y, 0.0);
        pnh_.param<double>("z", z, 0.0);
        lidar_to_camera_t_.x() = x;
        lidar_to_camera_t_.y() = y;
        lidar_to_camera_t_.z() = z;
        pnh_.param<double>("roll", roll, 0.0);
        pnh_.param<double>("pitch", pitch, 0.0);
        pnh_.param<double>("yaw", yaw, 0.0);
        lidar_to_camera_rpy_deg_.x() = pitch;
        lidar_to_camera_rpy_deg_.y() = yaw;
        lidar_to_camera_rpy_deg_.z() = -roll;
        // 保存原始 6DOF 微调值，用于动态调参的变化检测（避免与 mei 内部约定混淆）
        tune_x_ = x; tune_y_ = y; tune_z_ = z;
        tune_roll_ = roll; tune_pitch_ = pitch; tune_yaw_ = yaw;

        if (calib_file_.empty()) {
            throw std::runtime_error("calib_file is empty");
        }
        if (publish_rate_hz_ <= 0.0) {
            throw std::runtime_error("publish_rate_hz must be > 0");
        }
        if (max_depth_m_ <= min_depth_m_) {
            throw std::runtime_error("max_depth_m must be greater than min_depth_m");
        }
        if (display_width_ <= 0) {
            throw std::runtime_error("display_width must be > 0");
        }
    }

    void initCamera()
    {
        loadIntrinsicsFromYaml(calib_file_);

        mei_ = std::make_unique<camera::Mei>(rotation_order_);
        mei_->setIntrinsics(epsilon_, K_, D_, image_width_, image_height_);

        // 这里把 lidar frame 直接作为 Mei 的 world frame 使用。
        // 投影调用 projectWorld3dTo2d，不在节点里手动改点云坐标。
        mei_->setExtrinsics(lidar_to_camera_t_, toRadians(lidar_to_camera_rpy_deg_));

        ROS_INFO("[LidarCameraProjection] calib=%s size=%dx%d epsilon=%.6f",
                 calib_file_.c_str(), image_width_, image_height_, epsilon_);
        ROS_INFO("[LidarCameraProjection] world(lidar)->camera extrinsic: t=(%.4f, %.4f, %.4f) rpy_deg=(%.4f, %.4f, %.4f)",
                 lidar_to_camera_t_.x(), lidar_to_camera_t_.y(), lidar_to_camera_t_.z(),
                 lidar_to_camera_rpy_deg_.x(), lidar_to_camera_rpy_deg_.y(), lidar_to_camera_rpy_deg_.z());
    }

    void loadIntrinsicsFromYaml(const std::string& path)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("failed to open calib file: " + path);
        }

        std::string model_str = "PINHOLE";
        fs["camera_model"] >> model_str;
        if (model_str.empty()) {
            fs["model_type"] >> model_str;
        }

        fs["image_width"] >> image_width_;
        fs["image_height"] >> image_height_;
        if (image_width_ <= 0 || image_height_ <= 0) {
            throw std::runtime_error("invalid image size in calib file: " + path);
        }

        epsilon_ = 0.0;
        cv::FileNode mirror_node = fs["mirror_parameters"];
        if (!mirror_node.empty()) {
            mirror_node["xi"] >> epsilon_;
        } else {
            fs["xi"] >> epsilon_;
        }

        K_ = cv::Mat::eye(3, 3, CV_64F);
        cv::FileNode projection_node = fs["projection_parameters"];
        if (!projection_node.empty()) {
            projection_node["gamma1"] >> K_.at<double>(0, 0);
            projection_node["gamma2"] >> K_.at<double>(1, 1);
            projection_node["u0"] >> K_.at<double>(0, 2);
            projection_node["v0"] >> K_.at<double>(1, 2);
        } else {
            double fx = 0.0;
            double fy = 0.0;
            double cx = 0.0;
            double cy = 0.0;
            fs["fx"] >> fx;
            fs["fy"] >> fy;
            fs["cx"] >> cx;
            fs["cy"] >> cy;
            K_.at<double>(0, 0) = fx;
            K_.at<double>(1, 1) = fy;
            K_.at<double>(0, 2) = cx;
            K_.at<double>(1, 2) = cy;
        }

        D_ = cv::Mat::zeros(1, 4, CV_64F);
        cv::FileNode distortion_node = fs["distortion_parameters"];
        if (!distortion_node.empty()) {
            distortion_node["k1"] >> D_.at<double>(0, 0);
            distortion_node["k2"] >> D_.at<double>(0, 1);
            distortion_node["p1"] >> D_.at<double>(0, 2);
            distortion_node["p2"] >> D_.at<double>(0, 3);
        } else {
            fs["k1"] >> D_.at<double>(0, 0);
            fs["k2"] >> D_.at<double>(0, 1);
            fs["p1"] >> D_.at<double>(0, 2);
            fs["p2"] >> D_.at<double>(0, 3);
        }

        if (model_str != "MEI") {
            epsilon_ = 0.0;
        }
    }

    void setupPubSub()
    {
        if (endsWith(image_topic_, "/compressed")) {
            sub_compressed_image_ = nh_.subscribe(image_topic_,
                                                  1,
                                                  &LidarCameraProjectionNode::onCompressedImage,
                                                  this);
            ROS_INFO("[LidarCameraProjection] subscribe compressed image: %s", image_topic_.c_str());
        } else {
            sub_raw_image_ = nh_.subscribe(image_topic_,
                                           1,
                                           &LidarCameraProjectionNode::onRawImage,
                                           this);
            ROS_INFO("[LidarCameraProjection] subscribe raw image: %s", image_topic_.c_str());
        }

        sub_lidar_ = nh_.subscribe(lidar_topic_, 1, &LidarCameraProjectionNode::onLidarCloud, this);
        pub_overlay_ = it_.advertise(output_topic_, 1);
        srv_save_ = nh_.advertiseService("save", &LidarCameraProjectionNode::srvSave, this);
        ROS_INFO("[LidarCameraProjection] subscribe lidar: %s, publish overlay: %s",
                 lidar_topic_.c_str(), output_topic_.c_str());
    }

    void onRawImage(const sensor_msgs::Image::ConstPtr& msg)
    {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            std::lock_guard<std::mutex> lock(mutex_);
            latest_image_ = cv_ptr->image.clone();
            latest_image_stamp_ = msg->header.stamp;
            has_image_ = !latest_image_.empty();
        } catch (const cv_bridge::Exception& ex) {
            ROS_WARN_THROTTLE(1.0, "[LidarCameraProjection] raw image convert failed: %s", ex.what());
        }
    }

    void onCompressedImage(const sensor_msgs::CompressedImage::ConstPtr& msg)
    {
        try {
            cv::Mat encoded(1,
                            static_cast<int>(msg->data.size()),
                            CV_8UC1,
                            const_cast<unsigned char*>(msg->data.data()));
            cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
            if (image.empty()) {
                ROS_WARN_THROTTLE(1.0, "[LidarCameraProjection] compressed image decode failed");
                return;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            latest_image_ = image;
            latest_image_stamp_ = msg->header.stamp;
            has_image_ = true;
        } catch (const cv::Exception& ex) {
            ROS_WARN_THROTTLE(1.0, "[LidarCameraProjection] compressed image exception: %s", ex.what());
        }
    }

    void onLidarCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_cloud_ = msg;
    }

    void onParamTimer(const ros::TimerEvent&)
    {
        double x, y, z, roll, pitch, yaw;
        pnh_.param<double>("x", x, tune_x_);
        pnh_.param<double>("y", y, tune_y_);
        pnh_.param<double>("z", z, tune_z_);
        pnh_.param<double>("roll", roll, tune_roll_);
        pnh_.param<double>("pitch", pitch, tune_pitch_);
        pnh_.param<double>("yaw", yaw, tune_yaw_);

        const bool changed =
            x != tune_x_ || y != tune_y_ || z != tune_z_ ||
            roll != tune_roll_ || pitch != tune_pitch_ || yaw != tune_yaw_;
        if (!changed) return;

        tune_x_ = x; tune_y_ = y; tune_z_ = z;
        tune_roll_ = roll; tune_pitch_ = pitch; tune_yaw_ = yaw;

        // 与 loadParams 保持同一约定：t=[x,y,z], rpy_deg=[pitch, yaw, -roll]
        lidar_to_camera_t_ = Eigen::Vector3d(x, y, z);
        lidar_to_camera_rpy_deg_ = Eigen::Vector3d(pitch, yaw, -roll);
        mei_->setExtrinsics(lidar_to_camera_t_, toRadians(lidar_to_camera_rpy_deg_));
        ROS_INFO("[LidarCameraProjection] extrinsic updated: t=(%.4f, %.4f, %.4f) rpy_deg(pitch,yaw,-roll)=(%.4f, %.4f, %.4f)",
                 x, y, z, pitch, yaw, -roll);
    }

    void onTimer(const ros::TimerEvent&)
    {
        cv::Mat image;
        sensor_msgs::PointCloud2::ConstPtr cloud;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_image_ || !latest_cloud_) {
                return;
            }
            image = latest_image_.clone();
            cloud = latest_cloud_;
        }

        cv::Mat overlay = renderOverlay(image, cloud);
        if (overlay.empty()) {
            return;
        }

        pub_overlay_.publish(cv_bridge::CvImage(cloud->header, "bgr8", overlay).toImageMsg());
    }

    cv::Mat renderOverlay(const cv::Mat& raw_image,
                          const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        if (raw_image.empty() || !cloud_msg) {
            return cv::Mat();
        }

        cv::Size display_size = raw_image.size();
        if (raw_image.cols != display_width_) {
            const double scale = static_cast<double>(display_width_) /
                                 static_cast<double>(std::max(1, raw_image.cols));
            display_size = cv::Size(display_width_, cvRound(raw_image.rows * scale));
        }

        cv::Mat base_image;
        if (raw_image.size() == display_size) {
            base_image = raw_image.clone();
        } else {
            cv::resize(raw_image, base_image, display_size, 0, 0, cv::INTER_AREA);
        }

        cv::Mat overlay(display_size, base_image.type(), cv::Scalar::all(0));
        const double scale_x = static_cast<double>(display_size.width) /
                               static_cast<double>(std::max(1, image_width_));
        const double scale_y = static_cast<double>(display_size.height) /
                               static_cast<double>(std::max(1, image_height_));

        std::vector<ProjectedPoint> projected_points;
        projected_points.reserve(static_cast<size_t>(cloud_msg->width) * cloud_msg->height);

        std::vector<float> zbuffer(static_cast<size_t>(display_size.width) *
                                   static_cast<size_t>(display_size.height),
                                   std::numeric_limits<float>::max());

        const float kOcclusionDepthMarginMin = 0.2f;
        const float kOcclusionDepthMarginRatio = 0.03f;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const Eigen::Vector3d point_world(*iter_x, *iter_y, *iter_z);
            if (!isFinitePoint(point_world)) {
                continue;
            }

            Eigen::Vector3d point_camera;
            if (!mei_->world3dToCamera3d(point_world, point_camera)) {
                continue;
            }

            const float depth = static_cast<float>(point_camera.z());
            if (depth <= static_cast<float>(min_depth_m_) || depth >= static_cast<float>(max_depth_m_)) {
                continue;
            }

            cv::Point2d uv_raw;
            if (!mei_->projectWorld3dTo2d(point_world, uv_raw)) {
                continue;
            }

            const cv::Point2f uv_display(static_cast<float>(uv_raw.x * scale_x),
                                         static_cast<float>(uv_raw.y * scale_y));
            const int pixel_x = cvRound(uv_display.x);
            const int pixel_y = cvRound(uv_display.y);
            if (pixel_x < 0 || pixel_x >= overlay.cols || pixel_y < 0 || pixel_y >= overlay.rows) {
                continue;
            }

            const int pixel_index = pixel_y * overlay.cols + pixel_x;
            if (depth < zbuffer[static_cast<size_t>(pixel_index)]) {
                zbuffer[static_cast<size_t>(pixel_index)] = depth;
            }

            ProjectedPoint projected_point;
            projected_point.uv = uv_display;
            projected_point.depth = depth;
            projected_point.color = distanceToRainbowBgr(depth);
            projected_point.pixel_x = pixel_x;
            projected_point.pixel_y = pixel_y;
            projected_point.pixel_index = pixel_index;
            projected_points.push_back(projected_point);
        }

        for (size_t i = 0; i < projected_points.size(); ++i) {
            const ProjectedPoint& projected_point = projected_points[i];
            float neighborhood_min_depth = zbuffer[static_cast<size_t>(projected_point.pixel_index)];
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int neighbor_x = projected_point.pixel_x + dx;
                    const int neighbor_y = projected_point.pixel_y + dy;
                    if (neighbor_x < 0 || neighbor_x >= overlay.cols ||
                        neighbor_y < 0 || neighbor_y >= overlay.rows) {
                        continue;
                    }

                    const int neighbor_index = neighbor_y * overlay.cols + neighbor_x;
                    const float neighbor_depth = zbuffer[static_cast<size_t>(neighbor_index)];
                    if (neighbor_depth < neighborhood_min_depth) {
                        neighborhood_min_depth = neighbor_depth;
                    }
                }
            }

            const float depth_margin = std::max(kOcclusionDepthMarginMin,
                                                kOcclusionDepthMarginRatio * neighborhood_min_depth);
            if (projected_point.depth <= neighborhood_min_depth + depth_margin) {
                cv::circle(overlay, projected_point.uv, 1, projected_point.color, 1, cv::LINE_AA);
            }
        }

        cv::Mat result;
        cv::addWeighted(base_image, 0.8, overlay, 0.3, 0.0, result, -1);
        return result;
    }

    bool srvSave(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
    {
        ensureDir(output_dir_);
        std::string dir = output_dir_;
        if (!dir.empty() && dir.back() != '/') dir += '/';

        std::time_t t = std::time(nullptr);
        char tbuf[64];
        std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        std::ofstream ofs(dir + "calib_params.txt");
        if (!ofs.is_open()) {
            res.success = false;
            res.message = "Failed to open " + dir + "calib_params.txt";
            return true;
        }

        ofs << std::fixed << std::setprecision(10);
        ofs << "# LiDAR-Camera Projection Parameters\n";
        ofs << "# Generated: " << tbuf << "\n";
        ofs << "# Calib file: " << calib_file_ << "\n\n";
        ofs << "image_width: " << image_width_ << "\n";
        ofs << "image_height: " << image_height_ << "\n";
        ofs << "epsilon: " << epsilon_ << "\n";
        ofs << "K: [" << K_.at<double>(0,0) << ", " << K_.at<double>(0,1) << ", " << K_.at<double>(0,2) << ", "
                       << K_.at<double>(1,0) << ", " << K_.at<double>(1,1) << ", " << K_.at<double>(1,2) << ", "
                       << K_.at<double>(2,0) << ", " << K_.at<double>(2,1) << ", " << K_.at<double>(2,2) << "]\n";
        ofs << "D: [" << D_.at<double>(0,0) << ", " << D_.at<double>(0,1) << ", "
                       << D_.at<double>(0,2) << ", " << D_.at<double>(0,3) << "]\n\n";
        ofs << "extrinsic:\n";
        ofs << "  x: " << lidar_to_camera_t_.x() << "\n";
        ofs << "  y: " << lidar_to_camera_t_.y() << "\n";
        ofs << "  z: " << lidar_to_camera_t_.z() << "\n";
        ofs << "  roll: " << lidar_to_camera_rpy_deg_.x() << "\n";
        ofs << "  pitch: " << lidar_to_camera_rpy_deg_.y() << "\n";
        ofs << "  yaw: " << lidar_to_camera_rpy_deg_.z() << "\n";
        ofs.close();
        ROS_INFO("[LidarCameraProjection] Saved params to %scalib_params.txt", dir.c_str());

        std::string yaml_path = yaml_path_.empty() ? (dir + "camera_params.yaml") : yaml_path_;
        std::ofstream yfs(yaml_path);
        if (!yfs.is_open()) {
            res.success = false;
            res.message = "Failed to open " + yaml_path;
            return true;
        }

        yfs << std::fixed << std::setprecision(10);
        yfs << "image_width: " << image_width_ << "\n";
        yfs << "image_height: " << image_height_ << "\n";
        yfs << "epsilon: " << epsilon_ << "\n";
        yfs << "K: [" << K_.at<double>(0,0) << ", " << K_.at<double>(0,1) << ", " << K_.at<double>(0,2) << ", "
                       << K_.at<double>(1,0) << ", " << K_.at<double>(1,1) << ", " << K_.at<double>(1,2) << ", "
                       << K_.at<double>(2,0) << ", " << K_.at<double>(2,1) << ", " << K_.at<double>(2,2) << "]\n";
        yfs << "D: [" << D_.at<double>(0,0) << ", " << D_.at<double>(0,1) << ", "
                       << D_.at<double>(0,2) << ", " << D_.at<double>(0,3) << "]\n";
        yfs << "x: " << lidar_to_camera_t_.x() << "\n";
        yfs << "y: " << lidar_to_camera_t_.y() << "\n";
        yfs << "z: " << lidar_to_camera_t_.z() << "\n";
        yfs << "roll: " << lidar_to_camera_rpy_deg_.x() << "\n";
        yfs << "pitch: " << lidar_to_camera_rpy_deg_.y() << "\n";
        yfs << "yaw: " << lidar_to_camera_rpy_deg_.z() << "\n";
        yfs.close();
        ROS_INFO("[LidarCameraProjection] Saved params to %s", yaml_path.c_str());

        res.success = true;
        res.message = "Saved to " + dir;
        return true;
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    std::unique_ptr<camera::Mei> mei_;

    std::string calib_file_;
    std::string image_topic_;
    std::string lidar_topic_;
    std::string output_topic_;
    double publish_rate_hz_ = 10.0;
    double sync_tolerance_sec_ = 0.10;
    double min_depth_m_ = 0.3;
    double max_depth_m_ = 30.0;
    int display_width_ = 960;

    int image_width_ = 0;
    int image_height_ = 0;
    double epsilon_ = 0.0;
    cv::Mat K_;
    cv::Mat D_;

    std::array<int, 3> rotation_order_ = {{2, 1, 0}};
    Eigen::Vector3d lidar_to_camera_t_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lidar_to_camera_rpy_deg_ = Eigen::Vector3d::Zero();

    // 原始 6DOF 微调值，用于动态调参的变化检测
    double tune_x_ = 0.0, tune_y_ = 0.0, tune_z_ = 0.0;
    double tune_roll_ = 0.0, tune_pitch_ = 0.0, tune_yaw_ = 0.0;

    std::mutex mutex_;
    cv::Mat latest_image_;
    ros::Time latest_image_stamp_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
    bool has_image_ = false;

    std::string output_dir_;
    std::string yaml_path_;

    ros::Subscriber sub_raw_image_;
    ros::Subscriber sub_compressed_image_;
    ros::Subscriber sub_lidar_;
    image_transport::Publisher pub_overlay_;
    ros::Timer timer_;
    ros::Timer param_timer_;
    ros::ServiceServer srv_save_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_camera_projection_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        LidarCameraProjectionNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& ex) {
        ROS_FATAL("[lidar_camera_projection_node] fatal: %s", ex.what());
        return 1;
    }
    return 0;
}
