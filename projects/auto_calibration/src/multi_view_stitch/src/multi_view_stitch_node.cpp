#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <yaml-cpp/yaml.h>

#include "camera_model/mei.h"

namespace multi_view_stitch {
namespace {

struct CameraConfig {
    std::string logical_role;
    std::string camera_name;
    std::string yaml_path;
    std::string image_topic;
    int width = 0;
    int height = 0;
    double epsilon = 0.0;
    double vcs_z_offset = 0.0;
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(1, 4, CV_64F);
    std::vector<double> t_values;
    std::vector<double> r_euler_deg;
    camera::Mei model;

    explicit CameraConfig(const std::array<int, 3>& rotation_order = {{2, 1, 0}})
        : model(rotation_order)
    {}
};

struct CameraFrame {
    bool has_image = false;
    ros::Time stamp;
    cv::Mat image;
};

struct Batch {
    bool valid = false;
    CameraFrame left;
    CameraFrame main;
    CameraFrame right;
};

struct ProjectionGeometryCache {
    bool valid = false;
    double align_depth_z = 0.0;
    int left_projection_boundary_x = 0;
    int right_projection_boundary_x = 0;
    int left_projection_anchor_y = -1;
    int right_projection_anchor_y = -1;
    cv::Size left_image_size;
    cv::Size main_image_size;
    cv::Size right_image_size;
    cv::Point2d main_left_edge;
    cv::Point2d main_right_edge;
    Eigen::Vector3d front_left_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d front_right_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d left_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d left_in_left_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_in_right_camera = Eigen::Vector3d::Zero();
    cv::Point2d left_projection;
    cv::Point2d right_projection;
};

Eigen::Vector3d toEigenTranslation(const std::vector<double>& values, double y_offset)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    if (values.size() >= 1) {
        result.x() = values[0];
    }
    if (values.size() >= 2) {
        result.y() = values[1] + y_offset;
    } else {
        result.y() = y_offset;
    }
    if (values.size() >= 3) {
        result.z() = values[2];
    }
    return result;
}

Eigen::Vector3d toEigenEulerRadians(const std::vector<double>& values_deg)
{
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    if (values_deg.size() >= 1) {
        result.x() = values_deg[0] * M_PI / 180.0;
    }
    if (values_deg.size() >= 2) {
        result.y() = values_deg[1] * M_PI / 180.0;
    }
    if (values_deg.size() >= 3) {
        result.z() = values_deg[2] * M_PI / 180.0;
    }
    return result;
}

bool isFinitePoint(const cv::Point2d& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool isInsideImage(const cv::Point2d& point, int width, int height)
{
    return isFinitePoint(point) &&
           point.x >= 0.0 && point.x <= static_cast<double>(width - 1) &&
           point.y >= 0.0 && point.y <= static_cast<double>(height - 1);
}

std::string pointToString(const cv::Point2d& point)
{
    std::ostringstream oss;
    oss << "(" << point.x << ", " << point.y << ")";
    return oss.str();
}

std::string vectorToString(const Eigen::Vector3d& point)
{
    std::ostringstream oss;
    oss << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")";
    return oss.str();
}

std::string boolToString(bool value)
{
    return value ? "true" : "false";
}

bool endsWith(const std::string& value, const std::string& suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void drawAnchorMarker(cv::Mat& image, const cv::Point& point, const cv::Scalar& color)
{
    if (image.empty()) {
        return;
    }
    if (point.x < 0 || point.x >= image.cols || point.y < 0 || point.y >= image.rows) {
        return;
    }

    cv::circle(image, point, 8, color, 2, cv::LINE_AA);
    cv::line(image, cv::Point(point.x - 12, point.y), cv::Point(point.x + 12, point.y), color, 2, cv::LINE_AA);
    cv::line(image, cv::Point(point.x, point.y - 12), cv::Point(point.x, point.y + 12), color, 2, cv::LINE_AA);
}

}  // namespace

class MultiViewStitchNode {
public:
    MultiViewStitchNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh),
          pnh_(pnh),
          it_(nh)
    {
        loadParams();
        loadCameraConfigs();
        setupIo();
        process_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_max_hz_),
                                         &MultiViewStitchNode::onProcessTimer,
                                         this);
        ROS_INFO("[MultiViewStitchNode] init OK");
    }

private:
    void loadParams()
    {
        int rot0 = 2;
        int rot1 = 1;
        int rot2 = 0;
        pnh_.param<int>("rotation_order_0", rot0, 2);
        pnh_.param<int>("rotation_order_1", rot1, 1);
        pnh_.param<int>("rotation_order_2", rot2, 0);
        rotation_order_[0] = rot0;
        rotation_order_[1] = rot1;
        rotation_order_[2] = rot2;

        pnh_.param<double>("align_depth_z", default_align_depth_z_, 3.0);
        pnh_.param<double>("publish_max_hz", publish_max_hz_, 10.0);
        pnh_.param<double>("sync_tolerance_sec", sync_tolerance_sec_, 0.10);
        pnh_.param<bool>("log_debug", log_debug_, true);
        pnh_.param<bool>("crop_side_width", default_crop_side_width_, true);
        pnh_.param<bool>("crop_main_width", default_crop_main_width_, false);
        pnh_.param<int>("anchor/left/x", default_left_projection_boundary_x_, 0);
        pnh_.param<int>("anchor/right/x", default_right_projection_boundary_x_, 0);
        pnh_.param<int>("anchor/left/y", default_left_projection_anchor_y_, -1);
        pnh_.param<int>("anchor/right/y", default_right_projection_anchor_y_, -1);
        pnh_.param<int>("top_alignment_boundary_y", default_top_alignment_boundary_y_, 0);
        pnh_.param<int>("bottom_alignment_boundary_y", default_bottom_alignment_boundary_y_, 0);
        pnh_.param<std::string>("stitched_topic", stitched_topic_, "/multi_view_stitch/stitched_image");
        pnh_.param<std::string>("status_topic", status_topic_, "/multi_view_stitch/status");
        pnh_.param<std::string>("save_params_yaml_path", save_params_yaml_path_, "");

        main_camera_ = CameraConfig(rotation_order_);
        left_camera_ = CameraConfig(rotation_order_);
        right_camera_ = CameraConfig(rotation_order_);

        main_camera_.logical_role = "main";
        left_camera_.logical_role = "left";
        right_camera_.logical_role = "right";

        pnh_.param<std::string>("main_camera_yaml_path", main_camera_.yaml_path, "");
        pnh_.param<std::string>("left_camera_yaml_path", left_camera_.yaml_path, "");
        pnh_.param<std::string>("right_camera_yaml_path", right_camera_.yaml_path, "");
        pnh_.param<std::string>("main_image_topic", main_camera_.image_topic, "/driver/gmsl_camera/front_fish_eye/compressed");
        pnh_.param<std::string>("left_image_topic", left_camera_.image_topic, "/driver/gmsl_camera/left_fish_eye/compressed");
        pnh_.param<std::string>("right_image_topic", right_camera_.image_topic, "/driver/gmsl_camera/right_fish_eye/compressed");

        if (publish_max_hz_ <= 0.0) {
            throw std::runtime_error("publish_max_hz must be > 0");
        }
        if (sync_tolerance_sec_ < 0.0) {
            throw std::runtime_error("sync_tolerance_sec must be >= 0");
        }
    }

    void loadCameraConfigs()
    {
        std::string error;
        if (!loadBevCameraFromYaml(main_camera_, error)) {
            throw std::runtime_error("main camera load failed: " + error);
        }
        if (!loadBevCameraFromYaml(left_camera_, error)) {
            throw std::runtime_error("left camera load failed: " + error);
        }
        if (!loadBevCameraFromYaml(right_camera_, error)) {
            throw std::runtime_error("right camera load failed: " + error);
        }

        ROS_INFO("[MultiViewStitchNode] camera yaml loaded: main=%s left=%s right=%s",
                 main_camera_.camera_name.c_str(),
                 left_camera_.camera_name.c_str(),
                 right_camera_.camera_name.c_str());
    }

    bool loadBevCameraFromYaml(CameraConfig& camera, std::string& error)
    {
        if (camera.yaml_path.empty()) {
            error = camera.logical_role + " camera yaml path is empty";
            return false;
        }

        YAML::Node root;
        try {
            root = YAML::LoadFile(camera.yaml_path);
        } catch (const std::exception& ex) {
            error = "failed to load yaml " + camera.yaml_path + ": " + ex.what();
            return false;
        }

        YAML::Node node = root;
        if (!root["K"] && root.IsMap() && root.size() == 1) {
            YAML::const_iterator it = root.begin();
            camera.camera_name = it->first.as<std::string>();
            node = it->second;
        }

        if (!node || !node.IsMap()) {
            error = "yaml node is invalid: " + camera.yaml_path;
            return false;
        }

        if (camera.camera_name.empty()) {
            camera.camera_name = node["camera_name"] ? node["camera_name"].as<std::string>() : "";
        }
        if (camera.camera_name.empty()) {
            camera.camera_name = camera.logical_role;
        }

        const YAML::Node size = node["Size"];
        const YAML::Node k = node["K"];
        const YAML::Node d = node["D"];
        const YAML::Node t = node["t"];
        const YAML::Node r = node["R_euler"];
        if (!size || !k || !d || !t || !r || !node["Epsilon"]) {
            error = "yaml missing Size/K/D/t/R_euler/Epsilon: " + camera.yaml_path;
            return false;
        }
        if (!size.IsSequence() || size.size() < 2 || !k.IsSequence() || k.size() < 9 ||
            !d.IsSequence() || d.size() < 4 || !t.IsSequence() || t.size() < 3 ||
            !r.IsSequence() || r.size() < 3) {
            error = "yaml field size mismatch: " + camera.yaml_path;
            return false;
        }

        camera.width = size[0].as<int>();
        camera.height = size[1].as<int>();
        camera.epsilon = node["Epsilon"].as<double>();
        camera.vcs_z_offset = node["vcs_z_offset"] ? node["vcs_z_offset"].as<double>() :
                              (node["vcs_to_ground_height"] ? node["vcs_to_ground_height"].as<double>() : 0.0);
        camera.t_values.assign(3, 0.0);
        camera.r_euler_deg.assign(3, 0.0);
        for (size_t i = 0; i < 3; ++i) {
            camera.t_values[i] = t[i].as<double>();
            camera.r_euler_deg[i] = r[i].as<double>();
        }

        camera.K = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) {
            camera.K.at<double>(i / 3, i % 3) = k[i].as<double>();
        }

        camera.D = cv::Mat::zeros(1, 4, CV_64F);
        for (int i = 0; i < 4; ++i) {
            camera.D.at<double>(0, i) = d[i].as<double>();
        }

        camera.model = camera::Mei(rotation_order_);
        camera.model.setIntrinsics(camera.epsilon,
                                   camera.K,
                                   camera.D,
                                   camera.width,
                                   camera.height);
        camera.model.setGroundHeight(camera.vcs_z_offset);
        camera.model.setExtrinsics(toEigenTranslation(camera.t_values, camera.vcs_z_offset),
                                   toEigenEulerRadians(camera.r_euler_deg));
        error.clear();
        return true;
    }

    void setupIo()
    {
        subscribeCamera(left_camera_.image_topic, sub_left_raw_, sub_left_compressed_,
                        &MultiViewStitchNode::onLeftImage, &MultiViewStitchNode::onLeftCompressedImage);
        subscribeCamera(main_camera_.image_topic, sub_main_raw_, sub_main_compressed_,
                        &MultiViewStitchNode::onMainImage, &MultiViewStitchNode::onMainCompressedImage);
        subscribeCamera(right_camera_.image_topic, sub_right_raw_, sub_right_compressed_,
                        &MultiViewStitchNode::onRightImage, &MultiViewStitchNode::onRightCompressedImage);

        pub_stitched_ = it_.advertise(stitched_topic_, 1);
        pub_status_ = nh_.advertise<std_msgs::String>(status_topic_, 10);

        srv_save_params_ = pnh_.advertiseService("save_camera_params", &MultiViewStitchNode::onSaveCameraParams, this);
        ROS_INFO("[MultiViewStitchNode] service registered: save_camera_params");
    }

    bool onSaveCameraParams(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        if (!projection_geometry_cache_.valid) {
            ROS_WARN("[MultiViewStitchNode] projection geometry not computed yet, cannot save");
            return true;
        }

        std::string path = save_params_yaml_path_;
        pnh_.getParam("save_params_yaml_path", path);
        if (path.empty()) {
            ROS_WARN("[MultiViewStitchNode] save_params_yaml_path is empty, cannot save");
            return true;
        }

        const cv::Point2d& lp = projection_geometry_cache_.left_projection;
        const cv::Point2d& rp = projection_geometry_cache_.right_projection;
        const cv::Point2d& ml = projection_geometry_cache_.main_left_edge;
        const cv::Point2d& mr = projection_geometry_cache_.main_right_edge;

        try {
            YAML::Node root;
            root["left_front"]["projection_x"] = lp.x;
            root["left_front"]["projection_y"] = lp.y;
            root["right_front"]["projection_x"] = rp.x;
            root["right_front"]["projection_y"] = rp.y;
            root["front_h100"]["left_edge_x"] = ml.x;
            root["front_h100"]["left_edge_y"] = ml.y;
            root["front_h100"]["right_edge_x"] = mr.x;
            root["front_h100"]["right_edge_y"] = mr.y;

            std::ofstream ofs(path);
            if (!ofs.is_open()) {
                ROS_ERROR("[MultiViewStitchNode] failed to open %s for writing", path.c_str());
                return true;
            }
            ofs << root;
            ofs.close();
            ROS_INFO("[MultiViewStitchNode] save_camera_params -> %s", path.c_str());
        } catch (const std::exception& ex) {
            ROS_ERROR("[MultiViewStitchNode] save_camera_params failed: %s", ex.what());
        }
        return true;
    }

    void subscribeCamera(const std::string& topic,
                         image_transport::Subscriber& raw_subscriber,
                         ros::Subscriber& compressed_subscriber,
                         void (MultiViewStitchNode::*raw_cb)(const sensor_msgs::ImageConstPtr&),
                         void (MultiViewStitchNode::*compressed_cb)(const sensor_msgs::CompressedImageConstPtr&))
    {
        if (endsWith(topic, "/compressed")) {
            compressed_subscriber = nh_.subscribe(topic, 1, compressed_cb, this);
            ROS_INFO("[MultiViewStitchNode] subscribe compressed topic: %s", topic.c_str());
            return;
        }

        raw_subscriber = it_.subscribe(topic, 1, raw_cb, this);
        ROS_INFO("[MultiViewStitchNode] subscribe raw topic: %s", topic.c_str());
    }

    void onLeftImage(const sensor_msgs::ImageConstPtr& msg)
    {
        updateFrame(msg, left_frame_);
    }

    void onMainImage(const sensor_msgs::ImageConstPtr& msg)
    {
        updateFrame(msg, main_frame_);
    }

    void onRightImage(const sensor_msgs::ImageConstPtr& msg)
    {
        updateFrame(msg, right_frame_);
    }

    void onLeftCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        updateCompressedFrame(msg, left_frame_);
    }

    void onMainCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        updateCompressedFrame(msg, main_frame_);
    }

    void onRightCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        updateCompressedFrame(msg, right_frame_);
    }

    void updateFrame(const sensor_msgs::ImageConstPtr& msg, CameraFrame& frame)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& ex) {
            ROS_WARN_THROTTLE(1.0, "[MultiViewStitchNode] cv_bridge convert failed: %s", ex.what());
            return;
        }

        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame.has_image = true;
        frame.stamp = msg->header.stamp;
        frame.image = cv_ptr->image.clone();
    }

    void updateCompressedFrame(const sensor_msgs::CompressedImageConstPtr& msg, CameraFrame& frame)
    {
        try {
            const cv::Mat buffer(1, static_cast<int>(msg->data.size()), CV_8UC1,
                                 const_cast<unsigned char*>(msg->data.data()));
            cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_COLOR);
            if (decoded.empty()) {
                ROS_WARN_THROTTLE(1.0, "[MultiViewStitchNode] compressed image decode failed: %s",
                                  msg->header.frame_id.c_str());
                return;
            }

            std::lock_guard<std::mutex> lock(frame_mutex_);
            frame.has_image = true;
            frame.stamp = msg->header.stamp;
            frame.image = decoded;
        } catch (const cv::Exception& ex) {
            ROS_WARN_THROTTLE(1.0, "[MultiViewStitchNode] compressed image decode exception: %s", ex.what());
        }
    }

    Batch getLatestBatch() const
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        Batch batch;
        batch.left = left_frame_;
        batch.main = main_frame_;
        batch.right = right_frame_;
        batch.valid = batch.left.has_image && batch.main.has_image && batch.right.has_image;
        return batch;
    }

    void onProcessTimer(const ros::TimerEvent&)
    {
        const Batch batch = getLatestBatch();
        if (!batch.valid) {
            return;
        }

        const ros::Time publish_stamp = latestStamp(batch);
        const double sync_span = syncSpanSeconds(batch);
        if (sync_span > sync_tolerance_sec_) {
            publishFailure(publish_stamp, "image timestamp spread exceeds sync_tolerance_sec");
            return;
        }

        double align_depth_z = default_align_depth_z_;
        pnh_.getParam("align_depth_z", align_depth_z);
        bool crop_side_width = default_crop_side_width_;
        bool crop_main_width = default_crop_main_width_;
        pnh_.getParam("crop_side_width", crop_side_width);
        pnh_.getParam("crop_main_width", crop_main_width);
        int left_projection_boundary_x = default_left_projection_boundary_x_;
        int right_projection_boundary_x = default_right_projection_boundary_x_;
        int left_projection_anchor_y = default_left_projection_anchor_y_;
        int right_projection_anchor_y = default_right_projection_anchor_y_;
        int top_alignment_boundary_y = default_top_alignment_boundary_y_;
        int bottom_alignment_boundary_y = default_bottom_alignment_boundary_y_;
        pnh_.getParam("anchor/left/x", left_projection_boundary_x);
        pnh_.getParam("anchor/right/x", right_projection_boundary_x);
        pnh_.getParam("anchor/left/y", left_projection_anchor_y);
        pnh_.getParam("anchor/right/y", right_projection_anchor_y);
        pnh_.getParam("top_alignment_boundary_y", top_alignment_boundary_y);
        pnh_.getParam("bottom_alignment_boundary_y", bottom_alignment_boundary_y);

        cv::Mat stitched;
        std::string detail;
        if (!stitch(batch.left.image,
                    batch.main.image,
                    batch.right.image,
                    align_depth_z,
                    crop_side_width,
                    crop_main_width,
                    left_projection_boundary_x,
                    right_projection_boundary_x,
                    left_projection_anchor_y,
                    right_projection_anchor_y,
                    top_alignment_boundary_y,
                    bottom_alignment_boundary_y,
                    stitched,
                    detail)) {
            publishFailure(publish_stamp, detail);
            return;
        }

        publishSuccess(publish_stamp, stitched, detail);
    }

    bool stitch(const cv::Mat& left_image,
                const cv::Mat& main_image,
                const cv::Mat& right_image,
                double align_depth_z,
                bool crop_side_width,
                bool crop_main_width,
                int left_projection_boundary_x,
                int right_projection_boundary_x,
                int left_projection_anchor_y,
                int right_projection_anchor_y,
                int top_alignment_boundary_y,
                int bottom_alignment_boundary_y,
                cv::Mat& stitched_image,
                std::string& detail)
    {
        stitched_image.release();

        if (align_depth_z <= 0.0) {
            detail = "align_depth_z <= 0";
            return false;
        }
        if (left_image.empty() || main_image.empty() || right_image.empty()) {
            detail = "input image is empty";
            return false;
        }

        ProjectionGeometryCache geometry;
        std::string geometry_error;
        if (!resolveProjectionGeometry(left_image.size(),
                                       main_image.size(),
                                       right_image.size(),
                                       align_depth_z,
                                       left_projection_boundary_x,
                                       right_projection_boundary_x,
                                       left_projection_anchor_y,
                                       right_projection_anchor_y,
                                       geometry,
                                       geometry_error)) {
            detail = geometry_error;
            return false;
        }
        if (!isInsideImage(geometry.left_projection, left_image.cols, left_image.rows)) {
            detail = "left projection out of image: " + pointToString(geometry.left_projection);
            return false;
        }
        if (!isInsideImage(geometry.right_projection, right_image.cols, right_image.rows)) {
            detail = "right projection out of image: " + pointToString(geometry.right_projection);
            return false;
        }

        const int left_keep_width = static_cast<int>(std::ceil(geometry.left_projection.x));
        const int right_keep_start = static_cast<int>(std::floor(geometry.right_projection.x)) + 1;
        const int right_keep_width = right_image.cols - right_keep_start;
        if (crop_side_width && left_keep_width <= 0) {
            detail = "left crop width <= 0";
            return false;
        }
        if (crop_side_width && right_keep_width <= 0) {
            detail = "right crop width <= 0";
            return false;
        }

        const cv::Rect left_source_roi = crop_side_width
            ? cv::Rect(0, 0, left_keep_width, left_image.rows)
            : cv::Rect(0, 0, left_image.cols, left_image.rows);
        const cv::Rect right_source_roi = crop_side_width
            ? cv::Rect(right_keep_start, 0, right_keep_width, right_image.rows)
            : cv::Rect(0, 0, right_image.cols, right_image.rows);
        const cv::Rect main_source_roi = crop_main_width
            ? cv::Rect(geometry.left_projection_boundary_x,
                       0,
                       geometry.right_projection_boundary_x - geometry.left_projection_boundary_x + 1,
                       main_image.rows)
            : cv::Rect(0, 0, main_image.cols, main_image.rows);
        if (!isValidRoi(left_source_roi, left_image.size()) ||
            !isValidRoi(main_source_roi, main_image.size()) ||
            !isValidRoi(right_source_roi, right_image.size())) {
            detail = "computed source roi is invalid";
            return false;
        }

        const int left_main_anchor_y = clampToImageRow(static_cast<int>(std::lround(geometry.main_left_edge.y)), main_image.rows);
        const int right_main_anchor_y = clampToImageRow(static_cast<int>(std::lround(geometry.main_right_edge.y)), main_image.rows);
        const int left_anchor_y = clampToImageRow(static_cast<int>(std::lround(geometry.left_projection.y)), left_image.rows);
        const int right_anchor_y = clampToImageRow(static_cast<int>(std::lround(geometry.right_projection.y)), right_image.rows);
        const int resolved_top_alignment_boundary_y = resolveTopAlignmentBoundaryY(top_alignment_boundary_y,
                                                                                   main_image.rows);
        const int resolved_bottom_alignment_boundary_y = resolveBottomAlignmentBoundaryY(bottom_alignment_boundary_y,
                                                                                         main_image.rows);
        if (resolved_top_alignment_boundary_y < 0 || resolved_top_alignment_boundary_y >= main_image.rows) {
            detail = "top_alignment_boundary_y out of range";
            return false;
        }
        if (resolved_bottom_alignment_boundary_y < 0 || resolved_bottom_alignment_boundary_y >= main_image.rows) {
            detail = "bottom_alignment_boundary_y out of range";
            return false;
        }
        if (resolved_top_alignment_boundary_y > resolved_bottom_alignment_boundary_y) {
            detail = "alignment height boundaries overlap or invert";
            return false;
        }

        cv::Mat left_aligned(main_image.rows, left_source_roi.width, left_image.type(), cv::Scalar::all(0));
        cv::Mat main_panel = main_image(main_source_roi).clone();
        cv::Mat right_aligned(main_image.rows, right_source_roi.width, right_image.type(), cv::Scalar::all(0));
        if (!copyAlignedStrip(left_image(left_source_roi),
                              left_anchor_y,
                              left_main_anchor_y,
                              left_aligned)) {
            detail = "left aligned copy failed";
            return false;
        }
        if (!copyAlignedStrip(right_image(right_source_roi),
                              right_anchor_y,
                              right_main_anchor_y,
                              right_aligned)) {
            detail = "right aligned copy failed";
            return false;
        }

        const int left_anchor_x = clampToImageCol(static_cast<int>(std::lround(geometry.left_projection.x)) - left_source_roi.x,
                                                  left_source_roi.width);
        const int right_anchor_x = clampToImageCol(static_cast<int>(std::lround(geometry.right_projection.x)) - right_source_roi.x,
                                                   right_source_roi.width);
        const int main_left_anchor_x = clampToImageCol(geometry.left_projection_boundary_x - main_source_roi.x,
                                                       main_source_roi.width);
        const int main_right_anchor_x = clampToImageCol(geometry.right_projection_boundary_x - main_source_roi.x,
                                                        main_source_roi.width);

        drawAnchorMarker(left_aligned, cv::Point(left_anchor_x, left_main_anchor_y), cv::Scalar(0, 255, 255));
        drawAnchorMarker(main_panel, cv::Point(main_left_anchor_x, left_main_anchor_y), cv::Scalar(0, 255, 0));
        drawAnchorMarker(main_panel, cv::Point(main_right_anchor_x, right_main_anchor_y), cv::Scalar(0, 255, 0));
        drawAnchorMarker(right_aligned, cv::Point(right_anchor_x, right_main_anchor_y), cv::Scalar(0, 255, 255));

        const int output_y = resolved_top_alignment_boundary_y;
        const int output_height = resolved_bottom_alignment_boundary_y - resolved_top_alignment_boundary_y + 1;
        if (output_height <= 0) {
            detail = "output height <= 0";
            return false;
        }
        const cv::Rect output_roi(0, output_y, left_aligned.cols, output_height);
        const cv::Rect main_output_roi(0, output_y, main_panel.cols, output_height);
        const cv::Rect right_output_roi(0, output_y, right_aligned.cols, output_height);
        if (!isValidRoi(output_roi, left_aligned.size()) ||
            !isValidRoi(main_output_roi, main_panel.size()) ||
            !isValidRoi(right_output_roi, right_aligned.size())) {
            detail = "output height roi is invalid";
            return false;
        }

        std::vector<cv::Mat> strips;
        strips.push_back(left_aligned(output_roi).clone());
        strips.push_back(main_panel(main_output_roi).clone());
        strips.push_back(right_aligned(right_output_roi).clone());
        cv::hconcat(strips, stitched_image);
        if (stitched_image.empty()) {
            detail = "stitched image is empty";
            return false;
        }

        std::ostringstream oss;
        oss << "align_depth_z=" << align_depth_z
            << " crop_side_width=" << boolToString(crop_side_width)
            << " crop_main_width=" << boolToString(crop_main_width)
            << " left_projection_boundary_x=" << geometry.left_projection_boundary_x
            << " right_projection_boundary_x=" << geometry.right_projection_boundary_x
            << " top_alignment_boundary_y=" << resolved_top_alignment_boundary_y
            << " bottom_alignment_boundary_y=" << resolved_bottom_alignment_boundary_y
            << " p_main_left=" << pointToString(geometry.main_left_edge)
            << " p_main_right=" << pointToString(geometry.main_right_edge)
            << " P_left_world=" << vectorToString(geometry.left_world)
            << " P_right_world=" << vectorToString(geometry.right_world)
            << " p_left_img=" << pointToString(geometry.left_projection)
            << " p_right_img=" << pointToString(geometry.right_projection)
            << " left_source_roi=[" << left_source_roi.x << "," << left_source_roi.y << "," << left_source_roi.width << "," << left_source_roi.height << "]"
            << " main_source_roi=[" << main_source_roi.x << "," << main_source_roi.y << "," << main_source_roi.width << "," << main_source_roi.height << "]"
            << " right_source_roi=[" << right_source_roi.x << "," << right_source_roi.y << "," << right_source_roi.width << "," << right_source_roi.height << "]"
            << " left_main_anchor_y=" << left_main_anchor_y
            << " right_main_anchor_y=" << right_main_anchor_y
            << " left_anchor_y=" << left_anchor_y
            << " right_anchor_y=" << right_anchor_y
            << " output_roi_y=" << output_y
            << " output_roi_height=" << output_height
            << " stitched_size=" << stitched_image.cols << "x" << stitched_image.rows;
        detail = oss.str();
        return true;
    }

    bool copyAlignedStrip(const cv::Mat& source_strip,
                          int source_anchor_y,
                          int target_anchor_y,
                          cv::Mat& aligned_strip) const
    {
        if (source_strip.empty() || aligned_strip.empty() || source_strip.cols != aligned_strip.cols) {
            return false;
        }

        const int offset_y = target_anchor_y - source_anchor_y;
        const int dst_y = std::max(0, offset_y);
        const int src_y = std::max(0, -offset_y);
        const int copy_height = std::min(source_strip.rows - src_y, aligned_strip.rows - dst_y);
        if (copy_height <= 0) {
            return false;
        }

        source_strip(cv::Rect(0, src_y, source_strip.cols, copy_height))
            .copyTo(aligned_strip(cv::Rect(0, dst_y, aligned_strip.cols, copy_height)));
        return true;
    }

    bool resolveProjectionGeometry(const cv::Size& left_image_size,
                                   const cv::Size& main_image_size,
                                   const cv::Size& right_image_size,
                                   double align_depth_z,
                                   int left_projection_boundary_x,
                                   int right_projection_boundary_x,
                                   int left_projection_anchor_y,
                                   int right_projection_anchor_y,
                                   ProjectionGeometryCache& geometry,
                                   std::string& error)
    {
        const int resolved_left_projection_boundary_x = resolveLeftProjectionBoundaryX(left_projection_boundary_x,
                                                                                       main_image_size.width);
        const int resolved_right_projection_boundary_x = resolveRightProjectionBoundaryX(right_projection_boundary_x,
                                                                                         main_image_size.width);
        const int resolved_left_projection_anchor_y = resolveProjectionAnchorY(left_projection_anchor_y,
                                                                               main_image_size.height);
        const int resolved_right_projection_anchor_y = resolveProjectionAnchorY(right_projection_anchor_y,
                                                                                main_image_size.height);
        if (resolved_left_projection_boundary_x < 0 || resolved_left_projection_boundary_x >= main_image_size.width) {
            error = "left_projection_boundary_x out of range";
            return false;
        }
        if (resolved_right_projection_boundary_x < 0 || resolved_right_projection_boundary_x >= main_image_size.width) {
            error = "right_projection_boundary_x out of range";
            return false;
        }
        if (resolved_left_projection_boundary_x >= resolved_right_projection_boundary_x) {
            error = "projection boundaries overlap or invert";
            return false;
        }

        if (projection_geometry_cache_.valid &&
            std::abs(projection_geometry_cache_.align_depth_z - align_depth_z) <= 1e-12 &&
            projection_geometry_cache_.left_projection_boundary_x == resolved_left_projection_boundary_x &&
            projection_geometry_cache_.right_projection_boundary_x == resolved_right_projection_boundary_x &&
            projection_geometry_cache_.left_projection_anchor_y == resolved_left_projection_anchor_y &&
            projection_geometry_cache_.right_projection_anchor_y == resolved_right_projection_anchor_y &&
            projection_geometry_cache_.left_image_size == left_image_size &&
            projection_geometry_cache_.main_image_size == main_image_size &&
            projection_geometry_cache_.right_image_size == right_image_size) {
            geometry = projection_geometry_cache_;
            error.clear();
            return true;
        }

        ProjectionGeometryCache computed;
        computed.valid = true;
        computed.align_depth_z = align_depth_z;
        computed.left_projection_boundary_x = resolved_left_projection_boundary_x;
        computed.right_projection_boundary_x = resolved_right_projection_boundary_x;
        computed.left_projection_anchor_y = resolved_left_projection_anchor_y;
        computed.right_projection_anchor_y = resolved_right_projection_anchor_y;
        computed.left_image_size = left_image_size;
        computed.main_image_size = main_image_size;
        computed.right_image_size = right_image_size;
        computed.main_left_edge = cv::Point2d(static_cast<double>(resolved_left_projection_boundary_x),
                                              static_cast<double>(resolved_left_projection_anchor_y));
        computed.main_right_edge = cv::Point2d(static_cast<double>(resolved_right_projection_boundary_x),
                                               static_cast<double>(resolved_right_projection_anchor_y));

        const bool front_left_camera_ok =
            main_camera_.model.projectCamera2dToCamera3d(computed.main_left_edge, align_depth_z, computed.front_left_camera);
        const bool front_right_camera_ok =
            main_camera_.model.projectCamera2dToCamera3d(computed.main_right_edge, align_depth_z, computed.front_right_camera);
        if (!front_left_camera_ok) {
            logFrontPointDetails(align_depth_z,
                                 computed.main_left_edge, computed.main_right_edge,
                                 false, computed.front_left_camera,
                                 front_right_camera_ok, computed.front_right_camera,
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, cv::Point2d(),
                                 false, cv::Point2d());
            error = "main left edge projectCamera2dToCamera3d failed";
            return false;
        }
        if (!front_right_camera_ok) {
            logFrontPointDetails(align_depth_z,
                                 computed.main_left_edge, computed.main_right_edge,
                                 true, computed.front_left_camera,
                                 false, computed.front_right_camera,
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, cv::Point2d(),
                                 false, cv::Point2d());
            error = "main right edge projectCamera2dToCamera3d failed";
            return false;
        }
        if (!main_camera_.model.projectCamera2dToWorld3d(computed.main_left_edge, align_depth_z, computed.left_world)) {
            logFrontPointDetails(align_depth_z,
                                 computed.main_left_edge, computed.main_right_edge,
                                 true, computed.front_left_camera,
                                 true, computed.front_right_camera,
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, cv::Point2d(),
                                 false, cv::Point2d());
            error = "main left edge projectCamera2dToWorld3d failed";
            return false;
        }
        if (!main_camera_.model.projectCamera2dToWorld3d(computed.main_right_edge, align_depth_z, computed.right_world)) {
            logFrontPointDetails(align_depth_z,
                                 computed.main_left_edge, computed.main_right_edge,
                                 true, computed.front_left_camera,
                                 true, computed.front_right_camera,
                                 true, computed.left_world,
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, Eigen::Vector3d::Zero(),
                                 false, cv::Point2d(),
                                 false, cv::Point2d());
            error = "main right edge projectCamera2dToWorld3d failed";
            return false;
        }

        const bool left_in_left_camera_ok =
            left_camera_.model.world3dToCamera3d(computed.left_world, computed.left_in_left_camera);
        const bool right_in_right_camera_ok =
            right_camera_.model.world3dToCamera3d(computed.right_world, computed.right_in_right_camera);
        const bool left_projection_ok =
            left_camera_.model.projectWorld3dTo2d(computed.left_world, computed.left_projection);
        const bool right_projection_ok =
            right_camera_.model.projectWorld3dTo2d(computed.right_world, computed.right_projection);

        if (!left_projection_ok) {
            logFrontPointDetails(align_depth_z,
                                 computed.main_left_edge, computed.main_right_edge,
                                 true, computed.front_left_camera,
                                 true, computed.front_right_camera,
                                 true, computed.left_world,
                                 true, computed.right_world,
                                 left_in_left_camera_ok, computed.left_in_left_camera,
                                 right_in_right_camera_ok, computed.right_in_right_camera,
                                 left_projection_ok, computed.left_projection,
                                 right_projection_ok, computed.right_projection);
            error = "left camera projectWorld3dTo2d failed";
            return false;
        }
        if (!right_projection_ok) {
            logFrontPointDetails(align_depth_z,
                                 computed.main_left_edge, computed.main_right_edge,
                                 true, computed.front_left_camera,
                                 true, computed.front_right_camera,
                                 true, computed.left_world,
                                 true, computed.right_world,
                                 left_in_left_camera_ok, computed.left_in_left_camera,
                                 right_in_right_camera_ok, computed.right_in_right_camera,
                                 left_projection_ok, computed.left_projection,
                                 right_projection_ok, computed.right_projection);
            error = "right camera projectWorld3dTo2d failed";
            return false;
        }

        projection_geometry_cache_ = computed;
        geometry = computed;
        error.clear();
        return true;
    }

    int resolveLeftProjectionBoundaryX(int param_value, int image_width) const
    {
        if (image_width <= 0) {
            return 0;
        }
        return std::max(0, std::min(param_value, image_width - 1));
    }

    int resolveRightProjectionBoundaryX(int param_value, int image_width) const
    {
        if (image_width <= 0) {
            return 0;
        }
        if (param_value == 0) {
            return image_width - 1;
        }
        return std::max(0, std::min(param_value, image_width - 1));
    }

    int resolveTopAlignmentBoundaryY(int param_value, int image_height) const
    {
        if (image_height <= 0) {
            return 0;
        }
        return std::max(0, std::min(param_value, image_height - 1));
    }

    int resolveBottomAlignmentBoundaryY(int param_value, int image_height) const
    {
        if (image_height <= 0) {
            return 0;
        }
        if (param_value == 0) {
            return image_height - 1;
        }
        return std::max(0, std::min(param_value, image_height - 1));
    }

    int resolveProjectionAnchorY(int param_value, int image_height) const
    {
        if (image_height <= 0) {
            return 0;
        }
        if (param_value < 0) {
            return image_height / 2;
        }
        return std::max(0, std::min(param_value, image_height - 1));
    }

    int clampToImageRow(int row, int image_height) const
    {
        if (image_height <= 0) {
            return 0;
        }
        return std::max(0, std::min(row, image_height - 1));
    }

    int clampToImageCol(int col, int image_width) const
    {
        if (image_width <= 0) {
            return 0;
        }
        return std::max(0, std::min(col, image_width - 1));
    }

    bool isValidRoi(const cv::Rect& roi, const cv::Size& size) const
    {
        return roi.width > 0 &&
               roi.height > 0 &&
               roi.x >= 0 &&
               roi.y >= 0 &&
               roi.x + roi.width <= size.width &&
               roi.y + roi.height <= size.height;
    }

    ros::Time latestStamp(const Batch& batch) const
    {
        return std::max({batch.left.stamp, batch.main.stamp, batch.right.stamp});
    }

    double syncSpanSeconds(const Batch& batch) const
    {
        const ros::Time min_stamp = std::min({batch.left.stamp, batch.main.stamp, batch.right.stamp});
        const ros::Time max_stamp = std::max({batch.left.stamp, batch.main.stamp, batch.right.stamp});
        return (max_stamp - min_stamp).toSec();
    }

    void publishSuccess(const ros::Time& stamp, const cv::Mat& stitched_image, const std::string& detail)
    {
        cv_bridge::CvImage image;
        image.header.stamp = stamp;
        image.header.frame_id = "multi_view_stitch";
        image.encoding = sensor_msgs::image_encodings::BGR8;
        image.image = stitched_image;
        pub_stitched_.publish(image.toImageMsg());
        publishStatus(true, detail);
    }

    void publishFailure(const ros::Time& stamp, const std::string& reason)
    {
        sensor_msgs::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "multi_view_stitch";
        msg.encoding = sensor_msgs::image_encodings::BGR8;
        msg.height = 0;
        msg.width = 0;
        msg.step = 0;
        msg.data.clear();
        pub_stitched_.publish(msg);
        publishStatus(false, reason);
        maybeLog(false, reason);
    }

    void publishStatus(bool success, const std::string& message)
    {
        std_msgs::String msg;
        msg.data = (success ? "OK: " : "FAIL: ") + message;
        pub_status_.publish(msg);
    }

    void maybeLog(bool success, const std::string& message)
    {
        const ros::Time now = ros::Time::now();
        if (message != last_failure_message_ || (now - last_failure_log_time_).toSec() >= 1.0) {
            ROS_WARN("[MultiViewStitchNode] %s", message.c_str());
            last_failure_message_ = message;
            last_failure_log_time_ = now;
        }
    }

    void logFrontPointDetails(double align_depth_z,
                              const cv::Point2d& main_left_edge,
                              const cv::Point2d& main_right_edge,
                              bool front_left_camera_ok,
                              const Eigen::Vector3d& front_left_camera,
                              bool front_right_camera_ok,
                              const Eigen::Vector3d& front_right_camera,
                              bool left_world_ok,
                              const Eigen::Vector3d& left_world,
                              bool right_world_ok,
                              const Eigen::Vector3d& right_world,
                              bool left_in_left_camera_ok,
                              const Eigen::Vector3d& left_in_left_camera,
                              bool right_in_right_camera_ok,
                              const Eigen::Vector3d& right_in_right_camera,
                              bool left_projection_ok,
                              const cv::Point2d& left_projection,
                              bool right_projection_ok,
                              const cv::Point2d& right_projection) const
    {
        if (!log_debug_) {
            return;
        }

        std::ostringstream oss;
        oss << "[MultiViewStitchNode] full_points"
            << " align_depth_z=" << align_depth_z
            << " front_left_2d=" << pointToString(main_left_edge)
            << " front_left_camera3d_ok=" << boolToString(front_left_camera_ok)
            << " front_left_camera3d=" << vectorToString(front_left_camera)
            << " front_left_world3d_ok=" << boolToString(left_world_ok)
            << " front_left_world3d=" << vectorToString(left_world)
            << " left_camera3d_ok=" << boolToString(left_in_left_camera_ok)
            << " left_camera3d=" << vectorToString(left_in_left_camera)
            << " left_projection_ok=" << boolToString(left_projection_ok)
            << " left_projection_2d=" << pointToString(left_projection)
            << " front_right_2d=" << pointToString(main_right_edge)
            << " front_right_camera3d_ok=" << boolToString(front_right_camera_ok)
            << " front_right_camera3d=" << vectorToString(front_right_camera)
            << " front_right_world3d_ok=" << boolToString(right_world_ok)
            << " front_right_world3d=" << vectorToString(right_world)
            << " right_camera3d_ok=" << boolToString(right_in_right_camera_ok)
            << " right_camera3d=" << vectorToString(right_in_right_camera)
            << " right_projection_ok=" << boolToString(right_projection_ok)
            << " right_projection_2d=" << pointToString(right_projection);
        ROS_INFO("%s", oss.str().c_str());
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    ros::Timer process_timer_;
    image_transport::Subscriber sub_left_raw_;
    image_transport::Subscriber sub_main_raw_;
    image_transport::Subscriber sub_right_raw_;
    ros::Subscriber sub_left_compressed_;
    ros::Subscriber sub_main_compressed_;
    ros::Subscriber sub_right_compressed_;
    image_transport::Publisher pub_stitched_;
    ros::Publisher pub_status_;
    ros::ServiceServer srv_save_params_;

    std::array<int, 3> rotation_order_ = {{2, 1, 0}};
    CameraConfig left_camera_;
    CameraConfig main_camera_;
    CameraConfig right_camera_;

    mutable std::mutex frame_mutex_;
    CameraFrame left_frame_;
    CameraFrame main_frame_;
    CameraFrame right_frame_;

    double default_align_depth_z_ = 3.0;
    double publish_max_hz_ = 10.0;
    double sync_tolerance_sec_ = 0.10;
    bool log_debug_ = true;
    bool default_crop_side_width_ = true;
    bool default_crop_main_width_ = false;
    int default_left_projection_boundary_x_ = 0;
    int default_right_projection_boundary_x_ = 0;
    int default_left_projection_anchor_y_ = -1;
    int default_right_projection_anchor_y_ = -1;
    int default_top_alignment_boundary_y_ = 0;
    int default_bottom_alignment_boundary_y_ = 0;
    std::string stitched_topic_;
    std::string status_topic_;
    std::string save_params_yaml_path_;

    std::string last_failure_message_;
    ros::Time last_failure_log_time_;
    ProjectionGeometryCache projection_geometry_cache_;
};

}  // namespace multi_view_stitch

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_view_stitch_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        multi_view_stitch::MultiViewStitchNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& ex) {
        ROS_FATAL("[multi_view_stitch_node] %s", ex.what());
        return 1;
    }
    return 0;
}
