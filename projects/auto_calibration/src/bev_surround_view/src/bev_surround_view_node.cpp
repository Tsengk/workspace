#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "bev_surround_view/fisheye_auto_calibrator.h"

namespace bev_surround_view {

class BevSurroundViewNode {
public:
    BevSurroundViewNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh),
          pnh_(pnh),
          it_(nh)
    {
        loadParams();
        setupIo();
        startWorkers();
        startParamMonitor();
        ROS_INFO("[BevSurroundViewNode] init OK");
    }

    ~BevSurroundViewNode()
    {
        shutdown();
    }

private:
    struct CameraFrameCache {
        bool has_image = false;
        ros::Time stamp;
        cv::Mat image;
    };

    struct SyncedBatch {
        bool valid = false;
        uint64_t version = 0;
        ros::Time stamp;
        std::array<ros::Time, 4> stamps;
        std::array<cv::Mat, 4> images;
    };

    void loadParams()
    {
        // Output parameter names
        pnh_.param<std::string>("output/front_yaml_path", options_.output_front_yaml_path, "");
        pnh_.param<std::string>("output/rear_yaml_path", options_.output_rear_yaml_path, "");
        pnh_.param<std::string>("output/left_yaml_path", options_.output_left_yaml_path, "");
        pnh_.param<std::string>("output/right_yaml_path", options_.output_right_yaml_path, "");
        pnh_.param<std::string>("output/front_bin_path", bin_file_paths_["front"], "");
        pnh_.param<std::string>("output/rear_bin_path", bin_file_paths_["back"], "");
        pnh_.param<std::string>("output/left_bin_path", bin_file_paths_["left"], "");
        pnh_.param<std::string>("output/right_bin_path", bin_file_paths_["right"], "");
        pnh_.param<std::string>("output/debug_dir", options_.debug_dir, "/tmp/bev_surround_view");
        pnh_.param<bool>("output/save_debug_images", options_.save_debug_images, true);
        pnh_.param<bool>("output/save_bev_images", options_.save_bev_images, true);

        pnh_.param<double>("calibration/detector_score", options_.detector_score, 0.6);
        pnh_.param<std::string>("calibration/chessboard_yaml_path", options_.chessboard_yaml_path, "");
        pnh_.param<double>("calibration/solve_ground_z", options_.solve_ground_z, 0.0);
        pnh_.param<double>("calibration/back_sort_threshold", options_.back_sort_threshold, 0.15);
        pnh_.param<bool>("calibration/enable_joint_optimization", options_.enable_joint_optimization, true);
        pnh_.param<double>("calibration/ground_offset_z_m", options_.ground_offset_z_m, 0.37);
        pnh_.param<double>("vcs/z_offset", options_.ground_offset_z_m, options_.ground_offset_z_m);
        pnh_.param<int>("calibration/debug_boundary_scale", options_.debug_boundary_scale, 4);

        pnh_.param<double>("publish/max_hz", publish_max_hz_, 10.0);
        pnh_.param<std::string>("status_topic", status_topic_, "/bev_surround_view/status");
        pnh_.param<std::string>("bev_fused_topic", fused_bev_topic_, "/bev_surround_view/bev/fused");
        pnh_.param<std::string>("bev_front_topic", bev_topics_["front"], "/bev_surround_view/bev/front");
        pnh_.param<std::string>("bev_back_topic", bev_topics_["back"], "/bev_surround_view/bev/back");
        pnh_.param<std::string>("bev_left_topic", bev_topics_["left"], "/bev_surround_view/bev/left");
        pnh_.param<std::string>("bev_right_topic", bev_topics_["right"], "/bev_surround_view/bev/right");
        pnh_.param<std::string>("corners_front_topic", debug_topics_["front"], "/bev_surround_view/debug/corners/front");
        pnh_.param<std::string>("corners_back_topic", debug_topics_["back"], "/bev_surround_view/debug/corners/back");
        pnh_.param<std::string>("corners_left_topic", debug_topics_["left"], "/bev_surround_view/debug/corners/left");
        pnh_.param<std::string>("corners_right_topic", debug_topics_["right"], "/bev_surround_view/debug/corners/right");
        pnh_.param<int>("publish/status_queue_size", status_queue_size_, 10);
        pnh_.param<int>("publish/image_queue_size", image_queue_size_, 1);
        pnh_.param<bool>("publish/latch", publisher_latch_, true);
        pnh_.param<double>("runtime/param_monitor_period_sec", param_monitor_period_sec_, 1.0);
        pnh_.param<double>("runtime/worker_idle_sleep_sec", worker_idle_sleep_sec_, 0.01);
        pnh_.param<int>("subscribe/image_queue_size", subscribe_image_queue_size_, 1);

        // BEV parameters: support both old scale and new resolution
        double bev_resolution = 0.02;
        pnh_.param<double>("bev/resolution", bev_resolution, 0.02);
        pnh_.param<double>("bev/scale", options_.bev_scale, 1.0 / bev_resolution);
        pnh_.param<int>("bev/width", options_.bev_width, 1000);
        pnh_.param<int>("bev/height", options_.bev_height, 1000);
        pnh_.param<int>("output_width", options_.bev_width, options_.bev_width);
        pnh_.param<int>("output_height", options_.bev_height, options_.bev_height);
        pnh_.param<bool>("bev/enable_roi_crop", options_.enable_roi_crop, true);
        pnh_.param<double>("bev/range_front", options_.bev_range_front, 0.0);
        pnh_.param<double>("bev/range_rear", options_.bev_range_rear, 0.0);
        pnh_.param<double>("bev/range_left", options_.bev_range_left, 0.0);
        pnh_.param<double>("bev/range_right", options_.bev_range_right, 0.0);
        pnh_.param<double>("bev/origin_offset_x", options_.origin_offset_x, 0.0);
        pnh_.param<double>("bev/origin_offset_y", options_.origin_offset_y, 0.0);
        options_.bev_range_front_auto = options_.enable_roi_crop && options_.bev_range_front <= 0.0;
        options_.bev_range_rear_auto = options_.enable_roi_crop && options_.bev_range_rear <= 0.0;
        options_.bev_range_left_auto = options_.enable_roi_crop && options_.bev_range_left <= 0.0;
        options_.bev_range_right_auto = options_.enable_roi_crop && options_.bev_range_right <= 0.0;

        // Rotation order for camera::Mei
        int rot0 = 2, rot1 = 1, rot2 = 0;
        pnh_.param("rotation_order_0", rot0, 2);
        pnh_.param("rotation_order_1", rot1, 1);
        pnh_.param("rotation_order_2", rot2, 0);
        rotation_order_[0] = rot0;
        rotation_order_[1] = rot1;
        rotation_order_[2] = rot2;
        ROS_INFO("[BevSurroundViewNode] rotation_order=[%d, %d, %d]",
                 rotation_order_[0], rotation_order_[1], rotation_order_[2]);

        cameras_[0].role = "front";
        cameras_[1].role = "back";
        cameras_[2].role = "left";
        cameras_[3].role = "right";

        std::string front_topic, rear_topic, left_topic, right_topic;
        pnh_.param<std::string>("front_image_topic", front_topic, "/camera_front/image_raw");
        pnh_.param<std::string>("rear_image_topic", rear_topic, "/camera_rear/image_raw");
        pnh_.param<std::string>("left_image_topic", left_topic, "/camera_left/image_raw");
        pnh_.param<std::string>("right_image_topic", right_topic, "/camera_right/image_raw");

        loadCameraRequest("front", "front_fish_eye", front_topic, cameras_[0]);
        loadCameraRequest("back", "back_fish_eye", rear_topic, cameras_[1]);
        loadCameraRequest("left", "left_fish_eye", left_topic, cameras_[2]);
        loadCameraRequest("right", "right_fish_eye", right_topic, cameras_[3]);

        if (options_.bev_scale <= 0.0 || options_.bev_width <= 0 || options_.bev_height <= 0) {
            ROS_FATAL("[BevSurroundViewNode] invalid bev params: scale=%.3f width=%d height=%d",
                      options_.bev_scale,
                      options_.bev_width,
                      options_.bev_height);
            throw std::runtime_error("invalid bev params");
        }
        if (options_.back_sort_threshold <= 0.0) {
            ROS_FATAL("[BevSurroundViewNode] invalid back_sort_threshold: %.6f",
                      options_.back_sort_threshold);
            throw std::runtime_error("invalid back_sort_threshold");
        }
        if (sync_tolerance_sec_ < 0.0) {
            ROS_FATAL("[BevSurroundViewNode] invalid sync_tolerance_sec: %.6f", sync_tolerance_sec_);
            throw std::runtime_error("invalid sync_tolerance_sec");
        }
        if (publish_max_hz_ <= 0.0) {
            ROS_FATAL("[BevSurroundViewNode] invalid publish/max_hz: %.6f", publish_max_hz_);
            throw std::runtime_error("invalid publish/max_hz");
        }

        applyCameraConfigOverrides();

        calibrator_ = std::make_unique<FisheyeAutoCalibrator>(cameras_);

        ROS_INFO("[BevSurroundViewNode] Params loaded.");
        ROS_INFO("[BevSurroundViewNode] Cameras: front=%s back=%s left=%s right=%s",
                 cameras_[0].camera_name.c_str(),
                 cameras_[1].camera_name.c_str(),
                 cameras_[2].camera_name.c_str(),
                 cameras_[3].camera_name.c_str());
        ROS_INFO("[BevSurroundViewNode] Output: front_yaml=%s rear_yaml=%s left_yaml=%s right_yaml=%s",
                 options_.output_front_yaml_path.c_str(),
                 options_.output_rear_yaml_path.c_str(),
                 options_.output_left_yaml_path.c_str(),
                 options_.output_right_yaml_path.c_str());
        ROS_INFO("[BevSurroundViewNode] Output flags: debug_dir=%s save_debug=%s save_bev=%s",
                 options_.debug_dir.c_str(),
                 options_.save_debug_images ? "true" : "false",
                 options_.save_bev_images ? "true" : "false");
        ROS_INFO("[BevSurroundViewNode] Calibration: detector_score=%.2f ground_z=%.3f back_sort_threshold=%.3f joint_optimization=%s ground_offset_z=%.3f chessboard_yaml=%s sync_tolerance=%.3f",
                 options_.detector_score,
                 options_.solve_ground_z,
                 options_.back_sort_threshold,
                 options_.enable_joint_optimization ? "true" : "false",
                 options_.ground_offset_z_m,
                 options_.chessboard_yaml_path.c_str(),
                 sync_tolerance_sec_);
        ROS_INFO("[BevSurroundViewNode] Publish: max_hz=%.2f", publish_max_hz_);
        ROS_INFO("[BevSurroundViewNode] BEV: scale=%.1f width=%d height=%d",
                 options_.bev_scale,
                 options_.bev_width,
                 options_.bev_height);
    }

    void loadCameraRequest(const std::string& role,
                           const std::string& default_name,
                           const std::string& default_topic,
                           FisheyeAutoCameraConfig& camera)
    {
        pnh_.param<std::string>("cameras/" + role + "/name", camera.camera_name, default_name);
        pnh_.param<std::string>("cameras/" + role + "/topic", camera.image_topic, default_topic);
        camera.rotation_order = rotation_order_;
    }

    bool applyIntrinsicsOverrideFromFile(const std::string& path,
                                         FisheyeAutoCameraConfig& camera,
                                         std::string& error)
    {
        if (path.empty()) {
            error = "intrinsics file is empty for camera: " + camera.camera_name;
            return false;
        }

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            error = "failed to open intrinsics file: " + path;
            return false;
        }

        const cv::FileNode mirror = fs["mirror_parameters"];
        const cv::FileNode distortion = fs["distortion_parameters"];
        const cv::FileNode projection = fs["projection_parameters"];
        if (mirror.empty() || distortion.empty() || projection.empty()) {
            error = "invalid intrinsics file: " + path;
            return false;
        }

        int image_width = 0;
        int image_height = 0;
        double xi = 0.0;
        double gamma1 = 0.0;
        double gamma2 = 0.0;
        double u0 = 0.0;
        double v0 = 0.0;
        double k1 = 0.0;
        double k2 = 0.0;
        double p1 = 0.0;
        double p2 = 0.0;

        fs["image_width"] >> image_width;
        fs["image_height"] >> image_height;
        mirror["xi"] >> xi;
        projection["gamma1"] >> gamma1;
        projection["gamma2"] >> gamma2;
        projection["u0"] >> u0;
        projection["v0"] >> v0;
        distortion["k1"] >> k1;
        distortion["k2"] >> k2;
        distortion["p1"] >> p1;
        distortion["p2"] >> p2;

        if (image_width <= 0 || image_height <= 0) {
            error = "invalid image size in intrinsics file: " + path;
            return false;
        }

        camera.K = cv::Mat::eye(3, 3, CV_64F);
        camera.K.at<double>(0, 0) = gamma1;
        camera.K.at<double>(1, 1) = gamma2;
        camera.K.at<double>(0, 2) = u0;
        camera.K.at<double>(1, 2) = v0;
        camera.new_K = camera.K.clone();
        camera.D = cv::Mat::zeros(1, 4, CV_64F);
        camera.D.at<double>(0, 0) = k1;
        camera.D.at<double>(0, 1) = k2;
        camera.D.at<double>(0, 2) = p1;
        camera.D.at<double>(0, 3) = p2;
        camera.epsilon = xi;
        camera.width = image_width;
        camera.height = image_height;
        camera.new_width = image_width;
        camera.new_height = image_height;
        camera.vcs_to_ground_height = options_.ground_offset_z_m;

        ROS_INFO("[BevSurroundViewNode] intrinsics override applied: %s <- %s",
                 camera.camera_name.c_str(),
                 path.c_str());
        return true;
    }

    bool applyInstallationOverride(const std::string& install_role,
                                   FisheyeAutoCameraConfig& camera,
                                   std::string& error)
    {
        const std::string prefix = "installation/" + install_role + "/";
        const bool has_all_params =
            pnh_.hasParam(prefix + "x") &&
            pnh_.hasParam(prefix + "y") &&
            pnh_.hasParam(prefix + "z") &&
            pnh_.hasParam(prefix + "roll") &&
            pnh_.hasParam(prefix + "pitch") &&
            pnh_.hasParam(prefix + "yaw");
        if (!has_all_params) {
            error = "missing installation params for role: " + install_role;
            return false;
        }

        double install_x = 0.0;
        double install_y = 0.0;
        double install_z = 0.0;
        double install_roll = 0.0;
        double install_pitch = 0.0;
        double install_yaw = 0.0;
        pnh_.param<double>(prefix + "x", install_x, 0.0);
        pnh_.param<double>(prefix + "y", install_y, 0.0);
        pnh_.param<double>(prefix + "z", install_z, 0.0);
        pnh_.param<double>(prefix + "roll", install_roll, 0.0);
        pnh_.param<double>(prefix + "pitch", install_pitch, 0.0);
        pnh_.param<double>(prefix + "yaw", install_yaw, 0.0);

        camera.t.resize(3);
        camera.t[0] = install_y;
        camera.t[1] = install_z;
        camera.t[2] = -install_x;
        camera.r_euler.resize(3);
        camera.r_euler[0] = install_roll;
        camera.r_euler[1] = install_pitch;
        camera.r_euler[2] = install_yaw;

        ROS_INFO("[BevSurroundViewNode] installation override applied: %s input_xyz=[%.3f, %.3f, %.3f] -> t=[%.3f, %.3f, %.3f] R=[%.3f, %.3f, %.3f]",
                 camera.camera_name.c_str(),
                 install_x,
                 install_y,
                 install_z,
                 camera.t[0],
                 camera.t[1],
                 camera.t[2],
                 camera.r_euler[0],
                 camera.r_euler[1],
                 camera.r_euler[2]);
        return true;
    }

    void applyCameraConfigOverrides()
    {
        const std::string intrinsics_param_names[4] = {
            "front_intrinsics_file",
            "rear_intrinsics_file",
            "left_intrinsics_file",
            "right_intrinsics_file"
        };
        const std::string install_roles[4] = {
            "front",
            "rear",
            "left",
            "right"
        };

        std::string error;
        for (size_t i = 0; i < cameras_.size(); ++i) {
            std::string intrinsics_file;
            pnh_.param<std::string>(intrinsics_param_names[i], intrinsics_file, "");
            if (!applyIntrinsicsOverrideFromFile(intrinsics_file, cameras_[i], error)) {
                throw std::runtime_error(error);
            }
            if (!applyInstallationOverride(install_roles[i], cameras_[i], error)) {
                throw std::runtime_error(error);
            }
        }
    }

    void setupIo()
    {
        pub_status_ = pnh_.advertise<std_msgs::String>(status_topic_, status_queue_size_, publisher_latch_);
        pub_fused_bev_ = it_.advertise(fused_bev_topic_, image_queue_size_, publisher_latch_);
        pub_bev_["front"] = it_.advertise(bev_topics_["front"], image_queue_size_, publisher_latch_);
        pub_bev_["back"] = it_.advertise(bev_topics_["back"], image_queue_size_, publisher_latch_);
        pub_bev_["left"] = it_.advertise(bev_topics_["left"], image_queue_size_, publisher_latch_);
        pub_bev_["right"] = it_.advertise(bev_topics_["right"], image_queue_size_, publisher_latch_);
        pub_debug_["front"] = it_.advertise(debug_topics_["front"], image_queue_size_, publisher_latch_);
        pub_debug_["back"] = it_.advertise(debug_topics_["back"], image_queue_size_, publisher_latch_);
        pub_debug_["left"] = it_.advertise(debug_topics_["left"], image_queue_size_, publisher_latch_);
        pub_debug_["right"] = it_.advertise(debug_topics_["right"], image_queue_size_, publisher_latch_);
        srv_trigger_calibration_ = pnh_.advertiseService("trigger_calibration", &BevSurroundViewNode::handleTriggerCalibration, this);
        srv_save_calibration_ = pnh_.advertiseService("save_calibration", &BevSurroundViewNode::handleSaveCalibration, this);
        srv_reset_calibration_ = pnh_.advertiseService("reset_calibration", &BevSurroundViewNode::handleResetCalibration, this);

        for (size_t i = 0; i < cameras_.size(); ++i) {
            const std::string topic = cameras_[i].image_topic;
            subs_[i] = nh_.subscribe<sensor_msgs::CompressedImage>(
                topic,
                subscribe_image_queue_size_,
                boost::bind(&BevSurroundViewNode::imageCallback, this, _1, i));
            ROS_INFO("[BevSurroundViewNode] subscribe %s <- %s",
                     cameras_[i].camera_name.c_str(),
                     topic.c_str());
        }
    }

    void startWorkers()
    {
        calibration_thread_ = std::thread(&BevSurroundViewNode::runCalibrationLoop, this);
        bev_thread_ = std::thread(&BevSurroundViewNode::runBevLoop, this);
    }

    void startParamMonitor()
    {
        param_check_timer_ = nh_.createTimer(ros::Duration(param_monitor_period_sec_), &BevSurroundViewNode::checkParamsCallback, this);
    }

    void checkParamsCallback(const ros::TimerEvent&)
    {
        int new_bev_width = options_.bev_width;
        int new_bev_height = options_.bev_height;
        double new_bev_resolution = 1.0 / options_.bev_scale;
        double new_bev_range_front = options_.bev_range_front;
        double new_bev_range_rear = options_.bev_range_rear;
        double new_bev_range_left = options_.bev_range_left;
        double new_bev_range_right = options_.bev_range_right;
        double new_origin_offset_x = options_.origin_offset_x;
        double new_origin_offset_y = options_.origin_offset_y;
        bool new_enable_roi_crop = options_.enable_roi_crop;

        pnh_.getParam("bev/width", new_bev_width);
        pnh_.getParam("bev/height", new_bev_height);
        pnh_.getParam("output_width", new_bev_width);
        pnh_.getParam("output_height", new_bev_height);
        pnh_.getParam("bev/enable_roi_crop", new_enable_roi_crop);
        pnh_.getParam("bev/resolution", new_bev_resolution);
        pnh_.getParam("bev/range_front", new_bev_range_front);
        pnh_.getParam("bev/range_rear", new_bev_range_rear);
        pnh_.getParam("bev/range_left", new_bev_range_left);
        pnh_.getParam("bev/range_right", new_bev_range_right);
        pnh_.getParam("bev/origin_offset_x", new_origin_offset_x);
        pnh_.getParam("bev/origin_offset_y", new_origin_offset_y);

        double new_bev_scale = 1.0 / new_bev_resolution;
        const bool bev_width_changed = new_bev_width != options_.bev_width;
        const bool bev_height_changed = new_bev_height != options_.bev_height;
        const bool bev_scale_changed = std::abs(new_bev_scale - options_.bev_scale) > 0.001;
        const bool enable_roi_crop_changed = new_enable_roi_crop != options_.enable_roi_crop;
        const bool bev_range_front_changed = std::abs(new_bev_range_front - options_.bev_range_front) > 0.001;
        const bool bev_range_rear_changed = std::abs(new_bev_range_rear - options_.bev_range_rear) > 0.001;
        const bool bev_range_left_changed = std::abs(new_bev_range_left - options_.bev_range_left) > 0.001;
        const bool bev_range_right_changed = std::abs(new_bev_range_right - options_.bev_range_right) > 0.001;
        const bool origin_offset_x_changed = std::abs(new_origin_offset_x - options_.origin_offset_x) > 0.001;
        const bool origin_offset_y_changed = std::abs(new_origin_offset_y - options_.origin_offset_y) > 0.001;

        if (bev_width_changed ||
            bev_height_changed ||
            bev_scale_changed ||
            enable_roi_crop_changed ||
            bev_range_front_changed ||
            bev_range_rear_changed ||
            bev_range_left_changed ||
            bev_range_right_changed ||
            origin_offset_x_changed ||
            origin_offset_y_changed) {

            ROS_INFO("[BevSurroundViewNode] BEV params changed: width %d->%d, height %d->%d, scale %.3f->%.3f, enable_roi_crop %d->%d, front %.1f->%.1f, rear %.1f->%.1f, left %.1f->%.1f, right %.1f->%.1f, offset_x %.1f->%.1f, offset_y %.1f->%.1f",
                     options_.bev_width, new_bev_width,
                     options_.bev_height, new_bev_height,
                     options_.bev_scale, new_bev_scale,
                     static_cast<int>(options_.enable_roi_crop), static_cast<int>(new_enable_roi_crop),
                     options_.bev_range_front, new_bev_range_front,
                     options_.bev_range_rear, new_bev_range_rear,
                     options_.bev_range_left, new_bev_range_left,
                     options_.bev_range_right, new_bev_range_right,
                     options_.origin_offset_x, new_origin_offset_x,
                     options_.origin_offset_y, new_origin_offset_y);

            options_.bev_width = new_bev_width;
            options_.bev_height = new_bev_height;
            options_.bev_scale = new_bev_scale;
            options_.enable_roi_crop = new_enable_roi_crop;
            options_.bev_range_front = new_bev_range_front;
            options_.bev_range_rear = new_bev_range_rear;
            options_.bev_range_left = new_bev_range_left;
            options_.bev_range_right = new_bev_range_right;
            if (enable_roi_crop_changed || bev_range_front_changed) {
                options_.bev_range_front_auto = options_.enable_roi_crop && new_bev_range_front <= 0.0;
            }
            if (enable_roi_crop_changed || bev_range_rear_changed) {
                options_.bev_range_rear_auto = options_.enable_roi_crop && new_bev_range_rear <= 0.0;
            }
            if (enable_roi_crop_changed || bev_range_left_changed) {
                options_.bev_range_left_auto = options_.enable_roi_crop && new_bev_range_left <= 0.0;
            }
            if (enable_roi_crop_changed || bev_range_right_changed) {
                options_.bev_range_right_auto = options_.enable_roi_crop && new_bev_range_right <= 0.0;
            }
            options_.origin_offset_x = new_origin_offset_x;
            options_.origin_offset_y = new_origin_offset_y;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (has_best_calibration_) {
                    bev_maps_dirty_ = true;
                }
            }
        }
    }

    void shutdown()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_requested_) {
                return;
            }
            stop_requested_ = true;
        }

        // Stop timer first
        if (param_check_timer_) {
            param_check_timer_.stop();
        }

        if (calibration_thread_.joinable()) {
            calibration_thread_.join();
        }
        if (bev_thread_.joinable()) {
            bev_thread_.join();
        }
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg, size_t idx)
    {
        const cv::Mat encoded(1,
                              static_cast<int>(msg->data.size()),
                              CV_8UC1,
                              const_cast<unsigned char*>(msg->data.data()));
        const cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (decoded.empty()) {
            ROS_WARN("[BevSurroundViewNode] decode failed for %s", cameras_[idx].camera_name.c_str());
            return;
        }

        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_frames_[idx].has_image = true;
            latest_frames_[idx].stamp = stamp;
            latest_frames_[idx].image = decoded;
            updateSyncedBatchLocked();
        }
    }

    bool updateSyncedBatchLocked()
    {
        for (size_t i = 0; i < latest_frames_.size(); ++i) {
            if (latest_frames_[i].has_image == false) {
                return false;
            }
        }

        ros::Time min_stamp = latest_frames_[0].stamp;
        ros::Time max_stamp = latest_frames_[0].stamp;
        for (size_t i = 1; i < latest_frames_.size(); ++i) {
            if (latest_frames_[i].stamp < min_stamp) {
                min_stamp = latest_frames_[i].stamp;
            }
            if (max_stamp < latest_frames_[i].stamp) {
                max_stamp = latest_frames_[i].stamp;
            }
        }

        if ((max_stamp - min_stamp).toSec() > sync_tolerance_sec_) {
            return false;
        }

        bool same_as_last = latest_synced_batch_.valid;
        if (same_as_last) {
            for (size_t i = 0; i < latest_frames_.size(); ++i) {
                if (latest_synced_batch_.stamps[i] != latest_frames_[i].stamp) {
                    same_as_last = false;
                    break;
                }
            }
        }
        if (same_as_last) {
            return false;
        }

        latest_synced_batch_.valid = true;
        latest_synced_batch_.version = latest_synced_batch_version_ + 1;
        latest_synced_batch_.stamp = max_stamp;
        for (size_t i = 0; i < latest_frames_.size(); ++i) {
            latest_synced_batch_.stamps[i] = latest_frames_[i].stamp;
            latest_synced_batch_.images[i] = latest_frames_[i].image.clone();
        }
        latest_synced_batch_version_ = latest_synced_batch_.version;
        return true;
    }

    void runCalibrationLoop()
    {
        while (true) {
            SyncedBatch batch;
            bool has_new_batch = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stop_requested_) {
                    return;
                }
                if (latest_synced_batch_.valid && latest_synced_batch_version_ > calibration_consumed_version_) {
                    batch = latest_synced_batch_;
                    calibration_consumed_version_ = latest_synced_batch_version_;
                    has_new_batch = true;
                }
            }
            if (has_new_batch == false) {
                ros::WallDuration(worker_idle_sleep_sec_).sleep();
                continue;
            }

            std::array<FisheyeDetectionResult, 4> detections;
            std::array<bool, 4> detect_results;
            std::array<std::string, 4> detect_errors;
            std::array<cv::Mat, 4> debug_images;
            detect_results.fill(false);

            struct DetectBatchBody : public cv::ParallelLoopBody
            {
                DetectBatchBody(const FisheyeAutoCalibrator* calibrator,
                                const std::array<FisheyeAutoCameraConfig, 4>* cameras,
                                const SyncedBatch* batch,
                                const FisheyeAutoCalibrator::Options* options,
                                std::array<FisheyeDetectionResult, 4>* detections,
                                std::array<bool, 4>* detect_results,
                                std::array<std::string, 4>* detect_errors,
                                std::array<cv::Mat, 4>* debug_images)
                    : calibrator_(calibrator),
                      cameras_(cameras),
                      batch_(batch),
                      options_(options),
                      detections_(detections),
                      detect_results_(detect_results),
                      detect_errors_(detect_errors),
                      debug_images_(debug_images)
                {
                }

                void operator()(const cv::Range& range) const
                {
                    for (int idx = range.start; idx < range.end; ++idx) {
                        FisheyeAutoCameraConfig camera = (*cameras_)[idx];
                        camera.width = (*batch_).images[idx].cols;
                        camera.height = (*batch_).images[idx].rows;
                        if (camera.new_width <= 0) {
                            camera.new_width = camera.width;
                        }
                        if (camera.new_height <= 0) {
                            camera.new_height = camera.height;
                        }

                        std::string detect_error;
                        const bool detect_ok = calibrator_->detect(camera,
                                                                   (*batch_).images[idx],
                                                                   (*options_),
                                                                   (*detections_)[idx],
                                                                   detect_error);
                        (*detect_results_)[idx] = detect_ok;
                        (*detect_errors_)[idx] = detect_error;
                        (*debug_images_)[idx] = (*detections_)[idx].debug_image;
                    }
                }

                const FisheyeAutoCalibrator* calibrator_;
                const std::array<FisheyeAutoCameraConfig, 4>* cameras_;
                const SyncedBatch* batch_;
                const FisheyeAutoCalibrator::Options* options_;
                std::array<FisheyeDetectionResult, 4>* detections_;
                std::array<bool, 4>* detect_results_;
                std::array<std::string, 4>* detect_errors_;
                std::array<cv::Mat, 4>* debug_images_;
            };

            DetectBatchBody detect_body(calibrator_.get(),
                                        &cameras_,
                                        &batch,
                                        &options_,
                                        &detections,
                                        &detect_results,
                                        &detect_errors,
                                        &debug_images);
            cv::parallel_for_(cv::Range(0, static_cast<int>(cameras_.size())), detect_body);

            bool detect_all_ok = true;
            for (size_t i = 0; i < cameras_.size(); ++i) {
                const bool detect_ok = detect_results[i];
                const cv::Mat debug_image = detect_ok
                    ? debug_images[i]
                    : makeDetectionFailureImage(debug_images[i].empty() ? batch.images[i] : debug_images[i],
                                                detect_errors[i]);
                publishDebugImage(cameras_[i].role, batch.stamps[i], debug_image, i);

                if (detect_ok == false) {
                    detect_all_ok = false;
                    ROS_WARN_THROTTLE(1.0,
                                      "[BevSurroundViewNode] detect failed for %s: %s",
                                      cameras_[i].camera_name.c_str(),
                                      detect_errors[i].c_str());
                }
            }

            if (detect_all_ok == false) {
                publishStatus(false, "detect chessboard failed on synced batch");
                continue;
            }

            FisheyeAutoCalibrator::Output output;
            const bool ok = calibrator_->calibrate(cameras_, detections, options_, output);
            if (ok == false) {
                publishStatus(false, output.message);
                ROS_WARN("[BevSurroundViewNode] calibration failed: %s", output.message.c_str());
                continue;
            }

            const double current_mean_rmse = computeMeanReproj(output);
            double best_rmse_snapshot = std::numeric_limits<double>::infinity();
            bool improved = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (has_best_calibration_ == false || current_mean_rmse + 1e-9 < best_mean_reproj_rmse_px_) {
                    has_best_calibration_ = true;
                    best_mean_reproj_rmse_px_ = current_mean_rmse;
                    best_calibration_output_ = output;
                    bev_maps_dirty_ = true;
                    improved = true;
                }
                best_rmse_snapshot = best_mean_reproj_rmse_px_;
            }

            if (improved) {
                std::string yaml_error;
                if (calibrator_->savePoseYaml(cameras_, output, options_, yaml_error) == false) {
                    ROS_WARN("[BevSurroundViewNode] save yaml failed: %s", yaml_error.c_str());
                }

                FisheyeAutoCalibrator::Options render_opts = options_;
                std::string ipm_error;
                {
                    std::lock_guard<std::mutex> cal_lock(calibrator_mutex_);
                    if (calibrator_->refreshIpmCameras(cameras_, output, render_opts, ipm_error)) {
                        for (size_t i = 0; i < cameras_.size(); ++i) {
                            const std::string& role = cameras_[i].role;
                            const std::string& camera_name = cameras_[i].camera_name;
                            const auto bin_it = bin_file_paths_.find(role);
                            if (bin_it == bin_file_paths_.end() || bin_it->second.empty()) {
                                continue;
                            }

                            cv::Mat map_x, map_y;
                            if (!calibrator_->getIpmMaps(camera_name, map_x, map_y)) {
                                continue;
                            }

                            FisheyeAutoCalibrator::IpmRoiInfo roi_info;
                            if (!calibrator_->getIpmRoi(camera_name, roi_info)) {
                                continue;
                            }

                            std::vector<float> vx, vy;
                            if (!map_x.empty() && map_x.type() == CV_32F) {
                                vx.assign((float*)map_x.datastart, (float*)map_x.dataend);
                            }
                            if (!map_y.empty() && map_y.type() == CV_32F) {
                                vy.assign((float*)map_y.datastart, (float*)map_y.dataend);
                            }

                            serialize(vx, vy, bin_it->second,
                                      roi_info.x, roi_info.y, roi_info.width, roi_info.height);
                        }
                    }
                }

                publishStatus(true, std::string("accepted rmse=") + formatRmse(current_mean_rmse) + "px | " + output.message);
                logPose(cameras_, output);
            } else {
                publishStatus(true,
                              std::string("kept best rmse=") + formatRmse(best_rmse_snapshot) +
                                  "px, current=" + formatRmse(current_mean_rmse) + "px");
            }
        }
    }
    void runBevLoop()
    {
        uint64_t rendered_batch_version = 0;

        while (true) {
            bool need_render = false;
            bool refresh_ipm_info = false;
            FisheyeAutoCalibrator::Output calibration_output;
            FisheyeAutoCalibrator::Options render_options;
            SyncedBatch batch;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stop_requested_) {
                    return;
                }
                if (has_best_calibration_ && latest_synced_batch_.valid &&
                    (bev_maps_dirty_ || latest_synced_batch_version_ > rendered_batch_version)) {
                    calibration_output = best_calibration_output_;
                    render_options = options_;
                    batch = latest_synced_batch_;
                    for (auto& image : batch.images) {
                        image = image.clone();
                    }
                    refresh_ipm_info = bev_maps_dirty_;
                    need_render = true;
                }
            }
            if (!need_render) {
                ros::WallDuration(worker_idle_sleep_sec_).sleep();
                continue;
            }

            if (!allowPublish(last_bev_publish_time_)) {
                const double sleep_sec = 1.0 / publish_max_hz_;
                ros::WallDuration(sleep_sec).sleep();
            }

            
            std::string error;
            FisheyeBevRenderOutput render_output;
            bool render_ok = false;
            {
                std::lock_guard<std::mutex> cal_lock(calibrator_mutex_);
                render_ok = calibrator_->renderBevImages(cameras_,
                                                         calibration_output,
                                                         render_options,
                                                         batch.images,
                                                         refresh_ipm_info,
                                                         render_output,
                                                         error);
            }
            if (!render_ok) {
                if (refresh_ipm_info) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    bev_maps_dirty_ = false;
                }
                publishStatus(false, std::string("bev render failed: ") + error);
                ROS_WARN("[BevSurroundViewNode] bev render failed: %s", error.c_str());
                continue;
            }

            if (refresh_ipm_info) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    options_.bev_range_front = render_options.bev_range_front;
                    options_.bev_range_rear = render_options.bev_range_rear;
                    options_.bev_range_left = render_options.bev_range_left;
                    options_.bev_range_right = render_options.bev_range_right;
                    bev_maps_dirty_ = false;
                }

                pnh_.setParam("bev/range_front", render_options.bev_range_front);
                pnh_.setParam("bev/range_rear", render_options.bev_range_rear);
                pnh_.setParam("bev/range_left", render_options.bev_range_left);
                pnh_.setParam("bev/range_right", render_options.bev_range_right);
            }

            publishBevImages(cameras_, batch.stamp, render_output);
            rendered_batch_version = batch.version;
        }
    }

    cv::Mat makeDetectionFailureImage(const cv::Mat& image, const std::string& message) const
    {
        cv::Mat vis = image.clone();
        const std::string text = message.empty() ? std::string("detect failed") : message;
        cv::rectangle(vis,
                      cv::Rect(0, 0, vis.cols, std::min(60, vis.rows)),
                      cv::Scalar(0, 0, 255),
                      cv::FILLED);
        cv::putText(vis,
                    text,
                    cv::Point(10, std::min(40, vis.rows - 10)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(255, 255, 255),
                    2);
        return vis;
    }

    bool allowPublish(ros::WallTime& last_publish_time)
    {
        const ros::WallTime now = ros::WallTime::now();
        if (last_publish_time.isZero()) {
            last_publish_time = now;
            return true;
        }

        const double min_interval = 1.0 / publish_max_hz_;
        if ((now - last_publish_time).toSec() + 1e-9 < min_interval) {
            return false;
        }

        last_publish_time = now;
        return true;
    }

    void publishDebugImage(const std::string& role,
                           const ros::Time& stamp,
                           const cv::Mat& image,
                           size_t idx)
    {
        if (image.empty()) {
            return;
        }
        if (!allowPublish(last_debug_publish_times_[idx])) {
            return;
        }

        std_msgs::Header header;
        header.stamp = stamp;
        pub_debug_[role].publish(cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
    }

    void publishBevImages(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                          const ros::Time& stamp,
                          const FisheyeBevRenderOutput& render_output)
    {
        std_msgs::Header header;
        header.stamp = stamp;

        for (size_t i = 0; i < cameras.size(); ++i) {
            const std::string& role = cameras[i].role;
            const std::string& camera_name = cameras[i].camera_name;
            const std::map<std::string, cv::Mat>::const_iterator it =
                render_output.bev_images.find(camera_name);
            if (it == render_output.bev_images.end() || it->second.empty()) {
                continue;
            }
            pub_bev_[role].publish(cv_bridge::CvImage(header, "bgr8", it->second).toImageMsg());
        }

        if (render_output.fused_bev_image.empty() == false) {
            pub_fused_bev_.publish(cv_bridge::CvImage(header, "bgr8", render_output.fused_bev_image).toImageMsg());
        }
    }

    double computeMeanReproj(const FisheyeAutoCalibrator::Output& output) const
    {
        double sum = 0.0;
        size_t count = 0;
        for (std::map<std::string, FisheyePoseConfig>::const_iterator it = output.poses.begin();
             it != output.poses.end();
             ++it) {
            if (it->second.valid && it->second.reproj_rmse_px >= 0.0) {
                sum += it->second.reproj_rmse_px;
                count += 1;
            }
        }
        if (count == 0) {
            return std::numeric_limits<double>::infinity();
        }
        return sum / static_cast<double>(count);
    }

    std::string formatRmse(double value) const
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << value;
        return oss.str();
    }

    void logPose(const std::array<FisheyeAutoCameraConfig, 4>& cameras,
                 const FisheyeAutoCalibrator::Output& output)
    {
        for (size_t i = 0; i < cameras.size(); ++i) {
            std::map<std::string, FisheyePoseConfig>::const_iterator it =
                output.poses.find(cameras[i].camera_name);
            if (it == output.poses.end()) {
                continue;
            }
            const FisheyePoseConfig& pose = it->second;
            ROS_INFO("[BevSurroundViewNode] %s t=[%.2f, %.2f, %.2f] R_euler=[%.2f, %.2f, %.2f] reproj=%.2fpx",
                     cameras[i].camera_name.c_str(),
                     pose.t_values.size() > 0 ? pose.t_values[0] : 0.0,
                     pose.t_values.size() > 1 ? pose.t_values[1] : 0.0,
                     pose.t_values.size() > 2 ? pose.t_values[2] : 0.0,
                     pose.r_values.size() > 0 ? pose.r_values[0] : 0.0,
                     pose.r_values.size() > 1 ? pose.r_values[1] : 0.0,
                     pose.r_values.size() > 2 ? pose.r_values[2] : 0.0,
                     pose.reproj_rmse_px);
        }
    }

    bool handleTriggerCalibration(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        bool success = false;
        double mean_reproj_rmse_px = std::numeric_limits<double>::infinity();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            success = has_best_calibration_ && best_calibration_output_.valid;
            if (success) {
                mean_reproj_rmse_px = best_calibration_output_.shared_reproj_rmse_px >= 0.0
                    ? best_calibration_output_.shared_reproj_rmse_px
                    : computeMeanReproj(best_calibration_output_);
            }
        }

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << (success ? "success" : "fail") << ", mean_reproj_rmse_px=";
        if (std::isfinite(mean_reproj_rmse_px)) {
            oss << mean_reproj_rmse_px;
        } else {
            oss << "inf";
        }

        response.success = success;
        response.message = oss.str();
        return true;
    }

    bool handleSaveCalibration(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        FisheyeAutoCalibrator::Output output;
        FisheyeAutoCalibrator::Options options;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (has_best_calibration_ == false) {
                response.success = false;
                response.message = "no calibration result available";
                return true;
            }
            output = best_calibration_output_;
            options = options_;
        }

        std::string yaml_error;
        if (calibrator_->savePoseYaml(cameras_, output, options, yaml_error) == false) {
            response.success = false;
            response.message = yaml_error;
            return true;
        }

        for (size_t i = 0; i < cameras_.size(); ++i) {
            const std::string& role = cameras_[i].role;
            const std::string& camera_name = cameras_[i].camera_name;
            const auto bin_it = bin_file_paths_.find(role);
            if (bin_it == bin_file_paths_.end() || bin_it->second.empty()) {
                continue;
            }

            cv::Mat map_x, map_y;
            if (!calibrator_->getIpmMaps(camera_name, map_x, map_y)) {
                ROS_WARN("[BevSurroundViewNode] failed to get IPM maps for %s", camera_name.c_str());
                continue;
            }

            FisheyeAutoCalibrator::IpmRoiInfo roi_info;
            if (!calibrator_->getIpmRoi(camera_name, roi_info)) {
                continue;
            }

            std::vector<float> vx, vy;
            if (!map_x.empty() && map_x.type() == CV_32F) {
                vx.assign((float*)map_x.datastart, (float*)map_x.dataend);
            }
            if (!map_y.empty() && map_y.type() == CV_32F) {
                vy.assign((float*)map_y.datastart, (float*)map_y.dataend);
            }

            serialize(vx, vy, bin_it->second,
                      roi_info.x, roi_info.y, roi_info.width, roi_info.height);
        }

        response.success = true;
        response.message = "calibration saved";
        return true;
    }

    bool handleResetCalibration(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            has_best_calibration_ = false;
            bev_maps_dirty_ = false;
            best_mean_reproj_rmse_px_ = std::numeric_limits<double>::infinity();
            best_calibration_output_ = FisheyeAutoCalibrator::Output();
        }

        publishStatus(true, "calibration reset");
        response.success = true;
        response.message = "calibration reset";
        return true;
    }

    void publishStatus(bool success, const std::string& message)
    {
        std_msgs::String msg;
        msg.data = (success ? "OK: " : "FAIL: ") + message;
        pub_status_.publish(msg);
    }

    void serialize(const std::vector<float>& vx,
                   const std::vector<float>& vy,
                   const std::string& filename,
                   int a, int b, int c, int d)
    {
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            ROS_WARN("[BevSurroundViewNode] cannot open file %s", filename.c_str());
            return;
        }
        file.write(reinterpret_cast<const char*>(&a), sizeof(a));
        file.write(reinterpret_cast<const char*>(&b), sizeof(b));
        file.write(reinterpret_cast<const char*>(&c), sizeof(c));
        file.write(reinterpret_cast<const char*>(&d), sizeof(d));

        size_t vx_size = vx.size();
        file.write(reinterpret_cast<const char*>(&vx_size), sizeof(vx_size));
        if (!vx.empty()) {
            file.write(reinterpret_cast<const char*>(vx.data()), vx.size() * sizeof(float));
        }

        size_t vy_size = vy.size();
        file.write(reinterpret_cast<const char*>(&vy_size), sizeof(vy_size));
        if (!vy.empty()) {
            file.write(reinterpret_cast<const char*>(vy.data()), vy.size() * sizeof(float));
        }
        file.close();
        ROS_INFO("[BevSurroundViewNode] serialized to %s", filename.c_str());
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    ros::Timer param_check_timer_;

    std::unique_ptr<FisheyeAutoCalibrator> calibrator_;
    FisheyeAutoCalibrator::Options options_;
    std::array<int, 3> rotation_order_ = {{2, 1, 0}};

    std::string status_topic_ = "/bev_surround_view/status";
    std::string fused_bev_topic_ = "/bev_surround_view/bev/fused";
    std::map<std::string, std::string> bev_topics_;
    std::map<std::string, std::string> debug_topics_;
    int status_queue_size_ = 10;
    int image_queue_size_ = 1;
    int subscribe_image_queue_size_ = 1;
    bool publisher_latch_ = true;
    double param_monitor_period_sec_ = 1.0;
    double worker_idle_sleep_sec_ = 0.01;
    double sync_tolerance_sec_ = 0.08;
    double publish_max_hz_ = 10.0;

    std::array<FisheyeAutoCameraConfig, 4> cameras_;
    std::array<ros::Subscriber, 4> subs_;
    std::array<CameraFrameCache, 4> latest_frames_;
    SyncedBatch latest_synced_batch_;
    uint64_t latest_synced_batch_version_ = 0;
    uint64_t calibration_consumed_version_ = 0;

    std::mutex mutex_;
    std::mutex calibrator_mutex_;
    bool stop_requested_ = false;

    std::thread calibration_thread_;
    std::thread bev_thread_;

    bool has_best_calibration_ = false;
    bool bev_maps_dirty_ = false;
    double best_mean_reproj_rmse_px_ = std::numeric_limits<double>::infinity();
    FisheyeAutoCalibrator::Output best_calibration_output_;

    std::array<ros::WallTime, 4> last_debug_publish_times_;
    ros::WallTime last_bev_publish_time_;

    std::map<std::string, cv::Mat> latest_map_x_;
    std::map<std::string, cv::Mat> latest_map_y_;

    ros::ServiceServer srv_trigger_calibration_;
    ros::ServiceServer srv_save_calibration_;
    ros::ServiceServer srv_reset_calibration_;

    ros::Publisher pub_status_;
    image_transport::Publisher pub_fused_bev_;
    std::map<std::string, image_transport::Publisher> pub_bev_;
    std::map<std::string, image_transport::Publisher> pub_debug_;
    std::map<std::string, std::string> bin_file_paths_;
};

}  // namespace bev_surround_view

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bev_surround_view_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        bev_surround_view::BevSurroundViewNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[bev_surround_view_node] fatal: %s", e.what());
        return 1;
    }
    return 0;
}
