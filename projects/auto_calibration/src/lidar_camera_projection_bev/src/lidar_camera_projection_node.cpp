#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Trigger.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>

#include "perception_fusion_common/camera_service/camera_service_util.h"
#include "perception_fusion_common/camera_service/camera_service_api.h"
// Camera index constants
#define CAM_FRONT 0
#define CAM_LEFT_FRONT 1
#define CAM_RIGHT_FRONT 2
namespace {
using namespace cv;

constexpr int kBevMapWith = 1280;
constexpr int kBevMapHeight = 480;
constexpr float kBevPixelsMeter = 15.0f;

bool endsWith(const std::string& value, const std::string& suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
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
        initBEVMap();
        applyBevExtrinsic();
        InitCameraBevMaps();
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
	private:
    // BEV map processing
    bool bev_map_ready_{false};
    std::shared_ptr<::perception::PerCameraService> cam_service_ptr;
    cv::Mat bevcanvas;
    cv::Size bevsize;
    float scale = kBevPixelsMeter;
    cv::Point2f cct;

    // IPM maps
    cv::Mat mapx_current_, mapy_current_;

    float filter_cloud_z = -0.5;
    size_t filter_cloud_r = 1;

bool initSingleCamera(const ::perception::CalibrationData& calibration_data,
                      const std::string& camera_name){
    if (!cam_service_ptr || camera_name.empty()) return false;
    cam_service_ptr->cam_names.assign(1, camera_name);
    cam_service_ptr->cam_utils.resize(1);
    std::map<std::string, ::perception::CalibrationData> calibration_datas;
    calibration_datas[camera_name] = calibration_data;
    return cam_service_ptr->cam_utils[0].LoadParams(calibration_datas, camera_name);
}

bool initBEVMap(){
   bevsize = cv::Size(kBevMapWith, kBevMapHeight);
   bevcanvas = cv::Mat::zeros(bevsize,CV_8UC3);
   cct = cv::Point2f(kBevMapWith/2,bevsize.height/2);
   scale = kBevPixelsMeter;
   cam_service_ptr.reset(new ::perception::PerCameraService());
   ::perception::CalibrationData calibration;
   if (!loadCameraCalibration(calib_file_, vcs_to_ground_height_, calibration)) {
       throw std::runtime_error("failed to load camera intrinsics: " + calib_file_);
   }
   bev_map_ready_ = initSingleCamera(calibration, current_camera_);
   if (!bev_map_ready_) {
       throw std::runtime_error("failed to initialize camera: " + current_camera_);
   }
   return bev_map_ready_;
}

bool loadCameraCalibration(const std::string& path,
                           double vcs_to_ground_height,
                           ::perception::CalibrationData& calibration)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open camera calib file: " + path);
        return false;
    }

    std::string model = "PINHOLE";
    fs["camera_model"] >> model;
    if (model.empty()) fs["model_type"] >> model;

    fs["image_width"] >> image_width_;
    fs["image_height"] >> image_height_;
    if (image_width_ <= 0 || image_height_ <= 0) return false;

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double epsilon_ = 0.0;
    cv::FileNode projection = fs["projection_parameters"];
    if (!projection.empty()) {
        projection["gamma1"] >> fx;
        projection["gamma2"] >> fy;
        projection["u0"] >> cx;
        projection["v0"] >> cy;
    } else {
        fs["fx"] >> fx;
        fs["fy"] >> fy;
        fs["cx"] >> cx;
        fs["cy"] >> cy;
    }
    if (fx <= 0.0 || fy <= 0.0) return false;

    cv::FileNode mirror = fs["mirror_parameters"];
    if (!mirror.empty()) mirror["xi"] >> epsilon_;
    else fs["xi"] >> epsilon_;

    std::array<double, 4> distortion{{0.0, 0.0, 0.0, 0.0}};
    cv::FileNode distortion_node = fs["distortion_parameters"];
    if (!distortion_node.empty()) {
        distortion_node["k1"] >> distortion[0];
        distortion_node["k2"] >> distortion[1];
        distortion_node["p1"] >> distortion[2];
        distortion_node["p2"] >> distortion[3];
    } else {
        fs["k1"] >> distortion[0];
        fs["k2"] >> distortion[1];
        fs["p1"] >> distortion[2];
        fs["p2"] >> distortion[3];
    }

    calibration.Size = {image_width_, image_height_};
    calibration.new_Size = calibration.Size;
    calibration.K = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    calibration.D = {distortion[0], distortion[1], distortion[2], distortion[3], 0.0};
    calibration.Epsilon = (model == "MEI" || model == "FISHEYE") ? epsilon_ : 0.0;
    calibration.t = {0.0, 0.0, 0.0};
    calibration.R_euler = {0.0, 0.0, 0.0};
    calibration.vcs_to_ground_height = vcs_to_ground_height;
    return true;
}

void InitCameraBevMaps(){
    const cv::Point2f ipmcct(bevsize.width/2, bevsize.height/2);
    const float ipmscale = kBevPixelsMeter;

    mapx_current_ = Mat(bevsize,CV_32F,-1);
    mapy_current_ = Mat(bevsize,CV_32F,-1);

    float vcs_to_ground_height = cam_service_ptr->cam_utils[CAM_FRONT].vcs_to_ground_height;
    for(int y=0;y<bevsize.height;y++) {
        for(int x=0;x<bevsize.width;x++) {
            Point3f pvcs((x-ipmcct.x)/ipmscale,(y-ipmcct.y)/ipmscale,-vcs_to_ground_height), pimg;
            if(cam_service_ptr->PointVcsToImg(CAM_FRONT, pvcs, pimg)) {
                mapx_current_.at<float>(y,x) = pimg.x;
                mapy_current_.at<float>(y,x) = pimg.y;
            }
        }
    }
}

void applyBevExtrinsic(){
    if(!cam_service_ptr || !bev_map_ready_) return;
    const int cam_index = cam_service_ptr->GetCameraByName(current_camera_);
    const std::vector<double> t3   = {tune_x_, tune_y_, tune_z_};
    const std::vector<double> eul3 = {tune_pitch_, tune_yaw_, tune_roll_};
    cam_service_ptr->SetCameraExtrinsic(cam_index, t3, eul3);
}

cv::Point2f vcspoint2canvas(float x_vcs, float y_vcs) {
    return cv::Point2f(x_vcs,y_vcs)*scale+cct;
}

// Intensity to rainbow color mapping (consistent with RVIZ)
cv::Vec3b intensityToJetColor(float intensity, float min_val, float max_val) {
    float normalized = (intensity - min_val) / (max_val - min_val);
    normalized = (normalized < 0.0f) ? 0.0f : (normalized > 1.0f) ? 1.0f : normalized;

    float r, g, b;
    if (normalized <= 0.25) {
        b = 1.0f;
        g = normalized * 4;
        r = 0.0f;
    } else if (normalized <= 0.5) {
        b = 1.0f - (normalized - 0.25) * 4;
        g = 1.0f;
        r = 0.0f;
    } else if (normalized <= 0.75) {
        b = 0.0f;
        g = 1.0f;
        r = (normalized - 0.5) * 4;
    } else {
        b = 0.0f;
        g = 1.0f - (normalized - 0.75) * 4;
        r = 1.0f;
    }
    return cv::Vec3b(
        static_cast<uchar>(b * 255),
        static_cast<uchar>(g * 255),
        static_cast<uchar>(r * 255)
    );
}
void bevDrawMainLidarCloud(cv::Mat &canvas, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2ConstIterator<int> iter_intensity(*cloud_msg, "intensity");

    int count = 0;
    const int min_intensity=20, max_intensity=60;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        if(count++%2 == 0) continue;
        if (*iter_z<filter_cloud_z) continue;
        cv::Point2f pim = vcspoint2canvas(*iter_x, *iter_y);
        if(pim.x >= 1 && pim.x < canvas.cols-1 && pim.y >= 1 && pim.y < canvas.rows-1){
            if(*iter_z > 99) continue;
            auto color = intensityToJetColor((int)(*iter_intensity), min_intensity, max_intensity);
            cv::circle(canvas, pim, filter_cloud_r, color, -1, cv::LINE_8);
        }
    }
}

// Build and publish merged point cloud (camera IPM ground points + lidar intensity points)
void buildAndPublishCloud(const cv::Mat& ipm_canvas,
                          const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                          const std_msgs::Header& header)
{
    pcl::PointCloud<pcl::PointXYZRGB> out;
    out.reserve(static_cast<size_t>(ipm_canvas.cols) * ipm_canvas.rows / 2 +
                (cloud_msg ? cloud_msg->width * cloud_msg->height : 0));

    // Camera IPM: non-black pixels → ground color points
    const cv::Point2f ipm_cct(ipm_canvas.cols / 2.0f, ipm_canvas.rows / 2.0f);
    const float ipm_scale = kBevPixelsMeter;
    for (int py = 0; py < ipm_canvas.rows; ++py) {
        const cv::Vec3b* row = ipm_canvas.ptr<cv::Vec3b>(py);
        for (int px = 0; px < ipm_canvas.cols; ++px) {
            const cv::Vec3b& bgr = row[px];
            if (bgr[0] + bgr[1] + bgr[2] < 6) continue;
            pcl::PointXYZRGB p;
            p.x = (static_cast<float>(px) - ipm_cct.x) / ipm_scale;
            p.y = (static_cast<float>(py) - ipm_cct.y) / ipm_scale;
            p.z = 0.0f;
            p.b = cv::saturate_cast<uchar>(bgr[0] * 2);
            p.g = cv::saturate_cast<uchar>(bgr[1] * 2);
            p.r = cv::saturate_cast<uchar>(bgr[2] * 2);
            out.push_back(p);
        }
    }

    // Lidar points: keep real (x,y,z), color by intensity
    if (cloud_msg && cloud_msg->width * cloud_msg->height > 0) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<int> iter_intensity(*cloud_msg, "intensity");
        const int min_intensity = 20, max_intensity = 60;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            if (*iter_z < filter_cloud_z) continue;
            if (*iter_z > 99) continue;
            const cv::Vec3b color =
                intensityToJetColor(static_cast<float>(*iter_intensity),
                                    min_intensity, max_intensity);
            pcl::PointXYZRGB p;
            p.x = *iter_x;
            p.y = *iter_y;
            p.z = *iter_z;
            p.b = color[0];
            p.g = color[1];
            p.r = color[2];
            out.push_back(p);
        }
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header = header;
    if (msg.header.frame_id.empty()) msg.header.frame_id = cloud_frame_id_;
    pub_cloud_.publish(msg);
}

void bevDrawIPMImage(cv::Mat &canvas, cv::Mat& img) {
    if(bevsize != Size(kBevMapWith, kBevMapHeight)) {
        bevsize = Size(kBevMapWith, kBevMapHeight);
        bevcanvas = Mat::zeros(bevsize,CV_8UC3);
    }
    cct = cv::Point2f(bevsize.width/2, bevsize.height/2);
    scale = kBevPixelsMeter;

    Mat camera_bev = Mat::zeros(mapx_current_.size(),CV_8UC3);

    cv::Mat imgi = img.clone();
    if(imgi.empty()) return;

    const int cam_img_width = cam_service_ptr->cam_utils[0].camera_width;
    const int cam_img_height = cam_service_ptr->cam_utils[0].camera_height;

    cv::resize(imgi,imgi,cv::Size(cam_img_width,cam_img_height),0,0,INTER_NEAREST);

    remap(imgi, camera_bev, mapx_current_, mapy_current_, INTER_AREA, BORDER_CONSTANT);
    canvas = camera_bev;
}

    void loadParams()
    {
        pnh_.param<std::string>("current_camera", current_camera_, "");
        pnh_.param<std::string>("calib_file", calib_file_, "");
        pnh_.param<std::string>("cameras_dist_file", cameras_dist_file_, "");
        pnh_.param<std::string>("image_topic", image_topic_, "/driver/gmsl_camera/front_fish_eye/compressed");
        pnh_.param<std::string>("lidar_topic", lidar_topic_, "/lidar_points");
        pnh_.param<std::string>("output_topic", output_topic_, "/lidar_camera_projection/image");
        pnh_.param<std::string>("output_cloud_topic", output_cloud_topic_, "/lidar_camera_projection_bev/cloud");
        pnh_.param<std::string>("cloud_frame_id", cloud_frame_id_, "base_link");
        pnh_.param<std::string>("output_dir", output_dir_, "/tmp/lidar_camera_projection");
        pnh_.param<std::string>("output/yaml_path", yaml_path_, "");
        pnh_.param<double>("publish_rate_hz", publish_rate_hz_, 10.0);
        pnh_.param<double>("sync_tolerance_sec", sync_tolerance_sec_, 0.10);
        pnh_.param<int>("sync_queue_size", sync_queue_size_, 10);
        pnh_.param<int>("display_width", display_width_, 960);
        pnh_.param<double>("vcs_to_ground_height", vcs_to_ground_height_, 0.0);

        double x, y, z, roll, pitch, yaw;
        pnh_.param<double>("x", x, 0.0);
        pnh_.param<double>("y", y, 0.0);
        pnh_.param<double>("z", z, 0.0);
        pnh_.param<double>("roll", roll, 0.0);
        pnh_.param<double>("pitch", pitch, 0.0);
        pnh_.param<double>("yaw", yaw, 0.0);
        tune_x_ = x; tune_y_ = y; tune_z_ = z;
        tune_roll_ = roll; tune_pitch_ = pitch; tune_yaw_ = yaw;

        if (calib_file_.empty()) {
            throw std::runtime_error("calib_file is empty");
        }
        if (cameras_dist_file_.empty()) {
            throw std::runtime_error("cameras_dist_file is empty");
        }
        if (current_camera_.empty()) {
            throw std::runtime_error("current_camera is empty");
        }
        if (publish_rate_hz_ <= 0.0) {
            throw std::runtime_error("publish_rate_hz must be > 0");
        }
        if (display_width_ <= 0) {
            throw std::runtime_error("display_width must be > 0");
        }
        if (sync_tolerance_sec_ <= 0.0) {
            throw std::runtime_error("sync_tolerance_sec must be > 0");
        }
        if (sync_queue_size_ <= 0) {
            throw std::runtime_error("sync_queue_size must be > 0");
        }
    }

    void setupPubSub()
    {
        // 图像与点云通过 message_filters ApproximateTime 做时间同步，
        // 同步误差上限由 sync_tolerance_sec 配置（launch 可改）。
        sub_lidar_mf_.subscribe(nh_, lidar_topic_, sync_queue_size_);

        if (endsWith(image_topic_, "/compressed")) {
            sub_compressed_mf_.subscribe(nh_, image_topic_, sync_queue_size_);
            sync_compressed_.reset(new CompressedSync(
                CompressedPolicy(sync_queue_size_), sub_compressed_mf_, sub_lidar_mf_));
            sync_compressed_->setMaxIntervalDuration(ros::Duration(sync_tolerance_sec_));
            sync_compressed_->registerCallback(
                boost::bind(&LidarCameraProjectionNode::onSyncedCompressed, this, _1, _2));
            ROS_INFO("[LidarCameraProjection] sync compressed image + lidar (tol=%.3fs, queue=%d)",
                     sync_tolerance_sec_, sync_queue_size_);
        } else {
            sub_raw_mf_.subscribe(nh_, image_topic_, sync_queue_size_);
            sync_raw_.reset(new RawSync(
                RawPolicy(sync_queue_size_), sub_raw_mf_, sub_lidar_mf_));
            sync_raw_->setMaxIntervalDuration(ros::Duration(sync_tolerance_sec_));
            sync_raw_->registerCallback(
                boost::bind(&LidarCameraProjectionNode::onSyncedRaw, this, _1, _2));
            ROS_INFO("[LidarCameraProjection] sync raw image + lidar (tol=%.3fs, queue=%d)",
                     sync_tolerance_sec_, sync_queue_size_);
        }

        pub_overlay_ = it_.advertise(output_topic_, 1);
        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
        srv_save_ = nh_.advertiseService("save", &LidarCameraProjectionNode::srvSave, this);
        ROS_INFO("[LidarCameraProjection] subscribe lidar: %s image: %s, publish overlay: %s, publish cloud: %s",
                 lidar_topic_.c_str(), image_topic_.c_str(),
                 output_topic_.c_str(), output_cloud_topic_.c_str());
    }

    void onSyncedRaw(const sensor_msgs::Image::ConstPtr& img_msg,
                     const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
            storeSyncedPair(cv_ptr->image.clone(), img_msg->header.stamp, cloud_msg);
        } catch (const cv_bridge::Exception& ex) {
            ROS_WARN_THROTTLE(1.0, "[LidarCameraProjection] raw image convert failed: %s", ex.what());
        }
    }

    void onSyncedCompressed(const sensor_msgs::CompressedImage::ConstPtr& img_msg,
                            const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        try {
            cv::Mat encoded(1,
                            static_cast<int>(img_msg->data.size()),
                            CV_8UC1,
                            const_cast<unsigned char*>(img_msg->data.data()));
            cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
            if (image.empty()) {
                ROS_WARN_THROTTLE(1.0, "[LidarCameraProjection] compressed image decode failed");
                return;
            }
            storeSyncedPair(image, img_msg->header.stamp, cloud_msg);
        } catch (const cv::Exception& ex) {
            ROS_WARN_THROTTLE(1.0, "[LidarCameraProjection] compressed image exception: %s", ex.what());
        }
    }

    // 缓存一对时间同步后的 图像/点云，供 onTimer 按发布频率取用
    void storeSyncedPair(const cv::Mat& image, const ros::Time& stamp,
                         const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        if (image.empty()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        latest_image_ = image;
        latest_image_stamp_ = stamp;
        latest_cloud_ = cloud_msg;
        has_image_ = true;
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

        // BEV: update camera extrinsic and rebuild IPM maps
        applyBevExtrinsic();
        InitCameraBevMaps();

        ROS_INFO("[LidarCameraProjection] BEV extrinsic updated: t=(%.4f, %.4f, %.4f) rpy_deg=(%.4f, %.4f, %.4f)",
                 x, y, z, roll, pitch, yaw);
    }

    void onTimer(const ros::TimerEvent&)
    {
        cv::Mat image;
        sensor_msgs::PointCloud2::ConstPtr cloud;
        ros::Time image_stamp;

        {
            // latest_image_/latest_cloud_ 已由 message_filters 保证时间同步
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_image_ || !latest_cloud_) {
                return;
            }
            image = latest_image_.clone();
            image_stamp = latest_image_stamp_;
            cloud = latest_cloud_;
        }

        cv::Mat outimg  = cv::Mat::zeros(bevsize,CV_8UC3);
        bevDrawIPMImage(outimg, image);

        // 发布合并点云（相机真彩色地面点 + 主雷达强度彩色点）供 GUI BEV 视图显示
        {
            std_msgs::Header cloud_header = cloud->header;
            if (!image_stamp.isZero()) cloud_header.stamp = image_stamp;
            buildAndPublishCloud(outimg, cloud, cloud_header);
        }

        cv::Mat overlay = renderOverlay(outimg, cloud);  //叠加后的 图像
        if (overlay.empty()) {
            return;
        }

        cv::Mat outoutimg;
        cv::resize(overlay,outoutimg,image.size(),0,0,INTER_NEAREST);

        std_msgs::Header header = cloud->header;
        if (!image_stamp.isZero()) {
            header.stamp = image_stamp;
        }
        pub_overlay_.publish(cv_bridge::CvImage(header, "bgr8", outoutimg).toImageMsg());
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

        cv::Mat outoverlay  = cv::Mat::zeros(bevsize,CV_8UC3);
        bevDrawMainLidarCloud(outoverlay,cloud_msg);
 
         cv::Mat outoutimg;
        cv::resize(outoverlay,outoutimg,base_image.size(),0,0,INTER_NEAREST);
        cv::Mat result;
        cv::addWeighted(base_image, 0.8, outoutimg, 0.3, 0.0, result, -1);

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

        std::string yaml_path = yaml_path_.empty() ? (dir + "bev_extrinsic.yaml") : yaml_path_;
        std::ofstream yfs(yaml_path);
        if (!yfs.is_open()) {
            res.success = false;
            res.message = "Failed to open " + yaml_path;
            return true;
        }

        yfs << std::fixed << std::setprecision(6);
        yfs << "# BEV Camera Extrinsic Parameters\n";
        yfs << "# Generated: " << tbuf << "\n";
        yfs << "# Camera: " << current_camera_ << "\n\n";
        yfs << "x: " << tune_x_ << "\n";
        yfs << "y: " << tune_y_ << "\n";
        yfs << "z: " << tune_z_ << "\n";
        yfs << "roll: " << tune_roll_ << "\n";
        yfs << "pitch: " << tune_pitch_ << "\n";
        yfs << "yaw: " << tune_yaw_ << "\n";
        yfs.close();
        ROS_INFO("[LidarCameraProjection] Saved BEV extrinsic to %s", yaml_path.c_str());

        res.success = true;
        res.message = "Saved to " + yaml_path;
        return true;
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    std::string current_camera_;
    std::string calib_file_;
    std::string cameras_dist_file_;
    std::string image_topic_;
    std::string lidar_topic_;
    std::string output_topic_;
    std::string output_cloud_topic_;
    std::string cloud_frame_id_;
    double publish_rate_hz_ = 10.0;
    double sync_tolerance_sec_ = 0.10;
    int sync_queue_size_ = 10;
    int display_width_ = 960;

    int image_width_ = 0;
    int image_height_ = 0;
    double vcs_to_ground_height_ = 0.0;

    // BEV extrinsic tuning parameters
    double tune_x_ = 0.0, tune_y_ = 0.0, tune_z_ = 0.0;
    double tune_roll_ = 0.0, tune_pitch_ = 0.0, tune_yaw_ = 0.0;

    std::mutex mutex_;
    cv::Mat latest_image_;
    ros::Time latest_image_stamp_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
    bool has_image_ = false;

    std::string output_dir_;
    std::string yaml_path_;

    // ---- message_filters 时间同步 ----
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::PointCloud2> RawPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> CompressedPolicy;
    typedef message_filters::Synchronizer<RawPolicy> RawSync;
    typedef message_filters::Synchronizer<CompressedPolicy> CompressedSync;

    message_filters::Subscriber<sensor_msgs::Image>           sub_raw_mf_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub_compressed_mf_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>     sub_lidar_mf_;
    std::shared_ptr<RawSync>        sync_raw_;
    std::shared_ptr<CompressedSync> sync_compressed_;

    image_transport::Publisher pub_overlay_;
    ros::Publisher pub_cloud_;
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
