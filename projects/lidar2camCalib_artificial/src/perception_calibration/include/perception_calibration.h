#ifndef PERCEPTION_CALIBRATION_H_
#define PERCEPTION_CALIBRATION_H_

#include <fstream>
#include <thread>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <deque>
#include <mutex>
#include <atomic>
#include <memory>

#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int32.h>
// #include "cameracalib_msgs/ivlidarcameracalib.h"

#include "util.h"
#include "dense_cloud_generator.h"
#include "plane_segmentation.h"

namespace calibration {

const int MSG_BUFFER_SIZE = 15;
static const double DEG2RAD = (1.0 / 180.0 * 3.1415926);

struct PointCloudWithT {
  PointCloudWithT(): is_valid(false) {}
  PointCloudWithT(const PointCloudPtr& _cloud)
    : cloud(_cloud), is_valid(false) {}
  PointCloudWithT(const PointCloudPtr& _cloud,
                  const bool _is_valid)
    : cloud(_cloud), is_valid(_is_valid) {}
  PointCloudWithT(const PointCloudPtr& _cloud,
                  const Eigen::Matrix4f& _t,
                  const bool _is_valid = true)
    : cloud(_cloud), is_valid(_is_valid), t(_t) {}

  double GetTime() const {
    return cloud->header.stamp * 1e-6;
  }

  PointCloudPtr cloud;
  bool is_valid;
  Eigen::Matrix4f t;
  double rotation[3];
};

struct calib6d {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Matrix<double, 3, 1> t;
  Eigen::Matrix<double, 4, 4> RtLidar2Cam;
  float J;
  calib6d() {
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    J = 0;
  }
};

class PerceptionCalibration
{
public:

  PerceptionCalibration(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), it_(nh), capture_snapshot_(false) {
    cloud_sub_ = nh_.subscribe("/lidar_points", 10,
                               &PerceptionCalibration::CallbackPointCloud, this);

    image_sub_front_ = it_.subscribe("/camera_image_raw", 1,
                                     &PerceptionCalibration::CallbackImage, this);
    camera_info_sub_ = nh_.subscribe("/usb_cam/camera_info", 1,
                                     &PerceptionCalibration::CallbackCameraInfo, this);

    std::string intrinsics_file, extrinsics_file;
    private_nh_.param("intrinsics_file", intrinsics_file, std::string("camera_intrinsics.yaml"));
    private_nh_.param("extrinsics_file", extrinsics_file, std::string("camera_extrinsics.yaml"));

    private_nh_.param("enable_snapshot", enable_snapshot_, true);
    private_nh_.param("enable_visualize", enable_visualize_, true);
    private_nh_.param("enable_recorder", enable_recorder_, false);
    // private_nh_.param("enable_capture", enable_capture_, false);
    private_nh_.param("static_model", static_model_, false);

    //avos lidar calib param
    ros::param::get("perception_lidar_x_offset_top_center", gd_avos_cali_velo1_xoffset);
    ros::param::get("perception_lidar_y_offset_top_center", gd_avos_cali_velo1_yoffset);
    ros::param::get("perception_lidar_z_offset_top_center", gd_avos_cali_velo1_zoffset);
    ros::param::get("perception_lidar_roll_top_center", gd_avos_cali_velo1_roll);
    ros::param::get("perception_lidar_pitch_top_center", gd_avos_cali_velo1_pitch);
    ros::param::get("perception_lidar_yaw_top_center", gd_avos_cali_velo1_yaw);

    getPackagePath("perception_calibration", config_.package_path);

    config_.user_data_path = config_.package_path + "/../user_data";
    config_.user_image_path = config_.user_data_path + "/Cam0/";
    config_.user_cloud_path = config_.user_data_path + "/scans/";
    config_.intrinsics_path = config_.package_path + "/params/" + intrinsics_file;
    config_.extrinsics_path = config_.package_path + "/params/" + extrinsics_file;

    // if (config_.user_image_path != "") {
    //   if (access(config_.user_image_path.c_str(), F_OK) == -1) {
    //     int ret0 = std::system(std::string("mkdir -p " + config_.user_image_path).c_str());
    //   }
    // }

    initCameraParams();

    initMask();

    // ParseCloudFromeScan();

    if (enable_snapshot_) {
      snapshot_thread.reset(
        new std::thread(&PerceptionCalibration::DataSnapshot, this));
    }
    if (enable_visualize_) {
      visualization_thread.reset(
        new std::thread(&PerceptionCalibration::VisualizeCloudImage, this));
    }
  }

  ~PerceptionCalibration() {
    if (snapshot_thread != nullptr) {
      snapshot_thread->join();
    }
    if (visualization_thread != nullptr) {
      visualization_thread->join();
    }
  }

  std::string Name() const {return "PerceptionCalibration";}


private:
  // callback
  void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
  void CallbackImage(const sensor_msgs::ImageConstPtr& msg);
  void CallbackCameraInfo(const sensor_msgs::CameraInfoPtr& msg) {}
  // thread
  void DataSnapshot();
  void VisualizeCloudImage();
  void KeyboardMonitor();
  // sync
  bool SynchronizeMultiSensor(cv_bridge::CvImagePtr& cvimage,
                              PointCloudPtr& cloud,
  std::function<bool(size_t)> cloud_is_capturable = [](size_t i) {return true;});
  // init param
  void initCameraParams();
  void initEgoCarMarker();
  void initMask();
  
  calib6d buildCalib6d(const double x, const double y, const double z,
                                  const double roll, const double pitch, const double yaw);
  Eigen::Matrix<double, 3, 3> getRotationMat(const float roll, const float pitch, const float yaw);

  // func
  bool EnableCapture(size_t index);
  bool CheckCaptureCloud(size_t index);
  cv::Mat MergeCloud2Image(const PointCloudPtr& cloud, const cv::Mat& image);
  void ParseCloudFromeScan();
  void SaveImage(const cv_bridge::CvImagePtr& cvimage, const std::string& filename);
  void SaveCloud(const PointCloudPtr& cloud, const std::string& filename);
  void ConvertCloud2XYZI(const PointCloudPtr& input_cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& matching_cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& original_cloud);

  void fixScanPrecision();

  cv::Mat UndistortImage(const cv::Mat& image) const;

  template <typename Scalar>
  Scalar ConvertIntensity2Scalar(const int i) const;

  template <typename Scalar>
  Scalar ConvertDistance2Scalar(const float dis) const;

  template <typename Scalar>
  Scalar GetRainbowColor(const int i) const;

  PointCloudWithT Interpolate(PointCloudWithT start,
                              PointCloudWithT end, const double time) const;

  template <typename Matrix>
  void GetRPY(const Matrix& t, double* r, double* p, double* y) const;

  template <typename T>
  void GetMeanStdDev(const std::vector<T>& data, T* input_mean, T* input_dev) const;

  void CoordinateTransform_inv(const PointCloudPtr& inputPcPointer, PointCloudPtr& outputPcPointer, 
      const double Roll, const double Pitch, const double Yaw, 
      const double x_offset, const double y_offset, const double z_offset);

private:
  ros::NodeHandle& nh_;
  ros::NodeHandle& private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber camera_info_sub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_front_;

  std::unique_ptr<std::thread> monitor_thread;
  std::unique_ptr<std::thread> snapshot_thread;
  std::unique_ptr<std::thread> visualization_thread;

  CalibrationConfig config_;
  bool enable_snapshot_;
  bool enable_visualize_;
  bool enable_recorder_;
  bool enable_capture_;
  bool static_model_;

  int cloud_count_ = 0;

  calib6d init_rot;

  double gd_avos_cali_velo1_xoffset = 0.0;
  double gd_avos_cali_velo1_yoffset = 0.0;
  double gd_avos_cali_velo1_zoffset = 0.0;
  double gd_avos_cali_velo1_roll = 0.0;
  double gd_avos_cali_velo1_pitch = 0.0;
  double gd_avos_cali_velo1_yaw = 0.0;

  int image_width, image_height;
  Eigen::Matrix<double, 3, 3> camera_intrinsic_;
  Eigen::Matrix<double, 5, 1> camera_distortion_;
  Eigen::Matrix<double, 6, 1> camera_6DoF_;

  Eigen::Matrix<double, 3, 3> R_L2Cam;
  Eigen::Matrix<double, 3, 1> T_L2Cam;
  Eigen::Matrix<double, 3, 3> R_World2RtCar;
  Eigen::Matrix<double, 3, 1> C_tilde_;
  Eigen::Matrix<double, 3, 3> M_Mat_inv_;
  Eigen::Matrix<double, 3, 3> M_Mat;
  Eigen::Matrix<double, 3, 1> p_4;
  Eigen::Matrix<double, 4, 4> lidar2camera;


  //用于Opencv滑块的整形变量
  int slider_x_,slider_y_,slider_z_;
  int slider_roll_,slider_pitch_,slider_yaw_;


  std::mutex image_mutex_, cloud_mutex_;
  std::atomic<bool> capture_snapshot_;

  std::deque<cv_bridge::CvImagePtr> image_buffer_;
  std::deque<PointCloudWithT> cloud_buffer_;
  std::deque<PointCloudWithT> position_buffer_;

  DenseCloudGenerator<pcl::PointXYZI> dcloud_gen_;
  PlaneSegmentation<Point> plane_seg_;
};
}  // namespace calibration
#endif // PERCEPTION_CALIBRATION_H_
