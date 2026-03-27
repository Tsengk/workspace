#ifndef PERCEPTION_CALIBRATION_UTIL_H_
#define PERCEPTION_CALIBRATION_UTIL_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include "pcl_types.h"

namespace calibration {

using avos::perception_calib::pcl_util::Point;
using avos::perception_calib::pcl_util::PointCloud;
using avos::perception_calib::pcl_util::PointCloudPtr;

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::duration;

#define TIME_PROFILING_INIT(timer)                                             \
  static double timer##time = 0;                                               \
  static int timer##count = 0;                                                 \
  steady_clock::time_point timer##init = steady_clock::now();                  \

#define TIME_PROFILING_OUT(timer)                                              \
  double timer##duration =                                                     \
    (duration_cast<duration<double>>(                                          \
      steady_clock::now() - timer##init)).count() * 1000;                      \
  timer##time += timer##duration;                                              \
  ++timer##count;                                                              \
  ROS_ERROR_STREAM(Name() << " "#timer                                         \
                   << "\n\taverage time(ms) : " << timer##time / timer##count  \
                   << "\tcount : " << timer##count                             \
                   << "\tcurrent time(ms) : " << timer##duration);             \


int myexec(const char* cmd, std::vector<std::string>& resvec);
bool getPackagePath(const std::string& package_name, std::string& res);

struct CalibrationConfig {
  std::string package_path;
  std::string user_data_path;
  std::string user_image_path;
  std::string user_cloud_path;
  std::string intrinsics_path;
  std::string extrinsics_path;
};

class Parser {
public:
  explicit Parser(const std::string& file);
  ~Parser();

  void setPath(const std::string& file);

  bool ParseFromCSV(std::vector<std::vector<float>>& data,
                    const char separator, const bool ignore_title) const;
  bool ParseFromCSV(pcl::PointCloud<pcl::PointXYZI>& cloud,
                    const char separator, const bool ignore_title) const;
  bool ParseToCSV(const std::vector<std::vector<float>>& data,
                  const char separator, const std::string& title) const;
  bool ParseToCSV(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                  const char separator, const std::string& title) const;
  bool ParseCameraInfoFromYaml(int* width, int*height,
                               Eigen::Matrix<double, 3, 3>* intrinsic = nullptr,
                               Eigen::Matrix<double, 5, 1>* distortion = nullptr) const;
  bool ParseCamera6DofFromYaml(Eigen::Matrix<double, 6, 1>* camera_6DoF) const;

private:
  void ParseCSVLine(std::vector<float>& data,
                    const std::string& line, const char separator) const;
private:
  std::string file_;
};

}  // namespace calibration
#endif  // PERCEPTION_CALIBRATION_UTIL_H_ 
