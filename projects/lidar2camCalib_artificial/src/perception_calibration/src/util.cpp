#include "util.h"

#include <yaml-cpp/yaml.h>
#include <iomanip>

namespace calibration {

//execute shell command
int myexec(const char *cmd, std::vector<std::string> &resvec) {
  resvec.clear();
  FILE *pp = popen(cmd, "r"); //open pipeline
  if (!pp) {
    return -1;
  }
  char tmp[1024];
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0'; //remove line break
    }
    resvec.push_back(tmp);
  }
  pclose(pp); //close pipeline
  return resvec.size();
}

bool getPackagePath(const std::string& package_name, std::string &res) {
  std::vector<std::string> resvec;
  std::string cmd = "rospack find " + package_name;
  myexec(cmd.c_str(), resvec);
  if (resvec.size() == 0)
    return false;
  res = resvec[0];
  return true;
}

Parser::Parser(const std::string& file): file_(file) {}

Parser::~Parser() {}

void Parser::setPath(const std::string& file) {
  file_ = file;
}
bool Parser::ParseFromCSV(std::vector<std::vector<float>>& data,
                          const char separator, const bool ignore_title) const {
  std::ifstream ifs(file_);
  if (!ifs.is_open()) {
    std::cerr << "Parser: opening filed. check if " << file_ << " exists." << std::endl;
    return false;
  } else {
    std::cout << "Parser: parsing " << file_ << std::flush << std::endl;
  }
  if (ignore_title) {
    std::string title_line;
    std::getline(ifs, title_line);
  }
  std::string line;
  while (std::getline(ifs, line)) {
    std::vector<float> line_data;
    ParseCSVLine(line_data, line, separator);
    data.push_back(line_data);
  }
  ifs.close();
  //
  std::cout << "Parser: " << data.size() << " points parsed." << std::endl;
  return true;
}
bool Parser::ParseFromCSV(pcl::PointCloud<pcl::PointXYZI>& cloud,
                          const char separator, const bool ignore_title) const {
  std::ifstream ifs(file_);
  if (!ifs.is_open()) {
    std::cerr << "Parser: opening filed. check if " << file_ << " exists." << std::endl;
    return false;
  } else {
    std::cout << "Parser: parsing " << file_ << std::flush << std::endl;
  }
  if (ignore_title) {
    std::string title_line;
    std::getline(ifs, title_line);
  }
  std::string line;
  while (std::getline(ifs, line)) {
    pcl::PointXYZI p;
    std::vector<float> line_data;
    ParseCSVLine(line_data, line, separator);
    p.x = line_data[0];
    p.y = line_data[1];
    p.z = line_data[2];
    p.intensity = line_data[3];
    cloud.push_back(p);
  }
  ifs.close();
  cloud.height = 1;
  cloud.width = cloud.size();
  cloud.header.frame_id = "base_link";
  //
  std::cout << "Parser: " << cloud.size() << " points parsed." << std::endl;
  return true;
}
void Parser::ParseCSVLine(std::vector<float>& data,
                          const std::string& line, const char separator) const {
  std::stringstream ss(line);
  std::string subline;
  while (std::getline(ss, subline, separator)) {
    float t = 0;
    std::stringstream(subline) >> t;
    data.push_back(t);
  }
}
bool Parser::ParseToCSV(const std::vector<std::vector<float>>& data,
                        const char separator, const std::string& title) const {
  std::ofstream ofs(file_);
  if (!ofs.is_open()) {
    std::cerr << "Parser: parsing filed. check if " << file_ << " is valid." << std::endl;
    return false;
  } else {
    std::cout << "Parser: parsing " << data.size() << " points to " << file_ << std::flush << std::endl;
  }
  if (!title.empty()) {
    ofs << title << std::endl;
  }
  for (const auto& line_data : data) {
    if (!line_data.empty()) {
      for (size_t i = 0; i < line_data.size() - 1; ++i) {
        ofs << std::fixed << std::setprecision(6) << line_data[i] << separator;
      }
      ofs << static_cast<int>(line_data.back()) << std::endl;
    }
  }
  ofs.close();
  return true;
}
bool Parser::ParseToCSV(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                        const char separator, const std::string& title) const {
  std::ofstream ofs(file_);
  if (!ofs.is_open()) {
    std::cerr << "Parser: parsing filed. check if " << file_ << " is valid." << std::endl;
    return false;
  } else {
    std::cout << "Parser: parsing " << cloud.size() << " points to " << file_ << std::flush << std::endl;
  }
  if (!title.empty()) {
    ofs << title << std::endl;
  }
  for (const auto& p : cloud.points) {
    ofs << std::fixed << std::setprecision(6)
        << p.x << separator
        << p.y << separator
        << p.z << separator
        << static_cast<int>(p.intensity) << std::endl;
  }
  ofs.close();
  return true;
}

bool Parser::ParseCameraInfoFromYaml(int* width, int*height,
                                     Eigen::Matrix<double, 3, 3>* intrinsic,
                                     Eigen::Matrix<double, 5, 1>* distortion) const {
  std::ifstream ifs(file_);
  if (!ifs.is_open()) {
    std::cerr << "Parser: opening filed. check if " << file_ << " exists." << std::endl;
    return false;
  } else {
    std::cout << "Parser: parsing yaml " << file_ << std::flush << std::endl;
  }
  ifs.close();

  try {
    YAML::Node doc;
    doc = YAML::LoadFile(file_);

    *width = doc["image_width"].as<int>();
    *height = doc["image_height"].as<int>();

    if (intrinsic != nullptr) {
      const auto& camera_matrix = doc["camera_matrix"];
      for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
          (*intrinsic)(i, j) = camera_matrix["data"][i * 3 + j].as<double>();
        }
      }
    }

    if (distortion != nullptr) {
      const auto& distortion_coefficients = doc["distortion_coefficients"];
      for (size_t i = 0; i < 5; ++i) {
        (*distortion)(i) = distortion_coefficients["data"][i].as<double>();
      }
    }
  } catch (YAML::Exception& e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    return false;
  }

  return true;
}

bool Parser::ParseCamera6DofFromYaml(Eigen::Matrix<double, 6, 1>* camera_6DoF) const {
  std::ifstream ifs(file_);
  if (!ifs.is_open()) {
    std::cerr << "Parser: opening filed. check if " << file_ << " exists." << std::endl;
    return false;
  } else {
    std::cout << "Parser: parsing yaml " << file_ << std::flush << std::endl;
  }
  ifs.close();

  try {
    YAML::Node doc;
    doc = YAML::LoadFile(file_);
    const auto& node = doc["but_calibration_camera_velodyne"];
    for (size_t i = 0; i < 6; ++i) {
      (*camera_6DoF)(i) = node["6DoF"][i].as<double>();
    }
  } catch (YAML::Exception& e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    return false;
  }

  return true;
}

}  // namespace calibration
