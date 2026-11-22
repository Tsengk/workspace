#ifndef IPM_HPP
#define IPM_HPP
#include <string>
#include <opencv2/core.hpp>
#include <unordered_map>
#include "camera_utils.h"
class Ipm
{
private:
    const std::string _cfg_path;
    std::unordered_map<std::string, avos::perception::CameraUtils> _cameras;

public:
    Ipm(const std::string &cfg_path);
    ~Ipm();

public:
    bool get_bev_map(const std::string &camera_name, const size_t width, const size_t height, const size_t ipm_scale, cv::Mat &map_x, cv::Mat &map_y, const double z_offset = 0, const int shift_x = 0, const int shift_y = 0);
    bool get_cameras(std::unordered_map<std::string, avos::perception::CameraUtils> &cameras)
    {
        // return _cameras;
        cameras = _cameras;
        return true;
    }

    bool getCameraNames(std::vector<std::string> &names) const
    {
        names.clear(); // 清除旧内容，确保输出干净
        for (const auto &pair : _cameras)
        {
            names.push_back(pair.first);
        }
        return true;
    }

    bool setCameraExParam(const std::string &name, const std::vector<float> &r, const std::vector<float> &t)
    {
        auto it = _cameras.find(name);
        if (it == _cameras.end())
        {
            std::cerr << "Camera \"" << name << "\" not found.\n";
            return false;
        }
        return it->second.SetR(r) && it->second.SetT(t);
    }

    bool setCameraZOffset(const std::string &name,const float z){
        auto it = _cameras.find(name);
        if (it == _cameras.end())
        {
            std::cerr << "Camera \"" << name << "\" not found.\n";
            return false;
        }
        return it->second.SetZOffset(z);
    }
};

#endif