#include "ipm.hpp"
#include "camera_utils.h"

Ipm::Ipm(const std::string &cfg_path) : _cfg_path(cfg_path)
{
    // load
    YAML::Node config = YAML::LoadFile(_cfg_path.c_str());
    size_t index = 0;
    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        avos::perception::CameraUtils camera_utils;
        camera_utils.LoadCameraParams(index, it->second);
        _cameras[key] = camera_utils;
        index++;
        std::cout << "[debug] Load Camera Key: " << key << std::endl;
    }
}

Ipm::~Ipm()
{
}

bool Ipm::get_bev_map(const std::string &camera_name, const size_t width, const size_t height, const size_t ipm_scale, cv::Mat &map_x, cv::Mat &map_y, const double z_offset, const int shift_x, const int shift_y)
{
    auto it = _cameras.find(camera_name);
    if (it == _cameras.end())
    {
        std::cout << "[debug] cannot load camera: " << camera_name << std::endl;
        return false;
    }
    auto camera = it->second;

    auto bev_size = cv::Size(width, height);
    map_x = cv::Mat(bev_size, CV_32F, -1);
    map_y = cv::Mat(bev_size, CV_32F, -1);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            cv::Point3f pvcs(-(y - bev_size.height / 2), -(x - bev_size.width / 2), z_offset), pimg;
            pvcs.x += shift_x;
            pvcs.y += shift_y;
            pvcs.x /= ipm_scale;
            pvcs.y /= ipm_scale;
            camera.ConvertPointVCS2IMG(pvcs, pimg);
            if (pimg.x == 0 || pimg.y == 0)
            {
                pimg.x = -1;
                pimg.y == -1;
            }

            map_x.at<float>(y, x) = pimg.x;
            map_y.at<float>(y, x) = pimg.y;
        }
    }
    return true;
}
