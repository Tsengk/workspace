#include "calibration_gui/config_manager.h"
#include "vehicle_config/config_loader.h"
#include <algorithm>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace calibration_gui {

ConfigManager::ConfigManager() {}

bool ConfigManager::loadVehicleConfig(const std::string& file_path)
{
    vehicle_config::VehicleConfig tmp;
    if (!vehicle_config::ConfigLoader::loadFromFile(file_path, tmp)) {
        return false;
    }
    config_    = tmp;
    file_path_ = file_path;
    loaded_    = true;

    // 自动发现 intrinsics/<camera_name>.yaml，只记录路径，不解析内容
    const std::string dir = configDir();
    for (auto& cam : config_.cameras) {
        if (cam.intrinsics_file.empty()) {
            const std::string rel = "intrinsics/" + cam.name + ".yaml";
            std::ifstream f(dir + "/" + rel);
            if (f.good())
                cam.intrinsics_file = rel;
        }
    }

    return true;
}

std::string ConfigManager::configDir() const
{
    const size_t pos = file_path_.find_last_of("/\\");
    if (pos != std::string::npos) return file_path_.substr(0, pos);
    return ".";
}

std::vector<std::string> ConfigManager::lidarNames() const
{
    std::vector<std::string> names;
    for (const auto& l : config_.lidars) names.push_back(l.name);
    return names;
}

std::vector<std::string> ConfigManager::cameraNames() const
{
    std::vector<std::string> names;
    for (const auto& c : config_.cameras) names.push_back(c.name);
    return names;
}

std::vector<std::string> ConfigManager::radarNames() const
{
    std::vector<std::string> names;
    for (const auto& r : config_.radars) names.push_back(r.name);
    return names;
}

bool ConfigManager::getLidarInstall(const std::string& name,
                                     double& x, double& y, double& z,
                                     double& roll, double& pitch, double& yaw) const
{
    for (const auto& l : config_.lidars) {
        if (l.name == name) {
            x     = l.t.size() > 0 ? l.t[0] : 0.0;
            y     = l.t.size() > 1 ? l.t[1] : 0.0;
            z     = l.t.size() > 2 ? l.t[2] : 0.0;
            roll  = l.R.size() > 0 ? l.R[0] : 0.0;
            pitch = l.R.size() > 1 ? l.R[1] : 0.0;
            yaw   = l.R.size() > 2 ? l.R[2] : 0.0;
            return true;
        }
    }
    return false;
}

bool ConfigManager::getCameraInstall(const std::string& name,
                                      double& x, double& y, double& z,
                                      double& roll, double& pitch, double& yaw) const
{
    for (const auto& c : config_.cameras) {
        if (c.name == name) {
            x     = c.extrinsics.t.size() > 0 ? c.extrinsics.t[0] : 0.0;
            y     = c.extrinsics.t.size() > 1 ? c.extrinsics.t[1] : 0.0;
            z     = c.extrinsics.t.size() > 2 ? c.extrinsics.t[2] : 0.0;
            roll  = c.extrinsics.R_euler.size() > 0 ? c.extrinsics.R_euler[0] : 0.0;
            pitch = c.extrinsics.R_euler.size() > 1 ? c.extrinsics.R_euler[1] : 0.0;
            yaw   = c.extrinsics.R_euler.size() > 2 ? c.extrinsics.R_euler[2] : 0.0;
            return true;
        }
    }
    return false;
}

bool ConfigManager::getLidarRpProcessParams(const std::string& lidar_name,
                                            LidarRpProcessParams& params) const
{
    params = LidarRpProcessParams{};
    if (!loaded_ || lidar_name.empty()) return false;

    const std::string file = processDir() + "/lidar_rp_" + lidar_name + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;

    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node data = root;
        if (!root["calibrated"] && root.IsMap()) {
            for (auto it = root.begin(); it != root.end(); ++it) {
                if (it->second.IsMap()) {
                    data = it->second;
                    break;
                }
            }
        }

        YAML::Node calibrated = data["calibrated"];
        if (calibrated) {
            if (calibrated["z"]) {
                params.z = calibrated["z"].as<double>();
                params.has_z = true;
            }
            if (calibrated["roll_deg"]) {
                params.roll_deg = calibrated["roll_deg"].as<double>();
                params.has_roll = true;
            }
            if (calibrated["pitch_deg"]) {
                params.pitch_deg = calibrated["pitch_deg"].as<double>();
                params.has_pitch = true;
            }
        }

        if (!params.has_roll && data["roll_deg"]) {
            params.roll_deg = data["roll_deg"].as<double>();
            params.has_roll = true;
        }
        if (!params.has_pitch && data["pitch_deg"]) {
            params.pitch_deg = data["pitch_deg"].as<double>();
            params.has_pitch = true;
        }
        if (!params.has_z && data["z_m"]) {
            params.z = data["z_m"].as<double>();
            params.has_z = true;
        }
    } catch (const std::exception&) {
        return false;
    }

    return params.has_z || params.has_roll || params.has_pitch;
}

bool ConfigManager::getLidarSecondaryRpProcessParams(const std::string& lidar_name,
                                                     LidarSecondaryRpProcessParams& params) const
{
    params = LidarSecondaryRpProcessParams{};
    if (!loaded_ || lidar_name.empty()) return false;

    const std::string file = processDir() + "/secondary_rp_" + lidar_name + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;

    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node calibrated = root["calibrated"];
        if (calibrated) {
            if (calibrated["z"]) {
                params.z = calibrated["z"].as<double>();
                params.has_z = true;
            }
            if (calibrated["roll_deg"]) {
                params.roll_deg = calibrated["roll_deg"].as<double>();
                params.has_roll = true;
            }
            if (calibrated["pitch_deg"]) {
                params.pitch_deg = calibrated["pitch_deg"].as<double>();
                params.has_pitch = true;
            }
        }
    } catch (const std::exception&) {
        return false;
    }

    return params.has_z || params.has_roll || params.has_pitch;
}

bool ConfigManager::getLidarYawProcessParams(const std::string& lidar_name,
                                             LidarYawProcessParams& params) const
{
    params = LidarYawProcessParams{};
    if (!loaded_ || lidar_name.empty()) return false;

    const std::string file = processDir() + "/lidar_yaw_" + lidar_name + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;

    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node calibrated = root["calibrated"];
        if (calibrated && calibrated["yaw_deg"]) {
            params.yaw_deg = calibrated["yaw_deg"].as<double>();
            params.has_yaw = true;
        } else if (root["yaw_deg"]) {
            params.yaw_deg = root["yaw_deg"].as<double>();
            params.has_yaw = true;
        } else if (root["yaw_calib_deg"]) {
            params.yaw_deg = root["yaw_calib_deg"].as<double>();
            params.has_yaw = true;
        }
    } catch (const std::exception&) {
        return false;
    }

    return params.has_yaw;
}

bool ConfigManager::getLidarLidarProcessParams(const std::string& lidar_name,
                                               LidarLidarProcessParams& params) const
{
    params = LidarLidarProcessParams{};
    if (!loaded_ || lidar_name.empty()) return false;

    const std::string file = processDir() + "/lidar_lidar_" + lidar_name + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;

    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node calibrated = root["calibrated"];
        if (calibrated) {
            if (calibrated["x"]) {
                params.x = calibrated["x"].as<double>();
                params.has_x = true;
            }
            if (calibrated["y"]) {
                params.y = calibrated["y"].as<double>();
                params.has_y = true;
            }
            if (calibrated["yaw_deg"]) {
                params.yaw_deg = calibrated["yaw_deg"].as<double>();
                params.has_yaw = true;
            }
        }
    } catch (const std::exception&) {
        return false;
    }

    return params.has_x || params.has_y || params.has_yaw;
}

bool ConfigManager::getMergedLidarParams(const std::string& lidar_name,
                                         LidarMergedParams& params) const
{
    params = LidarMergedParams{};
    params.has_install = getLidarInstall(lidar_name,
                                         params.x, params.y, params.z,
                                         params.roll_deg, params.pitch_deg, params.yaw_deg);
    if (!params.has_install) return false;

    LidarRpProcessParams rp_params;
    if (getLidarRpProcessParams(lidar_name, rp_params)) {
        if (rp_params.has_z) {
            params.z = rp_params.z;
            params.z_from_process = true;
        }
        if (rp_params.has_roll) {
            params.roll_deg = rp_params.roll_deg;
            params.roll_from_process = true;
        }
        if (rp_params.has_pitch) {
            params.pitch_deg = rp_params.pitch_deg;
            params.pitch_from_process = true;
        }
    }

    LidarSecondaryRpProcessParams secondary_rp_params;
    if (getLidarSecondaryRpProcessParams(lidar_name, secondary_rp_params)) {
        if (secondary_rp_params.has_z) {
            params.z = secondary_rp_params.z;
            params.z_from_process = true;
        }
        if (secondary_rp_params.has_roll) {
            params.roll_deg = secondary_rp_params.roll_deg;
            params.roll_from_process = true;
        }
        if (secondary_rp_params.has_pitch) {
            params.pitch_deg = secondary_rp_params.pitch_deg;
            params.pitch_from_process = true;
        }
    }

    LidarYawProcessParams yaw_params;
    if (getLidarYawProcessParams(lidar_name, yaw_params) && yaw_params.has_yaw) {
        params.yaw_deg = yaw_params.yaw_deg;
        params.yaw_from_process = true;
    }

    LidarLidarProcessParams lidar_lidar_params;
    if (getLidarLidarProcessParams(lidar_name, lidar_lidar_params)) {
        if (lidar_lidar_params.has_x) {
            params.x = lidar_lidar_params.x;
            params.x_from_process = true;
        }
        if (lidar_lidar_params.has_y) {
            params.y = lidar_lidar_params.y;
            params.y_from_process = true;
        }
        if (lidar_lidar_params.has_yaw) {
            params.yaw_deg = lidar_lidar_params.yaw_deg;
            params.yaw_from_process = true;
        }
    }

    return true;
}

QString ConfigManager::getMergedLidarParamSourceText(const LidarMergedParams& params) const
{
    return QString("x: %1\ny: %2\nz: %3\nroll: %4\npitch: %5\nyaw: %6")
        .arg(params.x_from_process ? "已标定参数" : "安装参数")
        .arg(params.y_from_process ? "已标定参数" : "安装参数")
        .arg(params.z_from_process ? "已标定参数" : "安装参数")
        .arg(params.roll_from_process ? "已标定参数" : "安装参数")
        .arg(params.pitch_from_process ? "已标定参数" : "安装参数")
        .arg(params.yaw_from_process ? "已标定参数" : "安装参数");
}

bool ConfigManager::getRadarInstall(const std::string& name,
                                     double& x, double& y, double& z,
                                     double& roll, double& pitch, double& yaw) const
{
    for (const auto& r : config_.radars) {
        if (r.name == name) {
            x     = r.t.size() > 0 ? r.t[0] : 0.0;
            y     = r.t.size() > 1 ? r.t[1] : 0.0;
            z     = r.t.size() > 2 ? r.t[2] : 0.0;
            roll  = r.R.size() > 0 ? r.R[0] : 0.0;
            pitch = r.R.size() > 1 ? r.R[1] : 0.0;
            yaw   = r.R.size() > 2 ? r.R[2] : 0.0;
            return true;
        }
    }
    return false;
}

std::vector<std::string> ConfigManager::missingIntrinsics() const
{
    std::vector<std::string> missing;
    const std::string dir = configDir();
    for (const auto& cam : config_.cameras) {
        if (cam.intrinsics_file.empty()) {
            missing.push_back(cam.name);
            continue;
        }
        // 检查文件是否实际存在
        const std::string abs = (cam.intrinsics_file[0] == '/') ?
            cam.intrinsics_file : dir + "/" + cam.intrinsics_file;
        std::ifstream f(abs);
        if (!f.good())
            missing.push_back(cam.name);
    }
    return missing;
}

std::string ConfigManager::processDir() const
{
    if (!loaded_) return "";
    return configDir() + "/process";
}

std::string ConfigManager::templateDir() const
{
    if (!loaded_) return "";
    return configDir() + "/template";
}

std::string ConfigManager::getCameraIntrinsicsPath(const std::string& camera_name) const
{
    if (!loaded_) return "";
    for (const auto& cam : config_.cameras) {
        if (cam.name == camera_name) {
            if (cam.intrinsics_file.empty()) return "";
            if (cam.intrinsics_file[0] == '/') return cam.intrinsics_file;
            return configDir() + "/" + cam.intrinsics_file;
        }
    }
    return "";
}

bool ConfigManager::setCameraIntrinsicsFile(const std::string& camera_name,
                                             const std::string& file_path)
{
    if (!loaded_) return false;
    for (auto& cam : config_.cameras) {
        if (cam.name == camera_name) {
            const std::string dir = configDir() + "/";
            if (file_path.substr(0, dir.size()) == dir)
                cam.intrinsics_file = file_path.substr(dir.size());
            else
                cam.intrinsics_file = file_path;
            return true;
        }
    }
    return false;
}

std::string ConfigManager::getLidarLaunchFilePath(const std::string& lidar_name) const
{
    if (!loaded_) return "";
    for (const auto& l : config_.lidars) {
        if (l.name == lidar_name && !l.launch_file.empty()) {
            return configDir() + "/../../" + l.launch_file;
        }
    }
    return "";
}

std::string ConfigManager::getRadarLaunchFilePath(const std::string& radar_name) const
{
    if (!loaded_) return "";
    for (const auto& r : config_.radars) {
        if (r.name == radar_name && !r.launch_file.empty()) {
            return configDir() + "/../../" + r.launch_file;
        }
    }
    return "";
}

std::string ConfigManager::getLidarPacketsTopic(const std::string& lidar_name) const
{
    if (!loaded_) return "";
    for (const auto& l : config_.lidars) {
        if (l.name == lidar_name) return l.packets_topic;
    }
    return "";
}

std::string ConfigManager::getRadarPacketsTopic(const std::string& radar_name) const
{
    if (!loaded_) return "";
    for (const auto& r : config_.radars) {
        if (r.name == radar_name) return r.packets_topic;
    }
    return "";
}

std::string ConfigManager::getLidarPointCloudTopic(const std::string& lidar_name) const
{
    if (!loaded_) return "";
    for (const auto& l : config_.lidars) {
        if (l.name == lidar_name) return l.topic;
    }
    return "";
}

std::string ConfigManager::getRadarPointCloudTopic(const std::string& radar_name) const
{
    if (!loaded_) return "";
    for (const auto& r : config_.radars) {
        if (r.name == radar_name) return r.topic;
    }
    return "";
}

} // namespace calibration_gui
