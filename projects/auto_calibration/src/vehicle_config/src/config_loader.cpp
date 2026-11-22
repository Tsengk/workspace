#include "vehicle_config/config_loader.h"
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace vehicle_config {

bool ConfigLoader::loadFromFile(const std::string& file_path, VehicleConfig& config) {
    try {
        YAML::Node root = YAML::LoadFile(file_path);

        if (!root["vehicle"]) {
            return false;
        }
        if (!loadVehicleInfo(root["vehicle"], config.vehicle)) {
            return false;
        }

        if (root["cameras"]) {
            config.cameras.clear();
            for (const auto& cam_node : root["cameras"]) {
                CameraInfo cam;
                if (loadCameraInfo(cam_node, cam)) {
                    config.cameras.push_back(cam);
                }
            }
        }

        if (root["lidars"]) {
            config.lidars.clear();
            for (const auto& lidar_node : root["lidars"]) {
                LidarInfo lidar;
                if (loadLidarInfo(lidar_node, lidar)) {
                    config.lidars.push_back(lidar);
                }
            }
        }

        if (root["radars"]) {
            config.radars.clear();
            for (const auto& radar_node : root["radars"]) {
                RadarInfo radar;
                if (loadRadarInfo(radar_node, radar)) {
                    config.radars.push_back(radar);
                }
            }
        }

        if (root["gnss"]) {
            loadGnssInfo(root["gnss"], config.gnss);
        }

        if (root["bev_surround_view"]) {
            loadBevConfig(root["bev_surround_view"], config.bev);
        }

        return true;
    } catch (const YAML::Exception& e) {
        return false;
    }
}

bool ConfigLoader::saveToFile(const std::string& file_path, const VehicleConfig& config) {
    try {
        YAML::Node root;
        YAML::Node vehicle_node;
        saveVehicleInfo(vehicle_node, config.vehicle);
        root["vehicle"] = vehicle_node;

        YAML::Node cameras_node;
        for (size_t i = 0; i < config.cameras.size(); ++i) {
            YAML::Node cam_node;
            saveCameraInfo(cam_node, config.cameras[i]);
            cameras_node.push_back(cam_node);
        }
        root["cameras"] = cameras_node;

        YAML::Node lidars_node;
        for (size_t i = 0; i < config.lidars.size(); ++i) {
            YAML::Node lidar_node;
            saveLidarInfo(lidar_node, config.lidars[i]);
            lidars_node.push_back(lidar_node);
        }
        root["lidars"] = lidars_node;

        YAML::Node radars_node;
        for (size_t i = 0; i < config.radars.size(); ++i) {
            YAML::Node radar_node;
            saveRadarInfo(radar_node, config.radars[i]);
            radars_node.push_back(radar_node);
        }
        root["radars"] = radars_node;

        YAML::Node gnss_node;
        saveGnssInfo(gnss_node, config.gnss);
        root["gnss"] = gnss_node;

        YAML::Node bev_node;
        saveBevConfig(bev_node, config.bev);
        root["bev_surround_view"] = bev_node;

        std::ofstream fout(file_path);
        fout << root;
        return true;
    } catch (const YAML::Exception& e) {
        return false;
    }
}

bool ConfigLoader::validate(const VehicleConfig& config, std::string& error_msg) {
    if (config.vehicle.id.empty()) {
        error_msg = "Vehicle ID cannot be empty";
        return false;
    }
    if (config.vehicle.car_length <= 0 || config.vehicle.car_width <= 0) {
        error_msg = "Vehicle dimensions must be positive";
        return false;
    }
    if (config.vehicle.z_offset <= 0) {
        error_msg = "z_offset (rear axle height) must be positive";
        return false;
    }

    if (config.cameras.empty()) {
        error_msg = "At least one camera must be configured";
        return false;
    }
    if (config.lidars.empty()) {
        error_msg = "At least one LiDAR must be configured";
        return false;
    }

    for (size_t i = 0; i < config.cameras.size(); ++i) {
        const auto& cam = config.cameras[i];
        if (cam.name.empty() || cam.topic.empty()) {
            error_msg = "Camera " + std::to_string(i) + ": name and topic cannot be empty";
            return false;
        }
    }

    for (size_t i = 0; i < config.lidars.size(); ++i) {
        const auto& lidar = config.lidars[i];
        if (lidar.name.empty() || lidar.topic.empty()) {
            error_msg = "LiDAR " + std::to_string(i) + ": name and topic cannot be empty";
            return false;
        }
    }

    for (size_t i = 0; i < config.radars.size(); ++i) {
        const auto& radar = config.radars[i];
        if (radar.name.empty() || radar.topic.empty()) {
            error_msg = "Radar " + std::to_string(i) + ": name and topic cannot be empty";
            return false;
        }
    }

    if (config.gnss.name.empty() || config.gnss.topic.empty()) {
        error_msg = "GNSS name and topic cannot be empty";
        return false;
    }

    error_msg.clear();
    return true;
}

Eigen::Matrix3d ConfigLoader::eulerToRotMatrix(const std::vector<double>& euler) {
    if (euler.size() != 3) {
        return Eigen::Matrix3d::Identity();
    }

    const double roll = euler[0] * M_PI / 180.0;
    const double pitch = euler[1] * M_PI / 180.0;
    const double yaw = euler[2] * M_PI / 180.0;

    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
               .toRotationMatrix();
}

std::vector<double> ConfigLoader::rotMatrixToEuler(const Eigen::Matrix3d& R) {
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0); // ZYX order
    return {euler[0] * 180.0 / M_PI, euler[1] * 180.0 / M_PI, euler[2] * 180.0 / M_PI};
}

bool ConfigLoader::loadVehicleInfo(const YAML::Node& node, VehicleInfo& info) {
    info.id = node["id"] ? node["id"].as<std::string>("") : "";
    info.car_length = node["car_length"] ? node["car_length"].as<double>(0) : 0;
    info.car_width = node["car_width"] ? node["car_width"].as<double>(0) : 0;
    info.car_height = node["car_height"] ? node["car_height"].as<double>(0) : 0;
    info.front_overhang = node["front_overhang"] ? node["front_overhang"].as<double>(0) : 0;
    info.wheelbase = node["wheelbase"] ? node["wheelbase"].as<double>(0) : 0;
    info.rear_overhang = node["rear_overhang"] ? node["rear_overhang"].as<double>(0) : 0;
    info.z_offset = node["z_offset"] ? node["z_offset"].as<double>(0) : 0;
    return true;
}

bool ConfigLoader::loadCameraIntrinsics(const YAML::Node& node, CameraIntrinsics& intrinsics) {
    if (node["new_K"]) {
        intrinsics.new_K = node["new_K"].as<std::vector<double>>();
    }
    if (node["new_Size"]) {
        intrinsics.new_Size = node["new_Size"].as<std::vector<int>>();
    }
    if (node["scaled_K"]) {
        intrinsics.scaled_K = node["scaled_K"].as<std::vector<double>>();
    }
    if (node["scaled_Size"]) {
        intrinsics.scaled_Size = node["scaled_Size"].as<std::vector<int>>();
    }
    if (node["K"]) {
        intrinsics.K = node["K"].as<std::vector<double>>();
    }
    if (node["Size"]) {
        intrinsics.Size = node["Size"].as<std::vector<int>>();
    }
    if (node["D"]) {
        intrinsics.D = node["D"].as<std::vector<double>>();
    }
    intrinsics.Epsilon = node["Epsilon"] ? node["Epsilon"].as<double>(0) : 0;
    return true;
}

bool ConfigLoader::loadCameraExtrinsics(const YAML::Node& node, CameraExtrinsics& extrinsics) {
    if (node["t"]) {
        extrinsics.t = node["t"].as<std::vector<double>>();
    }
    if (node["R_euler"]) {
        extrinsics.R_euler = node["R_euler"].as<std::vector<double>>();
    }
    extrinsics.vcs_to_ground_height = node["vcs_to_ground_height"] ?
        node["vcs_to_ground_height"].as<double>(0) : 0;
    extrinsics.base_euler = node["base_euler"] ? node["base_euler"].as<double>(0) : 0;
    return true;
}

bool ConfigLoader::loadCameraInfo(const YAML::Node& node, CameraInfo& camera) {
    camera.name = node["name"] ? node["name"].as<std::string>("") : "";
    camera.topic = node["topic"] ? node["topic"].as<std::string>("") : "";
    camera.intrinsics_file = node["intrinsics_file"] ?
        node["intrinsics_file"].as<std::string>("") : "";

    if (node["intrinsics"]) {
        loadCameraIntrinsics(node["intrinsics"], camera.intrinsics);
    }
    if (node["extrinsics"]) {
        loadCameraExtrinsics(node["extrinsics"], camera.extrinsics);
    }

    return true;
}

bool ConfigLoader::loadLidarInfo(const YAML::Node& node, LidarInfo& lidar) {
    lidar.name          = node["name"]          ? node["name"].as<std::string>("")          : "";
    lidar.packets_topic = node["packets_topic"] ? node["packets_topic"].as<std::string>("") : "";
    lidar.topic         = node["topic"]         ? node["topic"].as<std::string>("")         : "";
    lidar.launch_file   = node["launch_file"]   ? node["launch_file"].as<std::string>("")   : "";

    if (node["t"]) {
        lidar.t = node["t"].as<std::vector<double>>();
    }
    if (node["R"]) {
        lidar.R = node["R"].as<std::vector<double>>();
    }

    return true;
}

bool ConfigLoader::loadRadarInfo(const YAML::Node& node, RadarInfo& radar) {
    radar.name          = node["name"]          ? node["name"].as<std::string>("")          : "";
    radar.packets_topic = node["packets_topic"] ? node["packets_topic"].as<std::string>("") : "";
    radar.topic         = node["topic"]         ? node["topic"].as<std::string>("")         : "";
    radar.launch_file   = node["launch_file"]   ? node["launch_file"].as<std::string>("")   : "";

    if (node["t"]) {
        radar.t = node["t"].as<std::vector<double>>();
    }
    if (node["R"]) {
        radar.R = node["R"].as<std::vector<double>>();
    }

    return true;
}

bool ConfigLoader::loadGnssInfo(const YAML::Node& node, GnssInfo& gnss) {
    gnss.name = node["name"] ? node["name"].as<std::string>("") : "";
    gnss.topic = node["topic"] ? node["topic"].as<std::string>("") : "";
    return true;
}

void ConfigLoader::saveVehicleInfo(YAML::Node& node, const VehicleInfo& info) {
    node["id"] = info.id;
    node["car_length"] = info.car_length;
    node["car_width"] = info.car_width;
    node["car_height"] = info.car_height;
    node["front_overhang"] = info.front_overhang;
    node["wheelbase"] = info.wheelbase;
    node["rear_overhang"] = info.rear_overhang;
    node["z_offset"] = info.z_offset;
}

void ConfigLoader::saveCameraIntrinsics(YAML::Node& node, const CameraIntrinsics& intrinsics) {
    for (size_t i = 0; i < intrinsics.new_K.size(); ++i) {
        node["new_K"].push_back(intrinsics.new_K[i]);
    }
    for (size_t i = 0; i < intrinsics.new_Size.size(); ++i) {
        node["new_Size"].push_back(intrinsics.new_Size[i]);
    }
    for (size_t i = 0; i < intrinsics.scaled_K.size(); ++i) {
        node["scaled_K"].push_back(intrinsics.scaled_K[i]);
    }
    for (size_t i = 0; i < intrinsics.scaled_Size.size(); ++i) {
        node["scaled_Size"].push_back(intrinsics.scaled_Size[i]);
    }
    for (size_t i = 0; i < intrinsics.K.size(); ++i) {
        node["K"].push_back(intrinsics.K[i]);
    }
    for (size_t i = 0; i < intrinsics.Size.size(); ++i) {
        node["Size"].push_back(intrinsics.Size[i]);
    }
    for (size_t i = 0; i < intrinsics.D.size(); ++i) {
        node["D"].push_back(intrinsics.D[i]);
    }
    node["Epsilon"] = intrinsics.Epsilon;
}

void ConfigLoader::saveCameraExtrinsics(YAML::Node& node, const CameraExtrinsics& extrinsics) {
    for (size_t i = 0; i < extrinsics.t.size(); ++i) {
        node["t"].push_back(extrinsics.t[i]);
    }
    for (size_t i = 0; i < extrinsics.R_euler.size(); ++i) {
        node["R_euler"].push_back(extrinsics.R_euler[i]);
    }
    node["vcs_to_ground_height"] = extrinsics.vcs_to_ground_height;
    node["base_euler"] = extrinsics.base_euler;
}

void ConfigLoader::saveCameraInfo(YAML::Node& node, const CameraInfo& camera) {
    node["name"] = camera.name;
    node["topic"] = camera.topic;
    if (!camera.intrinsics_file.empty()) {
        node["intrinsics_file"] = camera.intrinsics_file;
    } else {
        YAML::Node intrinsics_node;
        saveCameraIntrinsics(intrinsics_node, camera.intrinsics);
        node["intrinsics"] = intrinsics_node;
    }
    YAML::Node extrinsics_node;
    saveCameraExtrinsics(extrinsics_node, camera.extrinsics);
    node["extrinsics"] = extrinsics_node;
}

void ConfigLoader::saveLidarInfo(YAML::Node& node, const LidarInfo& lidar) {
    node["name"] = lidar.name;
    if (!lidar.packets_topic.empty()) node["packets_topic"] = lidar.packets_topic;
    node["topic"] = lidar.topic;
    if (!lidar.launch_file.empty()) node["launch_file"] = lidar.launch_file;
    for (size_t i = 0; i < lidar.t.size(); ++i) {
        node["t"].push_back(lidar.t[i]);
    }
    for (size_t i = 0; i < lidar.R.size(); ++i) {
        node["R"].push_back(lidar.R[i]);
    }
}

void ConfigLoader::saveRadarInfo(YAML::Node& node, const RadarInfo& radar) {
    node["name"] = radar.name;
    if (!radar.packets_topic.empty()) node["packets_topic"] = radar.packets_topic;
    node["topic"] = radar.topic;
    if (!radar.launch_file.empty()) node["launch_file"] = radar.launch_file;
    for (size_t i = 0; i < radar.t.size(); ++i) {
        node["t"].push_back(radar.t[i]);
    }
    for (size_t i = 0; i < radar.R.size(); ++i) {
        node["R"].push_back(radar.R[i]);
    }
}

void ConfigLoader::saveGnssInfo(YAML::Node& node, const GnssInfo& gnss) {
    node["name"] = gnss.name;
    node["topic"] = gnss.topic;
}

void ConfigLoader::loadBevConfig(const YAML::Node& node, BevSurroundViewConfig& bev) {
    bev.node_ns                    = node["node_ns"]                    ? node["node_ns"].as<std::string>()                    : bev.node_ns;
    bev.bev_image_topic            = node["bev_image_topic"]            ? node["bev_image_topic"].as<std::string>()            : bev.bev_image_topic;
    bev.detection_vis_front_topic  = node["detection_vis_front_topic"]  ? node["detection_vis_front_topic"].as<std::string>()  : bev.detection_vis_front_topic;
    bev.detection_vis_rear_topic   = node["detection_vis_rear_topic"]   ? node["detection_vis_rear_topic"].as<std::string>()   : bev.detection_vis_rear_topic;
    bev.detection_vis_left_topic   = node["detection_vis_left_topic"]   ? node["detection_vis_left_topic"].as<std::string>()   : bev.detection_vis_left_topic;
    bev.detection_vis_right_topic  = node["detection_vis_right_topic"]  ? node["detection_vis_right_topic"].as<std::string>()  : bev.detection_vis_right_topic;
    bev.output_width               = node["output_width"]               ? node["output_width"].as<int>()                       : bev.output_width;
    bev.output_height              = node["output_height"]              ? node["output_height"].as<int>()                      : bev.output_height;
    bev.resolution                 = node["resolution"]                 ? node["resolution"].as<double>()                      : bev.resolution;
    bev.range_front                = node["range_front"]                ? node["range_front"].as<double>()                     : bev.range_front;
    bev.range_rear                 = node["range_rear"]                 ? node["range_rear"].as<double>()                      : bev.range_rear;
    bev.range_left                 = node["range_left"]                 ? node["range_left"].as<double>()                      : bev.range_left;
    bev.range_right                = node["range_right"]                ? node["range_right"].as<double>()                     : bev.range_right;
    bev.origin_offset_x            = node["origin_offset_x"]           ? node["origin_offset_x"].as<double>()                 : bev.origin_offset_x;
    bev.origin_offset_y            = node["origin_offset_y"]           ? node["origin_offset_y"].as<double>()                 : bev.origin_offset_y;
}

void ConfigLoader::saveBevConfig(YAML::Node& node, const BevSurroundViewConfig& bev) {
    node["node_ns"]                   = bev.node_ns;
    node["bev_image_topic"]           = bev.bev_image_topic;
    node["detection_vis_front_topic"] = bev.detection_vis_front_topic;
    node["detection_vis_rear_topic"]  = bev.detection_vis_rear_topic;
    node["detection_vis_left_topic"]  = bev.detection_vis_left_topic;
    node["detection_vis_right_topic"] = bev.detection_vis_right_topic;
    node["output_width"]              = bev.output_width;
    node["output_height"]             = bev.output_height;
    node["resolution"]                = bev.resolution;
    node["range_front"]               = bev.range_front;
    node["range_rear"]                = bev.range_rear;
    node["range_left"]                = bev.range_left;
    node["range_right"]               = bev.range_right;
    node["origin_offset_x"]           = bev.origin_offset_x;
    node["origin_offset_y"]           = bev.origin_offset_y;
}

} // namespace vehicle_config