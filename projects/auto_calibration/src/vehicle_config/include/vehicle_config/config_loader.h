#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "vehicle_config/types.h"

namespace vehicle_config {

class ConfigLoader {
public:
    /// Load vehicle configuration from YAML file
    /// @param file_path Path to YAML configuration file
    /// @param config Output configuration structure
    /// @return true if successful, false otherwise
    static bool loadFromFile(const std::string& file_path, VehicleConfig& config);

    /// Save vehicle configuration to YAML file
    /// @param file_path Path to output YAML file
    /// @param config Configuration to save
    /// @return true if successful, false otherwise
    static bool saveToFile(const std::string& file_path, const VehicleConfig& config);

    /// Validate configuration
    /// @param config Configuration to validate
    /// @param error_msg Output error message if validation fails
    /// @return true if valid, false otherwise
    static bool validate(const VehicleConfig& config, std::string& error_msg);

    /// Convert camera extrinsics to rotation matrix
    /// @param euler ZYX Euler angles [roll, pitch, yaw] in degrees
    /// @return 3x3 rotation matrix
    static Eigen::Matrix3d eulerToRotMatrix(const std::vector<double>& euler);

    /// Convert rotation matrix to ZYX Euler angles
    /// @param R 3x3 rotation matrix
    /// @return ZYX Euler angles [roll, pitch, yaw] in degrees
    static std::vector<double> rotMatrixToEuler(const Eigen::Matrix3d& R);

    /// Load camera intrinsics from YAML node (public for external use)
    static bool loadCameraIntrinsics(const YAML::Node& node, CameraIntrinsics& intrinsics);

private:
    /// Load vehicle info from YAML node
    static bool loadVehicleInfo(const YAML::Node& node, VehicleInfo& info);

    /// Load camera extrinsics from YAML node
    static bool loadCameraExtrinsics(const YAML::Node& node, CameraExtrinsics& extrinsics);

    /// Load camera info from YAML node
    static bool loadCameraInfo(const YAML::Node& node, CameraInfo& camera);

    /// Load LiDAR info from YAML node
    static bool loadLidarInfo(const YAML::Node& node, LidarInfo& lidar);

    /// Load radar info from YAML node
    static bool loadRadarInfo(const YAML::Node& node, RadarInfo& radar);

    /// Load GNSS info from YAML node
    static bool loadGnssInfo(const YAML::Node& node, GnssInfo& gnss);

    /// Load BEV surround view config from YAML node
    static void loadBevConfig(const YAML::Node& node, BevSurroundViewConfig& bev);

    /// Save vehicle info to YAML node
    static void saveVehicleInfo(YAML::Node& node, const VehicleInfo& info);

    /// Save camera intrinsics to YAML node
    static void saveCameraIntrinsics(YAML::Node& node, const CameraIntrinsics& intrinsics);

    /// Save camera extrinsics to YAML node
    static void saveCameraExtrinsics(YAML::Node& node, const CameraExtrinsics& extrinsics);

    /// Save camera info to YAML node
    static void saveCameraInfo(YAML::Node& node, const CameraInfo& camera);

    /// Save LiDAR info to YAML node
    static void saveLidarInfo(YAML::Node& node, const LidarInfo& lidar);

    /// Save radar info to YAML node
    static void saveRadarInfo(YAML::Node& node, const RadarInfo& radar);

    /// Save GNSS info to YAML node
    static void saveGnssInfo(YAML::Node& node, const GnssInfo& gnss);

    /// Save BEV surround view config to YAML node
    static void saveBevConfig(YAML::Node& node, const BevSurroundViewConfig& bev);
};

} // namespace vehicle_config