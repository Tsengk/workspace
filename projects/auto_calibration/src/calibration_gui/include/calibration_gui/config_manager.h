#pragma once
#include <string>
#include <vector>
#include <QString>
#include "vehicle_config/types.h"

namespace calibration_gui {

/// 管理车辆配置文件加载与缓存
class ConfigManager {
public:
    struct LidarRpProcessParams {
        bool has_z{false};
        bool has_roll{false};
        bool has_pitch{false};
        double z{0.0};
        double roll_deg{0.0};
        double pitch_deg{0.0};
    };

    struct LidarSecondaryRpProcessParams {
        bool has_z{false};
        bool has_roll{false};
        bool has_pitch{false};
        double z{0.0};
        double roll_deg{0.0};
        double pitch_deg{0.0};
    };

    struct LidarYawProcessParams {
        bool has_yaw{false};
        double yaw_deg{0.0};
    };

    struct LidarLidarProcessParams {
        bool has_x{false};
        bool has_y{false};
        bool has_yaw{false};
        double x{0.0};
        double y{0.0};
        double yaw_deg{0.0};
    };

    struct LidarMergedParams {
        bool has_install{false};
        bool x_from_process{false};
        bool y_from_process{false};
        bool z_from_process{false};
        bool roll_from_process{false};
        bool pitch_from_process{false};
        bool yaw_from_process{false};
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double roll_deg{0.0};
        double pitch_deg{0.0};
        double yaw_deg{0.0};
    };

    ConfigManager();

    /// 从 YAML 文件加载车辆整体配置
    bool loadVehicleConfig(const std::string& file_path);

    /// 获取已加载的配置
    const vehicle_config::VehicleConfig& getConfig() const { return config_; }

    /// 是否已加载
    bool isLoaded() const { return loaded_; }

    /// 配置文件路径
    const std::string& filePath() const { return file_path_; }

    /// 配置文件所在目录（用于查找独立内参文件）
    std::string configDir() const;

    /// 获取 LiDAR 名称列表
    std::vector<std::string> lidarNames() const;

    /// 获取相机名称列表
    std::vector<std::string> cameraNames() const;

    /// 获取雷达名称列表
    std::vector<std::string> radarNames() const;

    /// 获取 LiDAR 安装参数（按名称）
    bool getLidarInstall(const std::string& name,
                         double& x, double& y, double& z,
                         double& roll, double& pitch, double& yaw) const;

    /// 获取相机安装参数（按名称）
    bool getCameraInstall(const std::string& name,
                          double& x, double& y, double& z,
                          double& roll, double& pitch, double& yaw) const;

    /// 获取毫米波雷达安装参数（按名称）
    bool getRadarInstall(const std::string& name,
                         double& x, double& y, double& z,
                         double& roll, double& pitch, double& yaw) const;

    /// 读取 process/lidar_rp_<lidar>.yaml 中已标定的 z/roll/pitch
    bool getLidarRpProcessParams(const std::string& lidar_name,
                                  LidarRpProcessParams& params) const;

    /// 读取 process/secondary_rp_<lidar>.yaml 中已标定的 z/roll/pitch
    bool getLidarSecondaryRpProcessParams(const std::string& lidar_name,
                                           LidarSecondaryRpProcessParams& params) const;

    /// 读取 process/lidar_yaw_<lidar>.yaml 中已标定的 yaw
    bool getLidarYawProcessParams(const std::string& lidar_name,
                                   LidarYawProcessParams& params) const;

    /// 读取 process/lidar_lidar_<lidar>.yaml 中已标定的 x/y/yaw
    bool getLidarLidarProcessParams(const std::string& lidar_name,
                                     LidarLidarProcessParams& params) const;

    /// 合并安装参数与 process 中已有标定结果
    bool getMergedLidarParams(const std::string& lidar_name,
                               LidarMergedParams& params) const;

    /// 生成激光雷达参数来源说明文本
    QString getMergedLidarParamSourceText(const LidarMergedParams& params) const;

    /// 获取 z_offset（后轴中心到地面高度）
    double zOffset() const { return config_.vehicle.z_offset; }

    /// 获取 BEV 环视标定节点 topic 配置
    const vehicle_config::BevSurroundViewConfig& getBevConfig() const { return config_.bev; }

    /// 返回内参缺失的相机名列表（文件不存在视为缺失）
    std::vector<std::string> missingIntrinsics() const;

    /// 返回 process/ 目录路径（configDir()/process），未加载时返回空串
    std::string processDir() const;
       /// 返回 template/ 目录路径（configDir()/template），未加载时返回空串
    std::string templateDir() const;

    /// 获取相机内参文件绝对路径（未设置时返回空串）
    std::string getCameraIntrinsicsPath(const std::string& camera_name) const;

    /// 设置相机内参文件路径（只记录路径，不解析内容）
    bool setCameraIntrinsicsFile(const std::string& camera_name,
                                  const std::string& file_path);

    /// 获取 LiDAR driver launch 文件绝对路径（launch_file 为空时返回空串）
    std::string getLidarLaunchFilePath(const std::string& lidar_name) const;

    /// 获取 Radar driver launch 文件绝对路径（launch_file 为空时返回空串）
    std::string getRadarLaunchFilePath(const std::string& radar_name) const;

    /// 获取 LiDAR packets_topic
    std::string getLidarPacketsTopic(const std::string& lidar_name) const;

    /// 获取 Radar packets_topic
    std::string getRadarPacketsTopic(const std::string& radar_name) const;

    /// 获取 LiDAR point_cloud_topic (the topic field)
    std::string getLidarPointCloudTopic(const std::string& lidar_name) const;

    /// 获取 Radar point_cloud_topic (the topic field)
    std::string getRadarPointCloudTopic(const std::string& radar_name) const;

private:
    vehicle_config::VehicleConfig config_;
    std::string file_path_;
    bool loaded_{false};
};

} // namespace calibration_gui
