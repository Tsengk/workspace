#pragma once

#include "lidar_rp_cali/types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <random>

namespace lidar_rp_cali {

/**
 * @brief 速腾（Robosense）RS-Ruby 80 线激光雷达仿真器
 *
 * 坐标系：FLU（前左上）
 *   X = 前方，Y = 左方，Z = 上方
 *   方位角：从 +X 轴开始，逆时针为正（向 +Y 方向）
 *   俯仰角：水平面以上为正
 *
 * 仿真参数：
 *   - 80 线垂直分布（-25° ~ +15°，非均匀）
 *   - 水平分辨率 0.2°（1800 线/圈）
 *   - 最大测距 100m
 *   - 可添加高斯测距噪声
 *
 * 场景几何：
 *   - 地面平面（车辆坐标系下 z = 0，可带法向量倾斜）
 *   - LiDAR 安装误差（roll / pitch）
 *   - 障碍物（AABB，车辆坐标系）
 */
class LidarSimulator {
public:
    struct Config {
        // LiDAR 安装误差（待标定量），单位：弧度
        double lidar_roll_rad;
        double lidar_pitch_rad;

        // LiDAR 离地高度（m）
        double lidar_height;

        // 地面倾斜（模拟坡道/横坡，单位：弧度）
        double ground_roll_rad;
        double ground_pitch_rad;

        // 水平分辨率（度）和最大量程（m）
        double azimuth_resolution_deg;
        double max_range;

        // 测距噪声标准差（m），0 表示无噪声
        double noise_std;

        // 障碍物列表（车辆坐标系，FLU）
        std::vector<ObstacleBox> obstacles;

        Config()
            : lidar_roll_rad(0), lidar_pitch_rad(0),
              lidar_height(1.8),
              ground_roll_rad(0), ground_pitch_rad(0),
              azimuth_resolution_deg(0.2),
              max_range(100.0),
              noise_std(0.02) {}
    };

    explicit LidarSimulator(const Config& cfg = Config());

    /**
     * @brief 生成一帧仿真点云（LiDAR 帧，FLU）
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr generate();

    void setConfig(const Config& cfg);
    const Config& getConfig() const { return cfg_; }

    /** 返回 80 线垂直角度列表（度） */
    static const std::vector<double>& getBeamAngles();

private:
    // 射线与地面平面求交（世界坐标系）
    // 地面在世界系下 z=0，地面法向量由 ground_roll/pitch 确定
    bool rayGroundIntersect(
        const Eigen::Vector3d& ray_dir_world,
        const Eigen::Vector3d& lidar_pos_world,
        const Eigen::Vector3d& ground_normal_world,
        double& t_out) const;

    // 射线与 AABB 求交（车辆坐标系）
    bool rayAABBIntersect(
        const Eigen::Vector3d& ray_origin_vehicle,
        const Eigen::Vector3d& ray_dir_vehicle,
        const ObstacleBox& box,
        double& t_out) const;

    Config cfg_;

    // 旋转矩阵缓存（避免每帧重算）
    Eigen::Matrix3d R_l2v_;  // LiDAR -> Vehicle（含安装误差）
    Eigen::Matrix3d R_v2w_;  // Vehicle -> World（含地面倾斜）
    Eigen::Matrix3d R_l2w_;  // LiDAR -> World

    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;

    void updateRotations();
};

} // namespace lidar_rp_cali
