#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace lidar_rp_cali {

// ============================================================
// 平面模型
// ============================================================
struct PlaneModel {
    Eigen::Vector4f coefficients;  // ax + by + cz + d = 0
    Eigen::Vector3f normal;        // (a, b, c) 归一化法向量
    float curvature;
    int inliers_count;
    float rms_error;
    bool valid;

    PlaneModel()
        : coefficients(Eigen::Vector4f::Zero()),
          normal(Eigen::Vector3f::UnitZ()),
          curvature(0.0f), inliers_count(0),
          rms_error(0.0f), valid(false) {}
};

// ============================================================
// 标定结果
// ============================================================
struct CalibrationResult {
    double roll_rad;
    double pitch_rad;
    double roll_deg;
    double pitch_deg;
    double confidence;            // [0, 1]
    Eigen::Quaterniond quaternion;
    bool valid;
    std::string message;

    CalibrationResult()
        : roll_rad(0), pitch_rad(0), roll_deg(0), pitch_deg(0),
          confidence(0), quaternion(Eigen::Quaterniond::Identity()),
          valid(false) {}
};

// ============================================================
// 平面拟合方法
// ============================================================
enum class FitMethod {
    RANSAC,
    LEAST_SQUARES,
    PCA
};

// ============================================================
// 障碍物包围盒（车辆坐标系，FLU，单位m）
// ============================================================
struct ObstacleBox {
    Eigen::Vector3f center;    // 中心位置
    Eigen::Vector3f half_size; // 半宽 (x, y, z)
    std::string label;         // "building" / "vehicle" 等
};

// ============================================================
// 测试场景配置
// ============================================================
struct SceneConfig {
    std::string name;
    std::string description;

    // LiDAR 安装误差（待标定量），单位：度
    double lidar_roll_deg;
    double lidar_pitch_deg;

    // LiDAR 离地高度（m）
    double lidar_height;

    // 测距噪声标准差（m）
    double noise_std;

    // 障碍物列表（车辆坐标系）
    std::vector<ObstacleBox> obstacles;

    // 期望标定输出（由 lidar_roll/pitch 决定）
    double expected_roll_deg;
    double expected_pitch_deg;

    // 验收阈值（度）
    double tolerance_deg;

    SceneConfig()
        : lidar_roll_deg(0), lidar_pitch_deg(0),
          lidar_height(1.8), noise_std(0.02),
          expected_roll_deg(0), expected_pitch_deg(0),
          tolerance_deg(0.3) {}
};

// ============================================================
// 安装参数（驱动使用的标称值）
// ============================================================
struct InstallationParams {
    std::string sensor_name;     // 传感器名称，用于标定文件键名
    double x;                    // 纵向偏移（m）
    double y;                    // 横向偏移（m）
    double z;                    // 垂直偏移（m），相对 VCS 原点（后轴中心）
    double roll_deg;             // 横滚角（°）
    double pitch_deg;            // 俯仰角（°）
    double yaw_deg;              // 偏航角（°）
    double z_offset;             // 后轴中心到地面高度（m），z + z_offset = 激光雷达到地面高度

    InstallationParams()
        : sensor_name("lidar"),
          x(0.0), y(0.0), z(1.8),
          roll_deg(0.0), pitch_deg(0.0), yaw_deg(0.0),
          z_offset(0.37) {}
};

// ============================================================
// 标定文件输出内容
// ============================================================
struct CalibrationOutput {
    InstallationParams installation;  // 原始安装参数

    // --- 标定增量（在安装后坐标系中测量）---
    double delta_z;           // z 修正量（m）
    double delta_roll_deg;    // roll 修正量（°），在安装后坐标系内测量
    double delta_pitch_deg;   // pitch 修正量（°），在安装后坐标系内测量

    // --- 融合后参数（四元数合成，正确处理大 yaw 情形）---
    // z_calib = z_install - (z_ground_measured + z_offset)
    // R_fused = R(delta_pitch, delta_roll) * R(yaw_install, pitch_install, roll_install)
    double z_calib;           // 融合后 z（m）
    double roll_calib_deg;    // 融合后 roll（°）
    double pitch_calib_deg;   // 融合后 pitch（°）
    double yaw_fused_deg;     // 融合后 yaw（°），通常与安装值一致，大角度时略有差异

    // --- 地面高度测量 ---
    double z_ground_measured; // VCS 中测量地面高度（m）
    double z_ground_expected; // 期望地面高度 = -z_offset（m）

    // --- 标定质量（稳定性评估）---
    double confidence;        // 平均置信度 [0,1]，由平面拟合质量（RMS+曲率）决定
    double std_roll_deg;      // 滑动窗口内 roll 标准差（°），越小越稳定
    double std_pitch_deg;     // 滑动窗口内 pitch 标准差（°），越小越稳定
    double std_z_m;           // 滑动窗口内 z_ground 标准差（m），越小越稳定
    int frames_used;          // 参与平均的帧数
    std::string quality_grade;// "EXCELLENT" / "GOOD" / "ACCEPTABLE" / "POOR"
    std::string timestamp;    // 标定时间戳

    CalibrationOutput()
        : delta_z(0.0), delta_roll_deg(0.0), delta_pitch_deg(0.0),
          z_calib(0.0), roll_calib_deg(0.0), pitch_calib_deg(0.0), yaw_fused_deg(0.0),
          z_ground_measured(0.0), z_ground_expected(0.0),
          confidence(0.0), std_roll_deg(0.0), std_pitch_deg(0.0), std_z_m(0.0),
          frames_used(0), quality_grade("UNKNOWN") {}
};

} // namespace lidar_rp_cali
