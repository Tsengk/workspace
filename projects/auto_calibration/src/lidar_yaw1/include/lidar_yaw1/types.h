#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace lidar_yaw1 {

// ============================================================
// 锥形桶聚类信息（VCS 坐标系，FLU）
// ============================================================
struct ConeCluster {
    Eigen::Vector3f center;    // 聚类质心（VCS，m）
    float radius_xy;           // XY 平面内最大半径（m）
    int   point_count;         // 点数
    bool  valid;

    ConeCluster()
        : center(Eigen::Vector3f::Zero()),
          radius_xy(0.0f), point_count(0), valid(false) {}
};

// ============================================================
// 单帧 Yaw 标定结果
// ============================================================
struct YawResult {
    double yaw_delta_rad;   // Yaw 误差（rad），正 = VCS 相对车辆 CCW
    double yaw_delta_deg;   // Yaw 误差（°）
    double confidence;      // 置信度 [0, 1]
    int    n_cones;         // 参与计算的锥桶数
    double line_rms;        // 直线拟合 RMS 误差（m），衡量锥桶共线性
    bool   valid;
    std::string message;

    YawResult()
        : yaw_delta_rad(0), yaw_delta_deg(0),
          confidence(0), n_cones(0), line_rms(0), valid(false) {}
};

// ============================================================
// 安装参数（驱动使用的标称值）
// ============================================================
struct InstallationParams {
    std::string sensor_name;  // 传感器名称（用于标定文件键名）
    double x, y, z;           // 位置偏移（m），z 相对后轴中心
    double roll_deg;          // 横滚（°）—— 已标定值
    double pitch_deg;         // 俯仰（°）—— 已标定值
    double yaw_deg;           // 偏航（°）—— 本模块待标定
    double z_offset;          // 后轴中心到地面高度（m）

    InstallationParams()
        : sensor_name("lidar"),
          x(0.0), y(0.0), z(1.8),
          roll_deg(0.0), pitch_deg(0.0), yaw_deg(0.0),
          z_offset(0.37) {}
};

// ============================================================
// Yaw 标定输出
// ============================================================
struct YawCalibOutput {
    InstallationParams installation;   // 原始安装参数

    // 标定增量
    double delta_yaw_deg;              // Yaw 修正量（°）= -θ（锥桶连线角度）

    // 融合后参数（四元数合成：Rz(-θ) * R_install）
    double x_calib;                    // 未变（m）
    double y_calib;                    // 未变（m）
    double z_calib;                    // 未变（m）
    double roll_calib_deg;             // 合成后 roll（°），roll/pitch 已标定故几乎不变
    double pitch_calib_deg;            // 合成后 pitch（°）
    double yaw_calib_deg;              // 合成后 yaw（°）= yaw_install - θ

    // 标定质量
    double confidence;                 // 平均置信度 [0, 1]
    double std_yaw_deg;                // 滑动窗口 yaw 标准差（°），越小越稳定
    int    frames_used;                // 参与平均的帧数
    std::string quality_grade;         // "EXCELLENT" / "GOOD" / "ACCEPTABLE" / "POOR"
    std::string timestamp;             // 标定时间戳

    YawCalibOutput()
        : delta_yaw_deg(0.0),
          x_calib(0.0), y_calib(0.0), z_calib(0.0),
          roll_calib_deg(0.0), pitch_calib_deg(0.0), yaw_calib_deg(0.0),
          confidence(0.0), std_yaw_deg(0.0),
          frames_used(0), quality_grade("UNKNOWN") {}
};

} // namespace lidar_yaw1
