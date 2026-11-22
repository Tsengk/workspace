#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace lidar_lidar_cali {

// ============================================================
// 单场景标定数据（双雷达同时观测标定板得到的4个圆孔中心）
// ============================================================
struct SceneData {
    // 4个圆孔中心在主雷达坐标系（VCS，FLU）中的3D坐标 (m)
    std::vector<Eigen::Vector3d> primary_centers;
    // 4个圆孔中心在辅雷达驱动输出坐标系（近似VCS）中的3D坐标 (m)
    std::vector<Eigen::Vector3d> secondary_centers;
    bool valid = false;
};

// ============================================================
// 辅激光雷达安装参数
// 约定：
//   - VCS（车体坐标系）: FLU（X=前, Y=左, Z=上），原点 = 后轴中心
//   - roll/pitch/z 已完成前序标定
//   - x_install / y_install / yaw_install_deg 待本模块标定
// ============================================================
struct InstallParams {
    std::string sensor_name;          // 传感器名称（用于输出文件键名）

    // 已标定参数（精确值，不再改变）
    double roll_calib_deg  = 0.0;
    double pitch_calib_deg = 0.0;
    double z_calib         = 1.43;   // LiDAR 在 VCS z 方向位置（m）
    double z_offset        = 0.37;   // 后轴中心到地面高度（m）

    // 待标定参数（安装标称值，本模块输出修正后的精确值）
    double x_install       = 0.0;
    double y_install       = 0.0;
    double yaw_install_deg = 0.0;

    InstallParams() : sensor_name("lidar_sub") {}
};

// ============================================================
// 标定结果
// ============================================================
struct CalibResult {
    // 标定后参数（写入驱动配置）
    double x_calib       = 0.0;
    double y_calib       = 0.0;
    double yaw_calib_deg = 0.0;

    // 标定增量（= calib - install）
    double delta_x       = 0.0;
    double delta_y       = 0.0;
    double delta_yaw_deg = 0.0;

    // 质量指标
    double rmse_m        = 0.0;  // 3D圆孔对齐 RMSE（m）
    int    num_scenes    = 0;    // 使用的有效场景数

    // 状态
    bool        success = false;
    std::string message;
    std::string timestamp;
    std::string quality_grade;  // "EXCELLENT" / "GOOD" / "ACCEPTABLE" / "POOR"
};

} // namespace lidar_lidar_cali
