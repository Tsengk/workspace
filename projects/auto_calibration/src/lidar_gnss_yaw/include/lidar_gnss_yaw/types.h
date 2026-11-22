#pragma once

/**
 * @file types.h
 * @brief LiDAR-GNSS 联合 yaw 标定核心数据结构
 *
 * 输入点云说明：
 *   驱动已使用安装参数（含 yaw_install）将原始 LiDAR 点云变换到 VCS（车体坐标系）。
 *   因此输入点云坐标系为 VCS（FLU，原点=后轴中心），而非原始 LiDAR 帧。
 *
 * 标定原理（全局投影一致性）：
 *   驱动以 yaw_install 变换点云，实际安装 yaw 为 δ_true，残差误差 ε = δ_true - yaw_install。
 *   对 VCS 中检测到的锥桶中心 q（已含 yaw_install 旋转），真实 VCS 位置为：
 *
 *     p_vcs = R(ε) · (q - t_lb) + t_lb
 *
 *   全局投影（VCS → UTM）：
 *     p_global = R_bg(ψ) · p_vcs + [xg, yg]
 *
 *   其中：
 *     R(ε)    = [[cos ε, -sin ε], [sin ε, cos ε]]，ε = δ_candidate - yaw_install
 *     t_lb    = [x_lb, y_lb]ᵀ，LiDAR 在 VCS 中的 XY 偏置（旋转中心）
 *     R_bg(ψ) = [[cos ψ, sin ψ], [sin ψ, -cos ψ]]，VCS → UTM（xg=北, yg=东）
 *     ψ       = GNSS 航向角（正北=0°，北偏东正，顺时针）
 *
 *   标定输出：
 *     ε* = argmin λ_min(Cov({p_global}))
 *     delta_yaw      = ε*（修正量，需更新到驱动安装参数）
 *     yaw_calibrated = yaw_install + ε*（标定后绝对 yaw）
 *
 * 坐标系约定：
 *   - VCS：FLU（X=前, Y=左, Z=上），原点 = 后轴中心，地面 z ≈ -z_offset
 *   - UTM：xg = 北（Northing），yg = 东（Easting）
 */

#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_gnss_yaw {

using PointT        = pcl::PointXYZI;
using PointCloud    = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

// ------------------------------------------------------------------
// GNSS 位姿（后轴中心，UTM 坐标系）
// ------------------------------------------------------------------
struct GnssPose {
    double timestamp{0.0};
    double xg{0.0};       // 北向（Northing，m）
    double yg{0.0};       // 东向（Easting，m）
    double heading{0.0};  // 航向角（°），正北=0，北偏东正，顺时针
    bool   valid{false};
};

// ------------------------------------------------------------------
// 单帧快照：GNSS 位姿 + VCS 中检测到的锥桶中心
// ------------------------------------------------------------------
struct Snapshot {
    GnssPose                     gnss;
    std::vector<Eigen::Vector2d> cones_vcs;  // 锥桶中心 (x,y) in VCS（已含 yaw_install）
};

// ------------------------------------------------------------------
// 全局参数
// ------------------------------------------------------------------
struct CalibParams {
    // ---- LiDAR 安装（VCS，后轴中心原点，FLU）----
    double x_lb{0.0};         // LiDAR 纵向偏移（m，正=前），即旋转中心 x 分量
    double y_lb{0.0};         // LiDAR 横向偏移（m，正=左），即旋转中心 y 分量
    double z_lb{1.8};         // LiDAR 垂直高度（m，相对后轴中心，仅记录用）
    double z_offset{0.37};    // 后轴中心到地面高度（m），VCS 地面 z ≈ -z_offset
    double yaw_install{0.0};  // 驱动使用的安装 yaw（°，CCW+），标定修正相对此值

    // ---- 地面去除（VCS 坐标系）----
    double ground_margin{0.15};  // 地面 z 以上保留余量（m）

    // ---- 锥桶检测 ----
    double cone_height_min{0.15};   // 锥桶最小高度（m）
    double cone_height_max{0.90};   // 锥桶最大高度（m）
    double cone_radius_max{0.35};   // 锥桶 XY 最大半径（m）
    double cone_max_range{25.0};    // 最大检测距离（m）
    double cone_min_range{1.0};     // 最小检测距离（m，过滤车体）
    double voxel_size{0.05};        // 降采样体素尺寸（m）
    double cluster_tolerance{0.25}; // 欧氏聚类距离阈值（m）
    int    min_cluster_points{5};   // 聚类最小点数
    int    max_cluster_points{500}; // 聚类最大点数

    // ---- 数据采集 ----
    int    min_snapshots{6};
    int    max_snapshots{60};
    int    min_cones_per_snapshot{3};
    double min_heading_spread{3.0};   // 最小航向角跨度（°），不足则告警
    double snapshot_interval{0.5};    // 采样间隔（s）

    // ---- 标定优化（搜索范围为修正量 ε = δ - yaw_install）----
    double search_range{12.0};   // 修正量搜索范围 ±（°）
    double coarse_step{0.05};    // 粗搜步长（°）
    double fine_tol{0.001};      // 精搜收敛精度（°）
};

// ------------------------------------------------------------------
// 标定结果
// ------------------------------------------------------------------
struct CalibResult {
    double yaw_install{0.0};     // 驱动先验 yaw（°）
    double yaw_calibrated{0.0};  // 标定后绝对 yaw（°），= yaw_install + delta_yaw
    double delta_yaw{0.0};       // 修正量 ε = yaw_calibrated - yaw_install（°）
    double rmse_m{0.0};          // 直线拟合 RMSE（m）
    double heading_spread{0.0};  // 快照航向角跨度（°）
    int    num_snapshots{0};
    int    num_cones_total{0};
    bool   success{false};
    std::string message;
};

}  // namespace lidar_gnss_yaw
