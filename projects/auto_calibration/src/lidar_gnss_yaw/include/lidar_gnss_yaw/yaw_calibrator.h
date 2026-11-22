#pragma once

/**
 * @file yaw_calibrator.h
 * @brief LiDAR-GNSS 联合 yaw 标定核心算法
 *
 * 输入：VCS 中的锥桶中心（已含 yaw_install 旋转），搜索残差修正量 ε。
 *
 * 投影公式（VCS 输入）：
 *   p_vcs_true = R(ε) · (q - t_lb) + t_lb     ε = δ_candidate - yaw_install
 *   p_global   = R_bg(ψ) · p_vcs_true + [xg, yg]
 *
 * 目标函数：f(ε) = λ_min(Cov(所有投影点))  →  最小化 → ε*
 * 两阶段搜索：粗搜（均匀格点）→ 黄金分割精搜
 *
 * 输出：
 *   delta_yaw      = ε* （需叠加到驱动 yaw_install 的修正量）
 *   yaw_calibrated = yaw_install + ε*
 */

#include "lidar_gnss_yaw/types.h"

namespace lidar_gnss_yaw {

class YawCalibrator {
public:
    explicit YawCalibrator(const CalibParams& p);

    void addSnapshot(const Snapshot& snap);
    void reset();

    int    numSnapshots() const { return static_cast<int>(snapshots_.size()); }
    double headingSpread() const;

    CalibResult calibrate() const;

private:
    CalibParams            params_;
    std::vector<Snapshot>  snapshots_;

    /**
     * @brief 将所有快照的 VCS 锥桶以 ε = (delta_yaw_deg - yaw_install) 修正后
     *        投影到全局 UTM 坐标系
     */
    std::vector<Eigen::Vector2d> projectAll(double delta_yaw_deg) const;

    /** 目标函数：投影点集对最优直线的方差（越小越好） */
    double objective(double delta_yaw_deg) const;

    /** 黄金分割搜索单峰极小值 */
    double goldenSearch(double lo, double hi, double tol) const;

    /** 计算投影点对直线的 RMSE（m） */
    double computeRMSE(double delta_yaw_deg) const;
};

}  // namespace lidar_gnss_yaw
