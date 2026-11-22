#include "lidar_gnss_yaw/yaw_calibrator.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace lidar_gnss_yaw {

static constexpr double DEG2RAD = M_PI / 180.0;

YawCalibrator::YawCalibrator(const CalibParams& p) : params_(p) {}

void YawCalibrator::addSnapshot(const Snapshot& snap)
{
    if (static_cast<int>(snap.cones_vcs.size()) >= params_.min_cones_per_snapshot)
        snapshots_.push_back(snap);
}

void YawCalibrator::reset() { snapshots_.clear(); }

// --------------------------------------------------------------------------
double YawCalibrator::headingSpread() const
{
    if (snapshots_.empty()) return 0.0;
    double h_min = snapshots_.front().gnss.heading;
    double h_max = snapshots_.front().gnss.heading;
    for (const auto& s : snapshots_) {
        h_min = std::min(h_min, s.gnss.heading);
        h_max = std::max(h_max, s.gnss.heading);
    }
    return h_max - h_min;
}

// --------------------------------------------------------------------------
// 核心投影（VCS 输入）
//
// 背景：
//   驱动以 yaw_install 将 LiDAR 点云变换到 VCS：
//     q = R(yaw_install) · p_lidar + t_lb
//   实际安装 yaw 为 δ_candidate，残差误差 ε = δ_candidate - yaw_install。
//   真实 VCS 位置：
//     p_vcs = R(ε) · (q - t_lb) + t_lb
//
//   全局投影（UTM：xg=北, yg=东）：
//     p_global = R_bg(ψ) · p_vcs + [xg, yg]
//
//   R_bg(ψ) = [[cos ψ,  sin ψ],   推导：ψ=0 时前→(1,0)=N ✓，左→(0,-1)=W ✓
//              [sin ψ, -cos ψ]]
//
// 参数：delta_yaw_deg = δ_candidate（候选绝对 yaw，搜索变量）
// --------------------------------------------------------------------------
std::vector<Eigen::Vector2d>
YawCalibrator::projectAll(double delta_yaw_deg) const
{
    // 残差修正角 ε = δ_candidate - yaw_install
    const double eps     = (delta_yaw_deg - params_.yaw_install) * DEG2RAD;
    const double cos_eps = std::cos(eps);
    const double sin_eps = std::sin(eps);

    const double tx = params_.x_lb;  // LiDAR 在 VCS 中的旋转中心 x
    const double ty = params_.y_lb;  // LiDAR 在 VCS 中的旋转中心 y

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(snapshots_.size() * 6);

    for (const auto& snap : snapshots_) {
        const double psi     = snap.gnss.heading * DEG2RAD;
        const double cos_psi = std::cos(psi);
        const double sin_psi = std::sin(psi);
        const double xg      = snap.gnss.xg;
        const double yg      = snap.gnss.yg;

        for (const auto& q : snap.cones_vcs) {
            // 1. 修正 yaw 误差：以 LiDAR 中心 t_lb 为旋转轴，旋转残差角 ε
            //    p_vcs = R(ε) · (q - t_lb) + t_lb
            const double dx = q.x() - tx;
            const double dy = q.y() - ty;
            const double vx = cos_eps * dx - sin_eps * dy + tx;
            const double vy = sin_eps * dx + cos_eps * dy + ty;

            // 2. VCS → UTM（xg=北, yg=东）：R_bg(ψ) · p_vcs + [xg, yg]
            const double gx = cos_psi * vx + sin_psi * vy + xg;  // 北向
            const double gy = sin_psi * vx - cos_psi * vy + yg;  // 东向

            pts.emplace_back(gx, gy);
        }
    }
    return pts;
}

// --------------------------------------------------------------------------
// 目标函数：PCA 最小特征值 = 投影点到最优拟合直线的方差（越小越好）
// --------------------------------------------------------------------------
double YawCalibrator::objective(double delta_yaw_deg) const
{
    auto pts = projectAll(delta_yaw_deg);
    if (pts.size() < 3) return 1e9;

    const double n = static_cast<double>(pts.size());

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : pts) mean += p;
    mean /= n;

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : pts) {
        Eigen::Vector2d d = p - mean;
        cov += d * d.transpose();
    }
    cov /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
    return eig.eigenvalues()(0);  // 最小特征值 = 垂直拟合直线方向的方差
}

// --------------------------------------------------------------------------
double YawCalibrator::goldenSearch(double lo, double hi, double tol) const
{
    static constexpr double PHI = 0.6180339887498949;  // (√5-1)/2

    double a = lo, b = hi;
    double c = b - PHI * (b - a);
    double d = a + PHI * (b - a);
    double fc = objective(c);
    double fd = objective(d);

    while ((b - a) > tol) {
        if (fc < fd) {
            b = d;  d = c;  fd = fc;
            c = b - PHI * (b - a);
            fc = objective(c);
        } else {
            a = c;  c = d;  fc = fd;
            d = a + PHI * (b - a);
            fd = objective(d);
        }
    }
    return (a + b) * 0.5;
}

// --------------------------------------------------------------------------
double YawCalibrator::computeRMSE(double delta_yaw_deg) const
{
    auto pts = projectAll(delta_yaw_deg);
    if (pts.size() < 2) return 1e9;

    const double n = static_cast<double>(pts.size());
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : pts) mean += p;
    mean /= n;

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : pts) {
        Eigen::Vector2d d = p - mean;
        cov += d * d.transpose();
    }
    cov /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
    return std::sqrt(eig.eigenvalues()(0));
}

// --------------------------------------------------------------------------
CalibResult YawCalibrator::calibrate() const
{
    CalibResult res;
    res.yaw_install   = params_.yaw_install;
    res.num_snapshots = static_cast<int>(snapshots_.size());
    for (const auto& s : snapshots_)
        res.num_cones_total += static_cast<int>(s.cones_vcs.size());

    // ---- 数据量检查 ----
    if (res.num_snapshots < params_.min_snapshots) {
        std::ostringstream oss;
        oss << "快照不足：" << res.num_snapshots << " < " << params_.min_snapshots;
        res.message = oss.str();
        return res;
    }

    res.heading_spread = headingSpread();
    if (res.heading_spread < params_.min_heading_spread) {
        ROS_WARN("[YawCalibrator] 航向角跨度 %.2f° < %.2f°，精度可能下降。"
                 "建议驾驶时带有转弯或从多角度经过锥桶。",
                 res.heading_spread, params_.min_heading_spread);
    }

    // ---- 阶段 1：粗搜（搜索绝对 yaw δ_candidate）----
    // 搜索范围：[yaw_install - range, yaw_install + range]，覆盖修正量 ε ∈ [−range, +range]
    const double range    = params_.search_range;
    const double step     = params_.coarse_step;
    const double base     = params_.yaw_install;

    double best_delta = base;
    double best_obj   = 1e9;

    for (double d = base - range; d <= base + range; d += step) {
        double obj = objective(d);
        if (obj < best_obj) {
            best_obj   = obj;
            best_delta = d;
        }
    }

    // ---- 阶段 2：黄金分割精搜 ----
    double refined = goldenSearch(best_delta - 2.0 * step,
                                  best_delta + 2.0 * step,
                                  params_.fine_tol);

    // ---- 结果 ----
    const double rmse    = computeRMSE(refined);
    const double eps_opt = refined - params_.yaw_install;  // 最优修正量

    res.yaw_calibrated = refined;
    res.delta_yaw      = eps_opt;
    res.rmse_m         = rmse;
    res.success        = true;

    std::ostringstream oss;
    oss.precision(4);
    oss << std::fixed;
    oss << "标定成功。"
        << "yaw_install=" << params_.yaw_install << "° "
        << "delta_yaw=" << eps_opt << "° "
        << "yaw_calibrated=" << refined << "° "
        << "RMSE=" << rmse * 100.0 << "cm "
        << "航向跨度=" << res.heading_spread << "°";
    res.message = oss.str();

    return res;
}

}  // namespace lidar_gnss_yaw
