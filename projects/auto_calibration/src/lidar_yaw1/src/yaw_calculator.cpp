#include "lidar_yaw1/yaw_calculator.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace lidar_yaw1 {

static constexpr double RAD2DEG = 180.0 / M_PI;

YawCalculator::YawCalculator(const Params& params)
    : params_(params) {}

// ---------------------------------------------------------------------------
// 单帧计算
// ---------------------------------------------------------------------------
YawResult YawCalculator::calculate(const std::vector<ConeCluster>& cones)
{
    YawResult result;

    if (static_cast<int>(cones.size()) < 2) {
        result.message = "too few cones: " + std::to_string(cones.size());
        return result;
    }

    // 1. 直线拟合：得到直线方向角 θ（rad）
    double rms = 0.0;
    double theta = fitLineAngle(cones, rms);

    // 2. Yaw 误差 = θ（锥桶连线偏离 X 轴的角度）
    //    修正量   = -θ（令连线回到 X 轴方向）
    double yaw_delta_rad = -theta;
    double yaw_delta_deg = yaw_delta_rad * RAD2DEG;

    // 3. 有效性检查
    if (std::abs(yaw_delta_deg) > params_.max_yaw_deg) {
        result.valid   = false;
        result.message = "yaw out of range: " + std::to_string(yaw_delta_deg) + "deg";
        return result;
    }

    // 4. 置信度
    double conf = computeConfidence(static_cast<int>(cones.size()), rms);

    result.yaw_delta_rad = yaw_delta_rad;
    result.yaw_delta_deg = yaw_delta_deg;
    result.confidence    = conf;
    result.n_cones       = static_cast<int>(cones.size());
    result.line_rms      = rms;
    result.valid         = true;
    result.message       = "OK";

    // 5. 置信度过滤后加入滑动窗口
    if (conf >= params_.min_confidence) {
        Entry e;
        e.yaw_delta_rad = yaw_delta_rad;
        e.confidence    = conf;
        window_.push_back(e);
        if (static_cast<int>(window_.size()) > params_.window_size) {
            window_.pop_front();
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// 滑动窗口加权平均
// ---------------------------------------------------------------------------
YawResult YawCalculator::getAveraged() const
{
    YawResult result;

    if (window_.empty()) {
        result.message = "window empty, no data";
        return result;
    }

    double sum_w   = 0.0;
    double sum_yaw = 0.0;
    for (const auto& e : window_) {
        sum_w   += e.confidence;
        sum_yaw += e.yaw_delta_rad * e.confidence;
    }

    if (sum_w < 1e-9) {
        result.message = "total weight near zero";
        return result;
    }

    result.yaw_delta_rad = sum_yaw / sum_w;
    result.yaw_delta_deg = result.yaw_delta_rad * RAD2DEG;
    result.confidence    = sum_w / static_cast<double>(window_.size());
    result.n_cones       = 0;   // averaged, not single frame
    result.line_rms      = 0.0;
    result.valid         = true;
    result.message       = "averaged (window=" + std::to_string(window_.size()) + " frames)";

    return result;
}

// ---------------------------------------------------------------------------
void YawCalculator::reset()
{
    window_.clear();
}

// ---------------------------------------------------------------------------
// PCA 直线拟合
//
// 推导：
//   cones 摆在车辆正前方（+X 方向），真实方向为 (1, 0)。
//   若 Yaw 有误差 δ，则在 VCS 中，锥桶中心投影到 XY 平面后
//   沿方向 (cos δ, sin δ) 分布，夹角 θ = δ。
//
//   PCA 提取最大方差方向 = 锥桶连线方向 (cos θ, sin θ)。
//   确保 X 分量为正（锥桶在正前方）。
//   返回 θ = atan2(dir_y, dir_x)。
//
// 当 N=2 时退化为两点连线。
// ---------------------------------------------------------------------------
double YawCalculator::fitLineAngle(
    const std::vector<ConeCluster>& cones,
    double& rms_error) const
{
    const int N = static_cast<int>(cones.size());

    // 收集 XY 坐标
    std::vector<double> xs(N), ys(N);
    double mx = 0.0, my = 0.0;
    for (int i = 0; i < N; ++i) {
        xs[i] = static_cast<double>(cones[i].center.x());
        ys[i] = static_cast<double>(cones[i].center.y());
        mx += xs[i];
        my += ys[i];
    }
    mx /= N;
    my /= N;

    // 中心化
    for (int i = 0; i < N; ++i) {
        xs[i] -= mx;
        ys[i] -= my;
    }

    // 2x2 协方差矩阵（散布矩阵）
    Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
    for (int i = 0; i < N; ++i) {
        C(0, 0) += xs[i] * xs[i];
        C(0, 1) += xs[i] * ys[i];
        C(1, 0) += ys[i] * xs[i];
        C(1, 1) += ys[i] * ys[i];
    }
    C /= N;

    // 对称矩阵特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(C);
    // 特征值升序，最大特征值对应的特征向量 = 主方向
    Eigen::Vector2d dir = solver.eigenvectors().col(1);

    // 保证 X 分量为正（锥桶在车辆正前方，线方向应朝 +X）
    if (dir.x() < 0.0) dir = -dir;

    double theta = std::atan2(dir.y(), dir.x());

    // 计算各点到直线的垂直距离 RMS
    // 直线：过质心，方向 dir = (dx, dy)
    // 法向量：n = (-dy, dx)
    // 距离 = |(-dy)*(xi) + (dx)*(yi)|
    double sum_sq = 0.0;
    double nx = -dir.y();
    double ny =  dir.x();
    for (int i = 0; i < N; ++i) {
        double d = nx * xs[i] + ny * ys[i];
        sum_sq += d * d;
    }
    rms_error = std::sqrt(sum_sq / N);

    return theta;
}

// ---------------------------------------------------------------------------
// 置信度评分
//
// 两个维度：
//   1. 锥桶数量：越多越好，4 个及以上满分
//   2. 共线性：RMS 越小越好，< 0.05m 满分，> 0.30m 零分
//
// confidence = 0.4 * cone_score + 0.6 * rms_score
// ---------------------------------------------------------------------------
double YawCalculator::computeConfidence(int n_cones, double rms_error) const
{
    // 锥桶数量分（4 个及以上满分）
    double cone_score = std::min(1.0, static_cast<double>(n_cones) / 4.0);

    // 直线拟合质量分（RMS < 0.05m 满分，> 0.30m 零分）
    double rms_score = 1.0 - std::min(1.0, rms_error / 0.30);

    double confidence = 0.4 * cone_score + 0.6 * rms_score;
    return std::max(0.0, std::min(1.0, confidence));
}

} // namespace lidar_yaw1
