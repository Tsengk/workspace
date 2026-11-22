#include "lidar_rp_cali/calibration_calculator.h"
#include <ros/ros.h>
#include <cmath>
#include <numeric>

namespace lidar_rp_cali {

static constexpr double RAD2DEG = 180.0 / M_PI;
static constexpr double DEG2RAD = M_PI / 180.0;

CalibrationCalculator::CalibrationCalculator(const Params& params)
    : params_(params) {}

// ---------------------------------------------------------------------------
CalibrationResult CalibrationCalculator::calculate(const PlaneModel& plane)
{
    CalibrationResult result;

    if (!plane.valid) {
        result.message = "invalid plane model";
        return result;
    }

    result = computeFromNormal(plane.normal);
    if (!result.valid) return result;

    result.confidence = computeConfidence(plane);

    if (result.confidence >= params_.min_confidence) {
        Entry e;
        e.roll_rad  = result.roll_rad;
        e.pitch_rad = result.pitch_rad;
        e.confidence = result.confidence;

        window_.push_back(e);
        if (static_cast<int>(window_.size()) > params_.window_size) {
            window_.pop_front();
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
CalibrationResult CalibrationCalculator::getAveraged() const
{
    CalibrationResult result;

    if (window_.empty()) {
        result.message = "window empty, no data";
        return result;
    }

    // 加权平均：以置信度为权重
    double sum_w = 0;
    double sum_roll  = 0;
    double sum_pitch = 0;
    for (const auto& e : window_) {
        sum_w     += e.confidence;
        sum_roll  += e.roll_rad  * e.confidence;
        sum_pitch += e.pitch_rad * e.confidence;
    }

    if (sum_w < 1e-9) {
        result.message = "total weight near zero";
        return result;
    }

    result.roll_rad  = sum_roll  / sum_w;
    result.pitch_rad = sum_pitch / sum_w;
    result.roll_deg  = result.roll_rad  * RAD2DEG;
    result.pitch_deg = result.pitch_rad * RAD2DEG;
    result.confidence = sum_w / window_.size();
    result.valid = true;
    result.message = "averaged (window=" +
                     std::to_string(window_.size()) + " frames)";

    // 生成四元数
    // R = Ry(pitch) * Rx(roll)
    Eigen::AngleAxisd roll_aa(result.roll_rad,  Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_aa(result.pitch_rad, Eigen::Vector3d::UnitY());
    result.quaternion = Eigen::Quaterniond(pitch_aa * roll_aa).normalized();

    return result;
}

// ---------------------------------------------------------------------------
void CalibrationCalculator::reset()
{
    window_.clear();
}

// ---------------------------------------------------------------------------
// 核心计算：从法向量提取 Roll / Pitch
//
// FLU 坐标系下，LiDAR 安装误差 (roll, pitch)：
//   R_l2v = Ry(pitch) * Rx(roll)
//
// 地面法向量在 LiDAR 帧：
//   n = R_l2v^T * [0, 0, 1]^T
//     = [-sin(pitch),  sin(roll)*cos(pitch),  cos(roll)*cos(pitch)]
//
// 反解：
//   Roll  = atan2(n_y, n_z)
//   Pitch = -atan2(n_x, sqrt(n_y^2 + n_z^2))
// ---------------------------------------------------------------------------
CalibrationResult CalibrationCalculator::computeFromNormal(
    const Eigen::Vector3f& normal) const
{
    CalibrationResult result;

    Eigen::Vector3f n = normal.normalized();

    double roll_rad  = std::atan2(static_cast<double>(n.y()),
                                  static_cast<double>(n.z()));
    double pitch_rad = -std::atan2(static_cast<double>(n.x()),
                                   std::sqrt(static_cast<double>(n.y()) *
                                             static_cast<double>(n.y()) +
                                             static_cast<double>(n.z()) *
                                             static_cast<double>(n.z())));

    double roll_deg  = roll_rad  * RAD2DEG;
    double pitch_deg = pitch_rad * RAD2DEG;

    // 有效性检查
    if (std::abs(roll_deg)  > params_.max_roll_deg ||
        std::abs(pitch_deg) > params_.max_pitch_deg) {
        result.valid = false;
        result.message = "angle out of range: roll=" +
                         std::to_string(roll_deg) + "deg pitch=" +
                         std::to_string(pitch_deg) + "deg";
        return result;
    }

    result.roll_rad  = roll_rad;
    result.pitch_rad = pitch_rad;
    result.roll_deg  = roll_deg;
    result.pitch_deg = pitch_deg;
    result.valid     = true;
    result.message   = "single frame OK";

    Eigen::AngleAxisd roll_aa (roll_rad,  Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_aa(pitch_rad, Eigen::Vector3d::UnitY());
    result.quaternion = Eigen::Quaterniond(pitch_aa * roll_aa).normalized();

    return result;
}

// ---------------------------------------------------------------------------
// 置信度 = f(内点比例, RMS误差)
// ---------------------------------------------------------------------------
double CalibrationCalculator::computeConfidence(const PlaneModel& plane) const
{
    // RMS 误差越小 → 置信度越高
    // 0.01m → 1.0，0.10m → 0.0（线性插值）
    double rms_score = 1.0 - std::min(1.0,
        static_cast<double>(plane.rms_error) / 0.10);

    // 曲率越小 → 平面越平整 → 置信度越高
    double curv_score = 1.0 - std::min(1.0,
        static_cast<double>(plane.curvature) * 100.0);

    double confidence = params_.rms_weight * rms_score +
                        (1.0 - params_.rms_weight) * curv_score;

    return std::max(0.0, std::min(1.0, confidence));
}

} // namespace lidar_rp_cali
