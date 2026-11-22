#pragma once

#include "lidar_rp_cali/types.h"
#include <deque>

namespace lidar_rp_cali {

/**
 * @brief 标定计算器
 *
 * 坐标系 FLU（前左上）：X=前, Y=左, Z=上
 *
 * 从地面平面法向量提取 Roll/Pitch：
 *   n_lidar = R_l2v^T * [0, 0, 1]^T
 *   其中 R_l2v = Ry(pitch_err) * Rx(roll_err)
 *
 *   展开得：n = [-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)]
 *
 *   Roll  = atan2(n_y, n_z)
 *   Pitch = -atan2(n_x, sqrt(n_y^2 + n_z^2))
 *
 * 多帧平均：维护滑动窗口，对法向量方向取加权平均
 */
class CalibrationCalculator {
public:
    struct Params {
        // 有效角度范围（超出则认为标定异常）
        double max_roll_deg;      // 默认 15°
        double max_pitch_deg;     // 默认 15°

        // 滑动窗口大小（帧数）
        int window_size;          // 默认 10

        // 置信度阈值（低于此值的结果不参与均值）
        double min_confidence;    // 默认 0.3

        // 置信度评分权重
        float rms_weight;         // RMS 误差影响权重，默认 0.5

        Params()
            : max_roll_deg(15.0), max_pitch_deg(15.0),
              window_size(10), min_confidence(0.3),
              rms_weight(0.5f) {}
    };

    explicit CalibrationCalculator(const Params& params = Params());

    /**
     * @brief 从单帧平面模型计算标定结果（并加入滑动窗口）
     */
    CalibrationResult calculate(const PlaneModel& plane);

    /**
     * @brief 返回滑动窗口均值标定结果
     */
    CalibrationResult getAveraged() const;

    void reset();
    void setParams(const Params& params) { params_ = params; }

private:
    struct Entry {
        double roll_rad;
        double pitch_rad;
        double confidence;
    };

    CalibrationResult computeFromNormal(const Eigen::Vector3f& normal) const;
    double computeConfidence(const PlaneModel& plane) const;

    Params params_;
    std::deque<Entry> window_;
};

} // namespace lidar_rp_cali
