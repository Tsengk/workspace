#pragma once

#include "lidar_yaw1/types.h"
#include <deque>
#include <vector>

namespace lidar_yaw1 {

/**
 * @brief Yaw 角标定计算器
 *
 * 算法：
 *   1. 将检测到的锥桶中心投影到 XY 平面
 *   2. PCA 直线拟合，得到直线方向向量
 *   3. 锥桶沿车辆正前方（+X）摆放，故直线角 θ = atan2(dir_y, dir_x)
 *   4. Yaw 误差：δ_yaw = θ，修正量 = -θ
 *      yaw_calib = yaw_install - θ（通过四元数合成实现）
 *
 * 置信度评分：
 *   - 锥桶数量越多，置信度越高
 *   - 直线拟合 RMS 越小（共线性越好），置信度越高
 *
 * 多帧稳定：
 *   - 维护滑动窗口，对 delta_yaw 做置信度加权平均
 */
class YawCalculator {
public:
    struct Params {
        double max_yaw_deg;      // 单帧最大允许 yaw 偏差（°），超出则异常，默认 30°
        int    window_size;      // 滑动窗口帧数，默认 10
        double min_confidence;   // 置信度阈值，低于此不入窗口，默认 0.30

        Params()
            : max_yaw_deg(30.0),
              window_size(10),
              min_confidence(0.30) {}
    };

    explicit YawCalculator(const Params& params = Params());

    /**
     * @brief 从单帧锥桶列表计算 Yaw 误差（并加入滑动窗口）
     *
     * @param cones 当前帧检测到的锥桶
     * @return 单帧标定结果（valid=false 表示本帧不可用）
     */
    YawResult calculate(const std::vector<ConeCluster>& cones);

    /**
     * @brief 返回滑动窗口的置信度加权平均结果
     */
    YawResult getAveraged() const;

    void reset();
    void setParams(const Params& p) { params_ = p; }
    const Params& getParams() const { return params_; }

private:
    struct Entry {
        double yaw_delta_rad;
        double confidence;
    };

    Params params_;
    std::deque<Entry> window_;

    /**
     * @brief PCA 直线拟合，返回直线方向角 θ（rad）
     *
     * 对 N≥2 的锥桶中心 XY 坐标做 PCA，取最大特征向量方向。
     * 保证方向的 X 分量为正（锥桶在正前方）。
     *
     * @param cones     锥桶列表（需 ≥ 2）
     * @param rms_error 输出：各点到直线的垂直距离 RMS（m）
     * @return 直线方向角 θ = atan2(dir_y, dir_x)（rad）
     */
    double fitLineAngle(const std::vector<ConeCluster>& cones,
                        double& rms_error) const;

    /**
     * @brief 计算置信度
     *
     * 综合锥桶数量和直线拟合质量：
     *   cone_score = min(1.0, n_cones / 4.0)
     *   rms_score  = 1.0 - min(1.0, rms / 0.3)
     *   confidence = 0.4 * cone_score + 0.6 * rms_score
     *
     * @param n_cones   锥桶数量
     * @param rms_error 直线拟合 RMS（m）
     */
    double computeConfidence(int n_cones, double rms_error) const;
};

} // namespace lidar_yaw1
