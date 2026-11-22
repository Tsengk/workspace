#pragma once

#include "lidar_rp_cali/types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_rp_cali {

/**
 * @brief 平面拟合器
 *
 * 支持三种算法：
 *   RANSAC        —— 适合含离群点的城市环境
 *   LEAST_SQUARES —— 适合干净数据，精度最高
 *   PCA           —— 效率高，基于协方差特征分解
 */
class PlaneFitter {
public:
    struct Params {
        FitMethod method;

        // RANSAC 参数
        float ransac_distance_threshold;  // 默认 0.05m
        int   ransac_max_iterations;      // 默认 200

        // 最小内点比例（低于此值则认为拟合失败）
        float min_inlier_ratio;           // 默认 0.5

        Params()
            : method(FitMethod::RANSAC),
              ransac_distance_threshold(0.05f),
              ransac_max_iterations(200),
              min_inlier_ratio(0.5f) {}
    };

    explicit PlaneFitter(const Params& params = Params());

    /**
     * @brief 拟合平面
     * @param cloud 地面点云
     * @param model 输出平面模型
     * @return true 拟合成功
     */
    bool fitPlane(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        PlaneModel& model);

    void setParams(const Params& params) { params_ = params; }
    const Params& getParams() const { return params_; }

private:
    bool fitRANSAC(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        PlaneModel& model);

    bool fitLeastSquares(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        PlaneModel& model);

    bool fitPCA(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        PlaneModel& model);

    // 计算 RMS 误差
    float computeRMSError(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const Eigen::Vector4f& coefficients);

    // 确保法向量 Z 分量为正（指向上方）
    void normalizeUpward(PlaneModel& model);

    Params params_;
};

} // namespace lidar_rp_cali
