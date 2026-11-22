#pragma once

#include <vector>
#include <Eigen/Dense>
#include "lidar_lidar_cali/types.h"

namespace lidar_lidar_cali {

// ---------------------------------------------------------------------------
// LidarCali
//
// 功能：通过两个激光雷达同时观测同一 FAST-LIVO2 标定板，
//       标定辅雷达的总体外参 (x_calib, y_calib, yaw_calib)。
//
// 原理：
//   两雷达各检测标定板的 4 个圆孔中心，得到 3D 点对应集合。
//   通过 SVD 3D-3D 配准，直接求解从辅雷达近似VCS到主雷达VCS的变换。
//
// 算法：
//   Step 1  对每个场景的圆孔中心做排序（角度 + CCW 保证对应关系一致）
//           辅雷达圆心先逆变换消除安装参数，还原到"水平LiDAR帧"：
//             q_rp = Rz(-yaw_s) · (p_secondary − [x_s, y_s, 0])
//
//   Step 2  SVD 配准 {(q_rp_i, p_primary_i)}，直接求解 T_calib：
//             P_primary ≈ Rz(yaw_calib) · q_rp + [x_calib, y_calib, z]
//
//   Step 3  从变换矩阵提取 x_calib/y_calib/yaw_calib_deg（驱动配置替换值）
// ---------------------------------------------------------------------------
class LidarCali {
public:
    struct Params {
        // 当前安装参数（用于逆变换消除安装偏置）
        double x_install       = 0.0;
        double y_install       = 0.0;
        double yaw_install_deg = 0.0;

        // 最少有效场景数
        int min_scenes = 3;
    };

    explicit LidarCali(const Params& p) : params_(p) {}

    // 添加一个场景（双雷达同步检测到的4个圆孔中心）
    void addScene(const SceneData& scene);

    // 是否具备标定最低数据量
    bool isReady() const;

    // 执行 SVD 3D-3D 标定
    CalibResult calibrate();

    // 重置所有场景数据
    void reset();

    int numScenes() const { return static_cast<int>(scenes_.size()); }

    void setParams(const Params& p) { params_ = p; }
    const Params& getParams() const { return params_; }

private:
    Params params_;
    std::vector<SceneData> scenes_;

    // 排序4个圆心（VCS/FLU坐标系，先转换到相机RDF再按角度排序以保证对应关系）
    void sortPatternCenters(const std::vector<Eigen::Vector3d>& in,
                            std::vector<Eigen::Vector3d>& out,
                            const std::string& axis_mode);

    // 3D RMSE
    double computeRMSE(const std::vector<Eigen::Vector3d>& src_transformed,
                       const std::vector<Eigen::Vector3d>& tgt) const;

    // 提取 ZYX 欧拉角（输出度）
    void extractEulerZYX(const Eigen::Matrix3d& R,
                         double& roll, double& pitch, double& yaw) const;
};

} // namespace lidar_lidar_cali
