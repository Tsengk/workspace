#include "lidar_lidar_cali/lidar_cali.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace lidar_lidar_cali {

static constexpr double R2D = 180.0 / M_PI;
static constexpr double D2R = M_PI / 180.0;

// ---------------------------------------------------------------------------
// sortPatternCenters
//
// 对 4 个圆孔中心按角度排序，保证两组点的对应关系一致。
// axis_mode="vcs"  : 输入为 FLU 坐标，先转到相机 RDF（cam.x=-vcs.y, cam.y=-vcs.z, cam.z=vcs.x）
//                   排序后再转回 VCS
// axis_mode="camera": 直接在 XY 平面排序
// ---------------------------------------------------------------------------
void LidarCali::sortPatternCenters(const std::vector<Eigen::Vector3d>& in,
                                    std::vector<Eigen::Vector3d>& out,
                                    const std::string& axis_mode)
{
    if (in.size() != 4) { out = in; return; }

    // 转换到工作坐标系
    std::vector<Eigen::Vector3d> work(4);
    if (axis_mode == "vcs") {
        for (int i = 0; i < 4; ++i) {
            work[i].x() = -in[i].y();
            work[i].y() = -in[i].z();
            work[i].z() =  in[i].x();
        }
    } else {
        work = in;
    }

    // 计算质心
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const auto& p : work) c += p;
    c /= 4.0;

    // 按角度排序
    std::vector<std::pair<double, int>> angle_idx;
    for (int i = 0; i < 4; ++i) {
        double dx = work[i].x() - c.x();
        double dy = work[i].y() - c.y();
        angle_idx.emplace_back(std::atan2(dy, dx), i);
    }
    std::sort(angle_idx.begin(), angle_idx.end());

    std::vector<Eigen::Vector3d> sorted(4);
    for (int i = 0; i < 4; ++i)
        sorted[i] = work[angle_idx[i].second];

    // CCW 检验（XY 平面叉积）
    Eigen::Vector3d v01 = sorted[1] - sorted[0];
    Eigen::Vector3d v12 = sorted[2] - sorted[1];
    if (v01.x() * v12.y() - v01.y() * v12.x() > 0)
        std::swap(sorted[1], sorted[3]);

    // 转回 VCS
    if (axis_mode == "vcs") {
        out.resize(4);
        for (int i = 0; i < 4; ++i) {
            out[i].x() =  sorted[i].z();
            out[i].y() = -sorted[i].x();
            out[i].z() = -sorted[i].y();
        }
    } else {
        out = sorted;
    }
}

// ---------------------------------------------------------------------------
double LidarCali::computeRMSE(const std::vector<Eigen::Vector3d>& src_t,
                               const std::vector<Eigen::Vector3d>& tgt) const
{
    if (src_t.size() != tgt.size() || src_t.empty()) return -1.0;
    double sum = 0.0;
    for (size_t i = 0; i < src_t.size(); ++i)
        sum += (src_t[i] - tgt[i]).squaredNorm();
    return std::sqrt(sum / static_cast<double>(src_t.size()));
}

// ---------------------------------------------------------------------------
void LidarCali::extractEulerZYX(const Eigen::Matrix3d& R,
                                  double& roll, double& pitch, double& yaw) const
{
    pitch = std::atan2(-R(2,0), std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
    roll  = std::atan2(R(2,1), R(2,2));
    yaw   = std::atan2(R(1,0), R(0,0));
    roll  *= R2D;
    pitch *= R2D;
    yaw   *= R2D;
}

// ---------------------------------------------------------------------------
void LidarCali::addScene(const SceneData& scene)
{
    if (scene.valid &&
        scene.primary_centers.size() == 4 &&
        scene.secondary_centers.size() == 4)
        scenes_.push_back(scene);
}

void LidarCali::reset()
{
    scenes_.clear();
}

bool LidarCali::isReady() const
{
    return static_cast<int>(scenes_.size()) >= params_.min_scenes;
}

// ---------------------------------------------------------------------------
// calibrate
//
// 算法：
//   对每个场景：
//     1. 辅雷达圆孔中心逆变换，消除安装参数，还原到水平LiDAR帧：
//          q_rp = Rz(-yaw_s) · (p_secondary − [x_s, y_s, 0])
//     2. 对 primary 和 q_rp 各做 sortPatternCenters("vcs") 保证对应
//   合并所有场景的点对，SVD 求最优刚体变换 T_calib：
//     P_primary = R_calib · q_rp + t_calib
//   输出：x_calib=t_calib.x, y_calib=t_calib.y, yaw_calib=atan2(R(1,0),R(0,0))
// ---------------------------------------------------------------------------
CalibResult LidarCali::calibrate()
{
    CalibResult result;

    // 收集有效场景
    std::vector<const SceneData*> valid;
    for (const auto& s : scenes_)
        if (s.valid && s.primary_centers.size() == 4 && s.secondary_centers.size() == 4)
            valid.push_back(&s);

    if (static_cast<int>(valid.size()) < params_.min_scenes) {
        result.message = "有效场景不足: " + std::to_string(valid.size())
                         + " < " + std::to_string(params_.min_scenes);
        return result;
    }

    // 逆变换系数（消除安装参数，还原到水平LiDAR帧）
    const double xi  = params_.x_install;
    const double yi  = params_.y_install;
    const double ci  = std::cos(-params_.yaw_install_deg * D2R);
    const double si  = std::sin(-params_.yaw_install_deg * D2R);

    // 合并所有场景的点对
    std::vector<Eigen::Vector3d> all_primary, all_q_rp;
    for (const auto* s : valid) {
        // 逆变换辅雷达圆心
        std::vector<Eigen::Vector3d> q_rp(4);
        for (int i = 0; i < 4; ++i) {
            double dx = s->secondary_centers[i].x() - xi;
            double dy = s->secondary_centers[i].y() - yi;
            q_rp[i].x() = ci * dx - si * dy;
            q_rp[i].y() = si * dx + ci * dy;
            q_rp[i].z() = s->secondary_centers[i].z();
        }

        // 排序（VCS 坐标系）
        std::vector<Eigen::Vector3d> pri_sorted, qrp_sorted;
        sortPatternCenters(s->primary_centers, pri_sorted, "vcs");
        sortPatternCenters(q_rp,               qrp_sorted, "vcs");

        for (int i = 0; i < 4; ++i) {
            all_primary.push_back(pri_sorted[i]);
            all_q_rp.push_back(qrp_sorted[i]);
        }
    }

    // SVD 3D-3D 配准
    // source = q_rp（水平LiDAR帧）, target = primary（VCS）
    Eigen::Vector3d c_qrp = Eigen::Vector3d::Zero();
    Eigen::Vector3d c_pri = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < all_q_rp.size(); ++i) {
        c_qrp += all_q_rp[i];
        c_pri += all_primary[i];
    }
    c_qrp /= static_cast<double>(all_q_rp.size());
    c_pri /= static_cast<double>(all_primary.size());

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < all_q_rp.size(); ++i)
        H += (all_q_rp[i] - c_qrp) * (all_primary[i] - c_pri).transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1.0;
        R = V * svd.matrixU().transpose();
    }

    Eigen::Vector3d t = c_pri - R * c_qrp;

    // RMSE
    std::vector<Eigen::Vector3d> transformed;
    transformed.reserve(all_q_rp.size());
    for (const auto& q : all_q_rp)
        transformed.push_back(R * q + t);
    double rmse = computeRMSE(all_primary, transformed);

    // 提取结果
    double roll_deg, pitch_deg, yaw_deg;
    extractEulerZYX(R, roll_deg, pitch_deg, yaw_deg);

    result.x_calib       = t.x();
    result.y_calib       = t.y();
    result.yaw_calib_deg = yaw_deg;

    result.delta_x       = result.x_calib       - params_.x_install;
    result.delta_y       = result.y_calib       - params_.y_install;
    result.delta_yaw_deg = result.yaw_calib_deg - params_.yaw_install_deg;

    result.rmse_m    = rmse;
    result.num_scenes = static_cast<int>(valid.size());
    result.success   = true;

    // 质量分级
    if      (rmse < 0.02 && result.num_scenes >= 3) result.quality_grade = "EXCELLENT";
    else if (rmse < 0.04 && result.num_scenes >= 2) result.quality_grade = "GOOD";
    else if (rmse < 0.08)                           result.quality_grade = "ACCEPTABLE";
    else                                             result.quality_grade = "POOR";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "[" << result.quality_grade << "] "
        << "scenes=" << result.num_scenes << " RMSE=" << rmse << "m"
        << " | x_calib=" << result.x_calib
        << "m y_calib=" << result.y_calib
        << "m yaw_calib=" << result.yaw_calib_deg << "°"
        << " (Δx=" << result.delta_x
        << " Δy=" << result.delta_y
        << " Δyaw=" << result.delta_yaw_deg << "°)";
    result.message = oss.str();
    return result;
}

} // namespace lidar_lidar_cali
