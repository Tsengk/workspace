/**
 * radar_cali_test_node.cpp
 *
 * 毫米波雷达标定纯合成数据自测节点
 * 不依赖任何传感器，使用合成点云直接验证 RadarCalibrator 算法。
 *
 * 测试策略：
 *   1. 生成已知安装位姿 (R_install, t_install) 的模拟地面/目标点云
 *   2. 调用 RadarCalibrator 求解
 *   3. 对比与真值的旋转误差 (°) 和位置误差 (m)
 */

#include <ros/ros.h>
#include <cmath>
#include <random>
#include <iomanip>
#include <sstream>
#include <Eigen/Dense>

#include "radar_cali/types.h"
#include "radar_cali/radar_calibrator.h"

namespace radar_cali {

// ----------------------------------------------------------------
// 辅助函数
// ----------------------------------------------------------------
static Eigen::Matrix3d eulerZYXDegToRot(double roll, double pitch, double yaw)
{
    const double r = roll  * M_PI / 180.0;
    const double p = pitch * M_PI / 180.0;
    const double y = yaw   * M_PI / 180.0;
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
               .toRotationMatrix();
}

static double rotationError(const Eigen::Matrix3d& R_est, const Eigen::Matrix3d& R_true)
{
    Eigen::Matrix3d R_err = R_est.transpose() * R_true;
    double cos_val = (R_err.trace() - 1.0) * 0.5;
    cos_val = std::max(-1.0, std::min(1.0, cos_val));
    return std::acos(cos_val) * 180.0 / M_PI;
}

// ----------------------------------------------------------------
// 生成合成地面点云（在雷达坐标系中）
// 地面在 VCS 坐标系中 z = z_ground（默认 -0.37m）
// 雷达安装矩阵 T_radar_to_vcs: p_vcs = R_install * p_radar + t_install
// 所以在雷达坐标系中地面法向量 n_r = R_install^T * n_vcs
// ----------------------------------------------------------------
static std::vector<Eigen::Vector3d> generateGroundPoints(
    const Eigen::Matrix3d& R_install,
    const Eigen::Vector3d& t_install,
    double z_ground_vcs,
    int n_pts,
    double noise_std,
    unsigned seed = 42)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, noise_std);

    // 地面在 VCS 中的 z，法向量 (0,0,1)
    // p_vcs = R_install * p_radar + t_install
    // => z_ground_vcs = (R_install * p_radar + t_install).z()
    // => (R_install.row(2)) * p_radar = z_ground_vcs - t_install.z()

    std::uniform_real_distribution<double> xy_dist(-8.0, 8.0);

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(n_pts);

    // R_install^T: 将 VCS 转换为雷达坐标系
    Eigen::Matrix3d R_inv = R_install.transpose();
    Eigen::Vector3d t_inv = -R_inv * t_install;

    for (int i = 0; i < n_pts; ++i) {
        double xv = xy_dist(rng);
        double yv = xy_dist(rng);
        double zv = z_ground_vcs + noise(rng);

        // 将 VCS 点转到雷达坐标系
        Eigen::Vector3d p_vcs(xv, yv, zv);
        Eigen::Vector3d p_radar = R_inv * p_vcs + t_inv;
        pts.push_back(p_radar);
    }
    return pts;
}

// ----------------------------------------------------------------
// 生成合成目标点（锥桶或角点，位置已知）
// ----------------------------------------------------------------
static std::vector<Eigen::Vector3d> generateTargetRadarPoints(
    const Eigen::Matrix3d& R_install,
    const Eigen::Vector3d& t_install,
    const std::vector<Eigen::Vector3d>& target_vcs,
    double noise_std,
    unsigned seed = 0)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, noise_std);

    Eigen::Matrix3d R_inv = R_install.transpose();
    Eigen::Vector3d t_inv = -R_inv * t_install;

    std::vector<Eigen::Vector3d> pts;
    for (const auto& p_vcs : target_vcs) {
        Eigen::Vector3d p_r = R_inv * p_vcs + t_inv;
        p_r.x() += noise(rng);
        p_r.y() += noise(rng);
        p_r.z() += noise(rng);
        pts.push_back(p_r);
    }
    return pts;
}

// ----------------------------------------------------------------
// 测试节点
// ----------------------------------------------------------------
class RadarCaliTestNode {
public:
    RadarCaliTestNode(ros::NodeHandle& nh, ros::NodeHandle& /*pnh*/)
        : nh_(nh) {}

    bool runAllTests()
    {
        ROS_INFO("========================================");
        ROS_INFO("  radar_cali self-test");
        ROS_INFO("========================================");

        struct TestCase {
            std::string name;
            double roll_install, pitch_install, yaw_install;
            double tx, ty, tz;
            double noise_std;
            bool   use_target;  // true=目标物标定，false=地面标定
            double rot_tol_deg;
            double trans_tol_m;
        };

        std::vector<TestCase> tests = {
            // 地面标定：估计 roll/pitch，yaw 来自安装参数
            {"ground_zero_install",  0.0,  0.0, 0.0,  4.0, 0.0, 0.5,  0.02, false, 2.0, 0.5},
            {"ground_small_roll",    2.0, -1.5, 0.0,  4.0, 0.0, 0.5,  0.02, false, 3.0, 0.5},
            // 目标物标定：全 6DOF 估计
            {"target_zero",          0.0,  0.0, 0.0,  4.0, 0.2, 0.5,  0.02, true,  2.0, 0.1},
            {"target_with_yaw",      1.0, -1.0, 3.0,  4.0, 0.2, 0.5,  0.05, true,  3.0, 0.15},
        };

        int passed = 0;
        for (const auto& tc : tests) {
            Eigen::Matrix3d R_true = eulerZYXDegToRot(tc.roll_install, tc.pitch_install, tc.yaw_install);
            Eigen::Vector3d t_true(tc.tx, tc.ty, tc.tz);

            InstallParams install;
            install.radar_name  = "test_radar";
            install.x           = tc.tx;
            install.y           = tc.ty;
            install.z           = tc.tz;
            install.roll_deg    = tc.roll_install;
            install.pitch_deg   = tc.pitch_install;
            install.yaw_deg     = tc.yaw_install;

            RadarCalibrator calibrator;
            RadarCalibResult result;
            bool ok = false;

            if (tc.use_target) {
                // 目标物标定
                std::vector<Eigen::Vector3d> targets_vcs = {
                    {3.0, 0.0, 0.0}, {5.0, 1.0, 0.0}, {7.0, -1.0, 0.0},
                    {4.0, 2.0, 0.0}, {6.0, -2.0, 0.0}, {8.0, 0.5, 0.0}
                };
                auto radar_pts = generateTargetRadarPoints(R_true, t_true, targets_vcs,
                                                           tc.noise_std, 42);
                ok = calibrator.calibrateWithTarget(radar_pts, targets_vcs, install, result);
            } else {
                // 地面标定
                auto radar_pts = generateGroundPoints(R_true, t_true, -0.37,
                                                      500, tc.noise_std, 42);
                ok = calibrator.calibrateGroundPlane(radar_pts, install, result);
            }

            double rot_err   = ok ? rotationError(result.R, R_true)    : 1e6;
            double trans_err = ok ? (result.t - t_true).norm() : 1e6;

            // 地面标定仅比较 roll/pitch，不比较平移（yaw 来自安装参数固定）
            bool pass;
            if (!tc.use_target) {
                double roll_err  = std::abs(result.roll_deg  - tc.roll_install);
                double pitch_err = std::abs(result.pitch_deg - tc.pitch_install);
                pass = ok && result.valid
                       && roll_err  < tc.rot_tol_deg
                       && pitch_err < tc.rot_tol_deg;
                ROS_INFO("[%s] %s  roll_err=%.2f°  pitch_err=%.2f°  (tol=%.1f°)  valid=%d",
                         tc.name.c_str(), pass ? "PASS" : "FAIL",
                         roll_err, pitch_err, tc.rot_tol_deg, result.valid ? 1 : 0);
            } else {
                pass = ok && result.valid
                       && rot_err   < tc.rot_tol_deg
                       && trans_err < tc.trans_tol_m;
                ROS_INFO("[%s] %s  rot_err=%.2f°(tol=%.1f°)  trans_err=%.3fm(tol=%.2fm)  valid=%d",
                         tc.name.c_str(), pass ? "PASS" : "FAIL",
                         rot_err, tc.rot_tol_deg, trans_err, tc.trans_tol_m,
                         result.valid ? 1 : 0);
            }

            if (pass) ++passed;
        }

        ROS_INFO("========================================");
        ROS_INFO("Result: %d/%d tests passed", passed, (int)tests.size());
        ROS_INFO("========================================");

        return passed == (int)tests.size();
    }

private:
    ros::NodeHandle& nh_;
};

} // namespace radar_cali

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_cali_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    radar_cali::RadarCaliTestNode node(nh, pnh);
    bool ok = node.runAllTests();

    ROS_INFO("[radar_cali_test] All tests %s", ok ? "PASSED" : "FAILED (see above)");
    return ok ? 0 : 1;
}
