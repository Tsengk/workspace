#include "lidar_rp_cali/lidar_simulator.h"
#include <cmath>
#include <ros/ros.h>

namespace lidar_rp_cali {

// ---------------------------------------------------------------------------
// RS-Ruby 80 线垂直角度（近似，单位：度）
// 分布：低层稀疏，近地平线密集，高层稀疏
// ---------------------------------------------------------------------------
static std::vector<double> buildBeamAngles()
{
    std::vector<double> angles;
    angles.reserve(80);

    // 低层 10 根（-25° ~ -16°，间距 1.0°）
    for (int i = 0; i < 10; ++i)
        angles.push_back(-25.0 + i * 1.0);

    // 次低层 15 根（-15.5° ~ -8.0°，间距 0.5°）
    for (int i = 0; i < 15; ++i)
        angles.push_back(-15.5 + i * 0.5);

    // 地平线附近 30 根（-7.5° ~ +2.5°，间距 ~0.34°）
    for (int i = 0; i < 30; ++i)
        angles.push_back(-7.5 + i * (10.0 / 29.0));

    // 上层 15 根（+2.7° ~ +9.7°，间距 0.5°）
    for (int i = 0; i < 15; ++i)
        angles.push_back(2.7 + i * 0.5);

    // 顶层 10 根（+10.0° ~ +15.0°，间距 ~0.56°）
    for (int i = 0; i < 10; ++i)
        angles.push_back(10.0 + i * (5.0 / 9.0));

    return angles;
}

const std::vector<double>& LidarSimulator::getBeamAngles()
{
    static const std::vector<double> angles = buildBeamAngles();
    return angles;
}

// ---------------------------------------------------------------------------
LidarSimulator::LidarSimulator(const Config& cfg)
    : cfg_(cfg), rng_(42), noise_dist_(0.0, cfg.noise_std)
{
    updateRotations();
}

void LidarSimulator::setConfig(const Config& cfg)
{
    cfg_ = cfg;
    noise_dist_ = std::normal_distribution<double>(0.0, cfg.noise_std);
    updateRotations();
}

// ---------------------------------------------------------------------------
// 预计算旋转矩阵
// R_l2v = Ry(pitch_err) * Rx(roll_err)   LiDAR → Vehicle
// R_v2w = Ry(gnd_pitch) * Rx(gnd_roll)   Vehicle → World
// R_l2w = R_v2w * R_l2v
// ---------------------------------------------------------------------------
void LidarSimulator::updateRotations()
{
    // Rx(α)
    auto Rx = [](double a) -> Eigen::Matrix3d {
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
        m(1, 1) =  std::cos(a);  m(1, 2) = -std::sin(a);
        m(2, 1) =  std::sin(a);  m(2, 2) =  std::cos(a);
        return m;
    };
    // Ry(α)
    auto Ry = [](double a) -> Eigen::Matrix3d {
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
        m(0, 0) =  std::cos(a);  m(0, 2) =  std::sin(a);
        m(2, 0) = -std::sin(a);  m(2, 2) =  std::cos(a);
        return m;
    };

    R_l2v_ = Ry(cfg_.lidar_pitch_rad) * Rx(cfg_.lidar_roll_rad);
    R_v2w_ = Ry(cfg_.ground_pitch_rad) * Rx(cfg_.ground_roll_rad);
    R_l2w_ = R_v2w_ * R_l2v_;
}

// ---------------------------------------------------------------------------
// 射线与地面（World 系 z=0 平面）求交
// ---------------------------------------------------------------------------
bool LidarSimulator::rayGroundIntersect(
    const Eigen::Vector3d& ray_dir_world,
    const Eigen::Vector3d& lidar_pos_world,
    const Eigen::Vector3d& /*ground_normal_world*/,
    double& t_out) const
{
    // 地面 z=0：lidar_pos_world.z + t * ray_dir_world.z = 0
    if (std::abs(ray_dir_world.z()) < 1e-9) return false;
    t_out = -lidar_pos_world.z() / ray_dir_world.z();
    return t_out > 0.01;  // 0.01m 最小距离，排除自身
}

// ---------------------------------------------------------------------------
// 射线与轴对齐包围盒（AABB）求交（车辆坐标系）
// ---------------------------------------------------------------------------
bool LidarSimulator::rayAABBIntersect(
    const Eigen::Vector3d& ray_origin,
    const Eigen::Vector3d& ray_dir,
    const ObstacleBox& box,
    double& t_out) const
{
    Eigen::Vector3d bmin = box.center.cast<double>() - box.half_size.cast<double>();
    Eigen::Vector3d bmax = box.center.cast<double>() + box.half_size.cast<double>();

    double tmin = 0.0;
    double tmax = cfg_.max_range;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(ray_dir[i]) < 1e-9) {
            // 射线平行于该轴
            if (ray_origin[i] < bmin[i] || ray_origin[i] > bmax[i])
                return false;
        } else {
            double inv = 1.0 / ray_dir[i];
            double t1 = (bmin[i] - ray_origin[i]) * inv;
            double t2 = (bmax[i] - ray_origin[i]) * inv;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
    }

    if (tmin < 0.01) tmin = tmax;
    if (tmin < 0.01 || tmin > cfg_.max_range) return false;

    t_out = tmin;
    return true;
}

// ---------------------------------------------------------------------------
// 生成一帧点云
// ---------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarSimulator::generate()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    const auto& beam_angles = getBeamAngles();
    const int   n_beams     = static_cast<int>(beam_angles.size());
    const double deg2rad    = M_PI / 180.0;

    // LiDAR 在世界系的位置（地面上方 H）
    // 世界系原点 = 车辆正下方地面点
    Eigen::Vector3d lidar_pos_world(0.0, 0.0, cfg_.lidar_height);

    // 水平扫描步数
    const int n_azimuth = static_cast<int>(
        std::round(360.0 / cfg_.azimuth_resolution_deg));

    cloud->reserve(static_cast<size_t>(n_beams) * n_azimuth);

    for (int ai = 0; ai < n_azimuth; ++ai) {
        double az_rad = ai * cfg_.azimuth_resolution_deg * deg2rad;
        double cos_az = std::cos(az_rad);
        double sin_az = std::sin(az_rad);

        for (int bi = 0; bi < n_beams; ++bi) {
            double el_rad = beam_angles[bi] * deg2rad;
            double cos_el = std::cos(el_rad);
            double sin_el = std::sin(el_rad);

            // 射线方向（LiDAR 帧，FLU）
            // x=前(cos*cos), y=左(cos*sin), z=上(sin)
            Eigen::Vector3d d_lidar(
                cos_el * cos_az,
                cos_el * sin_az,
                sin_el);

            // 转换到世界坐标系
            Eigen::Vector3d d_world = R_l2w_ * d_lidar;

            // ---- 地面交点 ----
            double t_ground = cfg_.max_range + 1.0;
            rayGroundIntersect(d_world, lidar_pos_world,
                               Eigen::Vector3d::UnitZ(), t_ground);

            // ---- 障碍物交点（车辆坐标系）----
            // 射线在车辆坐标系的原点和方向
            Eigen::Vector3d ray_origin_vehicle =
                R_v2w_.transpose() * lidar_pos_world;
            Eigen::Vector3d d_vehicle = R_v2w_.transpose() * d_world;

            double t_obstacle = cfg_.max_range + 1.0;
            for (const auto& box : cfg_.obstacles) {
                double t_tmp;
                if (rayAABBIntersect(ray_origin_vehicle, d_vehicle, box, t_tmp)) {
                    t_obstacle = std::min(t_obstacle, t_tmp);
                }
            }

            // 最近交点
            double t_hit = std::min(t_ground, t_obstacle);
            if (t_hit > cfg_.max_range || t_hit < 0.01) continue;

            // 世界系交点
            Eigen::Vector3d p_world = lidar_pos_world + t_hit * d_world;

            // 转回 LiDAR 帧
            Eigen::Vector3d p_lidar = R_l2w_.transpose() *
                                      (p_world - lidar_pos_world);

            // 测距噪声（沿射线方向）
            if (cfg_.noise_std > 0.0) {
                double noise = noise_dist_(rng_);
                p_lidar += noise * d_lidar;
            }

            // 强度：地面 ~ 0.5，障碍物 ~ 0.8
            float intensity = (t_hit < t_ground) ? 0.8f : 0.5f;

            pcl::PointXYZI pt;
            pt.x = static_cast<float>(p_lidar.x());
            pt.y = static_cast<float>(p_lidar.y());
            pt.z = static_cast<float>(p_lidar.z());
            pt.intensity = intensity;
            cloud->push_back(pt);
        }
    }

    cloud->width  = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

} // namespace lidar_rp_cali
