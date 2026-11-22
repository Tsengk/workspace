#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <Eigen/Dense>
#include <vector>
#include "image_lidar_calibration/types.h"

namespace image_lidar_calibration {

// LiDAR 圆孔中心检测器
// 参照 FAST-Calib demo (lidar_detect.hpp) 实现
// 支持两种模式：
//   Solid（固态/无ring字段）: 平面对齐 → BoundaryEstimation → 聚类 → 圆RANSAC
//   Mech（机械多线/有ring字段）: 平面对齐 → ring邻距跳变边缘 → 迭代圆RANSAC
class LidarProcessor {
public:
    struct Params {
        // ROI 过滤范围
        double roi_x_min = 1.5, roi_x_max = 5.0;
        double roi_y_min = -1.5, roi_y_max = 2.0;
        double roi_z_min = -0.5, roi_z_max = 2.0;
        // 平面拟合参数
        double ransac_threshold = 0.03;   // RANSAC距离阈值 (m)
        int    max_iterations   = 1000;   // RANSAC最大迭代次数
        // 圆孔几何参数（与 TargetParams 一致）
        double circle_radius         = 0.12;
        double delta_width_circles   = 0.50;
        double delta_height_circles  = 0.40;
        // 板面污染过滤参数
        double board_filter_margin_x = 0.14;
        double board_filter_margin_y = 0.14;
        double edge_filter_margin_x  = 0.16;
        double edge_filter_margin_y  = 0.16;
        double contamination_min_score = 0.58;
        double contamination_min_plane_keep_ratio = 0.28;
        double contamination_min_tight_inside_ratio = 0.55;
        // 局部圆弧支持参数
        double hole_search_radius_margin = 0.12;
        double hole_max_center_shift = 0.12;
        int    hole_min_annulus_points = 4;
        int    hole_max_inner_points = 16;
        double hole_min_local_score = 1.5;
    };

    LidarProcessor();
    void setParams(const Params& p);

    // 固态雷达模式（pcl::PointXYZI，无ring字段）
    // 使用 BoundaryEstimation + 欧氏聚类 + 每簇圆RANSAC
    bool detectCircleCenters(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        std::vector<Eigen::Vector3d>&                circle_centers,
        pcl::PointCloud<pcl::PointXYZ>::Ptr          edge_cloud_debug = nullptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr          plane_cloud_debug = nullptr);

    // 机械雷达模式（PointXYZIR，有ring字段）
    // 使用 ring邻距跳变边缘 + 迭代圆RANSAC
    bool detectCircleCentersMech(
        const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
        std::vector<Eigen::Vector3d>&            circle_centers,
        pcl::PointCloud<pcl::PointXYZ>::Ptr      edge_cloud_debug = nullptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr      plane_cloud_debug = nullptr);

    // ROI 过滤（通用）
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterROI(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;

    pcl::PointCloud<PointXYZIR>::Ptr filterROIMech(
        const pcl::PointCloud<PointXYZIR>::Ptr& cloud) const;

private:
    Params params_;

    // 内部辅助：平面RANSAC拟合
    bool fitPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                  pcl::ModelCoefficients::Ptr&                 coefficients,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr&        plane_cloud);

    bool fitPlaneMech(const pcl::PointCloud<PointXYZIR>::Ptr& cloud,
                      pcl::ModelCoefficients::Ptr&             coefficients,
                      pcl::PointCloud<PointXYZIR>::Ptr&        plane_cloud);

    // 将点对齐到Z=0平面（返回旋转矩阵和平均Z偏移）
    void alignToZPlane(const Eigen::Vector3d&                     normal,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& src,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr&         aligned,
                       Eigen::Matrix3d&                             R_align,
                       double&                                      avg_z);

    void alignEdgesToZPlane(const Eigen::Vector3d&                  normal,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr&      aligned,
                             Eigen::Matrix3d&                          R_align,
                             double&                                   avg_z);
};

} // namespace image_lidar_calibration
