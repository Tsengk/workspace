#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace vehicle_config {

/// Vehicle dimensions and parameters
struct VehicleInfo {
    std::string id;           ///< Vehicle identifier
    double car_length;        ///< Vehicle length (m)
    double car_width;         ///< Vehicle width (m)
    double car_height;        ///< Vehicle height (m)
    double front_overhang;    ///< Front overhang length (m)
    double wheelbase;         ///< Wheelbase length (m)
    double rear_overhang;     ///< Rear overhang length (m)
    double z_offset;          ///< Rear axle to ground height (m)

    VehicleInfo();
};

/// Camera intrinsic parameters
struct CameraIntrinsics {
    std::vector<double> new_K;        ///< New camera matrix (3x3 flattened)
    std::vector<int> new_Size;        ///< New image size [width, height]
    std::vector<double> scaled_K;     ///< Scaled camera matrix
    std::vector<int> scaled_Size;     ///< Scaled image size [width, height]
    std::vector<double> K;            ///< Original camera matrix (3x3 flattened)
    std::vector<int> Size;            ///< Original image size [width, height]
    std::vector<double> D;            ///< Distortion coefficients
    double Epsilon;                   ///< Calibration epsilon

    CameraIntrinsics();
};

/// Camera extrinsic (installation) parameters
struct CameraExtrinsics {
    std::vector<double> t;            ///< Translation [x, y, z] in VCS (m)
    std::vector<double> R_euler;      ///< Rotation as ZYX Euler angles [roll, pitch, yaw] (deg)
    double vcs_to_ground_height;      ///< VCS to ground height (m)
    double base_euler;                ///< Base Euler angle offset (deg)

    CameraExtrinsics();
};

/// Complete camera configuration
struct CameraInfo {
    std::string name;                 ///< Camera name
    std::string topic;                ///< ROS topic
    std::string intrinsics_file;      ///< Relative path to separate intrinsics YAML (optional)
    CameraIntrinsics intrinsics;      ///< Intrinsic parameters
    CameraExtrinsics extrinsics;      ///< Extrinsic (installation) parameters

    CameraInfo();
};

/// LiDAR configuration
struct LidarInfo {
    std::string name;                 ///< LiDAR name
    std::string packets_topic;        ///< ROS topic for raw packets (driver input)
    std::string topic;                ///< ROS topic for point cloud (driver output, used for visualization)
    std::string launch_file;          ///< Relative path to driver launch file
    std::vector<double> t;            ///< Translation [x, y, z] in VCS (m)
    std::vector<double> R;            ///< Rotation as ZYX Euler [roll, pitch, yaw] (deg)

    LidarInfo();
};

/// Millimeter wave radar configuration
struct RadarInfo {
    std::string name;                 ///< Radar name
    std::string packets_topic;        ///< ROS topic for raw packets (driver input)
    std::string topic;                ///< ROS topic for point cloud (driver output, used for visualization)
    std::string launch_file;          ///< Relative path to driver launch file
    std::vector<double> t;            ///< Translation [x, y, z] in VCS (m)
    std::vector<double> R;            ///< Rotation as ZYX Euler [roll, pitch, yaw] (deg)

    RadarInfo();
};

/// GNSS configuration
struct GnssInfo {
    std::string name;                 ///< GNSS name
    std::string topic;                ///< ROS topic with custom message

    // Message format:
    // xg, yg: position in ENU (East-North-Up) coordinates
    // heading: 0° = North, 90° = East (clockwise positive)
    // quality: positioning quality indicator

    GnssInfo();
};

/// BEV 环视标定节点 topic 配置
struct BevSurroundViewConfig {
    std::string node_ns;                       ///< 节点命名空间
    std::string bev_image_topic;               ///< BEV 拼接图 topic
    std::string detection_vis_front_topic;     ///< 前相机检测可视化 topic
    std::string detection_vis_rear_topic;      ///< 后相机检测可视化 topic
    std::string detection_vis_left_topic;      ///< 左相机检测可视化 topic
    std::string detection_vis_right_topic;     ///< 右相机检测可视化 topic
    int output_width;                          ///< 最终输出 BEV 图像宽度 (px)
    int output_height;                         ///< 最终输出 BEV 图像高度 (px)
    double resolution;                         ///< BEV 分辨率 (m/px)
    double range_front;                        ///< 前向 ROI (px)
    double range_rear;                         ///< 后向 ROI (px)
    double range_left;                         ///< 左向 ROI (px)
    double range_right;                        ///< 右向 ROI (px)
    double origin_offset_x;                    ///< 中心前后偏移 (px)
    double origin_offset_y;                    ///< 中心左右偏移 (px)

    BevSurroundViewConfig();
};

/// Complete vehicle configuration
struct VehicleConfig {
    VehicleInfo vehicle;              ///< Vehicle information
    std::vector<CameraInfo> cameras;  ///< All cameras
    std::vector<LidarInfo> lidars;    ///< All LiDARs
    std::vector<RadarInfo> radars;    ///< All radars
    GnssInfo gnss;                    ///< GNSS configuration
    BevSurroundViewConfig bev;        ///< BEV 环视标定节点配置

    VehicleConfig();
};

} // namespace vehicle_config