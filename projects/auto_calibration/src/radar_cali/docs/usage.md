# radar_cali 使用说明

## 功能

毫米波雷达外参标定（x/y/z/roll/pitch/yaw），yaw角精度小于0.5度。

支持两种标定方法：
1. **地面平面标定**：利用地面回波检测，估计roll/pitch角
2. **目标物标定**：利用已知位置的锥桶或标定板，估计全部6自由度

## 算法原理

### 地面平面标定

1. 从雷达点云中提取地面回波点
2. 使用RANSAC拟合地面平面方程 ax+by+cz+d=0
3. 从平面法向量估计roll/pitch角：
   - pitch = atan2(-normal.x, sqrt(normal.y² + normal.z²))
   - roll = atan2(normal.y, normal.z)
4. yaw角保持安装参数不变

### 目标物标定

1. 检测雷达点云中的目标物（锥桶、标定板等）
2. 获取目标物在VCS中的已知位置
3. 使用Umeyama算法进行点云配准：
   - 计算质心
   - 构建协方差矩阵 H = Σ(p_radar - c_radar)(p_vcs - c_vcs)ᵀ
   - SVD分解：H = UΣVᵀ
   - R = VUᵀ（确保右手系）
   - t = c_vcs - R·c_radar

## 参数配置

```yaml
# config/params.yaml
radar_name: "front_radar"
save_path: "/tmp/radar_calib_result.yaml"

topics:
  radar_topic: "/radar/points"

installation:
  x: 3.0           # 安装位置 X (m)
  y: 0.0           # 安装位置 Y (m)
  z: 0.8           # 安装位置 Z (m)
  roll_deg: 0.0    # 安装姿态 roll (deg)
  pitch_deg: 0.0   # 安装姿态 pitch (deg)
  yaw_deg: 0.0     # 安装姿态 yaw (deg)

calibrator:
  max_range: 100.0           # 最大检测距离 (m)
  min_range: 1.0             # 最小检测距离 (m)
  roi_x_min: -20.0           # ROI X最小值
  roi_x_max: 20.0            # ROI X最大值
  roi_y_min: -10.0           # ROI Y最小值
  roi_y_max: 10.0            # ROI Y最大值
  roi_z_min: -3.0            # ROI Z最小值
  roi_z_max: 5.0             # ROI Z最大值
  ground_height: -0.37       # 地面高度 (VCS中)
  ground_tolerance: 0.1      # 地面检测容差 (m)
  min_points: 50             # 最小点数
  max_iterations: 1000       # RANSAC最大迭代次数
  distance_threshold: 0.1    # RANSAC距离阈值 (m)
```

## C++ API 使用

### 地面平面标定

```cpp
#include "radar_cali/radar_calibrator.h"

radar_cali::RadarCalibrator calibrator;
radar_cali::RadarCalibrator::Params params;
params.max_range = 100.0;
params.min_points = 50;
params.distance_threshold = 0.1;
calibrator.setParams(params);

// 雷达点云（雷达坐标系）
std::vector<Eigen::Vector3d> radar_points;
// ... 填充雷达点云 ...

// 安装参数
radar_cali::InstallParams install;
install.radar_name = "front_radar";
install.x = 3.0; install.y = 0.0; install.z = 0.8;
install.roll_deg = 0.0; install.pitch_deg = 0.0; install.yaw_deg = 0.0;

// 执行标定
radar_cali::RadarCalibResult result;
bool ok = calibrator.calibrateGroundPlane(radar_points, install, result);

if (ok && result.valid) {
    std::cout << "Roll: " << result.roll_deg << " deg" << std::endl;
    std::cout << "Pitch: " << result.pitch_deg << " deg" << std::endl;
    std::cout << "Yaw: " << result.yaw_deg << " deg" << std::endl;
    std::cout << "Position: [" << result.x_m << ", " << result.y_m << ", " << result.z_m << "] m" << std::endl;
    std::cout << "Quality: " << result.quality_grade << std::endl;
}
```

### 目标物标定

```cpp
// 雷达检测到的目标点
std::vector<Eigen::Vector3d> radar_targets;
// 目标物在VCS中的已知位置
std::vector<Eigen::Vector3d> vcs_targets;

radar_cali::RadarCalibResult result;
bool ok = calibrator.calibrateWithTarget(radar_targets, vcs_targets, install, result);
```

### 点云变换

```cpp
// 将雷达点云变换到VCS
std::vector<Eigen::Vector3d> vcs_points;
calibrator.transformToVCS(radar_points, result, vcs_points);
```

## 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali
catkin_make --only-pkg-with-deps radar_cali
```

## 依赖

- ROS1 (roscpp, std_msgs, sensor_msgs, geometry_msgs, std_srvs)
- PCL (common, io, sample_consensus)
- Eigen3
- vehicle_config

## 精度指标

| 标定方法 | roll/pitch精度 | yaw精度 | 平移精度 |
|---------|---------------|---------|---------|
| 地面平面 | < 0.3° | 保持安装值 | 依赖安装先验 |
| 目标物 | < 0.3° | < 0.5° | < 0.05m |

## 注意事项

1. 地面平面标定需要平坦路面，80%以上区域为平地
2. 目标物标定时，目标物距离应在雷达有效检测范围内（5-40m）
3. 至少需要3个非共线目标点才能完成6自由度标定
4. 雷达点云质量直接影响标定精度，建议过滤噪声点
5. 安装先验参数（installation）应尽可能准确，误差< 10°

## 质量等级

- **EXCELLENT**：误差 < 0.02m，置信度 1.0
- **GOOD**：误差 < 0.05m，置信度 0.8
- **ACCEPTABLE**：误差 < 0.1m，置信度 0.6
- **POOR**：误差 >= 0.1m，置信度 0.3