# vehicle_config 使用说明

## 功能

车辆配置系统用于管理自动驾驶车辆的所有传感器参数和车辆尺寸信息。支持从YAML文件加载、保存、验证车辆配置。

## 配置文件格式

YAML格式，包含以下主要部分：

### 1. 车辆信息 (vehicle)

```yaml
vehicle:
  id: "CAR001"                    # 车辆标识
  car_length: 4.8                 # 车长 (m)
  car_width: 1.8                  # 车宽 (m)
  car_height: 1.5                 # 车高 (m)
  front_overhang: 0.9             # 前悬长度 (m)
  wheelbase: 2.8                  # 轴距 (m)
  rear_overhang: 1.1              # 后悬长度 (m)
  z_offset: 0.37                  # 后轴到地面高度 (m)
```

### 2. 相机信息 (cameras)

```yaml
cameras:
  - name: "front_camera"          # 相机名称
    topic: "/camera/image_raw"   # ROS话题
    intrinsics:
      new_K: [320.0, 0.0, 320.0, 0.0, 320.0, 160.0, 0.0, 0.0, 1.0]
      new_Size: [640, 320]
      scaled_K: [5014.642151485258, 0, 964.8852061954415, 0, 5012.721914802195, 540.8051489433708, 0, 0, 1]
      scaled_Size: [1920, 1080]
      K: [5014.642151485258, 0, 964.8852061954415, 0, 5012.721914802195, 540.8051489433708, 0, 0, 1]
      Size: [1920, 1080]
      D: [1.583124768456147, 12.375768453837075, -0.0011158506420116544, -0.0033198551651399135]
      Epsilon: 3.602978888758158
    extrinsics:
      t: [2.0, 0.0, 1.5]          # 平移 [x,y,z] (m)
      R_euler: [0.0, 5.0, 0.0]    # 欧拉角 [roll,pitch,yaw] (deg)
      vcs_to_ground_height: 0.0
      base_euler: 0.0
```

### 3. 激光雷达信息 (lidars)

```yaml
lidars:
  - name: "main_lidar"            # LiDAR名称
    topic: "/rslidar_points"     # ROS话题
    t: [0.0, 0.0, 0.5]           # 平移 [x,y,z] (m)
    R: [0.0, 0.0, 0.0]           # 欧拉角 [roll,pitch,yaw] (deg)
```

### 4. 毫米波雷达信息 (radars)

```yaml
radars:
  - name: "front_radar"           # 雷达名称
    topic: "/radar/points"       # ROS话题 (PointCloud2类型)
    t: [3.0, 0.0, 0.8]           # 平移 [x,y,z] (m)
    R: [0.0, 0.0, 0.0]           # 欧拉角 [roll,pitch,yaw] (deg)
```

### 5. GNSS信息 (gnss)

```yaml
gnss:
  name: "gnss_antenna"           # GNSS名称
  topic: "/gnss/odom"            # ROS话题
```

## C++ API 使用

### 加载配置

```cpp
#include "vehicle_config/config_loader.h"

vehicle_config::VehicleConfig config;
if (vehicle_config::ConfigLoader::loadFromFile("/path/to/config.yaml", config)) {
    // 使用配置
    std::cout << "Vehicle ID: " << config.vehicle.id << std::endl;
    std::cout << "Camera count: " << config.cameras.size() << std::endl;
}
```

### 保存配置

```cpp
vehicle_config::VehicleConfig config;
// ... 填充配置 ...
if (vehicle_config::ConfigLoader::saveToFile("/path/to/output.yaml", config)) {
    std::cout << "Configuration saved" << std::endl;
}
```

### 验证配置

```cpp
vehicle_config::VehicleConfig config;
// ... 加载或创建配置 ...
std::string error_msg;
if (!vehicle_config::ConfigLoader::validate(config, error_msg)) {
    std::cerr << "Config validation failed: " << error_msg << std::endl;
}
```

### 坐标转换

```cpp
// 欧拉角转旋转矩阵
std::vector<double> euler = {5.0, -8.0, 2.0};  // roll, pitch, yaw (deg)
Eigen::Matrix3d R = vehicle_config::ConfigLoader::eulerToRotMatrix(euler);

// 旋转矩阵转欧拉角
std::vector<double> euler_out = vehicle_config::ConfigLoader::rotMatrixToEuler(R);
```

## 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali
catkin_make --only-pkg-with-deps vehicle_config
```

## 依赖

- ROS1 (roscpp)
- yaml-cpp
- Eigen3

## 验证

配置验证包括：

1. 车辆ID非空
2. 车辆尺寸必须为正数
3. z_offset必须为正数
4. 至少配置一个相机和一个LiDAR
5. 所有传感器必须有名称和话题
6. GNSS必须有名称和话题

## 注意事项

1. 所有坐标均基于VCS（车辆坐标系）：
   - 原点在车辆后轴中心
   - X轴向前，Y轴向左，Z轴向上
   - z_offset为后轴中心到地面的高度

2. 欧拉角采用ZYX顺序（yaw-pitch-roll）

3. 相机内参K矩阵为3x3矩阵按行展开

4. 相机畸变系数D为[k1, k2, p1, p2, k3]格式