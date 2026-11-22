# LiDAR Yaw 角标定系统 — 使用说明文档

**模块名称**：`lidar_yaw1`
**适用平台**：Linux / ROS1 Noetic（或 Melodic）
**预估操作时间**：10～15 分钟（含场地准备）

---

## 目录

1. [前置条件](#1-前置条件)
2. [场地准备](#2-场地准备)
3. [配置文件修改](#3-配置文件修改)
4. [编译与安装](#4-编译与安装)
5. [标定操作流程](#5-标定操作流程)
6. [观察标定状态](#6-观察标定状态)
7. [保存标定结果](#7-保存标定结果)
8. [结果应用](#8-结果应用)
9. [故障排查](#9-故障排查)
10. [参数快速参考](#10-参数快速参考)

---

## 1. 前置条件

在开始 Yaw 标定前，须满足以下条件：

### 1.1 Roll / Pitch 标定已完成

本模块假设 Roll 和 Pitch 已经标定完毕。请确认：

- `lidar_rp_cali` 标定质量达到 **GOOD** 以上
- 已将标定后的 `roll_deg`、`pitch_deg` 值记录下来

### 1.2 软件依赖

```bash
# ROS1（Noetic 或 Melodic）
# PCL >= 1.8
# Eigen3
# catkin

# 确认依赖已安装
sudo apt install ros-noetic-pcl-ros ros-noetic-pcl-conversions \
                 ros-noetic-tf2-ros libeigen3-dev
```

### 1.3 激光雷达驱动正在运行

确保激光雷达驱动正在发布点云话题，默认为 `/rslidar_points`：

```bash
rostopic hz /rslidar_points   # 应有稳定输出，通常 10Hz 或 20Hz
```

---

## 2. 场地准备

### 2.1 锥形桶摆放规范

```
        ← 2m 以上间距 →
车头 ──→  ●  ●  ●  ●  ●
         锥1 锥2 锥3 锥4 锥5
         ↑                ↑
        2m~5m          最远40m
        （距车头）
```

**要求：**

| 项目 | 要求 | 说明 |
|------|------|------|
| 锥桶数量 | **4～5 个**（最少 2 个） | 数量越多精度越高 |
| 排列方式 | 沿车辆正前方 **X 轴方向** 成一直线 | 目视对齐即可，无需精确测量 |
| 相邻间距 | 建议 **1.5～3 m** | 间距过小可能被合并为同一聚类 |
| 起始距离 | 距车头 **2 m 以上** | 避免车身盲区 |
| 最远距离 | 不超过 **40 m** | 超出检测范围 |
| 周围环境 | 锥桶附近 **无其他障碍物** | 墙壁、路牌、其他车辆等均可能干扰聚类 |
| 地面要求 | **平坦地面** | 不平地面会导致锥桶倾斜，影响质心精度 |

### 2.2 车辆状态

- 车辆**静止**，驻车制动已启用
- 车头朝向与锥桶排列方向保持一致（车辆不需要精确对准，Yaw 误差会被测量出来）
- 激光雷达**正常工作**，无遮挡

---

## 3. 配置文件修改

打开配置文件：

```bash
# 路径示例（根据实际工作空间调整）
nano src/lidar_yaw1/config/params.yaml
```

### 3.1 必须修改的参数

#### ① 更新 Roll / Pitch 已标定值

```yaml
installation:
  sensor_name: top_lidar    # 与驱动配置保持一致
  x:         0.0            # 与驱动配置保持一致
  y:         0.0            # 与驱动配置保持一致
  z:         1.8            # 与驱动配置保持一致
  roll_deg:  0.12           # ← 填入 lidar_rp_cali 输出的 roll 标定值
  pitch_deg: -0.35          # ← 填入 lidar_rp_cali 输出的 pitch 标定值
  yaw_deg:   0.0            # ← 当前驱动使用的 yaw 值（标定前估计）
  z_offset:  0.37           # 与驱动配置保持一致
```

> **注意**：`roll_deg` / `pitch_deg` 必须使用 `lidar_rp_cali` 的标定结果，
> 而非原始安装标称值。错误填写会导致锥桶 z 过滤范围偏移。

#### ② 确认点云话题名

```yaml
input_topic: /rslidar_points   # 根据实际激光雷达驱动调整
```

#### ③ 设置输出文件路径

```yaml
calibration_output_file: /tmp/lidar_yaw_calibration.yaml   # 可自定义路径
```

### 3.2 按需调整的参数

根据实际锥桶摆放位置调整：

```yaml
cone:
  x_min:  2.0    # 最近锥桶距车头的距离
  x_max: 40.0    # 最远锥桶距车头的距离
  y_min: -5.0    # 若锥桶确保在中轴线附近，可适当收窄
  y_max:  5.0
  min_cones: 3   # 建议设为实际摆放数量 - 1（允许漏检一个）
```

---

## 4. 编译与安装

```bash
cd /path/to/your/catkin_ws

# 仅编译 lidar_yaw1 包（推荐，速度快）
catkin_make --only-pkg-with-deps lidar_yaw1

# 或编译整个工作空间
catkin_make

# source 环境
source devel/setup.bash
```

---

## 5. 标定操作流程

### 步骤 1：启动标定节点

```bash
roslaunch lidar_yaw1 lidar_yaw_cali.launch
```

启动后终端应输出类似：

```
[INFO] [LidarYawCali] init OK, listening: /rslidar_points
[INFO] [LidarYawCali] install: yaw=0.00° roll=0.12° pitch=-0.35° z=1.800m z_offset=0.370m
[INFO] [LidarYawCali] cone z range in VCS: [-0.29, 0.43]
[INFO] [LidarYawCali] output: /tmp/lidar_yaw_calibration.yaml
```

### 步骤 2：确认锥桶被检测到

```bash
# 另开终端，实时查看状态
rostopic echo /lidar_yaw_cali/status
```

正常检测时输出示例：

```
data: "OK | cones=4 yaw_delta=2.134° conf=0.87 rms=0.023m"
```

若输出 `cone detection failed`，请参考 [故障排查](#9-故障排查)。

### 步骤 3：等待数据积累（10～30 秒）

```bash
# 实时查看滑动窗口均值
rostopic echo /lidar_yaw_cali/result
```

`vector.x` 字段（Yaw 修正量）应在小范围内稳定波动。

### 步骤 4：查询当前标定结果

```bash
rosservice call /lidar_yaw_cali/trigger_calibration
```

输出示例：

```
success: True
message: "yaw_delta=-2.1345° yaw_calib=-2.1345° roll=0.1199° pitch=-0.3500° conf=0.873"
```

若 `confidence` 低于 0.5，建议重新摆放锥桶后继续。

### 步骤 5：保存标定结果

当数据稳定且 `confidence` 满意时：

```bash
rosservice call /lidar_yaw_cali/save_calibration
```

输出示例：

```
success: True
message: "[EXCELLENT] saved to /tmp/lidar_yaw_calibration.yaml | yaw_calib=-2.1345° (delta=-2.1345°) conf=0.873"
```

---

## 6. 观察标定状态

### 6.1 命令行监控

```bash
# 状态字符串（每帧输出）
rostopic echo /lidar_yaw_cali/status

# 数值结果（vector.x=Δyaw°, vector.y=锥桶数, vector.z=置信度）
rostopic echo /lidar_yaw_cali/result

# 查看锥桶点云帧率（应与 LiDAR 帧率一致）
rostopic hz /lidar_yaw_cali/cone_cloud
```

### 6.2 RViz 可视化（可选）

```bash
# 带 RViz 启动
roslaunch lidar_yaw1 lidar_yaw_cali.launch rviz:=true
```

在 RViz 中添加以下显示：

| Display 类型 | 话题 | 说明 |
|-------------|------|------|
| PointCloud2 | `/lidar_yaw_cali/cone_cloud` | 橙色：检测到的锥桶点 |
| MarkerArray | `/lidar_yaw_cali/cone_markers` | 橙色圆柱：各锥桶位置 |
| TF | — | 查看 `lidar_yaw_calibrated` 坐标系变化 |

**预期效果：** 橙色圆柱沿车辆 X 轴成一直线排列，连线与红色 X 轴重合表明检测正确。

### 6.3 稳定性判断标准

在 `trigger_calibration` 多次调用结果中，若 `yaw_delta` 变化不超过 0.3°，即可认为已收敛。

---

## 7. 保存标定结果

### 7.1 查看输出文件

```bash
cat /tmp/lidar_yaw_calibration.yaml
```

输出文件关键字段说明：

```yaml
top_lidar:
  calibrated:
    yaw_deg: -2.134500    # ← 此值填入驱动配置
  quality:
    quality_grade: EXCELLENT    # 质量等级
    confidence:    0.873000     # 置信度
    std_yaw_deg:   0.082000     # 标准差（越小越稳定）
    frames_used:   10           # 参与平均的帧数
```

### 7.2 重置重新标定

若需要清空数据重新开始：

```bash
rosservice call /lidar_yaw_cali/reset_calibration
```

---

## 8. 结果应用

### 8.1 更新驱动配置

将 `calibrated.yaw_deg` 填入激光雷达驱动的安装参数：

```yaml
# 驱动配置文件（以 rslidar 驱动为例）
lidar_param:
  device_ip: xxx.xxx.xxx.xxx
  msop_port: 6699
  difop_port: 7788
  angle_path: /xxx/angle.csv
  # ↓ 更新以下安装参数
  x:         0.0
  y:         0.0
  z:         1.8
  roll:      0.120          # lidar_rp_cali 标定值（不变）
  pitch:     -0.350         # lidar_rp_cali 标定值（不变）
  yaw:       -2.134500      # ← 本模块标定结果
```

### 8.2 验证标定效果

重启驱动后验证：

```bash
# 静态场景下，正前方锥桶连线应与 X 轴对齐
# 可再次运行标定节点，若 yaw_delta ≈ 0°，则标定已生效
roslaunch lidar_yaw1 lidar_yaw_cali.launch
rostopic echo /lidar_yaw_cali/result
# 期望：vector.x ≈ 0.0（< ±0.2°）
```

---

## 9. 故障排查

### 问题 1：`cone detection failed: insufficient cones`

每帧都检测不到足够锥桶。

| 可能原因 | 解决方案 |
|----------|----------|
| 锥桶不在 ROI 范围内 | 检查 `cone/x_min`、`x_max`、`y_min`、`y_max` |
| 锥桶被地面过滤掉 | 确认 `installation/roll_deg`、`pitch_deg` 为已标定值；或适当减小 `height_above_ground_min` |
| `min_cones` 设置过高 | 减小至实际锥桶数量 |
| 点云话题不对 | 确认 `input_topic` 设置正确；`rostopic hz /rslidar_points` 检查频率 |
| 锥桶距离过近（< 2m） | 减小 `cone/x_min` |

### 问题 2：`yaw out of range`

检测到 Yaw 偏差超过 `max_yaw_deg`（默认 30°）。

| 可能原因 | 解决方案 |
|----------|----------|
| 锥桶摆放方向错误（横向而非纵向） | 重新摆放，确保沿 +X 方向 |
| Roll/Pitch 严重错误导致点云扭曲 | 重新进行 Roll/Pitch 标定 |
| LiDAR Yaw 确实超过 30° | 将 `max_yaw_deg` 适当增大（如 45°）并重试 |

### 问题 3：置信度持续低（< 0.4）

| 可能原因 | 解决方案 |
|----------|----------|
| 锥桶共线性差（摆歪了） | 重新目视对齐后摆放 |
| 只有 2 个锥桶 | 增加到 4 个以上，cone_score 提升 |
| 锥桶间距太小被合并 | 增大锥桶间距，或减小 `cluster_tolerance` |
| 环境中有其他障碍物 | 清除锥桶附近障碍物 |

### 问题 4：每帧 yaw_delta 波动大（> 0.5°）

| 可能原因 | 解决方案 |
|----------|----------|
| 风吹动锥桶 | 选择无风环境操作 |
| 地面不平 | 换到更平坦地面 |
| 锥桶太远点云稀疏 | 减小 `x_max`；或提升 `voxel_size` 至 0.03m |
| 窗口太小 | 增大 `window_size`（如 20） |

### 问题 5：`cannot open file` 保存失败

```bash
# 确保目标目录有写权限
ls -la /tmp/
# 或修改输出路径为有权限的目录
```

---

## 10. 参数快速参考

```yaml
# src/lidar_yaw1/config/params.yaml

input_topic: /rslidar_points          # LiDAR 点云话题

installation:
  sensor_name: top_lidar              # 传感器名称
  x / y / z: 0.0 / 0.0 / 1.8        # 安装位置（m）
  roll_deg:   0.0   ← 填 rp_cali 结果
  pitch_deg:  0.0   ← 填 rp_cali 结果
  yaw_deg:    0.0   ← 驱动当前值
  z_offset:   0.37                    # 后轴到地面（m）

calibration_output_file: /tmp/lidar_yaw_calibration.yaml

cone:
  x_min / x_max:  2.0 / 40.0         # 前向检测范围（m）
  y_min / y_max: -10.0 / 10.0        # 横向检测范围（m）
  height_above_ground_min: 0.08      # 锥桶底部高度（m）
  height_above_ground_max: 0.80      # 锥桶顶部高度（m）
  voxel_size:          0.05          # 降采样（m）
  cluster_tolerance:   0.30          # 聚类距离（m）
  min_cluster_points:  5             # 单桶最少点数
  max_cluster_points:  500           # 单桶最多点数
  max_cluster_radius_xy: 0.50        # 锥桶最大半径（m）
  min_cones:           2             # 最少锥桶数

yaw_cali:
  max_yaw_deg:    30.0               # 最大允许 yaw 偏差（°）
  window_size:    10                 # 滑动窗口帧数
  min_confidence: 0.30               # 置信度阈值
```

---

## 操作流程速查卡

```
① 摆放 4～5 个锥桶（沿车辆正前方，间距 1.5～3m）
        ↓
② 修改 params.yaml（填入 roll/pitch 标定值）
        ↓
③ catkin_make --only-pkg-with-deps lidar_yaw1
        ↓
④ source devel/setup.bash
        ↓
⑤ roslaunch lidar_yaw1 lidar_yaw_cali.launch
        ↓
⑥ rostopic echo /lidar_yaw_cali/status  →  确认 "OK | cones=4..."
        ↓
⑦ 等待 10～30 秒（滑动窗口积累 10 帧）
        ↓
⑧ rosservice call /lidar_yaw_cali/trigger_calibration  →  查看结果
        ↓
⑨ rosservice call /lidar_yaw_cali/save_calibration    →  保存文件
        ↓
⑩ 将 calibrated.yaw_deg 填入驱动配置，重启驱动
```

---

*文档版本：v1.0 | 2026-03-25*
