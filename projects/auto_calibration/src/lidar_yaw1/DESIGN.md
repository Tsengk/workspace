# LiDAR Yaw 角标定系统 — 方案设计文档

**模块名称**：`lidar_yaw1`
**依赖前提**：Roll / Pitch 角已完成标定（`lidar_rp_cali`）
**实现语言**：Linux C++14 / ROS1 / PCL / Eigen3

---

## 目录

1. [背景与目标](#1-背景与目标)
2. [坐标系定义](#2-坐标系定义)
3. [标定原理](#3-标定原理)
4. [系统架构](#4-系统架构)
5. [模块设计](#5-模块设计)
   - 5.1 [ConeDetector — 锥桶检测器](#51-conedetector--锥桶检测器)
   - 5.2 [YawCalculator — Yaw 计算器](#52-yawcalculator--yaw-计算器)
   - 5.3 [LidarYawCaliNode — 主节点](#53-lidaryawcalinode--主节点)
6. [数据流](#6-数据流)
7. [置信度与质量评级](#7-置信度与质量评级)
8. [输出文件格式](#8-输出文件格式)
9. [关键参数说明](#9-关键参数说明)
10. [误差来源分析](#10-误差来源分析)

---

## 1. 背景与目标

激光雷达安装在车辆上时，实际安装姿态（Roll/Pitch/Yaw）往往与设计标称值存在偏差。
Roll 和 Pitch 可利用地面平面拟合进行标定（已在 `lidar_rp_cali` 中实现）。
Yaw 角偏差无法通过地面点云判断，需要借助具有已知方向的外部参照物。

**本方案目标：**

- 以车辆正前方排列的 4～5 个交通锥形桶为参照物
- 利用锥桶中心连线方向，从点云中自动估计激光雷达 Yaw 安装偏差
- 多帧滑动窗口平均，输出稳定的 Yaw 标定值
- 生成可直接用于驱动配置的 YAML 标定文件

---

## 2. 坐标系定义

```
        X（前）
        ↑
        │
  Y ←───┼  VCS（车辆坐标系，FLU）
        │
   Z（上，垂直纸面向外）
```

| 坐标系 | 原点 | 说明 |
|--------|------|------|
| **VCS**（Vehicle Coordinate System） | 后轴中心投影到地面 | FLU，X=前，Y=左，Z=上 |
| **LiDAR 帧** | 传感器中心 | 驱动将其变换到 VCS |
| **地面（VCS）** | — | z ≈ −z_offset（后轴到地面高度） |

驱动已使用安装参数 `R_install` 将点云从 LiDAR 帧变换到 VCS：

```
p_vcs = R_install × p_lidar + t_install
```

其中 `R_install = Rz(yaw_install) × Ry(pitch_install) × Rx(roll_install)`（ZYX 欧拉角顺序）。

---

## 3. 标定原理

### 3.1 几何直觉

锥形桶沿车辆正前方（+X 方向）依次摆放，如图所示：

```
车辆（俯视）
     ←Y→
  ┌──────┐
  │      │  ← 车头
  └──────┘
      │ X+（前方）
      │
      ● 锥桶1
      │
      ● 锥桶2
      │
      ● 锥桶3
      │
      ● 锥桶4
```

- **理想情况（Yaw = 0）**：锥桶中心连线与 VCS X 轴重合，夹角 θ = 0°
- **Yaw 有误差 δ**：VCS 相对真实车辆帧绕 Z 轴旋转了 δ，锥桶中心连线偏离 X 轴，夹角 θ = δ

### 3.2 数学推导

设 `yaw_true` 为真实 Yaw，`yaw_install` 为驱动使用的安装值，误差 `δ = yaw_install − yaw_true`。

驱动用 `R_install` 变换后，VCS 中的点相对真实车辆帧有额外旋转 `Rz(δ)`：

```
p_vcs = Rz(δ) × p_true_vcs
```

对沿真实 +X 排列的锥桶（真实坐标 `(xᵢ, 0)`）：

```
p_vcs_xy = Rz(δ) × (xᵢ, 0) = (xᵢ cosδ,  xᵢ sinδ)
```

所有锥桶中心落在同一射线上，该射线与 X 轴夹角恰好等于 δ。

### 3.3 标定公式

| 步骤 | 公式 |
|------|------|
| PCA 拟合锥桶连线方向角 | `θ = atan2(dir_y, dir_x)` |
| Yaw 误差 | `δ = θ` |
| Yaw 修正量 | `Δyaw = −θ` |
| 四元数合成（精确） | `R_calib = Rz(Δyaw) × R_install` |
| 提取 ZYX 欧拉角 | `yaw_calib = atan2(R(1,0), R(0,0))` |

> **为何用四元数合成而非直接相减：**
> 当 roll/pitch 非零时，欧拉角不满足线性叠加。四元数合成是唯一正确的做法，
> 在 Yaw 接近 0 时结果与 `yaw_install − θ` 基本一致；大角度时差异显著。

---

## 4. 系统架构

```
┌────────────────────────────────────────────────────────────┐
│                  lidar_yaw_cali_node                       │
│                                                            │
│  /rslidar_points ──→ [ConeDetector]──→ [YawCalculator]    │
│        PointCloud2      ↓ cones           ↓ YawResult      │
│                    cone_cloud          滑动窗口均值         │
│                         ↓                   ↓              │
│                  ~/cone_cloud          ~/result             │
│                  ~/cone_markers        ~/status             │
│                                             ↓              │
│                              Services: trigger/reset/save  │
│                                             ↓              │
│                                   标定结果 YAML 文件        │
└────────────────────────────────────────────────────────────┘
```

### 依赖关系

```
lidar_rp_cali（Roll/Pitch/Z 标定）
        │  输出：roll_deg, pitch_deg（已标定值）
        ▼
   params.yaml（installation.roll_deg / pitch_deg 填入已标定值）
        │
        ▼
   lidar_yaw1（本模块：Yaw 标定）
        │  输出：yaw_calib_deg
        ▼
   驱动配置更新
```

---

## 5. 模块设计

### 5.1 ConeDetector — 锥桶检测器

**职责**：从原始 VCS 点云中提取锥形桶聚类，输出各桶中心坐标。

#### 处理流程

```
原始点云（VCS）
      │
      ▼
 ① ROI 过滤（手动迭代，一次遍历）
      │  x ∈ [x_min, x_max]           // 前向距离范围
      │  y ∈ [y_min, y_max]           // 横向范围
      │  z ∈ [-z_offset+h_min,        // 锥桶高度范围
      │         -z_offset+h_max]
      ▼
 ② 体素网格降采样（VoxelGrid, leaf=0.05m）
      │  加速后续聚类，同时保留形状信息
      ▼
 ③ 欧几里得聚类（KdTree + EuclideanCluster）
      │  tolerance=0.30m
      │  min/max 点数过滤
      ▼
 ④ 锥桶形状过滤
      │  计算每个簇的 XY 平面最大半径
      │  radius_xy > max_cluster_radius_xy → 丢弃（非锥桶）
      ▼
 ⑤ 质心提取 → ConeCluster 列表
```

#### 锥桶高度 z 范围计算

```
地面高度（VCS）：z_ground = −z_offset = −0.37 m（默认）

锥桶 z 范围：
  z_min = z_ground + height_above_ground_min = −0.37 + 0.08 = −0.29 m
  z_max = z_ground + height_above_ground_max = −0.37 + 0.80 =  0.43 m
```

#### 接口定义

```cpp
class ConeDetector {
public:
    struct Params {
        float x_min, x_max;                   // 前向 ROI（m）
        float y_min, y_max;                   // 横向 ROI（m）
        float height_above_ground_min;        // 锥桶最低高度（m）
        float height_above_ground_max;        // 锥桶最高高度（m）
        float z_offset;                       // 后轴到地面（m）
        float voxel_size;                     // 降采样分辨率（m）
        float cluster_tolerance;              // 聚类距离阈值（m）
        int   min_cluster_points;
        int   max_cluster_points;
        float max_cluster_radius_xy;          // 最大 XY 半径（m）
        int   min_cones;                      // 最少有效锥桶数
    };

    bool detectCones(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        std::vector<ConeCluster>& cones,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& cone_cloud);
};
```

---

### 5.2 YawCalculator — Yaw 计算器

**职责**：从锥桶中心列表拟合方向角，计算 Yaw 误差，维护滑动窗口。

#### PCA 直线拟合（N ≥ 2）

```
输入：N 个锥桶中心 {(xᵢ, yᵢ)}

① 计算质心：(mx, my)
② 中心化：(x'ᵢ, y'ᵢ) = (xᵢ - mx, yᵢ - my)
③ 构建散布矩阵 C（2×2）：
     C = Σ [(x'ᵢ, y'ᵢ)ᵀ (x'ᵢ, y'ᵢ)] / N
④ 特征值分解（SelfAdjointEigenSolver）：
     最大特征值对应的特征向量 = 主方向 (dx, dy)
⑤ 方向修正：若 dx < 0，取反（保证指向正前方）
⑥ θ = atan2(dy, dx)

RMS（共线性）= sqrt( Σ(垂直距离²) / N )
  垂直距离 = |(-dy)·x'ᵢ + (dx)·y'ᵢ|
```

> N = 2 时退化为两点连线，RMS = 0（两点必然共线，置信度仅取决于锥桶数量）。

#### 置信度评分

```
cone_score = min(1.0, N / 4.0)                  // 4 个及以上满分
rms_score  = 1.0 - min(1.0, rms / 0.30)         // RMS < 0.05m 接近满分

confidence = 0.4 × cone_score + 0.6 × rms_score
```

#### 滑动窗口

```
每帧若 confidence ≥ min_confidence（默认 0.30）：
    将 (yaw_delta_rad, confidence) 压入窗口
    窗口超出 window_size（默认 10）时，弹出最旧帧

加权平均：
    yaw_avg = Σ(yaw_delta_rad_i × conf_i) / Σ(conf_i)
```

#### 接口定义

```cpp
class YawCalculator {
public:
    struct Params {
        double max_yaw_deg;     // 单帧最大允许偏差（默认 30°）
        int    window_size;     // 滑动窗口大小（默认 10）
        double min_confidence;  // 最小置信度阈值（默认 0.30）
    };

    YawResult calculate(const std::vector<ConeCluster>& cones);
    YawResult getAveraged() const;
    void reset();
};
```

---

### 5.3 LidarYawCaliNode — 主节点

#### 发布 / 订阅

| 类型 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| 订阅 | `/rslidar_points`（可配） | `sensor_msgs/PointCloud2` | 原始点云输入 |
| 发布 | `~/cone_cloud` | `sensor_msgs/PointCloud2` | 检测到的锥桶点云 |
| 发布 | `~/result` | `geometry_msgs/Vector3Stamped` | x=Δyaw(°) y=锥桶数 z=置信度 |
| 发布 | `~/status` | `std_msgs/String` | 每帧文字状态 |
| 发布 | `~/cone_markers` | `visualization_msgs/MarkerArray` | 锥桶位置可视化（RViz 橙色圆柱） |

#### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `~/trigger_calibration` | `std_srvs/Trigger` | 打印当前窗口均值结果 |
| `~/reset_calibration` | `std_srvs/Trigger` | 清空滑动窗口 |
| `~/save_calibration` | `std_srvs/Trigger` | 将结果写入 YAML 文件 |

#### TF 广播

每帧广播：`base_link → lidar_yaw_calibrated`，旋转量为四元数合成后的 `R_calib`。

#### 四元数合成（computeFusedEuler）

```cpp
// R_install = Rz(yaw_install) × Ry(pitch_install) × Rx(roll_install)
// R_delta   = Rz(delta_yaw)
// R_calib   = R_delta × R_install

// ZYX 欧拉角提取：
yaw   = atan2(R(1,0), R(0,0))
pitch = atan2(-R(2,0), sqrt(R(0,0)² + R(1,0)²))
roll  = atan2(R(2,1), R(2,2))
```

---

## 6. 数据流

```
                     ┌─ ROI 过滤 ─────────────────────────────────┐
sensor_msgs          │  x: [2, 40]m                               │
/rslidar_points  ────┤  y: [-10, 10]m                             │
(PointCloud2)        │  z: [-0.29, 0.43]m（z_offset=0.37默认）   │
                     └──────────────┬────────────────────────────┘
                                    │
                              VoxelGrid 0.05m
                                    │
                          EuclideanCluster (tol=0.30m)
                                    │
                         形状过滤 radius_xy < 0.50m
                                    │
                         ┌──────────┴──────────┐
                         │  ConeCluster 列表    │──→ ~/cone_cloud
                         │  (center, radius..)  │──→ ~/cone_markers
                         └──────────┬──────────┘
                                    │
                          PCA 直线拟合 θ
                                    │
                         Δyaw = −θ,  conf 评分
                                    │
                          滑动窗口（10帧加权平均）
                                    │
                     ┌──────────────┴──────────────────┐
                     │           YawResult              │
                     │  yaw_delta_deg  confidence       │──→ ~/result
                     │  n_cones        line_rms         │──→ ~/status
                     └──────────────────────────────────┘
                                    │
                         save_calibration 触发
                                    │
                    四元数合成 R_calib = Rz(Δyaw)×R_install
                                    │
                         /tmp/lidar_yaw_calibration.yaml
```

---

## 7. 置信度与质量评级

### 置信度分项

| 维度 | 计算方式 | 说明 |
|------|----------|------|
| 锥桶数量分 | `min(1.0, N / 4.0)` | 4 桶满分，2 桶 0.5 分 |
| 共线性分 | `1 − min(1.0, rms / 0.30)` | RMS < 0.05m 接近 1.0 |
| **综合** | `0.4 × cone_score + 0.6 × rms_score` | 共线性权重更高 |

### 质量等级判定

| 等级 | confidence | std_yaw | 帧数 | 建议 |
|------|-----------|---------|------|------|
| **EXCELLENT** | ≥ 0.80 | ≤ 0.10° | ≥ 8 | 可直接用于生产 |
| **GOOD** | ≥ 0.60 | ≤ 0.20° | ≥ 5 | 可接受 |
| **ACCEPTABLE** | ≥ 0.40 | ≤ 0.50° | ≥ 3 | 建议延长采集时间 |
| **POOR** | 其他 | — | — | 检查锥桶摆放，重新标定 |

---

## 8. 输出文件格式

保存至 `calibration_output_file`（默认 `/tmp/lidar_yaw_calibration.yaml`）：

```yaml
# LiDAR Yaw 标定文件
# 生成时间: 2026-03-25 10:30:00
# 坐标系: FLU (X=前,Y=左,Z=上)，VCS 原点为后轴中心投影到地面
# 融合方法: R_calib = Rz(delta_yaw) * R_install  (四元数合成)

top_lidar:

  # 原始安装参数（驱动使用的标称值）
  installation:
    x:         0.000000    # m
    y:         0.000000    # m
    z:         1.800000    # m
    roll_deg:  0.120000    # 度（已标定值，本步骤未改变）
    pitch_deg: -0.350000   # 度（已标定值，本步骤未改变）
    yaw_deg:   0.000000    # 度（标定前估计值）
    z_offset:  0.370000    # 后轴到地面高度 (m)

  # 标定后参数
  calibrated:
    x:         0.000000    # m     (未变)
    y:         0.000000    # m     (未变)
    z:         1.800000    # m     (未变)
    roll_deg:  0.119987    # 度    (精确合成值)
    pitch_deg: -0.350021   # 度    (精确合成值)
    yaw_deg:   -2.134500   # 度    (已更新) ← 标定结果
    z_offset:  0.370000    # m     (未变)

  # 标定增量
  delta:
    delta_yaw_deg: -2.134500    # 度（yaw 修正量 = -θ）

  # 标定质量
  quality:
    quality_grade:    EXCELLENT
    confidence:       0.873000
    std_yaw_deg:      0.082000
    frames_used:      10
    calibration_time: 2026-03-25 10:30:00
```

---

## 9. 关键参数说明

### 9.1 installation（安装参数）

| 参数 | 说明 | 注意事项 |
|------|------|----------|
| `sensor_name` | 写入标定文件的键名 | 须与驱动配置一致 |
| `z` / `z_offset` | 激光雷达垂直高度 / 后轴到地面 | 须与驱动配置保持一致 |
| `roll_deg` / `pitch_deg` | **上一步 roll/pitch 标定结果** | 不填写会导致锥桶高度过滤错误 |
| `yaw_deg` | 当前驱动使用的 yaw 估计值 | 标定结果将在此基础上修正 |

### 9.2 cone（锥桶检测参数）

| 参数 | 默认值 | 调优方向 |
|------|--------|----------|
| `x_min` | 2.0 m | 增大可排除近处杂波；减小可检测更近的锥桶 |
| `x_max` | 40.0 m | 根据实际摆放距离设置 |
| `height_above_ground_min` | 0.08 m | 若地面噪声大，适当增大（如 0.15m） |
| `height_above_ground_max` | 0.80 m | 覆盖锥桶最高点即可，一般无需修改 |
| `cluster_tolerance` | 0.30 m | 锥桶间距过小时须减小，避免多桶合并为一簇 |
| `max_cluster_radius_xy` | 0.50 m | 若误检路牌或灯柱，适当减小 |
| `min_cones` | 2 | 建议实际部署时设为 3，提高鲁棒性 |

### 9.3 yaw_cali（标定参数）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_yaw_deg` | 30.0° | 超出则认为检测异常，不入窗口 |
| `window_size` | 10 帧 | 越大稳定性越好，但收敛越慢 |
| `min_confidence` | 0.30 | 低置信帧过滤阈值 |

---

## 10. 误差来源分析

| 误差来源 | 影响量级 | 缓解措施 |
|----------|----------|----------|
| 锥桶摆放不直（人工误差） | 0.1°～0.5° | 多帧平均；使用更多锥桶 |
| 锥桶被风吹动 | 0.1°～1.0° | 静风环境下操作；增加帧数 |
| 点云稀疏（锥桶远处） | 影响共线性 | 缩小 x_max；增加锥桶数量 |
| Roll/Pitch 标定残余误差 | < 0.1°（通常） | 确保 R/P 标定质量为 GOOD 以上 |
| 地面不平（锥桶倾斜） | 0.05°～0.2° | 在平坦地面操作 |
| 聚类质心偏移 | < 0.02 m | 降低 voxel_size（如 0.03m） |

---

*文档版本：v1.0 | 2026-03-25*
