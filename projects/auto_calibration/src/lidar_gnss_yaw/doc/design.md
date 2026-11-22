# lidar_gnss_yaw 设计文档

## 1. 背景与目标

### 1.1 问题定义

LiDAR 安装时存在 **yaw 角安装误差**（偏航角偏差）。若不修正，LiDAR 感知到的目标方位与真实方位存在系统性偏差，影响定位、目标检测等下游任务的精度。

本模块在**车辆可以行驶**的条件下，利用已标定完毕的 GNSS（提供全局位置与航向）以及场地内摆放的锥形桶，对 LiDAR 的 yaw 安装角进行在线联合标定。

### 1.2 前提条件

| 条件 | 说明 |
|------|------|
| GNSS 已完成标定 | 输出后轴中心的全局坐标 (xg, yg) 及航向 heading |
| LiDAR Roll/Pitch 已知 | 可来自 `lidar_rp_cali` 或安装设计值 |
| 车辆可以行驶 | 需要从至少 2 个不同方向观测锥桶 |

### 1.3 精度目标

**标定误差 < 0.2°**，置信度 > 95%。

---

## 2. 坐标系定义

```
全局坐标系（UTM）          车体坐标系（VCS / FLU）     LiDAR 坐标系
  xg（北 / Northing）        X（前）                   X（前）
  ^                          ^                         ^
  |                          |                         |
  +---> yg（东 / Easting）   +---> Y（左）             +---> Y（左）
  Z = Up                     Z = Up（后轴中心原点）      Z = Up
```

| 量 | 符号 | 约定 |
|----|------|------|
| GNSS 输出位置 | (xg, yg) | 后轴中心，UTM；xg = Northing（北），yg = Easting（东），单位 m |
| GNSS 航向 | ψ | 正北 = 0°，北偏东为正，顺时针，单位 ° |
| LiDAR yaw | δ | 车体坐标系（FLU）内，CCW 为正，单位 ° |

> **注意**：全局系 xg=北、yg=东 与 ROS 常用 ENU（x=东, y=北）不同，导致 R_bg 形式不同，见第 3 节。

---

## 3. 标定算法

### 3.1 输入点云坐标系说明

> **驱动已使用安装参数（含 `yaw_install`）将原始 LiDAR 点云变换到 VCS**，本模块接收 VCS 点云，不再对原始 LiDAR 帧操作。

驱动变换：
```
q_vcs = R(yaw_install) · p_lidar + t_lb
```

若实际安装 yaw 为 δ_true，则 VCS 中锥桶的 **真实** 位置为：
```
p_vcs_true = R(δ_true) · p_lidar + t_lb
           = R(ε) · (q_vcs - t_lb) + t_lb     （ε = δ_true - yaw_install）
```

标定即求残差修正量 **ε**，进而得到 δ_true = yaw_install + ε。

### 3.2 核心原理：全局投影一致性

**观测模型**

在第 i 帧，车辆 GNSS 位姿为 (xgᵢ, ygᵢ, ψᵢ)，LiDAR 检测到第 j 个锥桶位于 **VCS**（已含 `yaw_install`）的 qᵢⱼ = (xv, yv)。以候选 yaw δ 修正后，该锥桶在全局坐标系中的投影为：

```
p_vcs_true = R(ε) · (q_vcs - t_lb) + t_lb      ε = δ - yaw_install
p_global   = R_bg(ψ) · p_vcs_true + [xg, yg]ᵀ
```

各矩阵定义：

```
         ┌ cos ε  -sin ε ┐               ┌ cos ψ   sin ψ ┐
R(ε)   = │               │   R_bg(ψ) =   │               │
         └ sin ε   cos ε ┘               └ sin ψ  -cos ψ ┘

t_lb = [x_lb, y_lb]ᵀ   （LiDAR 旋转中心在 VCS XY 平面内的偏置，来自安装参数）
```

> `ε` 是候选修正量，以 LiDAR 中心 t_lb 为轴旋转，修正驱动 `yaw_install` 与真实安装 yaw 之间的残差。

`R_bg(ψ)` 推导（全局系 xg=北，yg=东）：

车体 X 轴（前）在全局中的方向 = (cos ψ, sin ψ)：
- ψ=0°（北）：前→(1, 0) = 北 ✓
- ψ=90°（东）：前→(0, 1) = 东 ✓

车体 Y 轴（左）在全局中的方向 = (sin ψ, −cos ψ)：
- ψ=0°（北）：左→(0, −1) = 西 ✓
- ψ=90°（东）：左→(1, 0) = 北 ✓

```
       ┌ cos ψ   sin ψ ┐
R_bg = │               │   det = −1（xg=北,yg=东 水平面为左手系，正常）
       └ sin ψ  -cos ψ ┘
```

与 ROS 常用 ENU（x=东, y=北）对比：

| 全局系 | R_bg 第一行 | R_bg 第二行 |
|--------|------------|------------|
| ENU（x=东, y=北） | [sin ψ, −cos ψ] | [cos ψ, sin ψ] |
| **UTM（xg=北, yg=东）** | **[cos ψ, sin ψ]** | **[sin ψ, −cos ψ]** |

**一致性假设**

锥桶在全局坐标系中位置固定。当 δ 取**真值** δ* 时，同一个锥桶从不同车辆位姿投影得到的全局坐标完全一致；δ 偏差越大，不同位姿的投影越分散。

**目标函数**

N 帧快照的所有锥桶投影点集合 P = {pᵢⱼ}，锥桶在场地内成一条直线，则：

```
f(δ) = λ_min( Cov(P) )
```

即投影点云协方差矩阵的**最小特征值**（= 垂直于最优拟合直线方向的方差）。

```
δ* = argmin f(δ)
```

### 3.2 可观测性分析

**直线敏感度**

设锥桶沿 Y 轴分布，跨度 L（m），车辆在 X 方向行驶跨度 D（m），yaw 误差为 ε（rad）。

垂直残差近似：

```
σ_perp ≈ ε · √(L² + D²) / √12
```

对 L = 10 m、D = 10 m、ε = 0.2° ≈ 0.0035 rad：

```
σ_perp ≈ 0.0035 × 8.2 ≈ 0.029 m
```

这远大于典型 LiDAR 测量噪声（< 3 cm），因此**直线驾驶即可满足精度要求**。

**航向变化的提升作用**

当不同快照的航向角跨度 Δψ ≥ 5° 时，投影方向的多样性进一步降低交叉相关性，提高标定收敛速度和鲁棒性（实测对精度的改善约 2~3 倍）。

### 3.3 优化策略：两阶段搜索

```
阶段 1：粗搜（均匀格点）
  范围：[−search_range, +search_range]（默认 ±12°）
  步长：coarse_step（默认 0.05°）
  输出：δ_coarse

阶段 2：黄金分割精搜
  范围：[δ_coarse − 2·step, δ_coarse + 2·step]
  收敛精度：fine_tol（默认 0.001°）
  迭代数：约 log(4·step / fine_tol) / log(1.618) ≈ 50 次
  输出：δ*
```

黄金分割法收敛率稳定、无需导数，适合此一维单峰目标函数。

---

## 4. 系统架构

### 4.1 模块结构

```
┌──────────────────────────────────────────────────────┐
│                 LidarGnssYawNode                      │
│                                                       │
│  ┌─────────────┐    ┌──────────────┐                 │
│  │ ConeDetector│    │ YawCalibrator│                 │
│  │             │    │              │                 │
│  │ ROI过滤     │    │ addSnapshot()│                 │
│  │ 体素降采样  │    │ calibrate()  │                 │
│  │ 欧氏聚类    │    │              │                 │
│  │ 形状过滤    │    │ projectAll() │                 │
│  └─────────────┘    │ objective()  │                 │
│       ↑              │ goldenSearch()│                 │
│  PointCloud2        └──────────────┘                 │
│                            ↑                          │
│  nav_msgs/Odometry → GnssPose + Snapshot              │
│                                                       │
│  State: IDLE → COLLECTING → COMPUTING → DONE          │
└──────────────────────────────────────────────────────┘
```

### 4.2 数据流

```
LiDAR 点云 ──→ ConeDetector ──→ cones_lidar[]
                                      │
GNSS Odometry ──→ GnssPose ──────────┤
                                      ↓
                              Snapshot {gnss, cones_lidar}
                                      │
                              YawCalibrator::addSnapshot()
                                      │
                              [N 帧积累后触发]
                                      ↓
                              calibrate()
                              ├─ projectAll(δ)  → {p_global}
                              ├─ objective(δ)   → λ_min(Cov)
                              └─ goldenSearch() → δ*
                                      │
                              CalibResult
                              ├─ yaw_calibrated
                              ├─ delta_yaw
                              └─ rmse_m
```

### 4.3 节点状态机

```
              start()
  IDLE ──────────────────→ COLLECTING
   ↑                            │
   │ reset()             stop() / max_snapshots
   │                            ↓
   └──────────── DONE ←── COMPUTING
                  ↑
              success
```

---

## 5. 关键模块详述

### 5.1 ConeDetector

**流程**

```
输入点云（VCS，已含 yaw_install 变换）
  │
  ▼
ROI 过滤（z 范围 + 距离范围）
  地面高度估计：z_ground = −axis_offset       （VCS 中后轴中心高于地面 axis_offset m）
  锥桶 z 范围：[z_ground + ground_margin, z_ground + cone_height_max]
  距离范围：[cone_min_range, cone_max_range]
  │
  ▼
体素降采样（voxel_size = 0.05 m）
  │
  ▼
欧氏聚类（cluster_tolerance = 0.25 m）
  │
  ▼
形状过滤（每个 cluster）
  ├─ 高度 ∈ [cone_height_min, cone_height_max]  ✓
  ├─ XY 半径 ≤ cone_radius_max                  ✓
  └─ 点数 ∈ [min_cluster_points, max_cluster_points] ✓
  │
  ▼
锥桶中心（质心 XY）
```

**输出**：`std::vector<Eigen::Vector2d>` — 各锥桶中心 (x, y) in VCS

### 5.2 YawCalibrator

**关键函数**

| 函数 | 功能 |
|------|------|
| `addSnapshot(snap)` | 追加快照，过滤锥桶数不足的帧 |
| `projectAll(δ)` | 以给定 δ 将所有快照锥桶投影到全局 |
| `objective(δ)` | 计算 PCA 最小特征值 |
| `goldenSearch(lo,hi,tol)` | 黄金分割极小值搜索 |
| `calibrate()` | 执行两阶段优化，返回 `CalibResult` |
| `computeRMSE(δ)` | 计算直线拟合 RMSE（m） |

**objective() 计算细节**

```cpp
// 1. 所有投影点集合 {p}
auto pts = projectAll(delta_yaw_deg);

// 2. 均值
Eigen::Vector2d mean = sum(pts) / N;

// 3. 协方差矩阵 (2×2)
Cov = sum((p - mean)(p - mean)ᵀ) / N;

// 4. 最小特征值（SelfAdjointEigenSolver，精确且高效）
return eigenvalues(Cov)(0);   // 最小特征值
```

### 5.3 GNSS 航向转换

节点订阅 `nav_msgs/Odometry`（UTM 坐标系，xg=北，yg=东），从四元数提取航向：

```
yaw = atan2( 2(wz + xy), 1 − 2(y² + z²) )

在 xg=北、yg=东（z=Up）坐标系中，正 yaw（绕 +z 右手旋转）= 从北轴转向东轴
                                                            = 导航顺时针方向

因此：heading（导航，°）= yaw × 180/π   （直接，无需 90° 偏置）
```

**与 ENU 的对比：**

| 坐标系 | quaternion yaw=+90° 物理含义 | 到导航 heading 的转换 |
|--------|-----------------------------|--------------------|
| ENU（x=东, y=北） | 朝北，heading=0° | `heading = 90° − yaw_deg` |
| **UTM（xg=北, yg=东）** | **朝东，heading=90°** | **`heading = yaw_deg`（直接）** |

---

## 6. 误差分析

### 6.1 误差来源

| 误差源 | 典型量级 | 影响 |
|--------|----------|------|
| GNSS 位置噪声 | 1~5 cm | 引入锥桶投影散布，等效 yaw 误差 < 0.01°（@10m距离） |
| GNSS 航向噪声 | 0.05~0.1° | 直接叠加到标定结果中 |
| LiDAR 测距噪声 | 1~3 cm | 引入锥桶质心误差，等效 yaw 误差 < 0.01° |
| 锥桶非理想形状 | — | 质心偏差 < 2 cm，等效 < 0.1°（@15m） |
| 地面不平引起的点云畸变 | — | 用 z 范围滤波缓解 |

### 6.2 精度估计

在典型场景（锥桶跨度 10 m、20 帧快照、3 个方向驾驶、GNSS 航向精度 0.05°）：

```
理论 RMSE ≈ 0.05° (1σ)     << 0.2° 要求
```

实际精度主要受 GNSS 航向精度限制。若 GNSS 航向误差为 σ_ψ，标定精度约为：

```
σ_δ ≈ σ_ψ / √N_frames
```

---

## 7. 局限性与适用范围

| 条件 | 建议 |
|------|------|
| GNSS 信号遮挡 | 在开阔区域标定，避免高楼/树木遮挡 |
| 航向跨度 < 3° | 系统告警，建议换驾驶路线 |
| 锥桶少于 3 个 | 单帧不参与，需增加锥桶 |
| 大坡度（>2°） | 影响地面去除精度，建议在平地标定 |
| 锥桶不成直线 | 算法假设成立，若 L 形摆放会降低精度 |
| LiDAR Roll/Pitch 有较大误差 | 会影响地面去除，进而影响锥桶检测 |

---

## 8. 参数设计说明

| 参数 | 默认值 | 设计依据 |
|------|--------|----------|
| `coarse_step` | 0.05° | 粗搜精度远小于典型安装误差（1~3°），保证不漏极值 |
| `fine_tol` | 0.001° | 精搜精度是目标精度（0.2°）的 1/200，留出充裕裕量 |
| `search_range` | ±12° | 覆盖常见安装误差范围（通常 < 5°） |
| `min_snapshots` | 6 | 理论上 3 帧足够，6 帧提供统计平均抑制噪声 |
| `snapshot_interval` | 0.5 s | 对应 10 km/h 下约 1.4 m 的位移，保证快照位置多样性 |
| `cluster_tolerance` | 0.25 m | 大于锥桶直径（~0.3 m），小于锥桶间距（~1 m） |
| `cone_radius_max` | 0.35 m | 覆盖标准交通锥（底部直径约 0.4 m，取质心附近 0.35 m） |
