# LiDAR-LiDAR 外参标定方案文档

## 1. 问题描述

无人驾驶车辆上装有多个激光雷达：

| 雷达 | 标定状态 | 坐标系输出 |
|------|----------|------------|
| 主激光雷达（Master）| 全部参数已标定 | VCS（车体坐标系）|
| 辅激光雷达（Sub）| roll / pitch / z 已标定 | 近似 VCS（x/y/yaw 待定）|

**本次标定目标**：确定辅激光雷达的安装参数 `x_install`、`y_install`、`yaw_install`，使辅雷达输出的点云在 VCS 中与主雷达精确对齐。

---

## 2. 坐标系约定

```
VCS（车体坐标系，FLU）
  原点：后轴中心（投影到地面 + z_offset 高度）
  X：车头正前方
  Y：车头正左方
  Z：正上方
  地面 z ≈ −z_offset（通常 z_offset = 0.37 m）
```

---

## 3. 数学模型

### 3.1 变换链

对辅激光雷达中的原始点 **q**（LiDAR 本体坐标），变换到 VCS 的完整链为：

```
p_vcs = R(roll, pitch, yaw) · q  +  [x, y, z]^T
```

其中旋转矩阵（ZYX 顺序）：

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

**已标定**：`roll_calib`、`pitch_calib`、`z_calib`
**待标定**：`x_install`、`y_install`、`yaw_install`

### 3.2 驱动输出约定

驱动以**当前配置值**（含安装标称值 `x_install, y_install, yaw_install`）将辅雷达点云变换到近似 VCS：

```
p_wrong = T_install · q_raw
        = Rz(yaw_s)·Ry(pitch)·Rx(roll)·q_raw  +  [x_s, y_s, z_calib]
```

真实的 VCS 变换应为：

```
p_true = T_calib · q_raw
       = Rz(yaw_t)·Ry(pitch)·Rx(roll)·q_raw  +  [x_t, y_t, z_calib]
```

### 3.3 标定目标

驱动当前用 T_s = (Rz(yaw_s), [x_s, y_s, z]) 变换，但 (x_s, y_s, yaw_s) 是安装标称值，存在误差。

标定求解总体参数 **(x_calib, y_calib, yaw_calib)**，满足：

```
T_calib · q_raw = p_correct（正确的 VCS 位置）

T_calib = (Rz(yaw_calib), [x_calib, y_calib, z])
```

将 (x_calib, y_calib, yaw_calib) 写入驱动配置后，辅雷达输出即为正确 VCS。

---

## 4. 标定算法

### 4.1 目标

求总体参数 **(x_calib, y_calib, yaw_calib)**，使得驱动改用

```
T_calib = (Rz(yaw_calib), [x_calib, y_calib, z])
```

变换原始点云后，输出正确的 VCS 坐标。

### 4.2 核心思路

驱动用安装标称值 T_s 已将辅雷达点云变换到近似 VCS：

```
p_secondary ≈ T_s · q_raw
```

**先消除 T_s 的影响**，将锥桶质心从近似 VCS 还原到「水平 LiDAR 坐标系」：

```
q_rp = Rz(−yaw_s) · (p_secondary − [x_s, y_s])
```

此时 q_rp ≈ R_rp · q_raw（仅 roll/pitch 微小修正），再用 ICP 直接求解 T_calib：

```
p_primary ≈ Rz(yaw_calib) · q_rp + [x_calib, y_calib]
```

**输出的 (x_calib, y_calib, yaw_calib) 即驱动配置的直接替换值。**

### 4.3 算法流程

```
输入：
  P = {p_i}    主雷达锥桶质心（VCS，已稳定化）
  Q = {q_j}    辅雷达锥桶质心（近似 VCS，驱动已应用 T_s）

Step 1 – 跨帧稳定化
  对最近 N 帧的所有锥桶检测，按 XY 距离（< cluster_merge_dist）归并为簇，
  取出现次数 >= min_obs_per_cone 的簇均值质心

Step 2 – 逆变换还原到水平 LiDAR 坐标系
  q_rp = Rz(−yaw_s) · (q − [x_s, y_s])   （对每个 q ∈ Q）

Step 3 – 初始化
  (x₀, y₀, yaw₀) ← 安装标称值 (x_s, y_s, yaw_s)

Step 4 – 粗搜索（可选）
  在 [x₀±range_x, y₀±range_y, yaw₀±range_yaw] 内网格遍历，
  目标函数 = 平均最近邻距离平方，找到最优初值

Step 5 – 2D ICP 精细化
  迭代（最多 max_icp_iter 次）：
  a. 用当前 (x, y, yaw) 映射 Q_rp → Q'
  b. 为每个 q'_j 在 P 中找最近邻 p_match（距离 < max_assoc_dist）
  c. 若有效配对 < min_cones → 失败
  d. SVD 求解最优 2D 刚体变换（见 4.4）→ 更新 (x, y, yaw)
  e. 检查收敛（Δtrans + Δangle < tol）

Step 6 – 输出
  x_calib, y_calib, yaw_calib（驱动直接使用）
  delta_x, delta_y, delta_yaw（= calib − install，仅参考）
  RMSE（锥桶对齐误差，m）
```

### 4.4 SVD 求解 2D 刚体变换

已知对应关系 `{ (s_i, d_i) }`，求最优变换使 `d_i ≈ R·s_i + t`：

```
1. 计算质心：s̄ = mean(s_i),  d̄ = mean(d_i)

2. 交叉协方差矩阵（2×2）：
   H = Σ (s_i − s̄)(d_i − d̄)^T

3. SVD 分解：H = U · Σ · V^T

4. 旋转矩阵（处理反射，确保 det(R) = +1）：
   D = diag(1, det(V·U^T))
   R = V · D · U^T

5. 平移向量：
   t = d̄ − R · s̄

6. 提取旋转角：
   δyaw = atan2(R(1,0), R(0,0))
```

---

## 5. 场地要求

| 要求 | 说明 |
|------|------|
| 地面 | 水平平地（roll/pitch 标定前提已满足）|
| 锥桶数量 | ≥ 3 个（推荐 4~6 个，越多越稳定）|
| 锥桶分布 | 分布在主/辅雷达共同视野内，**不能共线**（共线退化），建议散布在不同方位 |
| 锥桶间距 | 建议 1~5 m，避免过密（聚类合并）或过疏（超出 ROI）|
| 锥桶规格 | 标准反光锥桶，高约 0.75 m，底座直径约 0.30 m |
| 车辆状态 | 静止停放（不要求行驶）|
| 共视范围 | 确保主/辅雷达在标定区域内均能看到同一批锥桶 |

**推荐布置（俯视图）：**

```
        车头方向
            ↑
    [P4]       [P5]
  [P2]    [P3]
      [P1]

  ← 车辆 →  （后轴中心 = VCS 原点）
```

- 锥桶距车头 5~20 m，在主/辅雷达均可见的区域散布
- 不要将所有锥桶排成一条直线

---

## 6. 质量等级

| 等级 | RMSE | 锥桶对数 | 说明 |
|------|------|----------|------|
| EXCELLENT | < 0.03 m | ≥ 4 | 高精度，直接使用 |
| GOOD | < 0.06 m | ≥ 3 | 满足大多数应用 |
| ACCEPTABLE | < 0.10 m | ≥ 3 | 勉强可用，建议重试 |
| POOR | ≥ 0.10 m | 任意 | 请检查锥桶布置或参数 |

---

## 7. 误差来源与对策

| 误差来源 | 对策 |
|----------|------|
| 锥桶检测噪声 | 增加累积帧数（accumulate_frames），利用多帧均值抑制抖动 |
| 初始值偏差过大 | 启用粗搜索（enable_coarse_search: true），扩大搜索范围 |
| 锥桶数量不足 | 增加锥桶（min_cones ≥ 3），确保主/辅雷达共视 |
| 锥桶共线分布 | 调整锥桶布置，确保非共线 |
| 时间同步偏差 | 当前为静止场景，时间同步要求低；动态场景需添加硬件同步 |
| 外参初始误差过大 | 调整 params.yaml 中的 x_install/y_install/yaw_install 初值 |
