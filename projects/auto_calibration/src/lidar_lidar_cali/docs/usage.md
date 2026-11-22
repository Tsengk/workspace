# lidar_lidar_cali 使用手册

## 1. 功能概述

`lidar_lidar_cali` 是一个双激光雷达联合标定功能包，以主雷达（已完全标定）为参考，标定辅雷达相对于车辆坐标系（VCS）的 X/Y/Yaw 安装偏差。

### 1.1 应用场景

当车辆搭载多个激光雷达时，所有雷达必须在统一的车辆坐标系（VCS）下工作，才能进行正确的点云融合和感知处理。本包通过让两雷达同时观测同一批锥形桶，利用粗搜索 + 2D ICP 算法求解辅雷达相对于主雷达的修正变换，从而将辅雷达的 X/Y/Yaw 安装参数精确标定。

### 1.2 标定维度

本包标定辅雷达的三个自由度：

| 标定量 | 含义 |
|--------|------|
| Δx | 辅雷达在 VCS 中 X 方向（前后）位置修正量 |
| Δy | 辅雷达在 VCS 中 Y 方向（左右）位置修正量 |
| Δyaw | 辅雷达 Yaw 角修正量（绕 Z 轴旋转偏差） |

Roll、Pitch、Z 三个自由度需提前由 `lidar_rp_cali` 标定完成。

### 1.3 算法数学原理

#### 1.3.1 坐标变换模型

设辅雷达到 VCS 的安装变换为：

```
T_install = { Rz(yaw_install) · Rx(roll) · Ry(pitch), [x_install, y_install, z_install] }
```

其中 Roll/Pitch/Z 已由 `lidar_rp_cali` 标定，本包求解修正量 ΔT = (Rz(Δyaw), [Δx, Δy])。

最终标定变换为：

```
T_calib = ΔT · T_install
```

展开后各参数的更新公式：

```
yaw_calib = yaw_install + Δyaw

x_calib = x_install + Δx · cos(yaw_install) - Δy · sin(yaw_install)
y_calib = y_install + Δx · sin(yaw_install) + Δy · cos(yaw_install)
```

#### 1.3.2 2D ICP（SVD 求解）

设主雷达检测到的锥桶中心集合为 `P = {p_i}`（VCS XY 平面坐标），辅雷达检测到的锥桶中心集合为 `Q = {q_i}`（VCS 近似坐标），ICP 目标为求解 2D 刚体变换 `(R, t)`，最小化：

```
E(R, t) = Σ || p_i - (R · q_i + t) ||²
```

SVD 求解步骤：

1. 计算各自的质心：`μ_p = mean(P)`，`μ_q = mean(Q)`
2. 去质心：`p'_i = p_i - μ_p`，`q'_i = q_i - μ_q`
3. 构造协方差矩阵：`H = Σ q'_i · p'^T_i`（2×2 矩阵）
4. SVD 分解：`H = U · S · V^T`
5. 旋转矩阵：`R = V · U^T`
6. 平移向量：`t = μ_p - R · μ_q`

迭代执行上述步骤（最多 `max_icp_iter = 100` 次），直到变换量小于 `icp_tolerance = 1e-6`。

#### 1.3.3 粗搜索 + 精搜索两阶段策略

1. **粗搜索（网格搜索）**：
   - 在 Δx ∈ [-coarse_range_x, +coarse_range_x]（步长 coarse_step_x）
   - Δy ∈ [-coarse_range_y, +coarse_range_y]（步长 coarse_step_y）
   - Δyaw ∈ [-coarse_range_yaw, +coarse_range_yaw]（步长 coarse_step_yaw）
   - 对每个网格点，计算主辅锥桶中心的最近邻匹配残差，找到最优初值
2. **精搜索（2D ICP SVD）**：以粗搜结果为初始值，迭代精化至收敛

### 1.4 帧缓冲融合

为提升锥桶中心检测的稳定性，本包维护 `accumulate_frames`（默认 10 帧）的历史点云缓冲，对每一根锥桶的多帧中心观测进行聚类融合，取均值作为最终锥桶中心位置，有效抑制单帧噪声。

---

## 2. 前置条件

### 2.1 必须完成的标定

| 雷达 | 必须标定的量 | 对应功能包 | 备注 |
|------|------------|------------|------|
| 主雷达 | Roll / Pitch / Z / Yaw | `lidar_rp_cali` + `lidar_yaw1` 或 `lidar_gnss_yaw` | 完全标定，X/Y 通常由机械安装保证 |
| 辅雷达 | Roll / Pitch / Z | `lidar_rp_cali` | 仅需地面相关的 3 个自由度 |

**主雷达"完全标定"是本算法的核心前提**：算法以主雷达的 VCS 点云为基准真值，若主雷达存在未补偿的偏差，会直接传递到辅雷达标定结果中。

### 2.2 验证主雷达标定状态

**方法 1：通过 rosparam 检查**

```bash
# 查看主雷达当前生效的外参参数
rosparam get /primary_lidar_driver/yaw_install_deg
rosparam get /primary_lidar_driver/roll_calib_deg
rosparam get /primary_lidar_driver/pitch_calib_deg
rosparam get /primary_lidar_driver/z_calib
```

所有参数应与最新标定结果一致，确认驱动已加载更新后的外参。

**方法 2：在平坦地面上验证地面平面**

```bash
# 启动主雷达，在 RViz 中查看点云
# 地面点应集中在 VCS z ≈ −z_offset（如 −0.37m），平面度残差 < 2cm
rostopic echo /lidar_rp_cali/result
```

**方法 3：通过直线/墙壁验证 Yaw**

将车辆停在直线墙壁旁（约 5m），在 RViz 中查看主雷达点云。主雷达点云中墙壁应呈完美直线；若墙壁点云有弯曲（> 2cm），说明主雷达 Yaw 标定有误，需重新标定后再进行双雷达标定。

---

## 3. 场地要求

### 3.1 锥形桶布置

- 数量：至少 3 个，建议 5~8 个
- 位置：两雷达共视区域内（通常为车辆前方），距车辆 5~40m
- 分布：随机错落摆放，**无需排成直线**（与 `lidar_gnss_yaw` 不同）
- 间距：相邻锥桶间距 > 1.0m（避免欧式聚类将多个锥桶合并为一个）
- 周边：锥桶半径 1m 内无遮挡物，避免干扰聚类

示例布置（俯视图）：

```
    [C4]         [C5]
         [C2] [C3]
    [C1]

← 主/辅雷达共视区域 →
```

### 3.2 车辆状态

- 车辆**静止停放**，无需行驶
- 确保两雷达均有清晰的锥桶视野（在 RViz 中预先确认）
- 地面平坦（坡度 < 1°），减少 Roll/Pitch 补偿引入的误差

---

## 4. 参数说明

所有参数位于 `config/params.yaml`，节点启动时加载。

### 4.1 主雷达参数（primary_lidar 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `topic` | `/primary_lidar_points` | — | 主雷达点云话题（已经外参补偿的 VCS 点云） |
| `x_min` | `-1.0` | m | VCS X 方向感兴趣区域下限（剔除车身后方点云） |
| `x_max` | `30.0` | m | VCS X 方向感兴趣区域上限 |
| `y_min` | `-15.0` | m | VCS Y 方向感兴趣区域下限 |
| `y_max` | `15.0` | m | VCS Y 方向感兴趣区域上限 |

### 4.2 辅雷达参数（secondary_lidar 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `topic` | `/secondary_lidar_points` | — | 辅雷达原始点云话题 |
| `sensor_name` | `secondary_lidar` | — | 传感器名称，用于输出文件标识 |
| `roll_calib_deg` | `0.0` | ° | 辅雷达 Roll 标定值（来自 lidar_rp_cali） |
| `pitch_calib_deg` | `0.0` | ° | 辅雷达 Pitch 标定值（来自 lidar_rp_cali） |
| `z_calib` | `1.5` | m | 辅雷达 Z 高度标定值（来自 lidar_rp_cali，单位 m，为 VCS 高度） |
| `z_offset` | `0.37` | m | 后轴中心到地面高度 |
| `x_install` | `0.0` | m | 辅雷达 X 名义安装位置 |
| `y_install` | `0.0` | m | 辅雷达 Y 名义安装位置 |
| `yaw_install` | `0.0` | ° | 辅雷达 Yaw 名义安装角 |

### 4.3 锥桶检测参数（cone 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `height_min` | `0.08` | m | 锥桶点云高度（相对地面）最小值，过滤地面噪声 |
| `height_max` | `0.80` | m | 锥桶点云高度最大值，过滤高大目标 |
| `voxel_size` | `0.05` | m | 体素下采样分辨率，加速聚类处理 |
| `cluster_tolerance` | `0.30` | m | 欧式聚类距离阈值 |
| `min_cluster_points` | `5` | 点 | 有效聚类最少点数（过少则丢弃） |
| `max_cluster_points` | `500` | 点 | 有效聚类最多点数（过多则丢弃，可能是大物体） |
| `max_cluster_radius_xy` | `0.50` | m | 聚类水平半径上限（超过视为非锥桶） |
| `min_cones` | `2` | 个 | 单帧检测有效性所需最少锥桶数 |

### 4.4 标定算法参数（calibration 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `max_icp_iter` | `100` | 次 | ICP 最大迭代次数 |
| `icp_tolerance` | `1e-6` | m/rad | ICP 收敛阈值（变换增量小于此值则停止） |
| `max_assoc_dist` | `2.0` | m | ICP 点对匹配最大距离，超过则视为无效匹配 |
| `enable_coarse_search` | `true` | — | 是否启用粗搜索（强烈建议开启，关闭时 ICP 直接从名义值开始） |
| `coarse_range_x` | `2.0` | m | 粗搜索 X 范围（±2.0m） |
| `coarse_range_y` | `2.0` | m | 粗搜索 Y 范围（±2.0m） |
| `coarse_range_yaw_deg` | `30.0` | ° | 粗搜索 Yaw 范围（±30°） |
| `coarse_step_x` | `0.10` | m | 粗搜索 X 步长 |
| `coarse_step_y` | `0.10` | m | 粗搜索 Y 步长 |
| `coarse_step_yaw_deg` | `2.0` | ° | 粗搜索 Yaw 步长 |
| `accumulate_frames` | `10` | 帧 | 帧缓冲大小，多帧融合提升锥桶中心稳定性 |
| `cluster_merge_dist` | `0.40` | m | 多帧融合时相同锥桶的聚类合并距离 |
| `min_obs_per_cone` | `2` | 次 | 进入优化的锥桶最少观测帧数（低于此值的锥桶被过滤） |
| `min_cones` | `3` | 个 | 触发标定所需的最少有效匹配锥桶对数 |

### 4.5 输出参数（output 节）

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `file` | `/tmp/lidar_lidar_calib_result.yaml` | 标定结果保存路径 |

---

## 5. 编译与启动步骤

### 5.1 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali
catkin_make --only-pkg-with-deps lidar_lidar_cali
source devel/setup.bash
```

### 5.2 填写配置文件

编辑 `src/lidar_lidar_cali/config/params.yaml`，填入实际参数：

```yaml
primary_lidar:
  topic: /rslidar_main_points      # 主雷达已标定 VCS 点云话题
  x_min: -1.0
  x_max: 30.0
  y_min: -15.0
  y_max: 15.0

secondary_lidar:
  topic: /rslidar_sub_points       # 辅雷达原始点云话题
  sensor_name: rslidar_sub
  roll_calib_deg: 0.12             # 来自辅雷达 lidar_rp_cali 结果
  pitch_calib_deg: -0.08           # 来自辅雷达 lidar_rp_cali 结果
  z_calib: 1.48                    # 来自辅雷达 lidar_rp_cali 结果
  z_offset: 0.37
  x_install: 0.0                   # 机械安装设计值（m）
  y_install: -0.8                  # 机械安装设计值（辅雷达在车辆右侧）
  yaw_install: 0.0                 # 机械安装设计值（°）

cone:
  height_min: 0.08
  height_max: 0.80
  voxel_size: 0.05
  cluster_tolerance: 0.30
  min_cluster_points: 5
  max_cluster_points: 500
  max_cluster_radius_xy: 0.50
  min_cones: 2

calibration:
  max_icp_iter: 100
  icp_tolerance: 1.0e-6
  max_assoc_dist: 2.0
  enable_coarse_search: true
  coarse_range_x: 2.0
  coarse_range_y: 2.0
  coarse_range_yaw_deg: 30.0
  coarse_step_x: 0.10
  coarse_step_y: 0.10
  coarse_step_yaw_deg: 2.0
  accumulate_frames: 10
  cluster_merge_dist: 0.40
  min_obs_per_cone: 2
  min_cones: 3

output:
  file: /tmp/lidar_lidar_calib_result.yaml
```

### 5.3 验证主雷达标定参数已生效

在启动标定节点前，确认主雷达驱动已加载正确外参：

```bash
# 检查主雷达驱动的外参 rosparam
rosparam get /primary_lidar_driver/yaw_calib_deg
rosparam get /primary_lidar_driver/roll_calib_deg
rosparam get /primary_lidar_driver/pitch_calib_deg

# 在 RViz 中订阅主雷达点云，验证地面高度约为 -0.37m
# 验证直线墙壁点云无明显弯曲
```

### 5.4 启动标定节点

```bash
roslaunch lidar_lidar_cali lidar_lidar_cali.launch
```

或者手动启动：

```bash
rosrun lidar_lidar_cali lidar_lidar_cali_node
```

### 5.5 服务调用操作序列

**步骤 1：确认锥桶可见**

在 RViz 中订阅 `~primary_cones` 和 `~secondary_cones`，确认两雷达均能检测到足够的锥桶：

```bash
rostopic echo /lidar_lidar_cali/status
# 应显示类似："主雷达检测到 5 个锥桶，辅雷达检测到 5 个锥桶，等待采集满 10 帧..."
```

**步骤 2：触发标定**

等待 `accumulate_frames` 帧采集完成（通常约 1 秒），然后调用：

```bash
rosservice call /lidar_lidar_cali/trigger_calibration "{}"
# 响应：success: True
# message: "[GOOD] Δx=0.047m Δy=-0.012m Δyaw=0.23° | x_calib=0.047m y_calib=-0.812m yaw_calib=0.23° | 锥桶对=5 RMSE=0.018m"
```

**步骤 3：查看结果**

```bash
rostopic echo /lidar_lidar_cali/result
# 输出：
#   vector.x = x_calib        (辅雷达标定后 X 安装量，单位 m)
#   vector.y = y_calib        (辅雷达标定后 Y 安装量，单位 m)
#   vector.z = yaw_calib_deg  (辅雷达标定后 Yaw 角，单位 °)
```

**步骤 4：保存结果**

```bash
rosservice call /lidar_lidar_cali/save_calibration "{}"
# 响应：success: True, message: "结果已保存到 /tmp/lidar_lidar_calib_result.yaml"
```

**步骤 5（可选）：重置**

```bash
rosservice call /lidar_lidar_cali/reset_calibration "{}"
# 清空帧缓冲，重新开始采集
```

---

## 6. 输出文件说明

标定结果保存到 `/tmp/lidar_lidar_calib_result.yaml`，格式如下：

```yaml
# lidar_lidar_cali 标定结果
# 生成时间：2026-03-26 11:00:00
timestamp: 1711421200.0
sensor_name: rslidar_sub

# 最终标定参数（应写入辅雷达驱动配置）
x_calib: 0.047
y_calib: -0.812
yaw_calib_deg: 0.23

# 修正量（相对于名义安装值的偏差）
delta_x: 0.047
delta_y: -0.012
delta_yaw_deg: 0.23

# 输入的名义安装参数
x_install: 0.0
y_install: -0.8
yaw_install_deg: 0.0

# 已用于补偿的 Roll/Pitch/Z
roll_calib_deg: 0.12
pitch_calib_deg: -0.08
z_calib: 1.48

# 标定质量指标
icp_rmse_m: 0.018           # ICP 收敛后的匹配残差（<0.05m 为优）
num_cones_used: 5            # 参与标定的锥桶对数
icp_iterations: 23           # ICP 实际迭代次数
```

---

## 7. 标定后如何更新驱动参数

### 7.1 方式一：直接修改驱动 launch 文件

打开辅雷达驱动的 launch 文件，将标定结果写入：

```xml
<!-- 辅雷达外参（由 lidar_lidar_cali 标定得到） -->
<param name="x_calib"         value="0.047"/>
<param name="y_calib"         value="-0.812"/>
<param name="yaw_calib_deg"   value="0.23"/>
<param name="roll_calib_deg"  value="0.12"/>
<param name="pitch_calib_deg" value="-0.08"/>
<param name="z_calib"         value="1.48"/>
```

### 7.2 方式二：通过外参配置文件

若驱动支持外参 YAML 文件，将 `/tmp/lidar_lidar_calib_result.yaml` 中 `x_calib`、`y_calib`、`yaw_calib_deg` 三个值复制到对应配置文件，重启驱动节点。

### 7.3 验证更新效果

更新参数并重启驱动后，在 RViz 中叠加主辅雷达点云（使用相同坐标系 `base_link`），对比同一静止锥桶的点云：

- 两雷达点云中同一锥桶中心的偏差应 < 5cm
- 地面点云高度应一致（均约为 −z_offset m）
- 直线边缘（如墙壁、路沿）在两雷达点云中应对齐

---

## 8. 手动点击模式（lidar_lidar_cali_clicked_node）

### 8.1 概述

`lidar_lidar_cali_clicked_node` 是**手动标定模式**节点，不需要 FAST-LIVO2 标定板，改用**锥形桶**作为对应点目标。操作员在 RViz 中分别点击主雷达和辅雷达视图中的同一批锥桶，节点自动提取各锥桶顶部位置，通过 **2D SVD（Procrustes 配准）**求解辅雷达的精确 x、y、yaw 外参。

**核心差异**：

| 对比项 | 标准模式 | 手动模式 |
|--------|---------|---------|
| 目标物 | FAST-LIVO2 标定板（4 圆孔） | 锥形桶（≥ 3 个，非共线） |
| 点采集 | 双雷达自动同步检测 | 用户逐一点击，服务切换活跃雷达 |
| 配准算法 | 3D SVD（4 点对） | 2D SVD Procrustes（N 点对） |
| 区分主/辅 | 固定订阅两路话题 | **`select_primary` / `select_secondary` 服务切换** |

---

### 8.2 锥桶布置要求

| 项目 | 要求 |
|------|------|
| 数量 | ≥ 3 个（2 个仅可求 x/y，yaw 不稳定）；推荐 4~6 个 |
| 排列 | **禁止共线**（全部在一条直线上时 yaw 退化）；推荐三角形或 L 形分布 |
| 位置 | 两雷达视野重叠区域内（一般为车辆前方 3~15m） |
| 高度 | 标准锥形桶（约 50~70cm），两雷达均能看到桶顶 |

---

### 8.3 使用流程

#### 第一步：启动节点

```bash
source /home/idriver/work/claude/test_lidar_rp_cali/devel/setup.bash
roslaunch lidar_lidar_cali lidar_lidar_cali_clicked.launch
```

终端输出：
```
[lidar_lidar_cali_clicked] 当前活跃雷达: primary
[lidar_lidar_cali_clicked] 用 select_primary / select_secondary 服务切换
```

#### 第二步：配置 RViz

1. **Fixed Frame** 设为 `base_link`
2. 添加两个 PointCloud2 显示，分别订阅主雷达和辅雷达话题
3. 添加 PointCloud2 显示 `~primary_cones`（绿色）和 `~secondary_cones`（蓝色）用于确认已选锥桶
4. 添加 MarkerArray 显示 `~cone_markers`，查看 M1/M2...（主）和 S1/S2...（辅）标签
5. 工具栏选 **"Publish Point"**（G 键）

#### 第三步：点击主雷达锥桶

节点启动后默认为**主雷达模式**（active = primary）。在主雷达点云中依次点击各锥桶（建议点到同一组锥桶即可，顺序不再要求固定）：

```bash
# 确认当前为主雷达模式（可选）
rosservice call /lidar_lidar_cali_clicked/select_primary
```

点击锥桶后：
```
[lidar_lidar_cali_clicked] 主雷达锥桶 #1: [8.24, -1.02, -0.018]
[lidar_lidar_cali_clicked] 主雷达锥桶 #2: [10.15,  0.35, -0.025]
[lidar_lidar_cali_clicked] 主雷达锥桶 #3: [ 9.87,  2.18, -0.020]
```

#### 第四步：切换到辅雷达并点击相同锥桶

```bash
rosservice call /lidar_lidar_cali_clicked/select_secondary
```

在**辅雷达点云**中点击同一组锥桶即可，节点会自动匹配主/辅锥桶的对应关系，点击顺序不需要与主雷达保持一致。

当辅雷达点击数量 = 主雷达点击数量 ≥ `min_cones` 时，自动触发 2D SVD 标定：

```
[lidar_lidar_cali_clicked] [GOOD] 锥桶=3 | x_calib=0.421m y_calib=-0.183m yaw_calib=-1.23°
                           | Δx=0.021m Δy=-0.033m Δyaw=-1.23° RMSE=0.015m
```

#### 第五步：保存结果

```bash
rosservice call /lidar_lidar_cali_clicked/save_calibration
# 输出: /tmp/lidar_lidar_calib_clicked.yaml
```

**重置（重新采集）**：
```bash
rosservice call /lidar_lidar_cali_clicked/reset_calibration
# 同时清空主/辅锥桶列表，活跃雷达重置为 primary
```

---

### 8.4 雷达切换服务说明

| 服务名 | 功能 |
|--------|------|
| `~select_primary` | 后续 `/clicked_point` 点击发送到**主雷达**锥桶列表 |
| `~select_secondary` | 后续 `/clicked_point` 点击发送到**辅雷达**锥桶列表 |
| `~trigger_calibration` | 主动触发标定（取主/辅较少的一方数量做截断） |
| `~reset_calibration` | 清空所有锥桶，活跃雷达重置为 primary |
| `~save_calibration` | 保存最新标定结果到 yaml |

> **注意**：点击事件只进入**当前活跃雷达**的列表，切换服务不影响已收集的锥桶。

---

### 8.5 手动模式参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `search_radius` | `0.5` m | 点击点周围搜索锥桶点云的 2D 水平半径 |
| `min_cluster_pts` | `3` | 邻域内最少点数 |
| `min_cones` | `3` | 自动触发标定所需的最少锥桶对数（建议 ≥ 3） |
| `secondary_lidar/*` | 同标准模式 | 辅雷达安装参数（x_install/y_install/yaw_install_deg） |

---

### 8.6 手动模式输出文件格式

```yaml
lidar_sub:

  installation:
    x_install:       0.400    # m（标称）
    y_install:      -0.150    # m（标称）
    z_calib:         1.430    # m（已标定，不变）
    roll_calib_deg:  0.000    # °（已标定，不变）
    pitch_calib_deg: 0.000    # °（已标定，不变）
    yaw_install_deg: 0.000    # °（标称）
    z_offset:        0.370

  calibrated:
    x:         0.421    # m  ← 写入驱动
    y:        -0.183    # m  ← 写入驱动
    z:         1.430    # m（不变）
    roll_deg:  0.000    # °（不变）
    pitch_deg: 0.000    # °（不变）
    yaw_deg:  -1.230    # °  ← 写入驱动

  delta:
    delta_x_m:     0.021
    delta_y_m:    -0.033
    delta_yaw_deg: -1.230

  quality:
    grade:       GOOD
    rmse_m:      0.015    # 2D 锥桶对齐 RMSE (m)，越小越好
    n_cones:     3        # 锥桶对数量（代替场景数）
    timestamp:   2026-04-01 10:30:00
```

**手动模式质量等级**：

| 等级 | RMSE | 锥桶对数 | 建议 |
|------|------|----------|------|
| EXCELLENT | < 0.03 m | ≥ 4 | 可直接使用 |
| GOOD | < 0.05 m | ≥ 3 | 可使用 |
| ACCEPTABLE | < 0.10 m | ≥ 2 | 建议增加锥桶数 |
| POOR | ≥ 0.10 m | — | 检查锥桶对应顺序或点击位置 |

---

### 8.7 常见问题（手动模式）

**Q：两路雷达坐标系不同，直接在同一 RViz 视图下比较位置时会偏移**

正常现象。辅雷达点云在驱动使用标称安装参数变换到 VCS 后，若标称值有误差，则显示偏移。标定的目的正是修正这个偏差。点击时只需关注**同一锥桶在对应雷达点云中的位置**即可。

**Q：点击顺序搞错了（主/辅锥桶顺序不同）**

调用 `reset_calibration` 重新采集。这是 2D SVD 的核心假设（同序号对应），顺序错误会导致标定结果完全错误。

**Q：标定结果 yaw 异常大（> 5°）**

可能原因：
1. 主/辅锥桶对应顺序搞错 → 重置后重新采集
2. 所有锥桶共线（全部在 X 轴上）→ 调整锥桶排列为非共线形状（L 形、三角形）
3. 点击位置离锥桶太远（点到地面或其他物体）→ 检查绿色/蓝色点云是否准确覆盖锥桶

**Q：如何同时运行标准模式和手动模式进行对比**

两个节点名称不同，可同时运行：
```bash
# 终端 1（标准模式，需要标定板）
roslaunch lidar_lidar_cali lidar_lidar_cali.launch
# 终端 2（手动模式，使用锥桶）
roslaunch lidar_lidar_cali lidar_lidar_cali_clicked.launch
```


## 9. 常见问题

**Q1：调用 `trigger_calibration` 提示"有效锥桶对数不足（需要 3 个）"，如何排查？**

A：检查以下几点：
1. 在 RViz 中查看 `~primary_cones` 和 `~secondary_cones`，确认两雷达各自能检测到足够锥桶
2. 检查共视区域：辅雷达可能视野不覆盖主雷达前方的锥桶，需调整锥桶摆放位置至两雷达共视区
3. 调整 `height_min/max` 参数，适当放宽范围（如 height_max 增大至 1.0m）
4. 检查 `min_obs_per_cone` 参数，若帧缓冲中某些锥桶观测次数少于此值，会被过滤
5. 确认 `accumulate_frames` 帧已全部采集完成（等待约 1~2 秒后再触发）

---

**Q2：主辅雷达检测到的锥桶数量不同（如主 5 个，辅 3 个），标定能否进行？**

A：可以进行。算法会对主辅锥桶中心进行最近邻匹配（`max_assoc_dist = 2.0m`），只使用匹配成功的锥桶对参与 ICP。只要匹配到的锥桶对 ≥ `min_cones = 3` 个，标定即可进行。建议调整辅雷达的感兴趣区域参数或锥桶布置，争取两雷达均能看到 ≥ 5 个锥桶，以提升精度。

---

**Q3：ICP 收敛后的 `icp_rmse_m` 较大（> 0.05m），如何改善？**

A：可能原因与改进方法：
1. **粗搜索初值不准**：若初始偏差大于粗搜索范围，ICP 可能陷入局部最优。可适当增大 `coarse_range_x/y/yaw_deg` 参数
2. **锥桶检测不稳定**：增大 `accumulate_frames` 至 20~30 帧，使锥桶中心均值更稳定
3. **两雷达时间同步问题**：若两雷达时间戳不同步（> 50ms 偏差），运动中采集会引入位置误差，确保静止状态下标定或使用硬件触发同步
4. **辅雷达 Roll/Pitch/Z 标定残余误差**：检查辅雷达地面点高度，若地面不平则重新进行 `lidar_rp_cali` 标定

---

**Q4：每次调用 `trigger_calibration` 的结果略有不同（Δx/Δy 差异约 ±0.02m），是否正常？**

A：在合理范围内是正常的。每次采集的 10 帧点云存在噪声，导致锥桶中心有轻微波动。改善方法：
1. 增大 `accumulate_frames`（如 30 帧）提升锥桶中心精度
2. 在标定结果稳定后（连续 3 次结果差异 < 0.01m 且 < 0.1°），取最后一次结果
3. 若差异 > 0.05m，需排查锥桶检测是否不稳定（查看 `~status` 输出中的锥桶数量波动）

---

**Q5：辅雷达的安装偏差 Δyaw 较大（> 5°），粗搜索能找到正确解吗？**

A：粗搜索范围为 ±`coarse_range_yaw_deg`（默认 ±30°），可覆盖较大的偏差。若偏差可能 > 30°，需增大此参数（如设为 45°），但同时粗搜索时间会增加。建议先通过目视检查点云对齐情况，估计大致偏差范围，再相应调整参数。

---

**Q6：如何判断标定是否成功？**

A：成功的标定应同时满足以下指标：
1. `icp_rmse_m < 0.05m`（优良 < 0.03m）
2. 在 RViz 中主辅雷达同一锥桶点云中心重合，偏差 < 5cm
3. 两雷达地面点云高度一致（均约 −z_offset m）
4. 重复调用标定服务，结果稳定（差异 < 0.02m，< 0.1°）

---

**Q7：粗搜索耗时较长（> 10 秒），如何加速？**

A：粗搜索网格点数 = (2×range_x/step_x + 1) × (2×range_y/step_y + 1) × (2×range_yaw/step_yaw + 1)。默认参数下约为 41 × 41 × 31 ≈ 52,000 个网格点。若耗时过长，可通过以下方式加速：
1. 增大步长：`coarse_step_x/y` 改为 0.20m，`coarse_step_yaw_deg` 改为 4°（略微降低精度）
2. 缩小范围：若已通过机械测量知道大致安装偏差，缩小 `coarse_range_x/y/yaw_deg`
3. 若已确认安装偏差 < 0.5m 且 < 5°，可将范围缩小至 (0.5m, 0.5m, 5°)，步长保持 0.1m/2°，网格点数降至约 121 × 121 × 6 ≈ 87,000... 仍较多，此时建议同时增大步长以减少网格点数
