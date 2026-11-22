# lidar_gnss_yaw 使用手册

## 1. 功能概述

`lidar_gnss_yaw` 是一个 LiDAR + GNSS 联合 Yaw 角标定功能包。与静态单帧方法不同，本包利用车辆从多个方向经过同一批锥形桶时的多帧观测，通过全局投影一致性优化求解激光雷达相对于车辆坐标系的 Yaw 角安装偏差，理论精度优于 0.2°。

### 1.1 坐标系定义

- **传感器坐标系（LiDAR Frame，L）**：激光雷达本体坐标系
- **车辆坐标系（Vehicle/Body Frame，B）**：原点位于后轴中心，X 轴指向正前方，Y 轴指向左方，Z 轴指向上方（FLU）
- **全局坐标系（Global/ENU Frame，G）**：东-北-上（ENU），原点为 GNSS 参考点

### 1.2 算法数学原理

设激光雷达到车辆坐标系的安装变换为：

```
T_lb = { R_lb(δ), t_lb }
```

其中 δ 为待标定的 Yaw 安装偏差（包含名义安装角 `yaw_install_deg` 加上未知误差）。

设某时刻车辆 GNSS 位姿为 `(xg, yg, ψ)`（ENU 坐标，ψ 为导航航向角），则激光雷达观测到的锥桶点 `q`（LiDAR 坐标系下）投影到全局坐标系的公式为：

```
p_global = R_bg(ψ) · (R_lb(δ) · q + t_lb) + [xg, yg]
```

其中：

```
R_bg(ψ) = [ sin(ψ)  -cos(ψ) ]
           [ cos(ψ)   sin(ψ) ]
```

该矩阵将 FLU 车辆坐标系（X=前）变换到 ENU 全局坐标系（X=东），其中导航航向角 ψ 满足"正北为 0°，北偏东为正"的定义。

**优化目标**：若 δ 正确，则所有快照中同一根锥桶的全局投影点应严格共线（锥桶排成一直线）。因此，以所有快照锥桶全局投影点云的 PCA 最小特征值（即垂直直线方向的离散度）作为代价函数，最小化该值即可求出最优 δ。

```
δ* = argmin λ_min( PCA( { p_global(δ) } ) )
```

### 1.3 两阶段搜索策略

1. **粗搜索**：在 `[yaw_install_deg - search_range_deg, yaw_install_deg + search_range_deg]` 区间内以 `coarse_step_deg`（默认 0.05°）为步长逐点计算代价函数，找到全局最优区间。
2. **黄金分割精搜**：在粗搜结果附近区间内使用黄金分割法迭代，直到搜索区间小于 `fine_tol_deg`（默认 0.001°）。

最终标定精度优于 0.001°（数值精度），实际精度受场地和观测质量影响，一般优于 0.2°。

---

## 2. 前置条件

### 2.1 必须完成的标定

| 标定项 | 对应功能包 | 说明 |
|--------|------------|------|
| LiDAR Roll / Pitch / Z | `lidar_rp_cali` | 激光雷达俯仰/横滚/高度外参，须先于 Yaw 标定完成 |
| GNSS 安装外参 | 车辆 GNSS 标定流程 | GNSS 天线到后轴中心的偏移 (xg_offset, yg_offset) 及 heading 对准 |

### 2.2 验证方法

**验证 LiDAR Roll/Pitch/Z 已正确标定**：
```bash
# 播放录包后查看点云话题，地面点应集中在 VCS z ≈ -0.37m（z_offset 值），且平面度残差 < 1cm
rostopic echo /lidar_rp_cali/result
```
输出中 `roll_deg` 和 `pitch_deg` 应接近 0（误差 < 0.1°），`z_calib` 应接近地面高度。

**验证 GNSS 信号质量**：
```bash
rostopic echo /gnss_topic   # 查看 nav_msgs/Odometry，covariance 对角线应 < 0.01
```
确保 GNSS 在标定区域有稳定固定解（RTK Fix 状态）。

---

## 3. GNSS 话题说明

本包订阅 `nav_msgs/Odometry` 格式的 GNSS 消息。

### 3.1 消息字段映射

| 消息字段 | 含义 | 单位 |
|----------|------|------|
| `pose.pose.position.x` | ENU 坐标系下东向位置 | m |
| `pose.pose.position.y` | ENU 坐标系下北向位置 | m |
| `pose.pose.orientation` | 四元数表示的姿态 | — |

从四元数提取偏航角（`yaw_enu`，ENU 坐标系，逆时针为正），再转换为导航航向角：

```
heading = 90° - yaw_enu_deg
```

其中 `heading` 满足：正北为 0°，北偏东为正（顺时针为正），与地图/导航惯例一致。

**示例**：车辆正朝东行驶时，`yaw_enu ≈ 0°`，`heading = 90°`。

### 3.2 话题配置

在 `config/params.yaml` 中设置：
```yaml
gnss_topic: /your_gnss_topic   # 实际 GNSS 话题名
```

---

## 4. 场地要求

### 4.1 锥形桶布置

- 数量：6 个（建议）
- 位置：车辆正前方 5~40m 范围内
- 排列：严格排成一条直线（直线度误差 < 5cm）
- 锥桶间距：1.0~3.0m 均匀分布
- 周边：半径 1m 内无其他障碍物或杂物，避免干扰点云聚类

### 4.2 车辆行驶要求

| 要求项 | 规范 |
|--------|------|
| 经过方向数 | 至少 2~3 个不同方向 |
| 航向跨度 | 总航向跨度 ≥ 3°（`min_heading_spread_deg` 参数，建议 ≥ 10°） |
| 行驶速度 | < 10 km/h（约 2.8 m/s） |
| 观测距离 | 经过锥桶群时距离锥桶 5~20m |
| 快照数量 | 建议采集 10~30 帧有效快照（`min_snapshots = 6`，`max_snapshots = 60`） |

**航向多样性的重要性**：不同航向观测可有效解耦 Yaw 误差与锥桶位置不确定性，航向跨度越大，标定结果越鲁棒。仅从同一方向观测会导致欠约束，无法收敛到正确值。

---

## 5. 参数说明

所有参数位于 `config/params.yaml`，节点启动时加载。

### 5.1 话题参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `lidar_topic` | `/lidar_points` | 激光雷达点云输入话题（PointCloud2） |
| `gnss_topic` | `/gnss_odom` | GNSS 里程计话题（nav_msgs/Odometry，ENU） |

### 5.2 安装参数（installation 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `x_lb` | `3.5` | m | 激光雷达在车辆坐标系下的 X 安装位置（前正后负） |
| `y_lb` | `0.0` | m | 激光雷达在车辆坐标系下的 Y 安装位置（左正右负） |
| `z_lb` | `1.5` | m | 激光雷达在车辆坐标系下的 Z 安装高度 |
| `z_offset` | `0.37` | m | 后轴中心到地面高度，用于地面高度修正 |
| `yaw_install_deg` | `0.0` | ° | 激光雷达名义 Yaw 安装角（设计值），搜索以此为中心 |

### 5.3 采集条件参数（collection 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `min_snapshots` | `6` | 帧 | 开始优化所需的最小快照数 |
| `max_snapshots` | `60` | 帧 | 快照缓冲区上限，超过后停止采集 |
| `min_heading_spread_deg` | `3.0` | ° | 所有快照航向值的跨度下限，低于此值优化结果不可靠 |

### 5.4 锥桶检测参数（cone 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `height_min` | `0.08` | m | 锥桶点云高度（相对地面）最小值，过滤地面噪声 |
| `height_max` | `0.80` | m | 锥桶点云高度最大值，过滤高大目标 |
| `max_cluster_radius_xy` | `0.50` | m | 聚类水平半径上限，超过此值认为不是单个锥桶 |
| `min_cones` | `4` | 个 | 每帧有效快照所需的最少锥桶数 |
| `cluster_tolerance` | `0.30` | m | 欧式聚类距离阈值 |
| `min_cluster_points` | `5` | 点 | 聚类最少点数（过少则丢弃） |
| `max_cluster_points` | `500` | 点 | 聚类最多点数（过多则丢弃） |

### 5.5 搜索参数（yaw_cali 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `search_range_deg` | `12.0` | ° | 粗搜索范围（以 yaw_install_deg 为中心，±12°） |
| `coarse_step_deg` | `0.05` | ° | 粗搜索步长 |
| `fine_tol_deg` | `0.001` | ° | 黄金分割精搜收敛阈值 |

---

## 6. 编译与启动步骤

### 6.1 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali
catkin_make --only-pkg-with-deps lidar_gnss_yaw
source devel/setup.bash
```

### 6.2 修改配置文件

编辑 `src/lidar_gnss_yaw/config/params.yaml`，填写实际话题名和安装参数：

```yaml
lidar_topic: /rslidar_points        # 替换为实际激光雷达话题
gnss_topic: /gnss/odom              # 替换为实际 GNSS 话题

installation:
  x_lb: 3.5                        # 激光雷达前向安装距离（m）
  y_lb: 0.0
  z_lb: 1.5
  z_offset: 0.37
  yaw_install_deg: 0.0             # 名义安装角

cone:
  height_min: 0.08
  height_max: 0.80
  max_cluster_radius_xy: 0.50
  min_cones: 4

yaw_cali:
  min_snapshots: 6
  max_snapshots: 60
  min_heading_spread_deg: 3.0
  search_range_deg: 12.0
  coarse_step_deg: 0.05
  fine_tol_deg: 0.001
```

### 6.3 启动节点

```bash
roslaunch lidar_gnss_yaw lidar_gnss_yaw_cali.launch
```

或者手动启动：
```bash
rosrun lidar_gnss_yaw lidar_gnss_yaw_node \
    _lidar_topic:=/rslidar_points \
    _gnss_topic:=/gnss/odom
```

### 6.4 服务调用操作序列

标定流程严格按以下顺序执行：

**步骤 1：开始采集**
```bash
rosservice call /lidar_gnss_yaw/start "{}"
# 响应：success: True, message: "采集已开始，请驾驶车辆经过锥桶群"
```

**步骤 2：驾驶车辆绕行锥桶群**

- 车辆以 < 10 km/h 的速度从 2~3 个不同方向经过锥桶群
- 每次经过时距锥桶 5~20m，确保至少 4 个锥桶在视野内
- 期间观察 `~status` 话题，确认快照数量在增加
  ```bash
  rostopic echo /lidar_gnss_yaw/status
  ```

**步骤 3：停止采集**
```bash
rosservice call /lidar_gnss_yaw/stop "{}"
# 响应：success: True, message: "采集完成，共 N 帧快照，正在优化..."
# 优化完成后自动发布结果到 ~result 话题
```

**步骤 4：查看标定结果**
```bash
rostopic echo /lidar_gnss_yaw/result
# 输出：
#   vector.x = yaw_calibrated_deg   (最终标定 Yaw 角，单位 °)
#   vector.y = delta_yaw_deg        (Yaw 修正量，单位 °)
#   vector.z = rmse_m               (全局投影直线拟合残差，单位 m)
```

**步骤 5：保存结果**
```bash
rosservice call /lidar_gnss_yaw/save "{}"
# 响应：success: True, message: "结果已保存到 /tmp/lidar_gnss_yaw_result.yaml"
```

**步骤 6（可选）：重置**
```bash
rosservice call /lidar_gnss_yaw/reset "{}"
# 清空所有快照，恢复初始状态，可重新采集
```

---

## 7. 输出文件说明

标定结果保存到 `/tmp/lidar_gnss_yaw_result.yaml`，格式如下：

```yaml
# lidar_gnss_yaw 标定结果
# 生成时间：2026-03-26 10:30:00
timestamp: 1711418400.0
yaw_calibrated_deg: 2.153       # 最终标定 Yaw 角（安装角 + 修正量）
delta_yaw_deg: 2.153            # Yaw 修正量 = yaw_calibrated - yaw_install
rmse_m: 0.018                   # 全局投影直线拟合 RMSE（越小越好）
num_snapshots: 24               # 参与优化的快照数
heading_spread_deg: 28.5        # 实际航向跨度
yaw_install_deg: 0.0            # 名义安装角（输入值）
```

**将结果写入驱动/launch 文件**：
```yaml
# 在激光雷达驱动 launch 文件中更新 yaw 参数
<param name="yaw_install_deg" value="2.153"/>
```

---

## 8. 操作注意事项

### 8.1 航向多样性的重要性

航向跨度是影响标定质量的核心因素。实际测试表明：

- 航向跨度 < 3°：优化欠约束，结果不稳定，误差可能 > 1°
- 航向跨度 3°~10°：结果可用，误差约 0.2°~0.5°
- 航向跨度 > 10°：结果可靠，误差通常 < 0.2°

**建议**：从正面（0°方向）、左斜 15°、右斜 15° 三个方向各经过一次锥桶群，总航向跨度约 30°，可达最佳标定效果。

### 8.2 速度要求

- 行驶速度严格控制在 < 10 km/h（约 2.8 m/s）
- 速度过快导致点云运动畸变，影响锥桶中心定位精度
- 过弯时速度应进一步降低至 < 5 km/h

### 8.3 GNSS 质量要求

- 必须使用 RTK 固定解（Fix 状态），浮动解（Float）精度不足
- 采集期间确保 GNSS 无遮挡，天空开阔
- 若 GNSS 信号中断，该帧快照会被自动丢弃

### 8.4 锥桶排列质量

- 锥桶直线度越高，PCA 残差越小，优化结果越精确
- 可用激光测距仪或拉线方式辅助对齐
- 标定完成后可通过 `rmse_m` 评估锥桶直线度（< 0.03m 为优，< 0.05m 为良）

---

## 9. 常见问题

**Q1：服务 `start` 调用成功，但快照数量一直为 0，如何排查？**

A：检查以下几点：
1. 确认激光雷达和 GNSS 话题在发布：`rostopic hz /your_lidar_topic`
2. 确认 GNSS Odometry 消息中姿态四元数不全为零
3. 确认视野内有 ≥ `min_cones`（默认 4）个锥桶，可查看 `~cone_cloud` 话题的点云可视化
4. 确认点云中锥桶高度在 `height_min`~`height_max` 范围内，可适当调整参数

---

**Q2：调用 `stop` 后提示"航向跨度不足"，无法优化，怎么办？**

A：`min_heading_spread_deg` 默认为 3.0°，需要从不同方向观测锥桶。解决方法：
1. 调用 `reset` 重置
2. 重新规划路线，确保从至少两个方向（角度差 > 5°）经过锥桶群
3. 若场地受限，可将 `min_heading_spread_deg` 临时降低至 1.5°，但精度会降低

---

**Q3：标定结果 `delta_yaw_deg` 每次运行都略有不同（±0.1°），是否正常？**

A：正常现象。影响因素包括：GNSS 定位噪声、锥桶检测质量、快照采样位置差异。建议：
1. 增加快照数量（至少 15 帧以上）
2. 确保更大的航向跨度（> 15°）
3. 多次标定取平均值，若标准差 > 0.2° 则需检查 GNSS 质量或锥桶布置

---

**Q4：`rmse_m` 值很大（> 0.1m），说明什么问题？**

A：`rmse_m` 反映全局投影的直线拟合残差，偏大的原因：
1. 锥桶未严格排成直线（最常见原因）
2. GNSS 精度不足，存在漂移
3. 快照采集时车速过快导致点云畸变
4. LiDAR Roll/Pitch/Z 标定不准确，影响锥桶高度分割

---

**Q5：如何验证标定结果是否正确？**

A：
1. 保存结果后，将 `yaw_calibrated_deg` 写入激光雷达外参
2. 在 RViz 中查看车辆前进方向与路面标线/直线目标的对齐情况
3. 重新启动节点进行一次验证性采集：若 `delta_yaw_deg` 接近 0（< 0.2°），说明标定成功

---

**Q6：节点占用 CPU 较高，是否正常？**

A：在采集阶段（`start` 后）节点对每帧点云进行欧式聚类，计算量较大。优化时会进行密集搜索，CPU 会短暂升高。可通过以下方式降低负载：
1. 降低激光雷达点云频率（如从 20Hz 降为 10Hz）
2. 增大 `coarse_step_deg`（如 0.1°）以减少粗搜索次数（略微降低精度）
3. 优化结束后 CPU 占用会自动恢复正常

---

**Q7：结果文件 `/tmp/lidar_gnss_yaw_result.yaml` 找不到？**

A：`/tmp` 目录在系统重启后会被清空。建议：
1. 每次标定完成后立即将结果复制到项目目录：
   ```bash
   cp /tmp/lidar_gnss_yaw_result.yaml ~/lidar_calibration/
   ```
2. 若需更改保存路径，修改 `params.yaml` 中的 `output_file` 参数（若已配置），或修改源码中的默认路径
