# lidar_gnss_yaw 使用说明

## 1. 功能概述

`lidar_gnss_yaw` 在车辆**行驶状态**下，利用 GNSS 全局位姿与 LiDAR 对锥形桶的检测，联合标定 LiDAR 的 **yaw 安装角**（偏航角）。

标定完成后输出：
- `yaw_calibrated`：LiDAR 在车体坐标系中的实际 yaw 角（度）
- `delta_yaw`：相对先验安装值的修正量（度）
- `rmse_m`：标定质量指标（直线拟合残差，m）

**精度要求：< 0.2°**

---

## 2. 环境与前提要求

### 2.1 软硬件前提

- ROS1（Noetic/Melodic）
- 已完成 GNSS 标定，GNSS 发布 `nav_msgs/Odometry`（**UTM 坐标系，xg=北，yg=东**）
- LiDAR 驱动正常，发布 `sensor_msgs/PointCloud2`
- 编译依赖：PCL ≥ 1.8，Eigen3，tf2

### 2.2 场地要求

| 要求 | 说明 |
|------|------|
| 平地 | 坡度 < 1°，避免坡地导致地面去除失效 |
| 锥形桶数量 | **4~8 个**（最少 3 个/帧有效），建议 6 个 |
| 锥桶间距 | **1~2 m**，不宜过密（影响聚类分离）或过疏（减少约束） |
| 锥桶排列 | **成一条直线**，总长度 6~15 m |
| 净空 | 锥桶周围 3 m 内无其他障碍物（车辆、路沿、墙壁等） |
| 开阔天空 | GNSS 上方无遮挡，保证定位精度 |

### 2.3 驾驶要求

| 要求 | 说明 |
|------|------|
| 速度 | < 10 km/h（约 2.8 m/s） |
| 路线 | 建议从 **2~3 个不同方向**经过锥桶区域（见第 3 节） |
| 航向跨度 | 各次经过时的 GNSS 航向差异 ≥ **5°**（越大越好） |
| 最小采集量 | 节点采集满 6 帧有效快照后方可触发标定 |

---

## 3. 推荐标定场景

### 场景 A（推荐）：两次不同方向驶过

```
        锥桶阵列（南北向）
        N ↑
          │
    ●─────┼─────●─────●─────●─────●
          │
    ①→→→→│→→→→→→→→→→→→→→→→  （车辆向东驶过，heading ≈ 90°）
          │
    ②↗↗↗↗│↗↗↗↗↗↗↗↗↗↗↗↗↗↗  （车辆向东北驶过，heading ≈ 75°）
          │
```

两次 heading 差约 15°，标定效果好。

### 场景 B（最简）：单次直线驶过

```
        锥桶阵列（任意方向）
    ①→→→→→→→→→→→→→→→→→→→→  （单次驶过，heading 保持基本不变）
```

单次驶过也可完成标定，但建议驾驶距离 ≥ 20 m（产生足够的位移多样性）。

### 场景 C（最优）：三角形路线

```
          ●
         ↗  ↘  锥桶阵列在中间
        ③    ①→→→→→→→→→
         ↖  ↙
          ②
```

三个方向经过，航向跨度可达 30°+，精度最高。

---

## 4. 安装与编译

```bash
# 进入工作空间
cd /home/idriver/work/claude/test_lidar_rp_cali

# 编译
catkin_make --only-pkg-with-deps lidar_gnss_yaw

# source
source devel/setup.bash
```

---

## 5. 参数配置

编辑 `src/lidar_gnss_yaw/config/params.yaml`，重点关注以下参数：

### 5.1 必须修改

```yaml
# 话题名称（改为实际话题）
lidar_topic: /rslidar_points     # 你的 LiDAR 点云话题
gnss_topic:  /sensorgps          # 你的 GNSS Odometry 话题

# LiDAR 安装参数（相对后轴中心）
installation:
  x_offset:    2.5     # LiDAR 距后轴前方距离（m），如 2.5 m
  y_offset:    0.0     # 横向偏置（m），居中则填 0
  z_offset:    1.8     # LiDAR 安装高度（m，仅记录用）
  axis_offset: 0.37    # 后轴中心到地面高度（m），决定地面 z 估计
  yaw_install: 0.0     # 先验安装 yaw（°），必须与驱动当前配置一致！

# 结果保存路径
save_path: /tmp/lidar_gnss_yaw_result.yaml
```

### 5.2 锥桶检测调优（通常默认值已足够）

```yaml
cone:
  height_min: 0.15   # 若使用小锥桶（< 30 cm），适当减小
  height_max: 0.90   # 若使用大锥桶（> 70 cm），适当增大
  max_range:  25.0   # 若锥桶较近，可减小以降低误检
```

### 5.3 输入点云要求

**输入点云必须是驱动已经用安装参数（含 `yaw_install`）变换到 VCS（车体坐标系）后的点云**，不是原始 LiDAR 坐标系的点云。本模块直接在 VCS 中检测锥桶，求取安装 yaw 的残差修正量。

> **关键**：`params.yaml` 中的 `yaw_install` 必须与驱动当前使用的安装 yaw 保持一致。若不一致，标定结果 `delta_yaw` 将包含两者的差值，导致错误。

### 5.4 GNSS 话题格式要求

节点订阅 `nav_msgs/Odometry`，**UTM 坐标系**（xg=北，yg=东）：

| 字段 | 含义 |
|------|------|
| `pose.pose.position.x` | xg，北向坐标 Northing（m） |
| `pose.pose.position.y` | yg，东向坐标 Easting（m） |
| `pose.pose.orientation` | 四元数（xg=北、yg=东、z=Up 系，yaw=0 朝北，顺时针为正） |

**航向提取公式（已内置）：**

```cpp
heading = atan2(2(wz + xy), 1 - 2(y² + z²)) * 180/π
// 在 xg=北、yg=东 坐标系中，yaw 直接等于导航航向，无需 90° 偏置
```

> **与 ENU 的区别**：若你的 GNSS 使用 ROS 标准 ENU（x=东，y=北），需将上式改为 `heading = 90° − yaw_enu_deg`，并同步修改 `yaw_calibrator.cpp` 中的 R_bg 矩阵（改回 ENU 版本）。

---

## 6. 标定操作步骤

### 步骤 1：准备场地

按第 2 节要求摆放锥桶，确认区域内无干扰物体。

### 步骤 2：启动节点

```bash
roslaunch lidar_gnss_yaw lidar_gnss_yaw_cali.launch
```

启动后检查状态：

```bash
rostopic echo /lidar_gnss_yaw/status
# 预期输出：IDLE: call ~start 开始采集
```

### 步骤 3：检验锥桶检测（可选但推荐）

在 RViz 中订阅 `/lidar_gnss_yaw/cone_cloud`，手动将车辆停在锥桶附近，确认锥桶点云被正确着色标记。若无点云或点云不对，先调整 `cone` 参数再继续。

### 步骤 4：开始采集

```bash
rosservice call /lidar_gnss_yaw/start "{}"
# 预期返回：success: True, message: "采集已启动"
```

### 步骤 5：驾驶采集

- 按第 3 节推荐路线缓慢驾车，速度 < 10 km/h
- 实时监控采集进度：

```bash
rostopic echo /lidar_gnss_yaw/status
# 预期：COLLECTING: 3/6 快照, 跨度=8.5°
```

- 观察 `/lidar_gnss_yaw/cone_cloud`，确认每次经过锥桶时有点云输出

### 步骤 6：停止并触发标定

当采集量足够（status 显示快照数 ≥ min_snapshots）：

```bash
rosservice call /lidar_gnss_yaw/stop "{}"
```

系统自动执行标定，控制台输出：

```
[LidarGnssYaw] ============ 标定结果 ============
  先验安装 yaw:     0.0000 °
  标定后 yaw:       1.2340 °
  修正量 delta_yaw: 1.2340 °
  直线拟合 RMSE:    0.0123 m (1.23 cm)
  航向角跨度:       12.50 °
  有效快照:         15
  锥桶检测总数:     87
=====================================
```

同时 `/lidar_gnss_yaw/result` 发布：
- `vector.x` = yaw_calibrated（°）
- `vector.y` = delta_yaw（°）
- `vector.z` = rmse_m（m）

### 步骤 7：保存结果

```bash
rosservice call /lidar_gnss_yaw/save "{}"
# 预期：success: True, message: "已保存到 /tmp/lidar_gnss_yaw_result.yaml"
```

查看结果文件：

```bash
cat /tmp/lidar_gnss_yaw_result.yaml
```

```yaml
# LiDAR-GNSS 联合 yaw 标定结果
# 生成时间: 2026-03-25 10:30:00
installation:
  yaw_install:    0.000000  # 先验安装值（°）
  yaw_calibrated: 1.234000  # 标定后 yaw（°），CCW正，车体坐标系
  delta_yaw:      1.234000  # 修正量（°）

quality:
  rmse_m:         0.012300  # 直线拟合 RMSE（m）
  rmse_cm:        1.230000  # 直线拟合 RMSE（cm）
  heading_spread: 12.500000 # 航向角跨度（°）
  num_snapshots:  15
  num_cones_total: 87
```

### 步骤 8：应用标定结果

将 `yaw_calibrated` 更新到 LiDAR 驱动的外参配置中，替换原 `yaw_deg` 字段。例如：

```yaml
# 驱动外参配置文件
installation:
  yaw_deg: 1.234   # 从 lidar_gnss_yaw 标定结果填入
```

如需**重新标定**，执行：

```bash
rosservice call /lidar_gnss_yaw/reset "{}"
# 然后重复步骤 4~7
```

---

## 7. 话题与服务参考

### 订阅话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `lidar_topic`（可配置） | `sensor_msgs/PointCloud2` | LiDAR 原始点云 |
| `gnss_topic`（可配置）  | `nav_msgs/Odometry` | GNSS 全局位姿（UTM, xg=北, yg=东） |

### 发布话题（均在 `~` 私有命名空间）

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `~cone_cloud` | `sensor_msgs/PointCloud2` | 检测到的锥桶点云（可视化） |
| `~result` | `geometry_msgs/Vector3Stamped` | 标定结果（x=yaw_cal, y=delta, z=rmse） |
| `~status` | `std_msgs/String` | 状态文本 |

### 服务（均在 `~` 私有命名空间）

| 服务 | 类型 | 说明 |
|------|------|------|
| `~start` | `std_srvs/Trigger` | 开始数据采集 |
| `~stop`  | `std_srvs/Trigger` | 停止采集并执行标定 |
| `~save`  | `std_srvs/Trigger` | 保存结果到文件 |
| `~reset` | `std_srvs/Trigger` | 重置，清空已采集数据 |

---

## 8. RViz 可视化配置

在 RViz 中添加以下 Display：

| Display 类型 | Topic | 用途 |
|---|---|---|
| PointCloud2 | `/lidar_gnss_yaw/cone_cloud` | 实时查看锥桶检测效果 |
| String | `/lidar_gnss_yaw/status` | 查看标定进度 |
| PointCloud2 | 原始 LiDAR 话题 | 参考背景点云 |
| TF | — | 查看坐标系 |

---

## 9. 故障排查

### 问题：快照数一直不增加

**检查步骤：**
1. 确认 GNSS 数据正常：`rostopic hz /sensorgps`（应有输出）
2. 确认 LiDAR 数据正常：`rostopic hz /rslidar_points`（应有输出）
3. 查看 RViz 中 `cone_cloud`，确认锥桶被检测到
4. 增大 `cone/max_range` 或减小 `cone/min_cluster_points`

### 问题：status 显示 `快照不足`

标定被拒绝，快照数 < `min_snapshots`。解决方法：
- 延长驾驶路线，多次经过锥桶
- 降低 `snapshot_interval`（如从 0.5 s 改为 0.3 s）
- 降低 `min_cones_per_snapshot`（如从 3 改为 2，锥桶较少时）

### 问题：RMSE 偏大（> 0.1 m）

可能原因：
- 锥桶区域有干扰物体（误检） → 清理场地
- 锥桶摆放不够直 → 重新摆放
- GNSS 精度差（多路径效应） → 换到开阔区域
- 驾驶速度过快（GNSS 时序与 LiDAR 不同步） → 降低速度

### 问题：航向角跨度 < 3°（告警）

解决方法：
- 在驾驶路线中加入一次 15° 以上的转弯，从斜向经过锥桶
- 若场地限制无法转弯，可增大 `min_snapshots` 至 15 以上，以数量弥补多样性

### 问题：检测不到锥桶（cone_cloud 为空）

**检查 z 范围：**

```bash
# 查看原始点云中锥桶的 z 值
rosrun pcl_ros pointcloud_to_pcd /rslidar_points  # 录制一帧
# 在 pcl_viewer 或 Python 中检查锥桶位置的 z 值
```

调整 `installation/axis_offset`（后轴到地面高度），使地面 z 估计（`z_ground = −axis_offset`）准确覆盖地面位置。

---

## 10. 注意事项

1. **标定前确认 LiDAR 外参中 roll/pitch 已填入正确值**，否则点云畸变影响锥桶检测高度过滤。

2. **每次重新安装 LiDAR 后需重新标定**，即使位置未变，安装误差也可能改变。

3. **GNSS 航向精度决定标定精度上限**。若 GNSS 航向精度为 ±0.1°，则本模块标定精度约为 ±0.07°（多帧平均后）。

4. **结果文件中 `yaw_calibrated` 为车体坐标系下的绝对值**（FLU，CCW 为正），直接填入驱动外参即可。`delta_yaw` 是相对先验值的修正量，两者关系：
   ```
   yaw_calibrated = yaw_install + delta_yaw
   ```

5. **标定结束后无需停车**，`~stop` 服务在车辆行进中也可调用。
