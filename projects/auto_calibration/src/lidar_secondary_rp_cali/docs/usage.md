# lidar_secondary_rp_cali 使用说明

辅激光雷达 Roll / Pitch / Z 手动标定工具。

---

## 前提条件

| 条件 | 说明 |
|------|------|
| 主雷达已标定 | 主雷达的 roll / pitch / z 已完成标定，其地面法线在 VCS 中接近 [0, 0, 1] |
| 驱动坐标系 | 主雷达和辅雷达的驱动**均已将点云变换到 VCS**（车辆坐标系，FLU，原点为后轴中心） |
| ROS 环境 | ROS Noetic，已完成 `catkin_make` |

---

## 标定原理

```
地面真实法线 ≈ 主雷达测量法线（主雷达已标定）

delta_roll  = roll(辅雷达法线)  - roll(主雷达法线)
delta_pitch = pitch(辅雷达法线) - pitch(主雷达法线)
z_calib     = z_install + (z_ground_primary - z_ground_secondary)

R_fused = R(delta_pitch, delta_roll) × R_install  （四元数合成）
```

差分做法可消除实际地面存在微小坡度带来的系统误差。

---

## 参数配置

标定前请编辑 `config/params.yaml`，填写辅雷达的**安装标称值**（与驱动配置保持一致）：

```yaml
installation:
  sensor_name: secondary_lidar   # 传感器名称，写入输出文件的键名
  x:           0.0               # 纵向安装偏移（m）
  y:           0.0               # 横向安装偏移（m）
  z:           1.8               # 垂直安装高度，相对后轴中心（m）
  roll_deg:    0.0               # 安装横滚角（°）
  pitch_deg:   0.0               # 安装俯仰角（°）
  yaw_deg:     0.0               # 安装偏航角（°）
  z_offset:    0.37              # 后轴中心到地面高度（m）

calibration_output_file: /tmp/secondary_lidar_rp_calibration.yaml
min_clicked_points: 4            # 每个阶段最少点击点数（建议 6~10）
```

也可在 launch 文件中用 `<param>` 覆盖任意参数，无需修改 yaml。

---

## 操作步骤

### 第一步：启动节点

```bash
roslaunch lidar_secondary_rp_cali lidar_secondary_rp_cali.launch
```

启动后终端会打印：

```
[SecondaryRpCali] === 阶段 1/2：请在 RViz 中显示【主雷达】点云 ...
```

---

### 第二步：采集主雷达地面点（阶段 1/2）

1. 打开 RViz，将 **Fixed Frame** 设置为 `base_link`（VCS 坐标系）

2. 添加 PointCloud2 显示，话题设为**主雷达**点云话题，确认能看到地面

3. 在工具栏选择 **"Publish Point"**（快捷键 `P`）

4. 在地面上依次点击若干位置，点要**分散**（前方、左侧、右侧均有），避免共线：
   - 最少 4 个点，建议 **6～10 个**
   - 不要点到车身、障碍物或远处噪点

5. 在 RViz 中添加 PointCloud2，话题设为 `/lidar_secondary_rp_cali/primary_cloud`，可实时查看已收集的点

6. 订阅状态话题观察实时拟合结果：
   ```bash
   rostopic echo /lidar_secondary_rp_cali/status
   ```
   输出示例：
   ```
   【阶段1/2 主雷达】8 个点 | roll=0.012° pitch=-0.005° z_gnd=-0.371m rms=0.003m | 满意后调用 confirm_primary 服务
   ```
   rms 越小越好，通常 < 0.01 m 即可认为地面点质量良好。

---

### 第三步：确认主雷达，切换到辅雷达阶段

```bash
rosservice call /lidar_secondary_rp_cali/confirm_primary
```

成功响应示例：
```
success: True
message: "主雷达平面已确认 | 点数=8 roll=0.012° pitch=-0.005° z_gnd=-0.371m rms=0.003m | 请在 RViz 中切换到辅雷达点云，继续点击地面点"
```

失败时（点数不足）：
```
success: False
message: "主雷达点击点不足（3 / 4），请继续点击地面"
```

---

### 第四步：采集辅雷达地面点（阶段 2/2）

1. 在 RViz 中将 PointCloud2 话题**切换为辅雷达**点云话题

2. 同样使用 **"Publish Point"** 工具点击地面（6～10 个点，分散分布）

3. 添加话题 `/lidar_secondary_rp_cali/secondary_cloud` 可查看已收集的辅雷达点

4. 观察状态话题，此时显示差分结果：
   ```
   【阶段2/2 辅雷达】7 个点 | delta_roll=1.230° delta_pitch=-0.850° z_gnd=-0.388m rms=0.004m | 满意后调用 save_calibration 服务
   ```

---

### 第五步：保存标定结果

```bash
rosservice call /lidar_secondary_rp_cali/save_calibration
```

成功响应示例：
```
success: True
message: "已保存至 /tmp/secondary_lidar_rp_calibration.yaml | z=1.818 roll=1.231° pitch=-0.848° | delta_roll=1.230° delta_pitch=-0.850° | 主(8点,rms=0.003m) 辅(7点,rms=0.004m)"
```

---

### 重新采集

若对结果不满意，或操作失误，调用 reset 服务从头开始：

```bash
rosservice call /lidar_secondary_rp_cali/reset_calibration
```

---

## 输出文件格式

输出文件（默认 `/tmp/secondary_lidar_rp_calibration.yaml`）结构如下：

```yaml
# 辅激光雷达 Roll/Pitch/Z 手动标定文件
# 生成时间: 2026-04-03 14:23:17
# 主雷达地面点数: 8  平面 RMS: 0.003 m
# 辅雷达地面点数: 7  平面 RMS: 0.004 m

secondary_lidar:

  installation:                        # 原始安装标称值（未变）
    x:         0.000000
    y:         0.000000
    z:         1.800000
    roll_deg:  0.000000
    pitch_deg: 0.000000
    yaw_deg:   0.000000
    z_offset:  0.370000

  calibrated:                          # 标定后参数（写入驱动配置）
    x:         0.000000
    y:         0.000000
    z:         1.818000               # 已更新
    roll_deg:  1.231000               # 已更新
    pitch_deg: -0.848000              # 已更新
    yaw_deg:   0.000000               # 合成后（yaw 未标定时与安装值相同）
    z_offset:  0.370000

  delta:                               # 残余误差（供调试参考）
    delta_roll_deg:  1.230000
    delta_pitch_deg: -0.850000
    delta_z:         -0.018000

  ground_measurement:
    z_ground_primary:   -0.371000     # 主雷达测量地面高度
    z_ground_secondary: -0.388000     # 辅雷达测量地面高度
    z_ground_expected:  -0.370000     # 理论值 = -z_offset

  quality:
    confidence:            0.912
    primary_plane_rms_m:   0.003
    secondary_plane_rms_m: 0.004
    n_primary_points:      8
    n_secondary_points:    7
```

将 `calibrated` 节中的 `z` / `roll_deg` / `pitch_deg` 更新到辅雷达驱动配置即完成标定。

---

## 话题与服务一览

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz Publish Point 点击点（两阶段共用） |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `~primary_cloud` | `sensor_msgs/PointCloud2` | 已收集的主雷达点击点（可在 RViz 中可视化） |
| `~secondary_cloud` | `sensor_msgs/PointCloud2` | 已收集的辅雷达点击点 |
| `~status` | `std_msgs/String` | 当前阶段和实时拟合结果文字 |
| `~result` | `geometry_msgs/Vector3Stamped` | 辅雷达 roll_deg(x) / pitch_deg(y) / confidence(z) |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `~confirm_primary` | `std_srvs/Trigger` | 确认主雷达点采集完成，切换到辅雷达阶段 |
| `~save_calibration` | `std_srvs/Trigger` | 计算并保存辅雷达标定结果 |
| `~reset_calibration` | `std_srvs/Trigger` | 清空全部数据，回到阶段1 |

---

## 质量判断参考

| 指标 | 优秀 | 可接受 | 需重采 |
|------|------|--------|--------|
| 平面 RMS 误差 | < 0.005 m | < 0.015 m | ≥ 0.015 m |
| 点击点数 | ≥ 8 | ≥ 6 | < 4 |
| 点分布 | 前/左/右均匀分散 | 基本分散 | 共线或聚集 |
| confidence | ≥ 0.85 | ≥ 0.60 | < 0.60 |

---

## 常见问题

**Q：点击后终端无反应？**
确认 RViz 的 Fixed Frame 与点云的 frame_id 一致，且 `/clicked_point` 话题有数据：
```bash
rostopic echo /clicked_point
```

**Q：`confirm_primary` 返回失败"平面拟合失败"？**
点击点数量虽然足够，但可能共线（例如全部点在车前方同一排）。请重置后重新采集，确保点在 XY 平面内分散分布。

**Q：标定结果 delta 很大（>5°）？**
检查辅雷达点云是否已经过驱动变换到 VCS。如果点云仍在雷达自身坐标系中，需先确认驱动配置。

**Q：两次标定结果不一致？**
增加点击点数量（建议 ≥ 8 个），并确保点击位置分散。也可对比主/辅雷达的平面 RMS，RMS 越小结果越稳定。
