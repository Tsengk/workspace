# lidar_rp_cali 使用说明

## 功能概述

`lidar_rp_cali` 是激光雷达外参标定系统的**第一步**，负责标定传感器安装的 **Roll（横滚）、Pitch（俯仰）和 Z（垂直高度）** 三个自由度。

工作原理：
1. 从输入点云中提取地面点（基于高度范围过滤 + 体素降采样 + 统计离群值剔除 + 欧几里得聚类）
2. 对地面点进行平面拟合（RANSAC / 最小二乘 / PCA 三种方法可选）
3. 从平面法向量提取 delta_roll 和 delta_pitch（地面法向量在理想情况下应指向 VCS 的 Z 轴正方向，偏差即为安装误差）
4. 从平面方程截距提取实测地面高度，与期望高度（`-z_offset`）对比计算 delta_z
5. 多帧滑动窗口平均，输出最终标定结果

坐标系约定：**FLU（前左上）**，X=前，Y=左，Z=上；VCS（车辆坐标系）原点位于后轴中心在地面的投影点向上偏移 `z_offset` 处。

精度指标：
- Roll / Pitch 标定误差 < 0.5°（理想场地 < 0.1°）
- Z 标定误差 < 0.02 m（理想场地 < 0.005 m）


## 标定依赖关系（前置条件）

```
[无前置条件]
       ↓
  lidar_rp_cali          ← 本功能包（第一步）
  输出：roll, pitch, z
       ↓
  lidar_yaw1             ← 第二步，依赖本步骤完成
  输出：yaw
       ↓
  lidar_gnss_yaw         ← 可选第三步（车辆行驶场景）
```

`lidar_rp_cali` 是整个标定流程的起点，**不依赖任何其他标定结果**。完成后，将输出的 `roll_deg` 和 `pitch_deg` 填入 `lidar_yaw1/config/params.yaml` 的 `installation` 节，再进行 Yaw 标定。


## 场地与环境要求

| 要求项 | 说明 |
|--------|------|
| 地面平坦度 | 30 m 范围内 80% 以上为水平地面（停车场、广场、道路直道均可） |
| 地面坡度 | 不超过 2°（超出则 roll/pitch 含路面坡度误差，无法准确标定） |
| 障碍物 | 允许存在停放车辆等障碍物，聚类算法会自动筛选最大地面点群 |
| 最小地面点数 | 经聚类后，最大地面点簇需 ≥ 200 个点，否则不参与标定 |
| 点云距离 | 地面点提取范围：2 m（排除车身遮挡）~ 30 m |
| 天气 | 避免雨雪（水面反射会产生假地面点），无特殊光照要求 |
| 车辆状态 | **静止停放**，不要求特定朝向 |
| 激光雷达运行 | 已正常出流，点云话题稳定发布 |

最佳标定场景：开阔停车场或空旷广场，地面洁净，无积水。


## 参数说明

以下所有参数均位于 `config/params.yaml`。

### 输入/输出参数

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `input_topic` | `/rslidar_points` | 输入点云话题 | 改为实际 LiDAR 驱动发布的话题 |
| `lidar_frame` | `lidar` | LiDAR 坐标系名称 | 与 TF 树保持一致 |
| `base_frame` | `base_link` | 车辆基础坐标系名称 | 与 TF 树保持一致 |
| `calibration_output_file` | `/tmp/lidar_calibration.yaml` | 标定结果输出路径 | 可改为项目目录下的持久化路径 |

### 安装参数（installation）

这些值应填写**驱动当前使用的标称安装参数**，必须与驱动配置保持一致。

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `installation/sensor_name` | `top_lidar` | 传感器名称（写入标定文件的键名） | 改为实际传感器标识符 |
| `installation/x` | `0.0` m | 纵向安装位置（相对后轴中心） | 从 CAD 图纸或实测获取 |
| `installation/y` | `0.0` m | 横向安装位置（左正右负） | 从 CAD 图纸或实测获取 |
| `installation/z` | `1.8` m | 垂直安装高度（相对后轴中心） | 从 CAD 图纸或实测获取 |
| `installation/roll_deg` | `0.0` ° | 横滚安装角（当前驱动用标称值） | 首次标定填 0，复标时填上次结果 |
| `installation/pitch_deg` | `0.0` ° | 俯仰安装角（当前驱动用标称值） | 首次标定填 0，复标时填上次结果 |
| `installation/yaw_deg` | `0.0` ° | 偏航安装角（当前驱动用标称值） | 本模块不标定 yaw，保持与驱动一致 |
| `installation/z_offset` | `0.37` m | 后轴中心到地面高度（m） | 根据车辆实测设置，影响地面 Z 期望值 |

### 地面检测参数（ground）

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `ground/min_z` | 自动计算 | 地面 Z 下限（m），`-(z_offset+0.5)` | 如地面点检测缺失，适当减小（更负） |
| `ground/max_z` | 自动计算 | 地面 Z 上限（m），`-(z_offset-0.3)` | 如地面点检测缺失，适当增大 |
| `ground/min_distance` | `2.0` m | 水平距离最小值（排除车身遮挡） | 车身遮挡更大时适当增大 |
| `ground/max_distance` | `30.0` m | 水平距离最大值 | 超远处点云稀疏时可减小 |
| `ground/voxel_size` | `0.2` m | 体素网格降采样分辨率 | 点云过密时增大可提速；稀疏时减小 |
| `ground/sor_mean_k` | `20` | 统计离群值去除的邻域点数 | 噪声多时增大 |
| `ground/sor_std_thresh` | `1.5` | 统计离群值去除的标准差倍数 | 减小可去除更多噪声点；过小会误删 |
| `ground/cluster_tolerance` | `0.5` m | 欧几里得聚类的距离阈值 | 地面不连续时增大；有斜坡时减小 |
| `ground/min_cluster_points` | `200` | 地面点簇最少点数 | 点云稀疏时减小；地面杂乱时增大 |

### 平面拟合参数（plane）

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `plane/method` | `RANSAC` | 拟合方法：`RANSAC` / `LEAST_SQUARES` / `PCA` | 含噪声/离群点时用 RANSAC；干净地面可用 LEAST_SQUARES 或 PCA |
| `plane/ransac_distance_threshold` | `0.05` m | RANSAC 内点判定距离阈值 | 地面较粗糙时增大（0.08~0.10）；平整地面可减小（0.02） |
| `plane/ransac_max_iterations` | `200` | RANSAC 最大迭代次数 | 内点比例低时增大；对精度要求高时可设 500 |
| `plane/min_inlier_ratio` | `0.5` | 平面拟合最小内点比例 | 障碍物多时适当减小 |

### 标定计算参数（cali）

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `cali/max_roll_deg` | `15.0` ° | Roll 超限阈值（超出则帧无效） | 不建议修改，超出说明场地或参数有问题 |
| `cali/max_pitch_deg` | `15.0` ° | Pitch 超限阈值 | 同上 |
| `cali/window_size` | `10` | 滑动窗口帧数 | 增大可提升稳定性但响应变慢；减小可快速响应 |
| `cali/min_confidence` | `0.3` | 进入滑动窗口的最低置信度 | 地面质量差时可适当降低（0.2）；高要求时提高（0.5） |
| `cali/rms_weight` | `0.5` | 置信度计算中 RMS 误差的权重（vs 曲率） | 一般不需调整 |


## 编译

```bash
# 进入工作空间根目录
cd /home/idriver/work/claude/test_lidar_rp_cali

# 仅编译本包
catkin_make --only-pkg-with-deps lidar_rp_cali

# 或编译全部包
catkin_make
```

编译成功后会生成三个可执行文件：
- `lidar_rp_cali_node` — 自动地面点标定节点
- `lidar_rp_cali_clicked_node` — 手动点击地面点标定节点（新）
- `calibration_test_node` — 仿真自测节点


## 启动步骤

### 1. 导入工作空间环境变量

```bash
source /home/idriver/work/claude/test_lidar_rp_cali/devel/setup.bash
```

### 2. 启动主标定节点

**默认启动（使用 config/params.yaml 中的话题名）：**

```bash
roslaunch lidar_rp_cali lidar_rp_cali.launch
```

**话题重映射示例（LiDAR 话题名与默认不同时）：**

```bash
# 方法一：通过 launch 参数覆盖
roslaunch lidar_rp_cali lidar_rp_cali.launch

# 方法二：直接运行节点并重映射话题
rosrun lidar_rp_cali lidar_rp_cali_node \
    /rslidar_points:=/velodyne_points \
    _input_topic:=/velodyne_points
```

**修改参数后启动（不更改 yaml 文件时）：**

```bash
rosrun lidar_rp_cali lidar_rp_cali_node \
    _plane/method:=PCA \
    _ground/max_distance:=25.0 \
    _cali/window_size:=15
```

### 3. 触发、查询、保存服务

节点启动后自动进入持续监听状态，每帧点云到来时自动计算。等待 10~20 秒（积累足够帧数）后：

**查询当前标定结果（不保存）：**

```bash
rosservice call /lidar_rp_cali/trigger_calibration
```

返回示例：
```
success: True
message: "Z=1.802135m Roll=2.003142deg Pitch=1.498876deg conf=0.823"
```

**重置标定数据（清空滑动窗口，重新开始）：**

```bash
rosservice call /lidar_rp_cali/reset_calibration
```

**保存标定结果到文件：**

```bash
rosservice call /lidar_rp_cali/save_calibration
```

返回示例：
```
success: True
message: "[EXCELLENT] saved to /tmp/lidar_calibration.yaml | z=1.802135 roll=2.003142deg pitch=1.498876deg"
```

### 4. 实时监控状态

```bash
# 查看状态字符串（含实时误差）
rostopic echo /lidar_rp_cali/status

# 查看标定结果（滑动窗口均值）
rostopic echo /lidar_rp_cali/result

# 查看地面点云
rostopic echo /lidar_rp_cali/ground_cloud
```


## 输出文件格式说明

标定成功保存后，`/tmp/lidar_calibration.yaml`（路径可配置）的结构如下：

```yaml
# LiDAR Roll/Pitch/Z 标定文件
# 生成时间: 2026-03-26 10:30:00
# 坐标系: FLU (X=前,Y=左,Z=上)，VCS 原点为后轴中心投影到地面
# 融合方法: R_fused = R(delta_pitch,delta_roll) * R_install  (四元数合成)

top_lidar:

  # 原始安装参数（驱动使用的标称值）
  installation:
    x:         0.000000    # m
    y:         0.000000    # m
    z:         1.800000    # m
    roll_deg:  0.000000    # 度
    pitch_deg: 0.000000    # 度
    yaw_deg:   0.000000    # 度
    z_offset:  0.370000    # 后轴到地面高度 (m)
    lidar_to_ground_height: 2.170000    # z+z_offset (m)

  # 标定后参数（写入驱动的目标值）
  calibrated:
    x:         0.000000    # m     (未变)
    y:         0.000000    # m     (未变)
    z:         1.802135    # m     (已更新) ← 将此值写入驱动
    roll_deg:  2.003142    # 度    (已更新) ← 将此值写入驱动
    pitch_deg: 1.498876    # 度    (已更新) ← 将此值写入驱动
    yaw_deg:   0.000000    # 度    (合成后，yaw≈0时与原值相同)
    z_offset:  0.370000    # m     (未变)
    lidar_to_ground_height: 2.172135    # z_calib+z_offset (m)

  # 标定增量（在安装后坐标系内测量的残余误差）
  delta:
    delta_z:         0.002135    # m
    delta_roll_deg:  2.003142    # 度（安装后坐标系）
    delta_pitch_deg: 1.498876    # 度（安装后坐标系）

  # 标定质量
  quality:
    quality_grade:       EXCELLENT
    confidence:          0.823000    # 平均置信度（越高越好）
    std_roll_deg:        0.045000    # roll 测量标准差（°，越小越稳定）
    std_pitch_deg:       0.038000    # pitch 测量标准差（°，越小越稳定）
    std_z_m:             0.003000    # z_ground 测量标准差（m，越小越稳定）
    z_ground_measured:  -0.372135    # 实测地面高度 (m)
    z_ground_expected:  -0.370000    # 期望地面高度 = -z_offset (m)
    frames_used:         10
    calibration_time:    2026-03-26 10:30:00
```

### 关键字段说明

| 字段 | 含义 | 使用方式 |
|------|------|----------|
| `calibrated.z` | 标定后激光雷达垂直安装高度 | **写入驱动配置** |
| `calibrated.roll_deg` | 标定后横滚角（四元数合成结果） | **写入驱动配置** |
| `calibrated.pitch_deg` | 标定后俯仰角（四元数合成结果） | **写入驱动配置** |
| `calibrated.yaw_deg` | 偏航角（本步骤不改变，与安装值一致） | 暂不修改，等 lidar_yaw1 标定后更新 |
| `delta.delta_roll_deg` | 在传感器坐标系下测量的 roll 残差 | 调试参考 |
| `delta.delta_pitch_deg` | 在传感器坐标系下测量的 pitch 残差 | 调试参考 |
| `quality.quality_grade` | 质量等级：EXCELLENT / GOOD / ACCEPTABLE / POOR | 低于 ACCEPTABLE 时需重新标定 |
| `quality.confidence` | 平面拟合置信度均值（越接近 1.0 越好） | < 0.3 表示地面质量差 |
| `quality.std_roll_deg` | 多帧 roll 测量的标准差 | > 0.5° 表示场地或安装不稳 |
| `quality.frames_used` | 参与平均的有效帧数 | < 3 时结果不可靠 |

### 质量等级判定标准

| 等级 | confidence | std_roll/pitch | std_z | frames | 建议操作 |
|------|-----------|----------------|-------|--------|----------|
| EXCELLENT | ≥ 0.80 | ≤ 0.10° | ≤ 0.005 m | ≥ 8 | 可直接使用 |
| GOOD | ≥ 0.60 | ≤ 0.20° | ≤ 0.010 m | ≥ 5 | 可使用，建议继续等待更多帧 |
| ACCEPTABLE | ≥ 0.40 | ≤ 0.50° | ≤ 0.020 m | ≥ 3 | 勉强可用，建议换更好场地重标 |
| POOR | 其他 | — | — | — | 不可用，请检查原因 |


## 可视化（RViz 配置提示）

### 启动 RViz

```bash
# 直接使用包自带的 rviz 配置
rosrun rviz rviz -d $(rospack find lidar_rp_cali)/config/lidar_rp_cali.rviz
```

### 推荐订阅的话题

| 话题 | 类型 | 在 RViz 中的作用 |
|------|------|-----------------|
| `/lidar_rp_cali/ground_cloud` | PointCloud2 | 显示检测到的地面点（绿色），用于验证地面提取效果 |
| `/lidar_rp_cali/result` | Vector3Stamped | 查看实时 roll/pitch/confidence 数值 |
| `/lidar_rp_cali/status` | String | 查看运行状态文字 |
| `/rslidar_points`（原始话题） | PointCloud2 | 显示原始点云，对比地面点提取范围 |

### RViz 配置建议

1. **Fixed Frame** 设置为 `lidar`（与 `lidar_frame` 参数一致）
2. 原始点云使用 **Z 轴颜色映射**（Flat Color 或 Axis Color，Axis=Z），便于直观看出倾斜
3. 地面点云使用**固定绿色**（Flat Color），便于与原始点云区分
4. 添加 **TF** 显示，可看到 `lidar_calibrated` 坐标系（即标定后的坐标系方向）
5. 如果需要查看平面拟合可视化，可运行测试节点 `calibration_test_node`，它会额外发布：
   - `/lidar_rp_cali/raw_cloud` — 仿真原始点云（按 Z 着色）
   - `/lidar_rp_cali/markers` — 平面薄板 + 法向量箭头 + PASS/FAIL 文字标签


## 手动点击模式（lidar_rp_cali_clicked_node）

### 概述

`lidar_rp_cali_clicked_node` 是**手动标定模式**节点，不依赖自动地面点检测算法，而是由操作员在 RViz 中通过 **"Publish Point"** 工具逐点点击地面，以这些点作为地面真值来拟合平面并计算 Roll / Pitch / Z。

**适用场景**：
- 地面环境复杂（坡道、台阶、地砖缝隙），自动检测易误判
- 需要指定特定区域（如车辆正前方平坦区域）作为标定参考面
- 自动模式调试困难，希望直接人工干预

**与自动模式的区别**：

| 对比项 | 自动模式（lidar_rp_cali_node） | 手动模式（lidar_rp_cali_clicked_node） |
|--------|-------------------------------|--------------------------------------|
| 地面点来源 | 自动从每帧点云提取 | 操作员在 RViz 逐点点击 |
| 订阅话题 | LiDAR 点云话题 | `/clicked_point` |
| 平面拟合触发 | 每帧点云到来时自动触发 | 每新增一个点击点后立即重拟合 |
| 质量指标 | 多帧滑动窗口标准差 | 点击点集合对平面的 RMS 误差 |
| 默认拟合方法 | RANSAC | LEAST_SQUARES（点少时更稳定） |
| 重置方式 | `reset_calibration` 服务 | `reset_calibration` 服务（同时清空点集） |

---

### 使用流程

#### 第一步：启动节点

```bash
source /home/idriver/work/claude/test_lidar_rp_cali/devel/setup.bash
roslaunch lidar_rp_cali lidar_rp_cali_clicked.launch
```

启动后终端会打印：
```
[LidarRpCaliClicked] 初始化完成，等待 /clicked_point 点击点
[LidarRpCaliClicked] 请在 RViz 中将 Fixed Frame 设为点云所在帧（通常为 base_link）
[LidarRpCaliClicked] 然后用 'Publish Point' 工具点击地面点。
```

#### 第二步：配置 RViz

1. 启动 RViz：
   ```bash
   rosrun rviz rviz
   ```

2. 将 **Fixed Frame** 设置为 `base_link`（即点云所在的 VCS 帧，与 `base_frame` 参数一致）

3. 添加 **PointCloud2** 显示，话题填写 LiDAR 点云话题（如 `/rslidar_points`），确认能看到地面点云

4. 再添加一个 **PointCloud2** 显示，话题填写 `/lidar_rp_cali_clicked/clicked_cloud`，颜色设置为固定绿色，用于可视化已选取的点击点

5. 工具栏点击 **"Publish Point"** 按钮（或按 **G** 键），鼠标变为十字准星

> **注意**：Fixed Frame 必须与点云坐标系一致。若点云在 `lidar` 帧下显示，则将 Fixed Frame 改为 `lidar`，并相应修改 `lidar_frame` 和 `base_frame` 参数。

#### 第三步：点击地面点

在 RViz 三维视图中，将鼠标移至地面区域，**左键单击**即可发布一个 `/clicked_point` 消息。

**点击建议**：
- 最少需要 **4 个点**（由 `min_clicked_points` 参数控制）才开始计算
- 建议点击 **6~10 个点**，分散在车辆前方不同位置（左侧、中间、右侧、近处、远处）
- 避免点击障碍物、车轮附近隆起区域、积水反射点等非地面位置
- 每次点击后，终端和 `~status` 话题会实时显示当前进度和计算结果：

```
[LidarRpCaliClicked] 收到第 5 个点 [8.120, -1.340, -0.375] frame=base_link
[LidarRpCaliClicked] OK | 共 5 点 | roll=1.823° pitch=-0.942° z_gnd=-0.371m rms=0.003m conf=0.871
```

#### 第四步：查询与保存

**查询当前标定结果（不保存）**：
```bash
rosservice call /lidar_rp_cali_clicked/trigger_calibration
```
返回示例：
```
success: True
message: "点数=7 Z=1.801232m Roll=1.823deg Pitch=-0.942deg conf=0.871"
```

**保存标定结果到文件**：
```bash
rosservice call /lidar_rp_cali_clicked/save_calibration
```
返回示例：
```
success: True
message: "[EXCELLENT] 已保存至 /tmp/lidar_calibration_clicked.yaml | z=1.801232 roll=1.823deg pitch=-0.942deg | 点数=7 rms=0.003m"
```

**重新采集（清空已有点击点，重新开始）**：
```bash
rosservice call /lidar_rp_cali_clicked/reset_calibration
```

---

### 手动模式参数说明

以下参数在 `launch/lidar_rp_cali_clicked.launch` 中可直接覆盖，或在 `config/params.yaml` 中修改公共参数。

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `min_clicked_points` | `4` | 开始计算所需最少点击点数 | 增大（6~8）可提高稳定性；减小（3）可更快得到初始结果 |
| `plane/method` | `LEAST_SQUARES` | 平面拟合方法 | 点少时（< 10）用 `LEAST_SQUARES`；点多时可换 `RANSAC` |
| `calibration_output_file` | `/tmp/lidar_calibration_clicked.yaml` | 输出文件路径 | 改为项目目录下的持久化路径 |
| `installation/*` | 与自动模式相同 | 安装参数 | 与驱动配置保持一致，含义同自动模式 |

---

### 手动模式输出文件格式

手动模式的输出文件与自动模式结构一致，`quality` 节中额外增加以下字段：

```yaml
top_lidar:

  # 原始安装参数（驱动使用的标称值）
  installation:
    ...（同自动模式）

  # 标定后参数（写入驱动的目标值）
  calibrated:
    x:         0.000000
    y:         0.000000
    z:         1.801232    # m     (已更新) ← 写入驱动
    roll_deg:  1.823000    # 度    (已更新) ← 写入驱动
    pitch_deg: -0.942000   # 度    (已更新) ← 写入驱动
    yaw_deg:   0.000000
    z_offset:  0.370000
    lidar_to_ground_height: 2.171232

  # 标定增量
  delta:
    delta_z:         0.001232
    delta_roll_deg:  1.823000
    delta_pitch_deg: -0.942000

  # 标定质量
  quality:
    quality_grade:       EXCELLENT
    confidence:          0.871000
    plane_rms_m:         0.003000    # 最终平面拟合 RMS (m)，越小越好 ← 手动模式专有
    n_clicked_points:    7           # 用户点击地面点数量            ← 手动模式专有
    std_roll_deg:        0.085000
    std_pitch_deg:       0.072000
    std_z_m:             0.004000
    z_ground_measured:  -0.371232
    z_ground_expected:  -0.370000
    calibration_time:    2026-04-01 10:30:00
```

**手动模式质量等级判定标准**（阈值较自动模式适当放宽）：

| 等级 | confidence | std_roll/pitch | std_z | 点击点数 | 建议操作 |
|------|-----------|----------------|-------|----------|----------|
| EXCELLENT | ≥ 0.80 | ≤ 0.15° | ≤ 0.008 m | ≥ 8 | 可直接使用 |
| GOOD | ≥ 0.60 | ≤ 0.30° | ≤ 0.015 m | ≥ 5 | 可使用，建议补充更多点 |
| ACCEPTABLE | ≥ 0.40 | ≤ 0.70° | ≤ 0.030 m | ≥ 3 | 勉强可用，建议重新选点 |
| POOR | 其他 | — | — | — | 不可用，检查点选位置 |

---

### 手动模式常见问题

**Q：点击后 `status` 一直显示"平面拟合失败"**

原因：点数不足（< 3），或所选点近似共线（无法确定平面）。
- 确保已点击 ≥ 4 个点，且分布在不同位置（不要全在同一直线上）

**Q：点击的是地面，但计算结果 roll/pitch 偏差极大（> 5°）**

原因：RViz 的 Fixed Frame 与点云坐标系不一致，导致点击坐标发生偏移。
- 检查 RViz 的 Fixed Frame 是否与点云的 `frame_id` 相同

**Q：想增加更多点，但不想重置已有结果**

直接继续点击即可。每次新增点后会用全量点重新拟合，结果会自动更新。无需重置。

**Q：如何同时运行自动模式和手动模式**

两个节点使用不同的节点名，可同时运行，互不干扰：
```bash
# 终端 1
roslaunch lidar_rp_cali lidar_rp_cali.launch
# 终端 2
roslaunch lidar_rp_cali lidar_rp_cali_clicked.launch
```


## 常见问题与排查

### Q1：节点启动后 `/lidar_rp_cali/status` 一直输出 `ground detection failed`

**原因**：地面点检测失败，可能是以下原因之一：

1. **Z 轴范围设置不匹配**：`z_offset` 与实际车辆配置不符，导致期望地面高度偏差超出 `min_z`/`max_z` 范围。
   - 排查：`rostopic echo /rslidar_points` 查看原始点云，在 RViz 中观察地面点的实际 Z 值
   - 修复：手动设置 `ground/min_z` 和 `ground/max_z` 覆盖自动计算值：
     ```yaml
     ground:
       min_z: -1.0   # 根据实际地面 Z 值调整
       max_z: -0.2
     ```

2. **水平距离范围不合适**：`min_distance` 过大导致车身遮挡区外无点，或 `max_distance` 过小。
   - 修复：调整 `ground/min_distance`（2.0→1.5）或 `ground/max_distance`（30→20）

3. **地面点聚类失败**：`min_cluster_points=200` 要求在过滤后仍有 200 个连续地面点，点云稀疏时可能不满足。
   - 修复：减小 `ground/min_cluster_points`（200→100）

4. **输入话题不对**：检查实际 LiDAR 点云话题是否为 `/rslidar_points`。
   - 排查：`rostopic list | grep points`
   - 修复：修改 `input_topic` 参数或在 launch 文件中重映射话题

---

### Q2：标定结果中 roll/pitch 误差始终偏大（> 1°），confidence 较低

**原因**：平面拟合质量差，常见于以下场景：

1. **地面不够平**：停车场有排水坡度、地砖缝隙等导致平面拟合 RMS 误差增大。
   - 解决：换更平整的场地，或减小 `ground/max_distance`（仅使用近处平整区域）

2. **障碍物点混入地面点云**：地面聚类不纯。
   - 解决：增大 `ground/min_cluster_points`，或清理场地后重标

3. **使用了 LEAST_SQUARES 但含较多噪声**：
   - 解决：改用 `plane/method: RANSAC`，RANSAC 对离群点更鲁棒

4. **窗口帧数不足**：刚启动时帧数少，结果不稳定。
   - 解决：等待至少 `window_size`（默认 10）帧后再触发标定

---

### Q3：`save_calibration` 返回 `no z_ground data yet` 或 `no valid calibration yet`

**原因**：没有积累足够的有效标定帧。

- 检查 `status` 话题，是否存在大量 `ground detection failed` 或 `plane fitting failed`
- 确保至少已经有 1 帧成功的标定结果（confidence ≥ `min_confidence`）
- 等待更长时间（10~30 秒）让滑动窗口积累足够数据

---

### Q4：标定文件中 `quality_grade: POOR`，应该怎么办

根据 POOR 产生的具体原因分别处理：

| POOR 的具体表现 | 处理方法 |
|----------------|----------|
| confidence 低（< 0.40） | 换更平整的场地；检查地面点提取效果 |
| std_roll 或 std_pitch 大（> 0.50°） | 车辆停稳后再标定；检查是否存在振动 |
| frames_used 少（< 3） | 等待更长时间；降低 `min_confidence` 阈值 |
| std_z 大（> 0.020 m） | 场地地面凹凸不平；缩小 `max_distance` |

---

### Q5：标定结果中 `calibrated.roll_deg` 和 `delta.delta_roll_deg` 不一样，哪个是真正的标定值

两者含义不同，应使用 `calibrated.roll_deg`：

- **`delta.delta_roll_deg`**：在传感器局部坐标系下测量的残余 roll 误差（原始测量值）
- **`calibrated.roll_deg`**：通过四元数合成 `R_fused = R_delta * R_install` 后，从旋转矩阵提取的 ZYX 欧拉角中的 roll 分量（融合了现有安装角度）

对于大多数前向安装的激光雷达（yaw_deg ≈ 0°），两者差异极小。但对于侧向或后向安装的传感器（yaw ≈ ±90° 或 180°），简单相加会有显著误差，必须使用四元数合成后的 `calibrated.*` 值。

**正确的驱动配置更新方式**：将 `calibrated.z`、`calibrated.roll_deg`、`calibrated.pitch_deg` 三个值写入驱动配置文件，替换原安装参数中对应的字段。

---

### Q6：连续多次运行标定，每次结果略有差异（0.05°~0.1°），是否正常

属于正常现象。差异来源包括：

1. 点云噪声（雷达测距噪声 σ ≈ 0.01~0.03 m）
2. 地面微小凹凸（无法避免）
3. 每次采集的帧数和地面点分布略有不同

建议：多次标定取均值，或运行更长时间（增大 `window_size` 至 20~30）以降低随机误差。若结果稳定在 ±0.1° 以内，标定质量良好。

---

### Q7：如何验证标定结果是否正确

可通过以下方法验证：

1. **更新驱动参数后重新采集点云**：在 RViz 中查看地面点云的 Z 值，应接近 `-z_offset`（如 -0.37 m）
2. **观察地面法向量**：标定正确后，地面平面拟合的法向量应与 VCS 的 Z 轴接近平行
3. **多场地交叉验证**：在不同停车场标定，结果一致说明标定可靠
4. **运行测试节点**：`roslaunch lidar_rp_cali calibration_test.launch` 查看仿真测试结果
