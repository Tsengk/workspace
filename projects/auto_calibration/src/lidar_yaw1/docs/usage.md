# lidar_yaw1 使用说明

## 功能概述

`lidar_yaw1` 是激光雷达外参标定系统的**第二步**，负责在 roll/pitch/z 已标定完毕的前提下，利用车辆正前方静态摆放的锥形桶标定激光雷达的 **Yaw（偏航）角**。

工作原理：
1. 从输入点云中检测锥形桶（ROI 过滤 → 体素降采样 → 欧几里得聚类 → 圆柱形过滤）
2. 提取各锥桶的 XY 平面质心坐标（ConeCluster.center）
3. 用 **PCA（主成分分析）** 对所有锥桶质心的 XY 坐标进行直线拟合，获取直线方向向量
4. 计算直线方向相对 VCS X 轴（车辆正前方）的偏角 θ
5. 根据标定原理 delta_yaw = -θ，计算 Yaw 修正量
6. 多帧滑动窗口平均，通过四元数合成输出最终融合标定结果

坐标系约定：**FLU（前左上）**，X=前，Y=左，Z=上；VCS 原点位于后轴中心。

精度指标：Yaw 标定误差 < 0.2°（理想场地无噪声时 < 0.05°）

### PCA 直线拟合原理

设 n 个锥桶质心的 XY 坐标为 {(x₁,y₁), ..., (xₙ,yₙ)}：

1. 计算质心均值 (x̄, ȳ)
2. 构造协方差矩阵 C = Σᵢ [(xᵢ-x̄, yᵢ-ȳ)ᵀ · (xᵢ-x̄, yᵢ-ȳ)]
3. 对 C 进行特征值分解，最大特征值对应的特征向量 (vx, vy) 即为直线方向
4. 计算偏角 θ = atan2(vy, vx)（对 X 轴的偏角）
5. Yaw 修正量 delta_yaw = -θ

PCA 方法的优势：对锥桶间距不均匀不敏感，天然处理锥桶数量 ≥ 2 的情况，计算效率高。

融合公式（四元数合成）：
```
R_install = Rz(yaw_install) * Ry(pitch_install) * Rx(roll_install)
R_delta   = Rz(delta_yaw)
R_calib   = R_delta * R_install
```
从 R_calib 提取 ZYX 欧拉角得到最终的 yaw_calib_deg。


## 标定依赖关系（前置条件）

```
[无前置条件]
       ↓
  lidar_rp_cali          ← 第一步（必须先完成）
  输出：calibrated.roll_deg, calibrated.pitch_deg, calibrated.z
       ↓
  lidar_yaw1             ← 本功能包（第二步）
  前提：将上一步输出的 roll_deg/pitch_deg 填入本包的 installation 节
  输出：calibrated.yaw_deg
       ↓
  lidar_gnss_yaw         ← 可选第三步（车辆行驶场景，更高精度）
```

**重要**：启动本节点前，必须将 `lidar_rp_cali` 的标定结果更新到 `config/params.yaml` 的 `installation` 节：

```yaml
installation:
  roll_deg:  2.003142   # ← 从 lidar_calibration.yaml 的 calibrated.roll_deg 复制
  pitch_deg: 1.498876   # ← 从 lidar_calibration.yaml 的 calibrated.pitch_deg 复制
  yaw_deg:   0.0        # ← 当前估计值（即将被本步骤更新）
```

若不更新 roll/pitch，Yaw 标定结果将包含 roll/pitch 残差引起的耦合误差，尤其是大角度安装误差时影响明显。


## 场地与环境要求

| 要求项 | 说明 |
|--------|------|
| 锥桶数量 | 至少 2 个（建议 4~6 个，锥桶越多精度越高） |
| 锥桶摆放方向 | **沿车辆正前方 X 轴方向**成一列排开，偏离角度应 < 30° |
| 锥桶间距 | 建议每个间距 2~5 m，不要聚堆 |
| 锥桶距离范围 | 第一个锥桶距车头 ≥ 2 m（排除车身遮挡），最远 ≤ 40 m |
| 锥桶横向范围 | 以 X 轴为中心，左右各 ≤ 10 m（即 `y_min/max` 范围内） |
| 锥桶高度 | 使用标准交通锥（底部直径约 0.35 m，高度 0.5~0.7 m） |
| 周边杂物 | 锥桶附近 1 m 范围内**无其他障碍物**，以免被误识别为锥桶 |
| 地面 | 相对平坦（roll/pitch 已标定，地面微坡不影响 Yaw 标定） |
| 车辆状态 | **静止停放**，车头正对锥桶列方向 |

### 推荐锥桶摆放方案

```
车辆（后轴中心为原点）
    ↑ X（前方）
    |
    |  ○ 锥桶6  x=30m
    |  ○ 锥桶5  x=25m
    |  ○ 锥桶4  x=20m
    |  ○ 锥桶3  x=15m
    |  ○ 锥桶2  x=10m
    |  ○ 锥桶1  x=5m
    |(车身遮挡区：0~2m 不检测)
   [车辆]
```

锥桶沿 X 轴中线（y ≈ 0）排列，间距均匀为 5 m（也可不均匀）。如果 Yaw 安装误差较大（如 5°），锥桶列在实际点云中会偏向一侧，但算法会自动补偿。


## 参数说明

以下所有参数均位于 `config/params.yaml`。

### 输入/输出参数

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `input_topic` | `/rslidar_points` | 输入点云话题 | 改为实际 LiDAR 驱动发布的话题 |
| `lidar_frame` | `lidar` | LiDAR 坐标系名称 | 与 TF 树保持一致 |
| `base_frame` | `base_link` | 车辆基础坐标系名称 | 与 TF 树保持一致 |
| `calibration_output_file` | `/tmp/lidar_yaw_calibration.yaml` | 标定结果输出路径 | 可改为项目目录下的持久化路径 |

### 安装参数（installation）

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `installation/sensor_name` | `top_lidar` | 传感器名称（写入标定文件的键名） | 与 lidar_rp_cali 保持一致 |
| `installation/x` | `0.0` m | 纵向安装位置 | 从 CAD 图纸或 lidar_rp_cali 输出获取 |
| `installation/y` | `0.0` m | 横向安装位置 | 同上 |
| `installation/z` | `1.8` m | 垂直安装高度 | 从 lidar_rp_cali 的 `calibrated.z` 获取 |
| `installation/roll_deg` | `0.0` ° | 横滚角 **（已标定值）** | **必须**填入 lidar_rp_cali 的 `calibrated.roll_deg` |
| `installation/pitch_deg` | `0.0` ° | 俯仰角 **（已标定值）** | **必须**填入 lidar_rp_cali 的 `calibrated.pitch_deg` |
| `installation/yaw_deg` | `0.0` ° | 偏航角（待标定当前估计值） | 填当前驱动使用的标称 yaw 值 |
| `installation/z_offset` | `0.37` m | 后轴中心到地面高度 | 与 lidar_rp_cali 保持一致 |

### 锥形桶检测参数（cone）

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `cone/x_min` | `2.0` m | X 方向最近检测距离 | 车身前悬更长时增大（如 3.0） |
| `cone/x_max` | `40.0` m | X 方向最远检测距离 | 远处锥桶点数极少时减小（如 25.0） |
| `cone/y_min` | `-10.0` m | Y 方向左侧边界 | 锥桶偏离 X 轴较远时放宽（-15.0） |
| `cone/y_max` | `10.0` m | Y 方向右侧边界 | 同上（15.0） |
| `cone/height_above_ground_min` | `0.08` m | 锥桶高于地面的最小值（地面噪声过滤） | 地面残差较多时增大（0.15） |
| `cone/height_above_ground_max` | `0.80` m | 锥桶高于地面的最大值 | 检测更高障碍物时增大（不建议，会引入误检） |
| `cone/voxel_size` | `0.05` m | 体素网格降采样分辨率 | 点云过密时增大（0.1）；稀疏时减小（0.03） |
| `cone/cluster_tolerance` | `0.30` m | 欧几里得聚类距离阈值 | 锥桶点云分散时增大（0.50）；聚类过度合并时减小（0.20） |
| `cone/min_cluster_points` | `5` | 聚类最少点数 | 远处锥桶点少，不要设置过大；近处过多噪声时增大 |
| `cone/max_cluster_points` | `500` | 聚类最多点数 | 聚类到大型障碍物时减小（200） |
| `cone/max_cluster_radius_xy` | `0.50` m | 锥桶 XY 轮廓半径上限（圆柱形过滤） | 标准交通锥底径约 0.35m，默认 0.50m 有余量 |
| `cone/min_cones` | `2` | 有效锥桶最少数量（少于此数量本帧跳过） | 建议保持 2，实际摆 4~6 个 |

### Yaw 标定计算参数（yaw_cali）

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|----------|
| `yaw_cali/max_yaw_deg` | `30.0` ° | Yaw 超限阈值（超出则帧无效） | 安装误差极大时可适当增大；一般不需修改 |
| `yaw_cali/window_size` | `10` | 滑动窗口帧数 | 增大可提升稳定性（20~30）；减小响应更快 |
| `yaw_cali/min_confidence` | `0.30` | 进入滑动窗口的最低置信度 | 锥桶质量差时降低（0.20）；要求高精度时提高（0.50） |


## 编译

```bash
# 进入工作空间根目录
cd /home/idriver/work/claude/test_lidar_rp_cali

# 仅编译本包
catkin_make --only-pkg-with-deps lidar_yaw1

# 或编译全部包
catkin_make
```

编译成功后会生成三个可执行文件：
- `lidar_yaw_cali_node` — 自动锥桶检测标定节点
- `lidar_yaw_cali_clicked_node` — 手动点击锥桶标定节点（新）
- `yaw_cali_test_node` — 仿真自测节点
- `yaw_cali_test_node` — 仿真自测节点


## 启动步骤

### 1. 前置检查：确认 roll/pitch 已填入参数

```bash
# 查看 lidar_rp_cali 的标定结果
cat /tmp/lidar_calibration.yaml | grep -A6 "calibrated:"
```

将输出的 `roll_deg` 和 `pitch_deg` 更新到 `config/params.yaml`：

```yaml
installation:
  roll_deg:  <填入 calibrated.roll_deg 的值>
  pitch_deg: <填入 calibrated.pitch_deg 的值>
```

### 2. 导入工作空间环境变量

```bash
source /home/idriver/work/claude/test_lidar_rp_cali/devel/setup.bash
```

### 3. 启动主标定节点

**默认启动（带 RViz 可视化）：**

```bash
roslaunch lidar_yaw1 lidar_yaw_cali.launch rviz:=true
```

**仅启动节点（不开 RViz）：**

```bash
roslaunch lidar_yaw1 lidar_yaw_cali.launch
```

**话题重映射示例：**

```bash
# LiDAR 话题名与默认不同时
rosrun lidar_yaw1 lidar_yaw_cali_node \
    /rslidar_points:=/top_lidar/points_raw \
    _input_topic:=/top_lidar/points_raw
```

**修改锥桶检测范围（如锥桶放到 45m 处）：**

```bash
rosrun lidar_yaw1 lidar_yaw_cali_node \
    _cone/x_max:=50.0 \
    _cone/y_min:=-15.0 \
    _cone/y_max:=15.0
```

### 4. 摆放锥形桶并等待标定

节点启动后，按照场地要求摆放锥形桶，然后静止等待 10~20 秒（积累足够帧数）。

通过以下方式监控运行状态：

```bash
# 查看运行状态（含锥桶数量、yaw_delta、置信度）
rostopic echo /lidar_yaw_cali/status

# 查看当前标定结果
rostopic echo /lidar_yaw_cali/result
# result.vector.x = yaw_delta_deg（Yaw 修正量）
# result.vector.y = cone_count（检测到的锥桶数）
# result.vector.z = confidence（置信度）
```

### 5. 触发、查询、保存服务

**查询当前标定结果（不保存）：**

```bash
rosservice call /lidar_yaw_cali/trigger_calibration
```

返回示例：
```
success: True
message: "yaw_delta=-2.0031° yaw_calib=-2.0031° roll=2.0032° pitch=1.4989° conf=0.876"
```

**重置标定数据（清空滑动窗口，重新开始）：**

```bash
rosservice call /lidar_yaw_cali/reset_calibration
```

**保存标定结果到文件：**

```bash
rosservice call /lidar_yaw_cali/save_calibration
```

返回示例：
```
success: True
message: "[EXCELLENT] saved to /tmp/lidar_yaw_calibration.yaml | yaw_calib=-2.0031° (delta=-2.0031°) conf=0.876"
```


## 输出文件格式说明

标定成功保存后，`/tmp/lidar_yaw_calibration.yaml` 的结构如下：

```yaml
# LiDAR Yaw 标定文件
# 生成时间: 2026-03-26 11:00:00
# 坐标系: FLU (X=前,Y=左,Z=上)，VCS 原点为后轴中心投影到地面
# 融合方法: R_calib = Rz(delta_yaw) * R_install  (四元数合成)
# 标定场景: 车辆正前方 4~5 个锥形桶，沿 X 轴排列

top_lidar:

  # 原始安装参数（roll/pitch 已完成前一步标定）
  installation:
    x:         0.000000    # m
    y:         0.000000    # m
    z:         1.802135    # m
    roll_deg:  2.003142    # 度（已标定值，本步骤未改变）
    pitch_deg: 1.498876    # 度（已标定值，本步骤未改变）
    yaw_deg:   0.000000    # 度（标定前估计值）
    z_offset:  0.370000    # 后轴到地面高度 (m)

  # 标定后参数（写入驱动的目标值）
  calibrated:
    x:         0.000000    # m     (未变)
    y:         0.000000    # m     (未变)
    z:         1.802135    # m     (未变)
    roll_deg:  2.003142    # 度    (精确合成值) ← 写入驱动
    pitch_deg: 1.498876    # 度    (精确合成值) ← 写入驱动
    yaw_deg:  -2.003100    # 度    (已更新)     ← 写入驱动
    z_offset:  0.370000    # m     (未变)

  # 标定增量
  # delta_yaw = -θ，θ 为锥桶连线相对 VCS X 轴的偏角
  delta:
    delta_yaw_deg: -2.003100    # 度（yaw 修正量）

  # 标定质量
  quality:
    quality_grade:   EXCELLENT
    confidence:      0.876000    # 平均置信度（锥桶数量+共线性）
    std_yaw_deg:     0.045000    # yaw 测量标准差（°）
    frames_used:     10
    calibration_time: 2026-03-26 11:00:00
```

### 关键字段说明

| 字段 | 含义 | 使用方式 |
|------|------|----------|
| `calibrated.yaw_deg` | 标定后偏航角（四元数合成结果） | **写入驱动配置** |
| `calibrated.roll_deg` | roll 角（与输入相同，精确四元数合成值） | 写入驱动配置（替换 lidar_rp_cali 的输出） |
| `calibrated.pitch_deg` | pitch 角（同上） | 写入驱动配置 |
| `delta.delta_yaw_deg` | Yaw 修正量（= -锥桶连线偏角θ） | 调试参考；delta > 0 表示安装偏 CCW |
| `quality.quality_grade` | 质量等级：EXCELLENT / GOOD / ACCEPTABLE / POOR | 低于 ACCEPTABLE 时需重新标定 |
| `quality.confidence` | 置信度均值（锥桶共线性 + 锥桶数量） | < 0.3 表示锥桶排列质量差 |
| `quality.std_yaw_deg` | 多帧 yaw 测量标准差 | > 0.5° 表示锥桶检测不稳定 |

### Yaw 质量等级判定标准

| 等级 | confidence | std_yaw | frames | 建议操作 |
|------|-----------|---------|--------|----------|
| EXCELLENT | ≥ 0.80 | ≤ 0.10° | ≥ 8 | 可直接使用 |
| GOOD | ≥ 0.60 | ≤ 0.20° | ≥ 5 | 可使用，建议继续等待 |
| ACCEPTABLE | ≥ 0.40 | ≤ 0.50° | ≥ 3 | 勉强可用，建议重检锥桶摆放 |
| POOR | 其他 | — | — | 不可用，请检查原因 |


## 如何判断锥桶检测质量

### 通过 RViz cone_markers 观察

在 RViz 中订阅 `/lidar_yaw_cali/cone_markers`（`visualization_msgs/MarkerArray`），检测到的每个锥桶会显示为**橙色圆柱体**（直径 0.30 m，高度 0.60 m），并标注编号（C1、C2 等）。

**正常情况**（标定可用）：
- 每帧稳定检测到 4~6 个橙色圆柱，位置帧间稳定（不抖动）
- 圆柱与实际锥桶位置对应，大小合适（不过大不过小）
- `status` 话题显示 `OK | cones=5 yaw_delta=...`

**异常情况及判断**：

| 现象 | 可能原因 | 处理方法 |
|------|----------|----------|
| 无 marker 出现 | 锥桶未被检测到 | 检查锥桶是否在 ROI 范围内；查看 `status` 是否报 `insufficient cones` |
| marker 数量少于实际锥桶 | 部分锥桶点云被过滤 | 增大 `x_max`；检查 `height_above_ground_max` 是否足够 |
| 出现多余 marker（非锥桶位置） | 其他障碍物被误检 | 清理锥桶附近障碍物；减小 `max_cluster_radius_xy` |
| marker 帧间跳动（位置不稳定） | 点云噪声大或距离过远 | 减小 `x_max`；增大 `min_cluster_points` |
| marker 圆柱明显偏离实际锥桶 | z_offset 参数错误 | 检查 `installation/z_offset` 是否与实际车辆一致 |

### 通过 cone_cloud 观察

订阅 `/lidar_yaw_cali/cone_cloud`（`PointCloud2`），显示所有被归为锥桶聚类的点云：
- 正常：每个锥桶对应一小团点（5~50 个点），分布在锥桶位置
- 异常：点云稀少（< 5 点）或连成一片（聚类合并了多个物体）

### 通过 status 话题监控

```bash
rostopic echo /lidar_yaw_cali/status
```

- `OK | cones=5 yaw_delta=-2.003° conf=0.876 rms=0.012m` — 正常
- `cone detection failed: insufficient cones` — 锥桶不足（< min_cones）
- `yaw calc failed: ...` — Yaw 计算失败（锥桶排列异常）


## 可视化（RViz 配置提示）

### 启动 RViz

```bash
# 启动节点时同时开启 RViz
roslaunch lidar_yaw1 lidar_yaw_cali.launch rviz:=true

# 或手动启动
rosrun rviz rviz -d $(rospack find lidar_yaw1)/config/yaw_cali_test.rviz
```

### 推荐订阅的话题

| 话题 | 类型 | 在 RViz 中的作用 |
|------|------|-----------------|
| `/lidar_yaw_cali/cone_cloud` | PointCloud2 | 显示检测到的锥桶点云（白色小团） |
| `/lidar_yaw_cali/cone_markers` | MarkerArray | 显示锥桶圆柱体和编号（橙色） |
| `/lidar_yaw_cali/result` | Vector3Stamped | 查看实时 yaw_delta/cone_count/confidence |
| `/lidar_yaw_cali/status` | String | 查看运行状态文字 |
| `/rslidar_points`（原始话题） | PointCloud2 | 显示原始点云，对比锥桶检测范围 |

### RViz 配置建议

1. **Fixed Frame** 设置为 `lidar`（与 `lidar_frame` 参数一致）
2. 原始点云使用 **Z 轴颜色映射**（Axis Color，Axis=Z），高度着色便于观察锥桶
3. 添加 **TF** 显示，可看到 `lidar_yaw_calibrated` 坐标系（Yaw 修正后的坐标系）
4. 锥桶点云使用**固定白色**（Flat Color），与原始点云区分
5. 如需查看 PCA 拟合直线等更丰富的可视化，运行测试节点：
   ```bash
   roslaunch lidar_yaw1 yaw_cali_test.launch
   ```
   它会额外发布：
   - `/lidar_yaw1_test/cone_cloud` — 合成锥桶点云
   - `/lidar_yaw1_test/cone_markers` — 锥桶圆柱体 + 编号
   - `/lidar_yaw1_test/fit_markers` — PCA 拟合直线（蓝色）+ X 轴参考线（黄色）+ PASS/FAIL 标签


## 手动点击模式（lidar_yaw_cali_clicked_node）

### 概述

`lidar_yaw_cali_clicked_node` 是**手动标定模式**节点，不依赖自动锥桶检测算法。操作员在 RViz 中点击每个锥形桶附近，节点自动在点击位置 **0.5m 半径内**搜索点云，取**最高点 Z**（锥桶顶部）和 **XY 质心**作为锥桶中心，然后调用与自动模式相同的 PCA 直线拟合算法计算 Yaw。

**适用场景**：
- 锥桶被其他物体遮挡，自动检测漏检
- 点云密度低（远处锥桶），自动聚类失败
- 需要精确控制哪些锥桶参与计算（剔除摆放不准确的锥桶）

**与自动模式的区别**：

| 对比项 | 自动模式 | 手动模式 |
|--------|---------|---------|
| 锥桶来源 | 每帧点云自动聚类 | 用户逐一点击 |
| 触发方式 | 每帧到来自动计算 | 每次新增锥桶后重算 |
| 质量指标 | 多帧滑动窗口标准差 | 逐步添加时的收敛性 |

---

### 使用流程

#### 第一步：布置锥桶并启动节点

将 **4~5 个锥形桶**沿车辆**正前方 X 轴方向**排成一排（不需要等间距），间距约 1m，距车辆 3~20m。

```bash
source /home/idriver/work/claude/test_lidar_rp_cali/devel/setup.bash
roslaunch lidar_yaw1 lidar_yaw_cali_clicked.launch
```

#### 第二步：配置 RViz

1. **Fixed Frame** 设为 `base_link`（点云所在的 VCS 帧）
2. 显示 LiDAR 点云话题（如 `/rslidar_points`）
3. 订阅 `/lidar_yaw_cali_clicked/cone_cloud`（已提取锥桶点云，橙色圆柱）
4. 工具栏点击 **"Publish Point"**（G 键）

#### 第三步：点击锥形桶

在 RViz 三维视图中，将鼠标**点击在每个锥形桶附近**（锥桶底部或中部点云处均可，不需要精确点击顶部）。节点会在点击位置 0.5m 半径内自动找到锥桶点云并提取顶部。

**点击建议**：
- 最少 **2 个锥桶**可计算（建议 4~5 个）
- 按从近到远或从左到右的顺序点击（顺序不影响计算结果）
- 点击后查看 RViz 中橙色圆柱和编号标签（C1、C2…）确认识别正确
- 如识别错误（如点到地面），调用 `reset_calibration` 重新开始

```
[LidarYawCaliClicked] 锥桶 #3: center=[8.120, 0.340, -0.021] points=47
[LidarYawCaliClicked] OK | 锥桶=3 yaw_delta=2.134° rms=0.021m conf=0.823
```

#### 第四步：查询与保存

**查询当前结果**：
```bash
rosservice call /lidar_yaw_cali_clicked/trigger_calibration
# 返回: 锥桶=4 yaw_delta=2.134° yaw_calib=2.134° conf=0.823 rms=0.021m
```

**保存结果**：
```bash
rosservice call /lidar_yaw_cali_clicked/save_calibration
# 输出: /tmp/lidar_yaw_calibration_clicked.yaml
```

**重置（重新点击）**：
```bash
rosservice call /lidar_yaw_cali_clicked/reset_calibration
```

---

### 手动模式参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `search_radius` | `0.5` m | 点击点周围搜索锥桶点云的 2D 水平半径 |
| `min_cluster_pts` | `3` | 邻域内最少点数，低于此视为误点击 |
| `min_cones` | `2` | 开始计算所需最少锥桶数 |
| `installation/*` | 同自动模式 | 安装参数（与驱动配置一致） |

---

### 手动模式输出文件格式

输出文件结构与自动模式一致，`quality` 节额外增加以下字段：

```yaml
  quality:
    quality_grade:    GOOD
    confidence:       0.823
    line_rms_m:       0.021    # 锥桶共线 RMS (m)，越小越好
    n_cones:          4        # 锥桶数量（代替帧数）
    std_yaw_deg:      0.195    # 逐步添加时 yaw 标准差（度）
    calibration_time: 2026-04-01 10:30:00
```

**手动模式质量等级**（标准略宽松）：

| 等级 | confidence | std_yaw | 锥桶数 | 建议 |
|------|-----------|---------|--------|------|
| EXCELLENT | ≥ 0.80 | ≤ 0.15° | ≥ 4 | 可直接使用 |
| GOOD | ≥ 0.60 | ≤ 0.30° | ≥ 3 | 可使用 |
| ACCEPTABLE | ≥ 0.40 | ≤ 0.70° | ≥ 2 | 建议增加锥桶数 |
| POOR | 其他 | — | — | 检查锥桶摆放和点击位置 |

---

### 常见问题（手动模式）

**Q：点击后 status 显示"附近只有 N 个点"**

锥桶距离过远，或 `search_radius` 偏小。增大 `search_radius`（0.7~1.0m），或将锥桶移近。

**Q：RViz 中橙色圆柱位置不对（偏移到地面）**

可能点击到地面区域而非锥桶。调用 `reset_calibration`，然后重新点击更靠近锥桶中心的位置。

**Q：yaw_delta 结果每次点击锥桶后变化很大**

锥桶共线性差（摆放不在一条直线上），或只有 2 个锥桶（直线拟合不稳定）。增加到 4~5 个锥桶，并精确沿 X 轴方向摆放。


## 常见问题与排查

### Q1：status 持续输出 `cone detection failed: insufficient cones`

锥桶检测失败，未找到足够数量的锥桶（< min_cones=2）。

排查步骤：
1. 确认锥桶在 RViz 中是否可见（在原始点云中）
2. 检查 ROI 参数（`x_min/max`, `y_min/max`）是否覆盖锥桶的实际位置
3. 检查高度参数：锥桶的 VCS Z 坐标应在 `[-z_offset + height_min, -z_offset + height_max]` 范围内
   - 当 z_offset=0.37，height_min=0.08，height_max=0.80 时，有效范围为 [-0.29, 0.43]
4. 减小 `min_cluster_points`（默认 5，远处锥桶点少时可设为 3）
5. 适当增大 `cluster_tolerance`（0.30→0.50）以合并分散点

---

### Q2：检测到的锥桶数量正确，但 yaw_delta 误差偏大（> 0.5°）

PCA 直线拟合质量差，可能原因：

1. **锥桶未沿 X 轴方向排列**：摆放时对准了车辆前方，但锥桶列存在横向偏移（仍可标定，但精度下降）
2. **锥桶间距过小**：所有锥桶聚在 3m 范围内，PCA 拟合结果对噪声敏感
   - 解决：拉大锥桶间距（每个 5m 以上）
3. **锥桶附近有杂物被误检**：一个"锥桶"实际上是多个物体合并的聚类
   - 解决：检查 `cone_markers` 的位置和大小，清理周边杂物
4. **窗口帧数不足**：刚启动时 < 3 帧
   - 解决：等待 10 秒以上再触发保存

---

### Q3：置信度 confidence 很低（< 0.30），帧频繁被丢弃

置信度由**锥桶共线性**（PCA 直线拟合 RMS 误差）和**锥桶数量**共同决定：

- 共线性差：锥桶不在一条直线上，或锥桶位置有大的随机偏移
  - 解决：重新摆放锥桶，使其尽量成一直线
- 锥桶数量少（只有 2 个）：2 点直线拟合无 RMS 概念，置信度较低
  - 解决：增加到 4~6 个锥桶

---

### Q4：标定结果中 `delta_yaw_deg` 为正还是负，代表什么意义

符号约定（FLU 坐标系，Z 轴向上为正）：

- **delta_yaw_deg > 0（正值）**：Yaw 修正量为正（CCW 方向），表示激光雷达实际指向相对 X 轴**顺时针偏转**了 |delta_yaw| 度（从上方俯视），即点云中锥桶列偏向了**右侧（-Y 方向）**
- **delta_yaw_deg < 0（负值）**：Yaw 修正量为负（CW 方向），表示激光雷达实际指向**逆时针偏转**，点云中锥桶列偏向了**左侧（+Y 方向）**

直觉验证：如果你在 RViz 中看到点云中的锥桶列整体偏向右侧（Y 值偏小），则 delta_yaw 应为正值。

---

### Q5：与 lidar_rp_cali 的依赖关系：如果 roll/pitch 标定未完成，直接运行 lidar_yaw1 会怎样

可以运行，但结果精度会降低，尤其是：

1. **大 roll/pitch 误差（> 2°）时**：未修正的 roll/pitch 会导致地面高度估计偏差，进而影响锥桶高度范围过滤，可能误检地面杂物或漏检锥桶
2. **耦合误差**：roll/pitch 误差通过旋转矩阵耦合到 Yaw 标定中，导致 delta_yaw 包含非 Yaw 来源的误差
3. **对于小 roll/pitch 误差（< 0.5°）**：影响较小，可以忽略

**建议**：始终先完成 lidar_rp_cali 标定，再运行 lidar_yaw1，即使 roll/pitch 标称值接近 0 度。

---

### Q6：标定后质量等级为 POOR，如何改善

| POOR 的具体表现 | 处理方法 |
|----------------|----------|
| confidence < 0.40 | 增加锥桶数量（4~6 个）；重新摆成一直线；清除周边杂物 |
| std_yaw > 0.50° | 确保车辆静止；检查锥桶是否被风吹动 |
| frames_used < 3 | 等待更长时间；降低 `min_confidence`（0.3→0.2） |

---

### Q7：如何验证 Yaw 标定结果是否正确

1. **点云目视检查**：将 `calibrated.yaw_deg` 填入驱动后，重新采集点云，在 RViz 中查看：
   - 停车场的标线应与 X 轴平行
   - 远处建筑的边缘应与 Y 轴平行
2. **锥桶复验**：重新对齐锥桶（沿 X 轴精确摆放），此时 `yaw_delta` 应接近 0°
3. **TF 验证**：在 RViz 中查看 `lidar_yaw_calibrated` 坐标系的 Z 轴（应垂直向上）和 X 轴（应指向车辆正前方）
4. **运行测试节点**：`roslaunch lidar_yaw1 yaw_cali_test.launch` 验证算法在仿真数据下的精度
