# 自动驾驶传感器标定工具 — 使用说明

## 概述

`calibration_gui` 是基于 Qt5 + ROS1 的传感器外参标定 GUI 工具，集成了以下 8 个标定流程：

| 步骤 | Tab 名称 | 对应 ROS 节点 | 标定内容 |
|------|----------|--------------|---------|
| 1 | LiDAR RP 标定 | `lidar_rp_cali` | LiDAR Roll / Pitch / Z |
| 2 | LiDAR Yaw 标定 | `lidar_yaw_cali` | LiDAR Yaw（静态锥桶） |
| 3 | GNSS+LiDAR Yaw 标定 | `lidar_gnss_yaw` | LiDAR Yaw（动态 GNSS 融合） |
| 4 | LiDAR-LiDAR 联合标定 | `lidar_lidar_cali` | 辅雷达 X / Y / Yaw |
| 5 | 相机-LiDAR 标定 | `image_lidar_calib_node` | 相机-LiDAR 6DOF 外参 |
| 6 | 相机-相机标定 | `camera_cali_node` | 相机间 6DOF 外参 |
| 7 | 环视 BEV 标定 | `bev_surround_view_node` | 四路环视单应矩阵 |
| 8 | 前视图像拼接 | `front_view_stitch_node` | 三路前视拼接参数 |

---

## 环境依赖

```
ROS Melodic / Noetic
Qt5 (5.12+)
VTK 7.1 (libvtk7-qt-dev)
PCL 1.10
OpenCV 4.x
Eigen3
yaml-cpp
```

---

## 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali
catkin_make --only-pkg-with-deps calibration_gui
```

---

## 启动 GUI

```bash
# 方式一：直接启动
roslaunch calibration_gui calibration_gui.launch

# 方式二：指定车辆配置文件
roslaunch calibration_gui calibration_gui.launch \
    vehicle_config:=/path/to/vehicle.yaml
```

---

## 车辆配置文件格式

GUI 启动后点击"**导入车辆配置文件**"加载 YAML，格式示例：

```yaml
vehicle:
  id: "vehicle_001"
  car_length: 4.8
  car_width: 1.9
  car_height: 1.5
  wheelbase: 2.85
  z_offset: 0.37        # 后轴中心到地面高度 (m)

lidars:
  - name: lidar_top
    topic: /lidar_top/points
    t: [1.5, 0.0, 1.8]          # x y z (m)，VCS 坐标系
    R: [0.0, 0.0, 0.0]          # roll pitch yaw (deg)

  - name: lidar_rear
    topic: /lidar_rear/points
    t: [-1.0, 0.0, 1.6]
    R: [0.0, 0.0, 180.0]

cameras:
  - name: camera_front
    topic: /camera_front/image_raw
    extrinsics:
      t: [1.8, 0.0, 1.4]
      R_euler: [0.0, 5.0, 0.0]  # roll pitch yaw (deg)
    intrinsics:
      K: [1200, 0, 960, 0, 1200, 540, 0, 0, 1]
      D: [0.0, 0.0, 0.0, 0.0, 0.0]
      Size: [1920, 1080]

  - name: camera_left
    topic: /camera_left/image_raw
    extrinsics:
      t: [0.5, 0.9, 1.3]
      R_euler: [0.0, 0.0, 90.0]
    intrinsics:
      K: [800, 0, 640, 0, 800, 360, 0, 0, 1]
      D: [0.0, 0.0, 0.0, 0.0, 0.0]
      Size: [1280, 720]

radars:
  - name: radar_front
    topic: /radar_front/points
    t: [2.0, 0.0, 0.5]
    R: [0.0, 0.0, 0.0]

gnss:
  name: gnss_main
  topic: /sensorgps
```

> **坐标系约定**：FLU（X=前，Y=左，Z=上），VCS 原点为后轴中心，`z_offset` 为后轴到地面高度。

---

## 各 Tab 使用说明

### Tab 1 — LiDAR RP 标定（Roll/Pitch/Z）

**前提**：无
**场景**：车辆停在平坦地面

**步骤**：
1. 启动标定节点：
   ```bash
   roslaunch lidar_rp_cali lidar_rp_cali.launch
   ```
2. 在 GUI 中选择对应激光雷达
3. 等待"实时状态"显示 `OK | roll=... pitch=...`（建议积累 ≥8 帧）
4. 点击"**触发标定**"查看当前均值结果
5. 结果满意后点击"**保存标定结果**"

**输出**：`/tmp/lidar_calibration.yaml`（含 roll/pitch/z 标定值及质量等级）

**质量等级**：
- `EXCELLENT`：conf≥0.80，std≤0.10°，帧数≥8
- `GOOD`：conf≥0.60，std≤0.20°，帧数≥5
- `ACCEPTABLE`：conf≥0.40，std≤0.50°，帧数≥3
- `POOR`：建议检查地面平整度或增加采集帧数

---

### Tab 2 — LiDAR Yaw 标定（静态锥桶）

**前提**：roll/pitch/z 已标定完毕
**场景**：车辆正前方沿 X 轴摆放 4~5 个锥形桶

**步骤**：
1. 摆放锥桶：沿车辆正前方 X 轴方向，间距约 1~2m，共 4~5 个
2. 启动节点：
   ```bash
   roslaunch lidar_yaw1 lidar_yaw_cali.launch
   ```
3. 在 GUI 中选择激光雷达，观察点云视图中黄色球（锥桶中心）
4. 等待状态显示 `OK | cones=4 yaw_delta=...`
5. 点击"**触发标定**"→"**保存**"

**输出**：`/tmp/lidar_yaw_calibration.yaml`

---

### Tab 3 — GNSS+LiDAR Yaw 标定（动态）

**前提**：roll/pitch/z 已标定，GNSS 已初始化（ENU 坐标系）
**场景**：车辆行驶时以锥桶为目标物

**步骤**：
1. 摆放锥桶：4~8 个，间距 1~2m，成一条直线
2. 启动节点：
   ```bash
   roslaunch lidar_gnss_yaw lidar_gnss_yaw_cali.launch
   ```
3. 在 GUI 中点击"**开始采集**"
4. 缓慢驾车绕过/经过锥桶，建议不同方向 2~3 次
5. 点击"**停止采集并计算 Yaw**"
6. 查看结果后点击"**保存**"

**GNSS 话题格式**：`nav_msgs/Odometry`（ENU 坐标系，四元数航向）

**输出**：`/tmp/lidar_gnss_yaw_calibration.yaml`

---

### Tab 4 — LiDAR-LiDAR 联合标定

**前提**：主雷达完全标定，辅雷达 roll/pitch/z 已标定
**场景**：两雷达共视范围内摆放 3~5 个锥桶

**步骤**：
1. 摆放锥桶（两雷达均可见）
2. 启动节点：
   ```bash
   roslaunch lidar_lidar_cali lidar_lidar_cali.launch \
       primary_topic:=/lidar_top/points \
       secondary_topic:=/lidar_rear/points
   ```
3. 在 GUI 中选择主/辅雷达，观察双色点云（红=主，绿=辅）
4. 等待约 10 帧积累后点击"**触发标定**"
5. 查看 Δx Δy Δyaw 修正量后保存

**输出**：`/tmp/lidar_lidar_calib_result.yaml`

---

### Tab 5 — 相机-LiDAR 标定

**前提**：LiDAR 完全标定
**场景**：棋盘格靶标置于相机与雷达共视区域

**步骤**：
1. 启动节点：
   ```bash
   roslaunch image_lidar_calibration image_lidar_calib.launch \
       lidar_topic:=/lidar_top/points \
       image_topic:=/camera_front/image_raw
   ```
2. 在 GUI 中选择激光雷达和相机
3. 将棋盘格靶标放置于共视区域，多角度采集 ≥10 对
4. 点击"**触发标定**"查看重投影误差
5. 满意后保存

**输出**：标定后 6DOF 外参（T_lidar_camera）

---

### Tab 6 — 相机-相机标定

**前提**：各相机内参已标定（`{camera_name}.yaml` 与车辆配置同目录）
**场景**：FASTLIVO2 靶标置于双相机共视区域

**步骤**：
1. 启动节点：
   ```bash
   roslaunch camera_cali camera_cali.launch \
       ref_image_topic:=/camera_front/image_raw \
       target_image_topic:=/camera_left/image_raw \
       vehicle_config_file:=/path/to/vehicle.yaml \
       ref_camera_name:=camera_front \
       target_camera_name:=camera_left
   ```
2. 在 GUI 中选择参考相机和待标定相机
3. 多角度采集 ≥10 对图像（双图像窗口显示角点检测结果）
4. 点击"**触发标定**"→"**保存**"

**相机内参文件**：`{config_dir}/{camera_name}.yaml`，格式：
```yaml
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
D: [k1, k2, p1, p2, k3]
Size: [width, height]
```

**输出**：`/tmp/camera_cali_result.yaml`（6DOF 外参）

---

### Tab 7 — 环视 BEV 标定

**前提**：四路相机内参已标定
**场景**：车辆四角各放棋盘格靶标

**步骤**：
1. 启动节点：
   ```bash
   roslaunch bev_surround_view bev_surround_view_cali.launch
   ```
2. 在 GUI 中绑定前/后/左/右四路相机
3. 调整 BEV 参数（分辨率、各方向范围）
4. 将棋盘格靶标放置于车辆四角
5. 点击"**触发 BEV 标定**"查看拼接效果
6. 满意后保存单应矩阵

**BEV 参数说明**：
- 分辨率：每像素对应的实际距离（m/px），默认 0.02
- 前/后/左/右范围：BEV 图像覆盖的实际距离（m）

**输出**：`/tmp/bev_calib_result.yaml`（四路单应矩阵）

---

### Tab 8 — 前视图像拼接

**前提**：三路前视相机内参已标定
**输出分辨率**：左前(640) + 正前(1280) + 右前(640) = **2560×720**

**步骤**：
1. 启动节点：
   ```bash
   roslaunch front_view_stitch front_view_stitch.launch
   ```
2. 在 GUI 中选择左前/正前/右前三路相机
3. **步骤1**：将棋盘格靶标放置于左前与正前相机重叠区域，点击"**步骤1：标定左前↔正前**"
4. **步骤2**：将靶标移至正前与右前相机重叠区域，点击"**步骤2：标定正前↔右前**"
5. 查看拼接预览图，调整裁剪宽度和水平偏移参数
6. 满意后保存

**输出**：`/tmp/front_stitch_result.yaml`（左右拼接单应矩阵及裁剪参数）

---

## ROS 节点命名空间约定

GUI 订阅的话题和服务基于各节点的固定名称：

| 节点名称 | 话题/服务前缀 |
|---------|-------------|
| `lidar_rp_cali` | `/lidar_rp_cali/` |
| `lidar_yaw_cali` | `/lidar_yaw_cali/` |
| `lidar_gnss_yaw` | `/lidar_gnss_yaw/` |
| `lidar_lidar_cali` | `/lidar_lidar_cali/` |
| `image_lidar_calib_node` | `/image_lidar_calib_node/` |
| `camera_cali_node` | `/camera_cali_node/` |
| `bev_surround_view_node` | `/bev_surround_view_node/` |
| `front_view_stitch_node` | `/front_view_stitch_node/` |

> 若需同时运行多个同类节点（如多雷达），需修改 launch 文件中的 `name=` 参数并相应调整 GUI 中的命名空间。

---

## 标准标定流程（推荐顺序）

```
Step 1: LiDAR RP 标定（所有 LiDAR）
    ↓
Step 2: LiDAR Yaw 标定（主 LiDAR）
    ↓
Step 3: GNSS+LiDAR Yaw 标定（可选，精度更高）
    ↓
Step 4: LiDAR-LiDAR 联合标定（辅 LiDAR）
    ↓
Step 5: 相机-LiDAR 标定（各相机）
    ↓
Step 6: 相机-相机标定（相邻相机对）
    ↓
Step 7: 环视 BEV 标定
    ↓
Step 8: 前视图像拼接标定
```

---

## 常见问题

**Q: 点击"触发标定"提示 "Service call failed"**
A: 对应的标定节点未启动，请先 `roslaunch` 对应节点。

**Q: 点云视图无显示**
A: 检查 LiDAR 话题是否正常发布：`rostopic hz /lidar_top/points`

**Q: 标定质量为 POOR**
A: 增加采集帧数（等待更长时间），检查地面平整度或锥桶摆放是否规范。

**Q: 相机内参加载失败**
A: 确认 `{camera_name}.yaml` 与车辆配置文件在同一目录下，且格式正确。

**Q: BEV 拼接有明显错位**
A: 检查各相机外参初始值是否正确，棋盘格靶标是否完整可见。
