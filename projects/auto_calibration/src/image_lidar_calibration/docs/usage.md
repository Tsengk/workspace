# image_lidar_calibration 使用说明

相机-LiDAR 联合外参标定包，基于 FASTLIVO2 标定板，求解相机到车辆坐标系（VCS）的 6DOF 变换。

---

## 1. 功能概述

本包实现相机与 LiDAR 的联合外参标定，输出 T_cam_to_vcs（P_vcs = R * P_cam + t），即将相机坐标系下的三维点变换到 VCS（Vehicle Coordinate System，以后轴中心为原点，FLU 方向定义）。

### 算法流程

1. **图像特征检测**：OpenCV `findChessboardCorners` + `cornerSubPix` 检测棋盘格 48 个内角点（8 列 × 6 行），亚像素精度。
2. **LiDAR 平面分割**：对 ROI 内点云做 RANSAC 平面拟合，提取标定板所在平面的法向量和质心；再用 PCA 估计板在 VCS 中的姿态。
3. **初始外参估计**：对每个位置用 EPnP（`solvePnP` EPNP 模式）求相机相对标定板的初始位姿，汇总后得到外参初值。
4. **多视图 Bundle Adjustment**：以 Ceres Solver 联合优化所有位置的重投影误差（Huber 鲁棒核）+ 法向量对齐约束，迭代收敛至最优外参。
5. **输出**：外参 + 重投影误差评估，写入 YAML 文件。

### FASTLIVO2 标定板相比传统棋盘格的优势

| 特性 | 传统棋盘格 | FASTLIVO2 标定板 |
|------|-----------|-----------------|
| LiDAR 定位方式 | 仅依赖板边缘点反射 | 4 个圆孔使激光穿透，获取地面深度，增强 3D 平面约束 |
| 板角点识别 | 边角检测 | 棋盘格内角点 + ArUco 辅助，鲁棒性更高 |
| 适用距离 | 3~8m（角点分辨率限制） | 5~15m（较大尺寸 96cm×72cm，远距仍清晰） |
| 标定板唯一性 | 无 ID | ArUco 标记提供唯一 ID，可多板同时使用 |
| 法向量精度 | 一般，依赖边缘点数量 | 孔洞区点云+板面点云联合拟合，法向量精度更高 |
| 典型重投影误差 | 1.5~3.0 px | < 1.0 px |

---

## 2. 前置条件

### 2.1 LiDAR 标定完成验证

启动前须确认 LiDAR 已完成全链路标定（roll/pitch/z/yaw），且驱动已将点云发布到 VCS 坐标系下。

验证方法：

```bash
# 订阅点云，在 rviz 中确认地面平面与 XY 平面平行（z ≈ -0.37m 处有稠密地面点层）
rosrun rviz rviz

# 或检查历次 lidar_rp_cali 和 lidar_yaw1 的标定结果文件
cat /tmp/lidar_rp_cali_result.yaml
cat /tmp/lidar_yaw1_result.yaml
```

验收标准：
- 地面平面法向量与 Z 轴夹角 < 0.5°
- Yaw 标定残差 < 0.2°
- VCS 中点云 X 轴指向车辆正前方（通过在车辆正前方放置已知位置目标物确认）

### 2.2 相机内参标定完成验证

相机内参须提前通过 `camera_calibration`（ROS）或 Kalibr 标定，并通过 `camera_info_topic` 话题正常发布。

验证方法：

```bash
# 确认 CameraInfo 话题正常
rostopic echo /camera/camera_info -n 1

# 检查关键字段
# D（畸变系数）、K（内参矩阵）、P（投影矩阵）应非零
# width/height 与实际分辨率一致
```

验收标准：
- RMS 重投影误差 < 0.5 px（来自内参标定报告）
- `camera_info` 话题刷新率 >= 10 Hz

---

## 3. FASTLIVO2 标定板说明

### 3.1 规格

| 参数 | 数值 |
|------|------|
| 总尺寸 | 96 cm × 72 cm |
| 棋盘格内角点 | 8 列 × 6 行（共 48 个角点） |
| 方格边长 | 12 cm |
| 圆孔数量 | 4 个 |
| 圆孔直径 | 8 cm |
| 圆孔位置（相对板中心） | (±24 cm, ±18 cm) |
| ArUco 标记 | 4 个，位于板四角 |

### 3.2 圆孔作用

4 个圆孔使激光束能穿透标定板到达后方地面，在标定板对应空间区域形成"地面孔洞"点云，与板面点云联合拟合时能更精确地约束板的法向量和空间位置，尤其在板面点云密度较低（远距场景）时效果更明显。

### 3.3 摆放要求

**距离**：8~12 m 为最佳（允许范围 5~15 m）。
- 距离 < 5 m：LiDAR 点云过密，反射率梯度大，平面拟合受遮挡影响。
- 距离 > 15 m：图像角点分辨率下降（棋盘格在图像中小于 30×30 px/方格），检测失败率上升。

**倾斜角度**：板面法向量与相机光轴夹角建议 10°~45°。
- 垂直正对（0°）：深度观测条件差，法向量不确定性高。
- 倾斜 > 60°：棋盘格透视变形过大，角点检测准确性下降。

**使用注意事项**：
- 标定板须保持竖直且稳定固定，采集期间不得抖动（建议用支架固定）。
- 标定板正面（有棋盘格的一侧）须朝向相机和 LiDAR 共视方向。
- 标定板背后须留有至少 30 cm 空间，使孔洞穿透的激光能落在地面或后方平面上，而非被遮挡。
- 阳光直射可能导致相机过曝，建议在阴天或室内进行。
- 确认标定板表面无污损、折痕，避免影响角点检测精度。

---

## 4. 场地与环境要求

### 4.1 距离要求

| 参数 | 要求 |
|------|------|
| 车辆到标定板最近距离 | >= 5 m |
| 车辆到标定板最远距离 | <= 15 m（推荐 <= 12 m） |
| 最佳标定距离 | 8~12 m |
| 相机与 LiDAR 共视区域 | 须覆盖整块标定板 |

### 4.2 光照要求

| 条件 | 说明 |
|------|------|
| 推荐光照 | 阴天散射光或室内均匀照明 |
| 可接受光照 | 晴天但标定板无直射阳光反光 |
| 不可接受 | 背光（标定板处于强逆光中）、阴影斑驳（斑马线式阴影覆盖棋盘格） |
| 图像亮度 | 标定板区域平均亮度 80~200（8bit），不过曝也不欠曝 |

### 4.3 遮挡要求

- 标定板周围 30 cm 内无遮挡物（车辆、锥桶、行人等），LiDAR 视线和相机视线均须完整覆盖板面。
- 标定期间禁止人员在标定板与传感器之间穿行。
- LiDAR 水平角扫描须覆盖整块标定板，即标定板宽度（96 cm）在 LiDAR 的水平扫描角分辨率下对应至少 10 根扫描线（水平方向），避免点云过稀。

### 4.4 多位置采集建议

建议至少采集 6 个不同位置，覆盖以下维度，以提升标定约束多样性和收敛精度：

| 位置编号 | 距离 | 水平偏角（相对车辆正前方） | 板面倾斜角 | 说明 |
|---------|------|--------------------------|-----------|------|
| P1 | 8 m | 0°（正前方） | 15°（向左倾） | 基准位置 |
| P2 | 8 m | +10°（偏左） | 15°（向右倾） | 增加左侧约束 |
| P3 | 8 m | -10°（偏右） | 15°（向左倾） | 增加右侧约束 |
| P4 | 5 m | 0° | 30°（向右倾） | 近距、大倾角 |
| P5 | 12 m | +5° | 10° | 远距 |
| P6 | 10 m | -5° | 20°（向上倾） | 增加俯仰方向约束 |

每次移动标定板后：
1. 等待标定板静止（至少 2 s）。
2. 观察 `debug_image` 确认棋盘格 48 个角点均已检测（绿色圆点标注）。
3. 观察 `board_cloud` 确认标定板点云平面分割成功（点云着色与地面点云分离）。
4. 确认无问题后，调用 `trigger_calibration` 服务触发单次采集（或设置为自动采集模式）。

---

## 5. 参数说明表格

配置文件路径：`config/default_params.yaml` 与 `config/front_camera_calib.yaml`

### 5.1 话题参数

| 参数名 | 默认值 | 说明 | 调整建议 |
|--------|--------|------|---------|
| `lidar_topic` | `/rslidar_points` | LiDAR 点云话题（sensor_msgs/PointCloud2） | 修改为实际驱动发布的 VCS 点云话题 |
| `image_topic` | `/camera/image_raw` | 相机图像话题（sensor_msgs/Image） | 修改为实际相机驱动话题；若为压缩图像需添加 `/compressed` 并调整订阅类型 |
| `camera_info_topic` | `/camera/camera_info` | 相机内参话题（sensor_msgs/CameraInfo） | 需与 `image_topic` 同一相机命名空间 |
| `save_path` | `/tmp/camera_lidar_calib_result.yaml` | 标定结果保存路径 | 修改为项目标定结果目录，确保目录已存在且有写权限 |

### 5.2 安装参数（installation）

| 参数名 | 默认值 | 单位 | 说明 | 调整建议 |
|--------|--------|------|------|---------|
| `installation/camera_name` | `front_camera` | — | 相机名称，用于结果文件中的标识 | 与实际相机名称对应，多相机时须唯一 |
| `installation/x` | `2.0` | m | 相机在 VCS 中的 X 坐标（正=前） | 填写安装设计值，作为优化初值 |
| `installation/y` | `0.0` | m | 相机在 VCS 中的 Y 坐标（正=左） | 填写安装设计值 |
| `installation/z` | `1.5` | m | 相机在 VCS 中的 Z 坐标（正=上） | 填写安装设计值 |
| `installation/roll_deg` | `0.0` | ° | 相机安装 Roll 角（ZYX欧拉） | 填写安装设计值；前视相机通常 0° |
| `installation/pitch_deg` | `5.0` | ° | 相机安装 Pitch 角（正=下俯） | 填写安装设计值；前视下俯相机约 3°~10° |
| `installation/yaw_deg` | `0.0` | ° | 相机安装 Yaw 角（正=左偏） | 填写安装设计值；正前视通常 0° |

### 5.3 标定板参数（board）

| 参数名 | 默认值 | 单位 | 说明 | 调整建议 |
|--------|--------|------|------|---------|
| `board/pattern_rows` | `6` | — | 棋盘格内角点行数 | FASTLIVO2 标准板固定为 6，不可修改 |
| `board/pattern_cols` | `8` | — | 棋盘格内角点列数 | FASTLIVO2 标准板固定为 8，不可修改 |
| `board/square_size` | `0.12` | m | 方格边长 | FASTLIVO2 标准板固定为 0.12 m；使用自制板时按实测值填写 |
| `board/hole_diameter` | `0.08` | m | 圆孔直径 | FASTLIVO2 标准板固定为 0.08 m；仅影响孔洞特征提取范围 |

### 5.4 图像检测参数（detector）

| 参数名 | 默认值 | 单位 | 说明 | 调整建议 |
|--------|--------|------|------|---------|
| `detector/corner_quality_level` | `0.01` | — | goodFeaturesToTrack 质量阈值，控制角点检测灵敏度 | 如角点漏检，适当减小（0.005）；如误检过多，适当增大（0.02） |
| `detector/min_distance` | `10` | px | 相邻角点最小像素距离 | 标定板在图像中较小时（远距场景）可适当减小至 5~8 px |
| `detector/window_size` | `11` | px | `cornerSubPix` 精化窗口半径（实际窗口 2w+1 像素） | 通常无需调整；图像分辨率极低时可减小至 7 |
| `detector/use_hole_detection` | `true` | — | 是否启用孔洞辅助特征（增强 LiDAR 平面约束） | 建议保持 true；仅当孔洞被遮挡时设为 false |

### 5.5 LiDAR 处理参数（lidar）

| 参数名 | 默认值 | 单位 | 说明 | 调整建议 |
|--------|--------|------|------|---------|
| `lidar/ransac_threshold` | `0.01` | m | RANSAC 内点判定距离阈值 | 通常无需调整；点云噪声大（如雨天）时可增大至 0.02 |
| `lidar/min_inlier_ratio` | `0.3` | — | 平面拟合内点比例下限 | 若远距场景点云稀疏导致拟合失败，可降至 0.2 |
| `lidar/max_iterations` | `200` | — | RANSAC 最大迭代次数 | 通常无需调整；对精度要求极高的场景可增大至 500 |
| `lidar/roi_x_min` | `2.0` | m | ROI X 轴最小值（最近距离） | 标定场景近于 5 m 时可减小；须大于 LiDAR 盲区距离 |
| `lidar/roi_x_max` | `20.0` | m | ROI X 轴最大值（最远距离） | 通常覆盖 15 m 标定范围即可；过大会引入无关平面 |
| `lidar/roi_y_min` | `-5.0` | m | ROI Y 轴最小值（右侧边界） | 根据实际标定板摆放范围调整 |
| `lidar/roi_y_max` | `5.0` | m | ROI Y 轴最大值（左侧边界） | 根据实际标定板摆放范围调整 |
| `lidar/roi_z_min` | `-0.5` | m | ROI Z 轴最小值（下界） | 通常无需调整；地面以下为负值 |
| `lidar/roi_z_max` | `3.0` | m | ROI Z 轴最大值（上界） | 若标定板安装高度超过 2 m 则适当增大 |

### 5.6 求解器参数（solver）

| 参数名 | 默认值 | 单位 | 说明 | 调整建议 |
|--------|--------|------|------|---------|
| `solver/max_ceres_iterations` | `200` | — | Ceres 最大迭代次数 | 通常 100 次内收敛；如 cost 未充分下降可增大至 500 |
| `solver/function_tolerance` | `1.0e-6` | — | Ceres 收敛判定：函数值相对变化量 | 通常无需调整；精度要求极高时可减小至 1e-8 |
| `solver/use_huber_loss` | `true` | — | 是否启用 Huber 鲁棒核（抑制外点影响） | 建议保持 true；仅在确认数据极干净时可设 false |
| `solver/huber_loss_param` | `1.0` | px | Huber 核阈值（超过此误差按线性损失处理） | 一般无需调整；若检测有系统性 1~2 px 偏差可适当增大 |
| `solver/min_pairs` | `3` | — | 最少有效观测数，低于此值不触发优化 | 建议 >= 5 以提升精度；仅在数据极少时设为 3 |

---

## 6. 编译与启动步骤

### 6.1 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali

# 仅编译 image_lidar_calibration 包
catkin_make --only-pkg-with-deps image_lidar_calibration

# 若编译成功，输出类似：
# [100%] Linking CXX executable image_lidar_calib_node
# [100%] Built target image_lidar_calib_node
```

依赖项（须已安装）：
- ROS Melodic 或 Noetic
- PCL（`libpcl-dev`）
- OpenCV（`libopencv-dev`，>=3.4）
- Eigen3（`libeigen3-dev`）
- Ceres Solver（`libceres-dev`）
- `cv_bridge`、`image_transport`、`pcl_ros`（ROS 软件包）

### 6.2 source 环境

```bash
source /home/idriver/work/claude/test_lidar_rp_cali/devel/setup.bash
```

### 6.3 启动标定节点

**方式一：使用 launch 文件（推荐）**

```bash
roslaunch image_lidar_calibration image_lidar_calibration_gui.launch
```

使用自定义话题：

```bash
roslaunch image_lidar_calibration image_lidar_calibration_gui.launch \
  lidar_topic:=/lidar/vcs_points \
  image_topic:=/front_cam/image_raw \
  camera_info_topic:=/front_cam/camera_info
```

**方式二：直接 rosrun**

```bash
rosrun image_lidar_calibration image_lidar_calib_node \
  _lidar_topic:=/lidar/vcs_points \
  _image_topic:=/front_cam/image_raw \
  _camera_info_topic:=/front_cam/camera_info \
  _save_path:=/data/calib/front_camera_result.yaml
```


---

## 7. 操作流程

### 步骤 1：确认传感器数据正常

```bash
# 确认话题正常发布
rostopic hz /rslidar_points    # 应 >= 10 Hz
rostopic hz /camera/image_raw  # 应 >= 10 Hz
rostopic echo /camera/camera_info -n 1  # 检查内参非零
```

### 步骤 2：打开 rviz 监控

在 rviz 中添加以下显示项（Fixed Frame 设为 `base_link` 或 VCS 坐标系）：

| 话题 | 类型 | 用途 |
|------|------|------|
| `~debug_image` | Image | 实时观察棋盘格角点检测结果 |
| `~board_cloud` | PointCloud2 | 观察 LiDAR 标定板平面分割结果 |
| `~status` | String | 观察标定状态和提示信息 |

```bash
# 或直接在终端订阅状态
rostopic echo /image_lidar_calib_node/status
```

### 步骤 3：将标定板放到第一个位置

按第 4 节建议，将标定板放置于距车辆 8~12 m、相机与 LiDAR 共视区域内，稳定固定。

### 步骤 4：确认检测成功

观察 `debug_image`：
- 应显示绿色圆点标注的 48 个角点（8×6 网格清晰可见）。
- 角点连线方向与棋盘格行列方向一致。
- 若角点未全部检测（少于 48 个）：重新调整标定板位置或检查光照，参见第 10 节常见问题。

观察 `board_cloud`：
- 应显示从背景点云中分割出的标定板平面点云（通常 50~300 个点）。
- 若点云为空：检查 ROI 参数是否覆盖当前标定板距离，参见第 10 节常见问题。

### 步骤 5：触发单次采集（"抓对"）

确认检测无误后，调用 `trigger_calibration` 服务，系统将当前图像-点云对保存为一条观测记录：

```bash
rosservice call /image_lidar_calib_node/trigger_calibration "{}"
```

返回示例：
```
success: True
message: "Pair 1/3 captured. Need 2 more pairs to calibrate."
```

重复步骤 3~5，移动标定板至不同位置，直至采集至少 6 对（推荐）。

### 步骤 6：触发标定计算

采集足够对数（>= `min_pairs`，建议 >= 6）后，再次调用触发服务：

```bash
rosservice call /image_lidar_calib_node/trigger_calibration "{}"
```

当对数达到 `min_pairs` 后，触发服务会自动启动 Ceres 优化。也可先用 reset 清空后重新采集。

### 步骤 7：检查标定质量

```bash
rostopic echo /image_lidar_calib_node/result -n 1
rostopic echo /image_lidar_calib_node/status -n 5
```

输出示例：
```
header:
  stamp: ...
  frame_id: "front_camera"
vector:
  x: 2.003   # x_m
  y: 0.001   # y_m
  z: 1.498   # z_m
```

状态信息示例：
```
Calibration GOOD: RMS=0.62px, pairs=6, roll=0.15deg, pitch=4.98deg, yaw=0.03deg
```

质量等级：
- `EXCELLENT`：RMS < 0.5 px，置信度 >= 0.95
- `GOOD`：RMS < 1.0 px，置信度 >= 0.8
- `ACCEPTABLE`：RMS < 2.0 px，置信度 >= 0.6
- `POOR`：RMS >= 2.0 px，不建议使用

若质量为 `POOR` 或 `ACCEPTABLE`，建议增加观测位置（调用 `trigger_calibration` 继续采集）或排查问题后重新标定。

### 步骤 8：保存结果

```bash
rosservice call /image_lidar_calib_node/save_calibration "{}"
```

返回示例：
```
success: True
message: "Saved to /tmp/camera_lidar_calib_result.yaml"
```

### 步骤 9：重置（可选）

如需重新采集：

```bash
rosservice call /image_lidar_calib_node/reset_calibration "{}"
```

---

## 8. 输出文件格式

保存路径由 `save_path` 参数指定，默认 `/tmp/camera_lidar_calib_result.yaml`。

示例文件内容及字段含义：

```yaml
# camera_lidar_calib_result.yaml
# 由 image_lidar_calibration 自动生成
# 时间戳: 2026-03-26 14:23:05

camera_name: front_camera        # 相机名称（来自 installation/camera_name）
timestamp: "2026-03-26 14:23:05" # 标定时间

# --- 外参结果（T_cam_to_vcs）---
# P_vcs = R * P_cam + t
# 旋转以 ZYX 欧拉角表示（°），平移单位 m
extrinsics:
  roll_deg:  0.152    # Roll 角（ZYX欧拉序）
  pitch_deg: 4.983    # Pitch 角
  yaw_deg:   0.031    # Yaw 角

  x_m: 2.003          # 相机在 VCS 中 X 坐标（正=前）
  y_m: 0.001          # Y 坐标（正=左）
  z_m: 1.498          # Z 坐标（正=上）

  # 4×4 齐次变换矩阵（行主序）
  T_cam_to_vcs:
    - [  0.9993, -0.0005,  0.0374,  2.003 ]
    - [  0.0005,  1.0000,  0.0026,  0.001 ]
    - [ -0.0374, -0.0026,  0.9993,  1.498 ]
    - [  0.0000,  0.0000,  0.0000,  1.000 ]

# --- 精度评估 ---
quality:
  rms_reprojection: 0.623    # RMS 重投影误差（px），越小越好，目标 < 1.0 px
  mean_reprojection: 0.581   # 平均重投影误差（px）
  num_pairs: 6               # 参与标定的有效观测数
  quality_grade: GOOD        # EXCELLENT / GOOD / ACCEPTABLE / POOR
  confidence: 0.87           # 置信度 [0,1]

# --- 安装先验（参考值）---
installation_prior:
  x: 2.0
  y: 0.0
  z: 1.5
  roll_deg:  0.0
  pitch_deg: 5.0
  yaw_deg:   0.0
```

---

## 9. 标定验证方法

将标定后的外参用于将 LiDAR 点云投影到图像，通过目视检查边缘对齐精度。

### 9.1 验证步骤

1. 加载标定结果（T_cam_to_vcs），提取 R 和 t。
2. 对 LiDAR 点云中的每个点 P_vcs，按如下公式变换到相机坐标系：

   ```
   P_cam = R^T * (P_vcs - t)   # T_cam_to_vcs 的逆变换
   ```

3. 用相机内参矩阵 K 和畸变系数 D，将 P_cam 投影到图像：

   ```
   u = fx * P_cam.x / P_cam.z + cx
   v = fy * P_cam.y / P_cam.z + cy
   ```

4. 在原始图像上叠加投影点（按深度着色），观察边缘对齐情况。

### 9.2 验收标准

- LiDAR 点云投影后，点云边缘（障碍物轮廓、地面边界）与图像中对应物体边缘的偏差 < 2 px。
- 在 8~12 m 距离处，柱体（锥桶、路牌）轮廓吻合良好，无明显系统性偏移。

### 9.3 快速验证命令

```bash
# 在 rviz 中使用 image_view 叠加显示
rosrun image_view image_view image:=/camera/image_raw

# 使用 ros-perception 中的 image_proc 或自定义节点将 LiDAR 点云投影
# 也可用 Autoware/Apollo 工具包中的 lidar_camera_projection 节点进行可视化
```

---

## 10. 常见问题

**Q1：棋盘格检测失败（debug_image 无角点标注）**

原因及处理：
- 光照不足或过曝：调整光照，检查相机曝光参数（降低曝光时间或增加曝光补偿）。
- 标定板与背景对比度低：确保标定板背景为纯色（白墙或空旷地面），避免背景杂乱。
- 标定板距离过远（> 15 m）：将标定板移近至 12 m 以内。
- 标定板倾斜过大（> 60°）：减小倾斜角度至 45° 以内。
- 部分角点被遮挡：确保标定板正面无遮挡，48 个角点均须完整可见。

**Q2：LiDAR 平面检测失败（board_cloud 为空）**

原因及处理：
- 标定板不在 ROI 范围内：检查 `lidar/roi_x_min/max` 是否覆盖当前标定板距离，适当调整。
- ROI 内点云太稀疏：LiDAR 水平扫描角分辨率不足以覆盖标定板宽度，尝试减小标定距离或调整 `min_inlier_ratio` 至 0.2。
- RANSAC 阈值过小：在点云噪声较大的环境（雨天、扬尘）中，适当增大 `ransac_threshold` 至 0.02~0.03 m。
- 标定板材质反射率低：FASTLIVO2 标准板已针对反射率优化，若使用自制板须确保表面漫反射特性良好。

**Q3：重投影误差大（RMS > 1.0 px）**

原因及处理：
- 观测位置多样性不足：增加不同距离、角度的观测位置，覆盖 5~12 m 和 ±20° 偏角范围。
- 时间同步偏差：确认图像与点云时间戳对齐（建议同步精度 < 10 ms）；检查采集时标定板是否完全静止。
- 相机内参精度不足：重新用更多图像（>= 50 张）进行内参标定，确认 RMS < 0.5 px。
- 安装先验值偏差过大：将 `installation` 参数修正为更接近实际安装值，以改善 Ceres 优化初值。
- 数据中存在异常对：调用 reset 后重新采集，排除震动或遮挡导致的坏数据。

**Q4：LiDAR 点云太稀疏，板面点不足**

原因及处理：
- 使用低线数 LiDAR（< 16 线）：在 5~8 m 近距采集，同时适当增大 ROI Y 范围以捕获更多扫描线。
- 标定板过于倾斜（接近激光扫描平面）：调整标定板使其法向量与 LiDAR 水平面夹角 > 15°。
- 降低 `min_inlier_ratio` 至 0.15~0.2，允许以更少内点完成平面拟合。

**Q5：Ceres 优化不收敛（质量为 POOR）**

原因及处理：
- 观测对数量过少（< 3 对）：继续采集更多位置直至 >= 6 对。
- 初始外参与真实值偏差过大（> 10°）：修正 `installation` 参数中的先验欧拉角，使之与目测估计值更接近。
- 所有标定板位置共面（例如全在同一距离、相同高度）：增加不同深度和高度的位置，破坏位置退化。

**Q6：服务调用失败（trigger_calibration 返回 success: False）**

原因及处理：
- 节点未接收到 `camera_info`（相机内参话题未发布）：确认 `camera_info_topic` 话题名称正确且持续发布。
- 当前帧图像检测失败：等待 `debug_image` 显示检测成功后再调用服务。
- 当前帧 LiDAR 平面分割失败：观察 `status` 话题信息确认原因，处理方法见 Q2。

**Q7：输出外参与安装先验差异过大**

原因及处理：
- 差异 < 5°，平移 < 0.1 m：属正常安装误差，以标定结果为准。
- 差异 > 10°：检查 LiDAR 是否已完成 yaw 标定（`lidar_yaw1` 包），确认点云已在 VCS 下发布；检查标定板是否使用了正确尺寸参数（`square_size`）。
- 检查坐标系定义：确认相机坐标系为光学坐标系（Z 向前，X 向右，Y 向下）还是 FLU（X 向前，Y 向左，Z 向上），确保与代码约定一致。

**Q8：时间同步问题（图像与点云不对齐）**

原因及处理：
- 采集 `trigger_calibration` 时使用的是节点缓存的最新图像和最新点云，二者时间戳差异若大于 50 ms，标定误差会显著增大。
- 解决方法一：使用 `message_filters::ApproximateTimeSynchronizer` 确保图像与点云时间对齐（节点内已实现，检查 `cloud_mutex_` 保护逻辑是否正常）。
- 解决方法二：降低 LiDAR 和相机帧率至一致值（如均为 10 Hz），或使用硬件触发信号同步。
- 解决方法三：标定期间保持标定板完全静止，以减小时间不对齐带来的误差影响。
