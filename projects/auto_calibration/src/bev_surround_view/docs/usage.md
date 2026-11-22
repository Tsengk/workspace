# bev_surround_view 使用手册

## 1. 功能概述

`bev_surround_view` 是一个环视 BEV（Bird's Eye View，鸟瞰视图）标定功能包，利用车辆周围铺设的专用标定板，为 4 个鱼眼/广角相机（前/后/左/右）计算地面单应矩阵，实现精度 < 2cm 的 360° 环视拼接。

### 1.1 标定板布局

本包设计配合两种标定板使用（尺寸定义见 `config/chasiss_board.yaml`）：

**前后大板（front_rear_board，4.4m × 1.2m）**

```
板本地坐标（X=横向，Y=纵深）：

X:  0.0    0.1            1.3  1.5              2.9  3.1            4.3   4.4
    |margin|←─左黑块1.2m─→|gap|←──棋盘格1.4m──→|gap|←─右黑块1.2m─→|margin|
Y:        [0.1 ~ 1.1]          [6列×4行内角点,0.2m格]    [0.1 ~ 1.1]
```

- 棋盘格：6列×4行内角点，格子尺寸 0.2m，`findChessboardCorners` 检测 24 个点
- 左/右黑色实心方块：各 4 个角点，共 8 个角点，用于扩展约束

**左右小板（left_right_board，1.2m × 1.6m）**

- 纯棋盘格：4列×6行内角点，格子尺寸 0.2m，检测 24 个点
- 无黑块

### 1.2 标定输出

每个相机输出一个 3×3 单应矩阵 H（VCS 地面坐标 → 图像像素）：

```
s · [u, v, 1]^T = H · [x_vcs, y_vcs, 1]^T
```

同时保存逆矩阵 `H_inv = H^{-1}`（图像像素 → VCS 地面坐标）。

---

## 2. 坐标系约定

```
VCS（车辆坐标系，FLU）：
  X = 前方，Y = 左方，Z = 上方
  原点 = 后轴中心
  地面 z_vcs = -z_offset = -0.37m

BEV 图像坐标系：
  列 u（向右） ↔ VCS -Y（从车左到车右）
  行 v（向下） ↔ VCS -X（从车前到车后）
  VCS 原点对应像素坐标 (origin_x, origin_y)

板本地坐标系（board local）：
  原点 = 板的一个参考角（由 board_origin 在 VCS 中定位）
  X 轴 = 沿板横向（4.4m 方向）
  Y 轴 = 沿板纵深方向
```

### VCS ↔ BEV 像素转换

```
VCS -> BEV 像素：
  u_bev = origin_x - y_vcs / resolution   （Y 轴：左为正，图像列向右）
  v_bev = origin_y - x_vcs / resolution   （X 轴：前为正，图像行向上）

BEV 像素 -> VCS：
  x_vcs = (origin_y - v_bev) × resolution
  y_vcs = (origin_x - u_bev) × resolution
```

默认参数（resolution=0.02，range_front=5.0，range_rear=3.0，range_left/right=2.5）：
- BEV 图像：宽 250px × 高 400px
- origin = (125, 250)

---

## 3. 标定算法设计

### 3.1 核心思路：直接单应矩阵（无需外参）

**旧方案（已废弃）**：用 PnP 求相机外参，再从外参数学推导 H。

**新方案**：直接用已知 VCS 坐标的点与检测到的图像角点，调用
`cv::findHomography(vcs_pts, img_pts, RANSAC, 2.0)` 求 H。

优点：
- 不依赖相机外参（无需事先标定安装位置）
- 不存在坐标系换算错误（PnP 给的是相机到板的变换，不是相机到 VCS）
- 直接建立 VCS↔图像的映射，是 BEV 标定的本质

### 3.2 两阶段标定（前后相机）

**阶段 1：棋盘格 24 角点 → H_init**

1. `undistort` / `fisheye::undistortImage` 去畸变
2. `cv::findChessboardCorners()` + `cornerSubPix()` 检测 24 个角点 → `img_corners`
3. `placement.cornerVcs(row, col)` 计算每个角点的 VCS 坐标 → `vcs_pts`
4. `cv::findHomography(vcs_pts, img_corners, RANSAC, 2.0)` → **H_init**

**阶段 2：黑块 8 角点精化 → H_final**（仅前后相机）

1. `computeBlackBlockVcsCorners(placement, cb_params)` 计算 8 个黑块角点的 VCS 坐标
2. 对每个黑块角点：
   a. 用 H_init 将 VCS 坐标投影到图像 → 期望像素位置
   b. 以期望位置为中心，取 ±60px ROI
   c. OTSU 阈值 → 反色 → `findContours` → 取面积最大轮廓
   d. `approxPolyDP(epsilon = 0.04×arcLength)` 多边形近似
   e. 取多边形顶点中距期望点最近者 → `cornerSubPix(7×7)` 亚像素精化
3. 合并：24 棋盘格点 + M 个成功检测的黑块点（M ≤ 8）
4. `cv::findHomography(merged_vcs, merged_img, RANSAC, 2.0)` → **H_final**

效果：将约束点从 1.4m×1.0m 的棋盘格区域扩展到 4.4m×1.2m 的整板，
显著改善 H 在板两端的外推精度。

**左右相机**：仅执行阶段 1（无黑块），直接用 24 个棋盘格点求 H。

### 3.3 BoardPlacement 结构体

`BoardPlacement` 描述一块标定板的空间位置，包含两组参数：

**A. 棋盘格角点 VCS 坐标（用于阶段 1）**

```
cornerVcs(row, col) = (
    corner0_x + row × row_dx + col × col_dx,
    corner0_y + row × row_dy + col × col_dy
)
```

`corner0` 是 `findChessboardCorners` 返回的第 0 个角点的 VCS 坐标；
`row_dx/row_dy` 和 `col_dx/col_dy` 是行列方向的 VCS 步长向量。

**确定方法**：运行 `visualizeDetection`，观察角点编号顺序，
用卷尺测量 corner0 的实际 VCS 坐标，再确认步长方向。

**B. 板本地 → VCS 坐标变换（用于阶段 2 黑块角点）**

```
boardLocalToVcs(bx, by) = (
    board_origin_x + bx × board_lx_dx + by × board_ly_dx,
    board_origin_y + bx × board_lx_dy + by × board_ly_dy
)
```

其中 `(bx, by)` 是板本地坐标（bx 沿 4.4m 横向，by 沿 1.2m 纵深）。

**前板示例**：板横向（bx+）= VCS -Y，板纵深（by+）= VCS +X：
```yaml
board_origin_x: 2.0    # 板本地(0,0)的 VCS X = 近边X
board_origin_y: 2.2    # 板本地(0,0)的 VCS Y = 板左端 = +2.2m
board_lx_dx:  0.0      # bx 方向对 VCS X 无贡献
board_lx_dy: -1.0      # bx 方向对 VCS Y 分量（横向右移 → Y 减小）
board_ly_dx:  1.0      # by 方向对 VCS X 分量（纵深增加 → X 增大）
board_ly_dy:  0.0
```

### 3.4 黑块角点 VCS 坐标计算

```cpp
// 左黑块 4 个角点（板本地坐标）
(black_left_x1,  black_y1)   // 底左 = (0.1, 0.1)
(black_left_x2,  black_y1)   // 底右 = (1.3, 0.1)
(black_left_x1,  black_y2)   // 顶左 = (0.1, 1.1)
(black_left_x2,  black_y2)   // 顶右 = (1.3, 1.1)

// 右黑块 4 个角点（板本地坐标）
(black_right_x1, black_y1)   // (3.1, 0.1)
(black_right_x2, black_y1)   // (4.3, 0.1)
(black_right_x1, black_y2)   // (3.1, 1.1)
(black_right_x2, black_y2)   // (4.3, 1.1)
```

每个板本地坐标通过 `boardLocalToVcs()` 转换为 VCS 坐标。

### 3.5 BEV 合成（逆向采样）

```
对 BEV 图像每个像素 (u_bev, v_bev)：
  1. 像素 → VCS 地面坐标 (x_vcs, y_vcs)
  2. [s·u, s·v, s]^T = H · [x_vcs, y_vcs, 1]^T → 相机像素 (u_cam, v_cam)
  3. 双线性插值采样相机图像颜色
  4. 重叠区域按距车辆距离权重混合
```

---

## 4. 可视化

`visualizeDetection()` 在原图上叠加检测结果，方便调试标定板布置。

**输出图像内容**：

| 元素 | 含义 |
|------|------|
| 彩色圆点序列 | 棋盘格角点编号（`cv::drawChessboardCorners` 风格） |
| 绿色实心圆（r=6） | 黑块角点检测成功 |
| 红色空心圆（r=6） | 黑块角点期望位置（检测失败） |
| 左上角文字 | `Chess:N Black:M/8`（N=检测到角点数，M=成功黑块点数） |

**使用方法**：

```cpp
std::array<DetectionVis, 4> vis_results;
calibrator.visualizeDetection(img_array, cb_params, vis_results, &board_placements_);

// vis_results[i].vis_image   : 可视化图像
// vis_results[i].detected    : 棋盘格是否检测成功
// vis_results[i].num_corners : 检测到的角点数
```

---

## 5. 参数说明

所有参数位于 `config/params.yaml`。

### 5.1 BEV 参数（bev 节）

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `resolution` | `0.02` | m/pixel | BEV 分辨率 |
| `range_front` | `5.0` | m | 前视范围（VCS +X） |
| `range_rear` | `3.0` | m | 后视范围（VCS -X） |
| `range_left` | `2.5` | m | 左视范围（VCS +Y） |
| `range_right` | `2.5` | m | 右视范围（VCS -Y） |

### 5.2 棋盘格参数（checkerboard 节）

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `front_rear_corner_rows` | `4` | 前后板棋盘格角点行数 |
| `front_rear_corner_cols` | `6` | 前后板棋盘格角点列数 |
| `front_rear_square_size` | `0.2` | 前后板格子尺寸（m） |
| `left_right_corner_rows` | `6` | 左右板棋盘格角点行数 |
| `left_right_corner_cols` | `4` | 左右板棋盘格角点列数 |
| `left_right_square_size` | `0.2` | 左右板格子尺寸（m） |
| `black_left_x1` | `0.1` | 左黑块左边 X（板本地） |
| `black_left_x2` | `1.3` | 左黑块右边 X（板本地） |
| `black_right_x1` | `3.1` | 右黑块左边 X（板本地） |
| `black_right_x2` | `4.3` | 右黑块右边 X（板本地） |
| `black_y1` | `0.1` | 黑块下边 Y（板本地） |
| `black_y2` | `1.1` | 黑块上边 Y（板本地） |

### 5.3 车辆参数（vehicle 节）

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `wheelbase` | `2.7` | m，轴距（后轴到前轴） |
| `front_overhang` | `0.9` | m，前悬（前轴到车头） |
| `rear_overhang` | `0.5` | m，后悬（后轴到车尾） |
| `width` | `1.8` | m，车宽（左右对称） |

VCS 原点在后轴中心，由此推算：
```
车头 VCS X = wheelbase + front_overhang
车尾 VCS X = -rear_overhang
车身左侧 VCS Y = +width/2
车身右侧 VCS Y = -width/2
```

### 5.4 标定板摆放（board_placement 节）

只需用卷尺现场测量 **3 个距离**（前后板各 1 个，左右板各 2 个），其余 VCS 坐标由代码自动推算。

| 参数名 | 说明 | 测量方法 |
|--------|------|---------|
| `front/dist_to_vehicle` | 前板近边 距 车头 的距离 | 从车头保险杠量到板的最近侧 |
| `rear/dist_to_vehicle` | 后板近边 距 车尾 的距离 | 从车尾保险杠量到板的最近侧 |
| `left/dist_from_front` | 左板前缘 距 车头 的距离 | 沿车轴方向，从车头量到板的前边 |
| `left/dist_from_side` | 左板近边 距 车身左侧 的距离 | 垂直于车身，从车身侧面量到板近边 |
| `right/dist_from_front` | 右板前缘 距 车头 的距离 | 同左板，量右侧 |
| `right/dist_from_side` | 右板近边 距 车身右侧 的距离 | 同左板，量右侧 |

**布置要求**：
- 前/后板：中心线必须与车辆纵向中心线对齐（Y=0）
- 左/右板：前后位置可任意，只需准确量 `dist_from_front` 和 `dist_from_side`

**自动推算逻辑**（代码中 `computeBoardPlacements()`）：

```
前板近边 VCS X  = wheelbase + front_overhang + front/dist_to_vehicle
后板近边 VCS X  = -(rear_overhang + rear/dist_to_vehicle)
左板前缘 VCS X  = (wheelbase + front_overhang) - left/dist_from_front
左板近边 VCS Y  = width/2 + left/dist_from_side
右板近边 VCS Y  = -(width/2 + right/dist_from_side)
```

棋盘格角点 VCS 坐标、步长方向向量、黑块坐标变换矩阵均由代码根据上述近边位置 + 板规格常数（4.4m/1.2m/1.6m 等）计算得出，无需手工填写。

**标定板规格常数**（硬编码在 `computeBoardPlacements()`，对应 `chasiss_board.yaml`）：

| 常数 | 值 | 含义 |
|------|----|------|
| `FR_HALF_W` | 2.2 m | 前后板横向半宽（4.4/2） |
| `FR_CX0` | 1.5 m | 棋盘格在板本地 X 的起始位置 |
| `FR_CY0` | 0.1 m | 棋盘格在板本地 Y（纵深）的起始位置 |
| `LR_LENGTH` | 1.2 m | 左右板在 VCS X 方向的长度 |
| `LR_CX0` | 0.1 m | 棋盘格在板本地 X（VCS X）的起始位置 |
| `LR_CY0` | 0.1 m | 棋盘格在板本地 Y（VCS Y 深度）的起始位置 |

---

## 6. ROS 接口

### 6.1 订阅话题

| 话题名（默认） | 消息类型 | 说明 |
|----------------|----------|------|
| `/camera_front/image_raw` | `sensor_msgs/Image` | 前相机图像 |
| `/camera_rear/image_raw` | `sensor_msgs/Image` | 后相机图像 |
| `/camera_left/image_raw` | `sensor_msgs/Image` | 左相机图像 |
| `/camera_right/image_raw` | `sensor_msgs/Image` | 右相机图像 |
| `/camera_*/camera_info` | `sensor_msgs/CameraInfo` | 各相机内参（K、D） |

话题名可通过参数 `front_image_topic` 等覆盖。

### 6.2 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `~bev_image` | `sensor_msgs/Image` | BEV 合成图像 |
| `~detection_vis_front` | `sensor_msgs/Image` | 前相机检测可视化 |
| `~detection_vis_rear` | `sensor_msgs/Image` | 后相机检测可视化 |
| `~detection_vis_left` | `sensor_msgs/Image` | 左相机检测可视化 |
| `~detection_vis_right` | `sensor_msgs/Image` | 右相机检测可视化 |
| `~status` | `std_msgs/String` | 标定状态消息 |

### 6.3 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `~trigger_calibration` | `std_srvs/Trigger` | 触发标定（检测+计算H） |
| `~reset_calibration` | `std_srvs/Trigger` | 重置标定状态 |
| `~save_calibration` | `std_srvs/Trigger` | 保存结果到文件 |

---

## 7. 编译与使用

### 7.1 编译

```bash
cd /home/idriver/work/claude/test_lidar_rp_cali
catkin_make --only-pkg-with-deps bev_surround_view --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
```

### 7.2 运行

```bash
roslaunch bev_surround_view bev_surround_view_cali.launch
```

### 7.3 触发标定

```bash
# 等待4路相机图像就绪后：
rosservice call /bev_surround_view_node/trigger_calibration
```

成功响应示例：
```
success: True
message: "BEV calibration OK, stitch_error=0.023m"
```

### 7.4 查看可视化

```bash
rqt_image_view /bev_surround_view_node/detection_vis_front
rqt_image_view /bev_surround_view_node/bev_image
```

---

## 8. 常见问题

**Q1：棋盘格检测失败（`Chessboard not found in front camera`）**

1. 确认标定板角点数参数与实物一致（前后板 6×4，左右板 4×6）
2. 光照：避免强光直射，角点间距应 > 15px
3. 标定板平整无遮挡
4. 图像话题是否有数据：`rostopic hz /camera_front/image_raw`

**Q2：黑块角点检测失败（Black:0/8）**

1. 检查 `board_placement.front.board_origin/lx/ly` 参数是否正确
2. 查看 `detection_vis_front`：红圈（期望位置）是否在图像内
3. 若红圈偏离黑块，说明 `board_origin` 位置或 `board_lx/ly` 方向设置有误
4. ROI 范围 ±60px 内需有黑块对比度，检查曝光设置

**Q3：BEV 图像中相机拼接处错位**

1. 检查 `detection_vis` 图像，确认棋盘格角点编号从正确方向排列
2. 核查 `vehicle` 参数和 `board_placement` 距离测量是否准确（用卷尺重新量）
3. 检查前/后板是否真正与车辆纵向中心线（Y=0）对齐

**Q4：BEV 图像某区域出现拉伸或空洞**

- 相机俯角不足（建议 > 30°）
- `range_*` 超出相机实际视野范围，调小对应方向的范围参数

**Q5：如何验证推算的 BoardPlacement 是否正确**

启动节点后查看日志，会打印：
```
[BevSurroundViewNode] BoardPlacement 推算完成 (front_x=2.40, rear_x=-0.80, left_y=1.10, right_y=-1.10)
```
这些是各板的 board_origin（近边 VCS 坐标）。对照测量值验证：
- `front_x` = wheelbase + front_overhang + front/dist_to_vehicle
- `rear_x` = -(rear_overhang + rear/dist_to_vehicle)
- `left_y` = width/2 + left/dist_from_side
- `right_y` = -(width/2 + right/dist_from_side)

查看 `detection_vis` 图像，绿圈出现在黑块角点处则推算正确；若整体偏移则重新测量距离参数。
