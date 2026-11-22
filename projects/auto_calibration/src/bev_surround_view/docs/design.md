# bev_surround_view 模块设计文档

## 1. 模块定位

`bev_surround_view` 是一个 ROS 节点，负责对车辆四路鱼眼相机（前/后/左/右）进行**自动外参标定**，并生成 BEV（Bird's Eye View，鸟瞰图）拼接图像及 IPM（Inverse Perspective Mapping）查找表文件。

标定结果以 YAML（外参）和二进制 `.bin`（IPM 查找表）两种格式输出，供下游感知模块使用。

---

## 2. 整体架构

### 2.1 线程模型

```
ROS 主线程
  ├── 图像订阅回调 (front/rear/left/right compressed)
  │     └── updateSyncedBatchLocked()  → 同步帧缓存
  │
  ├── calibration_thread_              ← 标定工作线程
  │     ├── detect() × 4              ← 角点检测
  │     ├── calibrate()               ← 外参求解
  │     └── 最优结果追踪 (best_mean_reproj_rmse_px_)
  │
  ├── bev_thread_                      ← BEV 渲染工作线程
  │     ├── renderBevImages()
  │     └── 发布 bev_fused / bev_front / ...
  │
  └── param_check_timer_ (0.1s)        ← 参数热更新监控
        └── checkParamsCallback()
```

### 2.2 服务接口

| 服务名 | 功能 |
|--------|------|
| `trigger_calibration` | 触发一次标定 |
| `save_calibration` | 保存当前最优标定结果 |
| `reset_calibration` | 清空标定状态 |

### 2.3 关键类

| 类 | 文件 | 职责 |
|----|------|------|
| `BevSurroundViewNode` | `src/bev_surround_view_node.cpp` | ROS 节点、线程调度、参数管理 |
| `FisheyeAutoCalibrator` | `src/fisheye_auto_calibrator.cpp` | 标定算法核心、IPM 生成 |
| `camera::Mei` | `include/camera_model/mei.h` | MEI 鱼眼相机模型 |

---

## 3. 相机模型

采用 **MEI（Mei-Rives）鱼眼模型**，参数组成：

```
xi        — 镜面参数（mirror parameter）
K (3×3)   — 内参矩阵 [gamma1, gamma2, u0, v0]
D (4×1)   — 畸变系数 [k1, k2, p1, p2]
```

示例（前相机 `cam_front.yaml`）：

```yaml
model_type: MEI
image_width: 1920
image_height: 1280
mirror_parameters:
  xi: 1.2976321578
projection_parameters:
  gamma1: 1005.2248867690
  gamma2: 1004.8641487212
  u0: 961.7279923110
  v0: 641.879743340
distortion_parameters:
  k1: 0.2862457897
  k2: -0.2759162530
  p1: -0.0018146652
  p2: -0.0005326020
```

---

## 4. 坐标系约定

### 4.1 VCS（Vehicle Coordinate System）

- X 轴：车辆前方
- Y 轴：车辆左方
- Z 轴：向上
- 地面高度：z = −0.37 m（`vcs_z_offset`）

### 4.2 相机外参存储格式

`FisheyeAutoCameraConfig.t` 存储顺序为 **[y_vcs, z_vcs, −x_vcs]**（非标准顺序，注意）。

旋转使用 **ZYX 欧拉角**（`rotation_order = {2,1,0}`），单位为度。

### 4.3 T_c_w 变换

```
P_cam = R_c_w * P_world + t_c_w
```

`resolvePoseConfig()` 将 T_c_w 转换为 VCS 下的 [y, z, −x] 平移 + ZYX 欧拉角（度）输出到 YAML。

---

## 5. 标定板布局

使用 `config/chasiss_board_V2.yaml` 定义的底盘标定板，`unit_size: 0.2`（单位：m）。

标定板分为顶/底（top_bottom_board）和左/右（left_right_board）两类，每类含 3 组角点区域：

```
  ┌──────────────────────────────────────────┐
  │  group0 (4×4)  │  group1 (4×6)  │  group2 (4×4)  │
  └──────────────────────────────────────────┘
       左相机用          前相机用          右相机用
```

- **group1（中间 4×6）**：前相机标定主用
- **group0（左侧 4×4）**：左相机 + 后相机标定用
- **group2（右侧 4×4）**：右相机 + 后相机标定用

---

## 6. 标定流程（5 阶段）

### Phase 1：前相机标定

- 使用 top_bottom_board 的 **group1（4×6）**
- 3D 点偏移：`{0.0f, -7.75f*sq, -17.5f*sq}`（sq = unit_size）
- 方法：`solvePnP(ITERATIVE)`

### Phase 2：左相机标定

- 使用前相机 group0（4×4）的 2D 角点
- 通过 `backProjectToGround()` 将前相机角点反投影到地面，得到 3D 世界点
- 对 3D 点执行 `rotateGridPoints90(..., 4, 4, false)` 旋转到左相机视角
- 方法：`solvePnP(ITERATIVE)`

### Phase 3：右相机标定

- 使用前相机 group2（4×4）的 2D 角点
- 同 Phase 2，`backProjectToGround()` + `rotateGridPoints90(..., 4, 4, true)`
- 方法：`solvePnP(ITERATIVE)`

### Phase 4：后相机标定

- 融合左相机 group0 + 右相机 group2 的地面 3D 点
- 通过 `sortPointsGrid()` 重排点序
- 方法：`solvePnP(ITERATIVE)`

### Phase 5：联合优化（可选）

- 对左/右/后相机的 rvec + tvec（共 18 个参数）执行**坐标下降**
- 迭代 30 次，每次对单个参数做 ±step 扰动，保留 RMSE 更小的结果
- 由 `enable_joint_optimization` 控制开关

### 流程图

```
图像输入 (4路)
    │
    ▼
角点检测 detect() × 4   ← CornerDetAC 算法
    │
    ▼
Phase 1: 前相机 PnP
    │
    ├──▶ Phase 2: 左相机 PnP (依赖前相机地面点)
    ├──▶ Phase 3: 右相机 PnP (依赖前相机地面点)
    │
    ▼
Phase 4: 后相机 PnP (依赖左+右地面点)
    │
    ▼
Phase 5: 联合优化 (坐标下降 30 iter)
    │
    ▼
输出: poses + BEV 图像 + IPM .bin 文件
```

---

## 7. IPM 输出格式

### 7.1 YAML 外参文件

每个相机输出一个 YAML，包含：
- `t`：[y_vcs, z_vcs, −x_vcs]（米）
- `r_euler`：ZYX 欧拉角（度）
- 相机内参（K、D、xi）

### 7.2 二进制 `.bin` 文件

```
[int32 roi_x] [int32 roi_y] [int32 roi_w] [int32 roi_h]
[size_t vx_size] [float[] map_x]
[size_t vy_size] [float[] map_y]
```

`map_x` / `map_y` 为 OpenCV `remap` 所需的查找表，尺寸为 `roi_w × roi_h`。

ROI 由 `refreshIpmCameras()` 自动裁剪：从中心像素向外扫描，找到第一个有效 IPM 像素确定边界。

---

## 8. BEV 渲染

- BEV 画布：默认 1000×1000 px，分辨率 0.01 m/px
- 每路相机通过 IPM 查找表 `remap` 到 BEV 空间
- 融合策略：固定权重 0.7，各相机叠加后取最大值（`fuseBev()`）

---

## 9. 发现的问题

### 🔴 严重 Bug

#### Bug 1：`static last_rmse_sum` 导致 poses 为空但 valid=true

**位置**：`src/fisheye_auto_calibrator.cpp` 约第 1747 行

```cpp
static double last_rmse_sum = std::numeric_limits<double>::max();
if (last_rmse_sum > front_rmse + left_rmse + right_rmse + back_rmse) {
    last_rmse_sum = ...;
    output.poses[...] = ...;  // 仅在 if 内赋值
}
output.valid = true;  // 无条件设置，即使 poses 为空
```

**问题**：当 RMSE 未改善时，`output.poses` 为空，但 `output.valid = true`，下游代码会访问空 map 导致逻辑错误。

**修复**：移除 `static last_rmse_sum`，由节点层的 `best_mean_reproj_rmse_px_` 统一管理最优结果追踪（节点层已有此逻辑）。

---

#### Bug 2：`rotateGridPoints90` 硬编码 4×4 尺寸

**位置**：`src/fisheye_auto_calibrator.cpp` Phase 2/3

```cpp
rotateGridPoints90(..., 4, 4, false);  // 硬编码，应读取 group0/group2 实际尺寸
```

**问题**：若标定板配置修改了 group0/group2 的行列数，此处不会自动适配。

**修复**：改为读取 `side_group0_count` / `side_group2_count` 的实际行列数。

---

### 🟡 设计问题

#### 问题 3：`fuseBev()` 固定权重导致重叠区过曝

**位置**：`src/fisheye_auto_calibrator.cpp` `fuseBev()`

固定权重 0.7，4 路相机叠加最大值可达 2.8，超出 uint8 范围后被截断，导致重叠区域过曝发白。

**建议**：改为归一化融合或使用距离权重（离相机越近权重越高）。

---

#### 问题 4：`options_` 无互斥锁保护

**位置**：`src/bev_surround_view_node.cpp` `checkParamsCallback()`

`param_check_timer_` 在定时器线程修改 `options_`，而 `calibration_thread_` 和 `bev_thread_` 同时读取，存在数据竞争。

**修复**：在 `checkParamsCallback()` 中加 `std::lock_guard<std::mutex>` 保护 `options_`。

---

#### 问题 5：`unit_size` 注释与实际值矛盾

**位置**：`config/chasiss_board_V2.yaml`

```yaml
unit_size: 0.2   # 注释写的是 0.4m
```

需确认实际标定板物理尺寸，统一注释与数值。

---

### 🟠 调试代码残留

#### 问题 6：硬编码路径的 `save()` 函数

**位置**：`src/fisheye_auto_calibrator.cpp` 第 1466–1512 行

```cpp
// 硬编码路径
save("/home/idriver/work/agent_space/test/save.txt", ...);
```

在 `calibrate()` 中被调用 6 次，应在正式版本中移除。

#### 问题 7：裸 `printf` 调试输出

**位置**：`src/fisheye_auto_calibrator.cpp` 第 1570、1598、1625、1679 行

应替换为 `ROS_DEBUG` 或直接删除。

#### 问题 8：文件中部的 `#include`

**位置**：`src/fisheye_auto_calibrator.cpp` 第 1461–1464 行

`#include` 语句出现在函数实现中间，应移至文件顶部。

---

### ⚪ 死代码

#### 问题 9：`ExtrinsicSolver` 类从未使用

**位置**：`include/bev_surround_view/extrinsic_solver.h`

定义了完整的 `ExtrinsicSolver` 类（PnP+LM 求解器），但在 `fisheye_auto_calibrator.cpp` 中从未 `#include` 或调用。

**建议**：确认是否为历史遗留，若无用则删除。

---

## 10. 问题汇总

| # | 严重程度 | 位置 | 描述 |
|---|----------|------|------|
| 1 | 🔴 严重 | `fisheye_auto_calibrator.cpp:~1747` | `static last_rmse_sum` 导致 poses 空但 valid=true |
| 2 | 🔴 严重 | `fisheye_auto_calibrator.cpp` Phase 2/3 | `rotateGridPoints90` 硬编码 4×4 |
| 3 | 🟡 设计 | `fisheye_auto_calibrator.cpp` `fuseBev()` | 固定权重导致重叠区过曝 |
| 4 | 🟡 设计 | `bev_surround_view_node.cpp` | `options_` 无互斥锁，存在数据竞争 |
| 5 | 🟡 设计 | `config/chasiss_board_V2.yaml` | `unit_size` 注释与数值矛盾 |
| 6 | 🟠 调试 | `fisheye_auto_calibrator.cpp:1466-1512` | 硬编码路径 `save()` 调用 6 次 |
| 7 | 🟠 调试 | `fisheye_auto_calibrator.cpp:1570,1598,1625,1679` | 裸 `printf` 调试输出 |
| 8 | 🟠 调试 | `fisheye_auto_calibrator.cpp:1461-1464` | `#include` 出现在文件中部 |
| 9 | ⚪ 死代码 | `include/bev_surround_view/extrinsic_solver.h` | `ExtrinsicSolver` 类从未使用 |
