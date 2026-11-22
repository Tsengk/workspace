# lidar_camera_projection

`lidar_camera_projection` 是一个 ROS `catkin` 包，用于将 LiDAR 点云投影到相机图像平面，并输出带有投影结果的可视化图像。

默认实现包含以下能力：

- 读取相机标定文件并初始化相机模型
- 订阅图像与 `PointCloud2` 点云
- 根据 LiDAR 到相机的外参完成三维点投影
- 按深度范围过滤点云并进行遮挡抑制
- 发布叠加投影后的图像
- 通过 ROS 服务保存当前使用的参数

## 目录结构

```text
lidar_camera_projection/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── lidar_camera_projection.launch
├── config/
│   ├── front_camera_calib.yaml
│   └── vis.rviz
└── src/
    └── lidar_camera_projection_node.cpp
```

各目录作用如下：

- `src/`：节点主实现，包含参数读取、订阅发布、投影渲染、保存服务等逻辑
- `launch/`：统一启动入口，封装默认话题、标定文件、外参和显示参数
- `config/front_camera_calib.yaml`：默认相机内参与畸变参数文件
- `config/vis.rviz`：RViz 可视化配置，默认订阅投影结果图像
- `CMakeLists.txt` / `package.xml`：编译配置与依赖声明

## 依赖环境

从当前包配置可见，编译依赖包括：

- ROS `catkin`
- `roscpp`
- `std_msgs`
- `std_srvs`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `camera_model`
- OpenCV
- Eigen3

编译标准为 `C++14`，并启用了：

- `-O2`
- `-Wall`
- `-Wextra`

## 编译方式

该项目是标准 `catkin` 包，放在工作空间的 `src/` 目录下进行编译。

在工作空间根目录执行：

```bash
catkin_make
```

编译完成后加载环境：

```bash
source devel/setup.bash
```

如果只编译当前包，可执行：

```bash
catkin_make --pkg lidar_camera_projection
```

生成的可执行文件为：

```text
lidar_camera_projection_node
```

安装规则中包含：

- 可执行文件安装到 `CATKIN_PACKAGE_BIN_DESTINATION`
- `launch/` 与 `config/` 安装到 `CATKIN_PACKAGE_SHARE_DESTINATION`

## 运行方式

### 1. 使用 launch 文件启动

```bash
roslaunch lidar_camera_projection lidar_camera_projection.launch
```

默认启动文件：

```text
launch/lidar_camera_projection.launch
```

### 2. 带参数覆盖启动

例如覆盖输入输出话题与外参：

```bash
roslaunch lidar_camera_projection lidar_camera_projection.launch \
  image_topic:=/driver/gmsl_camera/front_h100/compressed \
  lidar_topic:=/driver/rotate_lidar/cloud/right/offline \
  output_topic:=/lidar_camera_projection/image \
  x:=0.0 y:=0.0 z:=0.0 \
  roll:=0.0 pitch:=0.0 yaw:=0.0
```

### 3. 结合 RViz 查看结果

```bash
rviz -d $(rospack find lidar_camera_projection)/config/vis.rviz
```

该 RViz 配置默认显示：

- `rviz/Image`
- 话题 `/lidar_camera_projection/image`

## 节点说明

启动的节点为：

```text
lidar_camera_projection_node
```

节点内部主要流程如下：

1. 从参数服务器读取标定文件、输入输出话题、显示参数和外参
2. 读取相机标定 YAML，构造相机模型
3. 根据图像话题类型订阅原始图像或压缩图像
4. 订阅 `sensor_msgs/PointCloud2`
5. 定时检查图像与点云是否可用，并进行时间差过滤
6. 将点云投影到图像平面，绘制彩色投影点
7. 发布叠加后的图像
8. 轮询外参参数，支持运行时更新外参
9. 通过服务将当前参数写入输出目录

## 默认输入输出

### 输入图像

默认图像话题：

```text
/driver/gmsl_camera/front_h100/compressed
```

图像订阅方式由话题名决定：

- 若话题以 `/compressed` 结尾，则按 `sensor_msgs/CompressedImage` 订阅并解码
- 否则按 `sensor_msgs/Image` 订阅原始图像

### 输入点云

默认点云话题：

```text
/driver/rotate_lidar/cloud/right/offline
```

消息类型：

```text
sensor_msgs/PointCloud2
```

节点在点云中读取字段：

- `x`
- `y`
- `z`

### 输出图像

默认输出话题：

```text
/lidar_camera_projection/image
```

发布类型：

```text
sensor_msgs/Image
```

编码格式：

```text
bgr8
```

## Launch 参数

`launch/lidar_camera_projection.launch` 提供了完整的启动参数。

### 标定与输出

- `camera_file`：默认 `$(find lidar_camera_projection)/config/front_camera_calib.yaml`
- `output_dir`：默认 `/tmp/lidar_camera_projection`
- `output_yaml_path`：默认 `/tmp/lidar_camera_projection/camera_params.yaml`

### 话题

- `image_topic`：默认 `/driver/gmsl_camera/front_h100/compressed`
- `lidar_topic`：默认 `/driver/rotate_lidar/cloud/right/offline`
- `output_topic`：默认 `/lidar_camera_projection/image`

### 处理参数

- `publish_rate_hz`：默认 `10.0`
- `sync_tolerance_sec`：默认 `0.2`
- `min_depth_m`：默认 `0.3`
- `max_depth_m`：默认 `30.0`
- `display_width`：默认 `960`

### 旋转顺序

- `rotation_order_0`：默认 `2`
- `rotation_order_1`：默认 `1`
- `rotation_order_2`：默认 `0`

launch 文件中将该顺序解释为：

```text
yaw-pitch-roll = 2,1,0
```

### LiDAR 到相机外参

- `x`：默认 `0.0`
- `y`：默认 `0.0`
- `z`：默认 `0.0`
- `roll`：默认 `0.0`
- `pitch`：默认 `0.0`
- `yaw`：默认 `0.0`

## 相机标定文件

默认标定文件为：

```text
config/front_camera_calib.yaml
```

当前配置内容对应：

- 相机名：`front_camera`
- 模型类型：`MEI`
- 分辨率：`1920 x 1080`

文件中包含以下参数组：

- `mirror_parameters`
- `distortion_parameters`
- `projection_parameters`

节点会从该 YAML 中读取：

- `image_width`
- `image_height`
- `xi`
- 投影矩阵参数 `gamma1/gamma2/u0/v0`
- 畸变参数 `k1/k2/p1/p2`

## 投影与渲染逻辑

当前实现中的投影流程如下：

1. 将 LiDAR 点作为 world 坐标输入相机模型
2. 使用外参将 world 坐标转换到相机坐标系
3. 以相机坐标系下的 `z` 作为深度
4. 按 `min_depth_m` 与 `max_depth_m` 过滤深度范围
5. 将有效三维点投影到二维像素坐标
6. 根据显示宽度对图像与投影坐标进行统一缩放
7. 使用 z-buffer 记录每个像素位置的最近深度
8. 结合 3x3 邻域最小深度进行遮挡抑制
9. 使用彩虹色映射根据距离为投影点着色
10. 将投影层与底图按权重混合输出

图像混合方式为：

- 原图权重 `0.8`
- 投影层权重 `0.3`

## 时间同步方式

节点未使用 message_filters，而是采用“缓存最近图像 + 缓存最近点云 + 定时处理”的方式。

处理定时器频率由 `publish_rate_hz` 控制，默认 `10 Hz`。

当图像时间戳与点云时间戳都有效时，会检查：

```text
abs(image_stamp - cloud_stamp) <= sync_tolerance_sec
```

默认允许时间差：

```text
0.2 秒
```

## 运行时外参更新

节点内部额外启动了一个 `0.1` 秒周期的参数轮询定时器，用于读取：

- `x`
- `y`
- `z`
- `roll`
- `pitch`
- `yaw`

当这些参数发生变化时，节点会重新设置相机外参并继续投影处理，因此可以在运行过程中直接调整参数服务器中的外参值。

## 参数保存服务

节点提供了一个保存服务：

```text
save
```

使用示例：

```bash
rosservice call /save
```

服务类型：

```text
std_srvs/Trigger
```

调用后会在 `output_dir` 下生成：

```text
calib_params.txt
```

保存内容包括：

- 生成时间
- 标定文件路径
- 图像宽高
- `epsilon`
- 内参矩阵 `K`
- 畸变参数 `D`
- 当前外参 `x/y/z/roll/pitch/yaw`

默认输出目录为：

```text
/tmp/lidar_camera_projection
```

## 代码组织

当前源码集中在 `src/lidar_camera_projection_node.cpp`，按功能可分为以下部分：

- 工具函数
  - 字符串后缀判断
  - 有效点检查
  - 距离到彩虹色映射
  - 角度转弧度
  - 输出目录创建
- `LidarCameraProjectionNode` 类
  - 参数读取 `loadParams`
  - 相机初始化 `initCamera`
  - 标定文件读取 `loadIntrinsicsFromYaml`
  - 订阅发布初始化 `setupPubSub`
  - 图像回调
  - 点云回调
  - 参数轮询回调 `onParamTimer`
  - 主处理定时器 `onTimer`
  - 投影渲染 `renderOverlay`
  - 参数保存服务 `srvSave`

整体上，这是一个单节点、单源码文件的投影可视化包，启动入口、标定配置和 RViz 配置均已经独立放在 `launch/` 与 `config/` 目录中。
