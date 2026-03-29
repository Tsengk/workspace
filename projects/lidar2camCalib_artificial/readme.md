Perception Calibration (LiDAR-Camera 联合标定验证与微调工具)
📌 项目简介
本项目是一个基于 ROS 和 C++ 的激光雷达与相机联合标定（LiDAR-Camera Calibration）验证与动态微调工具。它可以实时同步相机的图像数据与激光雷达的点云数据，根据内外参将点云投影到图像平面进行融合显示。

经过深度优化，本工具引入了实时可视化 UI 交互和参数自动写回功能，极大提升了人工校验和微调标定参数的效率。

✨ 核心升级功能 (Changelog)
动态 6-DoF 滑块微调 (Trackbar Tuning)

在 OpenCV 可视化窗口中集成了 6 个滑动条，支持对平移（X, Y, Z）和旋转（Roll, Pitch, Yaw）进行高精度的实时动态调整。

彻底解决了 OpenCV 滑块不支持负数和浮点数的底层限制，采用偏移量映射 (Offset Mapping) 机制：滑块中心点（如 5000）即代表真正的 0 零点。

OSD 真实物理量显示 (On-Screen Display)

摒弃了滑块旁边的晦涩整数，直接在融合图像的左上角以绿色高亮字体实时渲染当前的真实物理数值（精确到毫米 m 和度 deg，支持完美显示负号）。

一键保存与自动覆写 (Auto YAML Overwrite)

在调参满意后，将鼠标焦点停留在图像窗口，按下键盘 s 键，程序会自动将当前的 6 DoF 结果保存并覆写回 camera_extrinsics.yaml 文件。

四元数自动转换：代码内置了欧拉角到四元数（Quaternion [x, y, z, w]）的自动转换逻辑，并按格式一并写入 YAML。

安全无损：采用智能正则/字符匹配算法，仅修改参数数值，完美保留原文件中的所有中文注释。

防崩溃与日志优化

修复了由于等待数据同步阻塞 UI 事件循环导致的 GTK 崩溃（Exit code -6）问题，确保 UI 线程以稳定的 10Hz 刷新。

引入静默模式：启动时终端会实时打印时间戳用于核对传感器同步状态；一旦按下 s 键保存，无关的日志滚动会立即停止，终端仅保留清晰的参数保存结果。

🛠️ 环境与依赖
OS: Ubuntu (配合 ROS 环境)

核心依赖:

ROS (roscpp, sensor_msgs, cv_bridge, tf)

OpenCV (需支持 GUI 功能)

PCL (点云处理与转换)

Eigen3 (矩阵运算)

🚀 运行与使用指南
1. 启动节点
确保您已经 catkin_make 编译成功，然后通过 launch 文件启动节点：
cd lidar2camCalib_artificial
source devel/setup.bash
roslaunch src/perception_calibration/launch/perception_cali_pt.launch
2. UI 交互说明
节点启动后，会弹出一个名为 Calibration View 的图像窗口，包含点云融合画面和 6 个控制滑块：

X / Y / Z [5000=0]: 控制平移矩阵，精度为 1e-3 m (毫米级)。滑块拨到 5000 代表 0m。

Roll / Pitch / Yaw [18000=0]: 控制旋转矩阵，精度为 1e-2 deg (0.01度)。滑块拨到 18000 代表 0度。

参数读取：请直接观看图像左上角的绿色文字，此处显示的即为将要写入配置文件的真实物理参数。

3. 快捷键操作
请确保鼠标焦点位于 OpenCV 图像窗口内：

按下 s 键：

终端停止滚动打印时间戳日志。

终端输出当前保存的 6DoF 矩阵和 Quaternion 四元数。

程序自动修改 params/camera_extrinsics.yaml、camera_intrinsics.yaml，永久保存当前外参。

📂 关键文件说明
perception_cali_pt.launch: 启动配置与 Topic 重映射（订阅 /driver/lidar/rs_top_points 和 /gmsl_image0）。

camera_intrinsics.yaml: 存放相机内参矩阵、畸变系数 (plumb_bob 模型与 Quaternion) 等。

camera_extrinsics.yaml: 存放 LiDAR 到相机的外参 (6 DoF)。本工具将自动修改此文件。

perception_calibration.cpp: 核心逻辑实现，包含数据同步、点云重投影、GUI 渲染和文件 I/O 读写。
