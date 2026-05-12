# ARS408 SPI Radar ROS Driver

基于 ROS 1 的大陆（Continental）ARS408 毫米波雷达驱动节点。通过 SPI 接口读取底层打包好的 CAN 报文（SPI 转 CAN 桥接），按 ARS408 协议（Motorola 大端序）解析目标模式数据，以 `visualization_msgs::MarkerArray` 在 RViz 中提供双雷达（前向与后向）的 3D 目标可视化。

---

## 核心特性

- **双雷达通道支持**：单次 SPI 读取同步解析 CAN3（前向雷达）与 CAN2（后向雷达）数据
- **精准协议解析**：通过位运算提取 0x60A（帧起始）与 0x60B（目标距离、速度），无需 DBC 依赖
- **硬件与仿真无缝切换**：通过 Launch 参数一键切换实车硬件读取或本地 Bin 文件读取
- **RViz 可视化**：将雷达目标转化为 MarkerArray（不同颜色的 Bounding Box、ID 与纵向速度），含 TF 坐标系转换
- **超时保护**：目标超时自动丢弃（默认 0.3s），防止丢帧时出现幽灵目标
- **解析失步保护**：未知 CAN ID 自动跳过 3 字，防止单帧错位导致后续全部解析失败

---

## 环境依赖

| 环境 | 版本 |
|---|---|
| 操作系统 | Ubuntu 18.04+ |
| ROS | Melodic / Noetic |
| 依赖包 | roscpp, std_msgs, visualization_msgs, tf2_ros |

---

## 目录结构

```
ars408_radar/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── run_radar.launch         # 核心启动文件
├── src/
│   └── ars408_spi_node.cpp     # C++ 节点源码
└── test_data/
    ├── generate_mock_spi.py     # 离线模拟数据生成脚本
    └── mock_radar_spi.bin       # 生成的测试二进制流
```

---

## 编译

### 通用（x86 / Xavier 均适用）

```bash
cd ~/radar_ws
catkin_make
source devel/setup.bash
```

建议将 source 写入 `~/.bashrc`：

```bash
echo "source ~/radar_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 运行

### 模式 A：离线仿真（首次验证推荐）

无需真实硬件，读取本地生成的模拟二进制流。

```bash
roslaunch ars408_radar run_radar.launch sim:=true
```

默认模拟数据包含两路雷达目标：

| 雷达 | Topic | 目标数据 | RViz 颜色 |
|---|---|---|---|
| 前雷达 (CAN3) | `/radar_front/objects` | ID=2, X=10m, Y=-3m, Vx=15m/s | 绿色 |
| 后雷达 (CAN2) | `/radar_rear/objects` | ID=1, X=5m, Y=1m, Vx=-5m/s | 红色 |

如需自定义模拟数据，修改 `test_data/generate_mock_spi.py` 后重新生成：

```bash
cd src/ars408_radar/test_data
python3 generate_mock_spi.py
```

### 模式 B：实车硬件模式

连接真实底盘与 SPI 线束，读取 `/dev/spidev0.0` 硬件节点。

```bash
roslaunch ars408_radar run_radar.launch
```

---

## 节点参数

所有参数通过 `private_nh_.param` 读取，可在 Launch 文件中配置：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `is_mock` | false | `true` 时从本地文件读取，`false` 时读取 SPI 硬件 |
| `device_path` | `/dev/spidev0.0` | SPI 设备路径；sim 模式下自动指向 `.bin` 文件 |
| `spi_speed_hz` | 10000000 | SPI 通信时钟频率 (Hz) |
| `object_timeout` | 0.3 | 目标超时时间（秒），超过此时间未刷新的目标自动丢弃；设为 0 禁用 |

---

## RViz 可视化说明

Launch 文件会自动打开 RViz。首次打开无画面时：

1. **Fixed Frame** 设为 `base_link`
2. **Add → MarkerArray**，Topic 选择 `/radar_front/objects`（绿色，车头前方）
3. **Add → MarkerArray**，Topic 选择 `/radar_rear/objects`（红色，车尾后方）

前后雷达的 TF 变换：

| 雷达 | 相对于 base_link |
|---|---|
| `radar_front_link` | 平移 (2.5, 0, 0.5)，无旋转 |
| `radar_rear_link` | 平移 (-1.0, 0, 0.5)，绕 Z 轴旋转 180° |

---

## 在 Xavier（Jetson AGX/Orin/NX）上编译部署

### 1. 安装 ROS

Xavier 为 arm64（aarch64）架构，建议使用 Ubuntu 20.04 + ROS Noetic：

```bash
# 安装 ROS Noetic (aarch64)
sudo apt update
sudo apt install ros-noetic-ros-base

# 安装依赖
sudo apt install ros-noetic-visualization-msgs ros-noetic-tf2-ros

# 初始化 rosdep
sudo rosdep init
rosdep update
```

### 2. 编译

直接在 Xavier 上编译（不支持交叉编译，因涉及 ioctl SPI 系统调用）：

```bash
mkdir -p ~/radar_ws/src
cd ~/radar_ws/src
# 将本功能包拷贝或同步到 src/ars408_radar/
cd ~/radar_ws
catkin_make
source devel/setup.bash
```

### 3. SPI 设备权限

Xavier 的 SPI 设备路径通常为 `/dev/spidev0.0` 或 `/dev/spidev1.0`，默认权限为 `root`，普通用户无读写权限：

**临时赋权：**
```bash
sudo chmod 777 /dev/spidev0.0
```

**持久化权限（推荐）：**
```bash
sudo usermod -aG spi $USER
# 重新登录后生效，或执行 newgrp spi
```

确认设备节点存在：
```bash
ls -l /dev/spidev*
```

若设备节点不存在，需检查设备树是否使能了 SPI 控制器：
```bash
sudo dmesg | grep spi
```

### 4. 启动（实车模式）

```bash
source ~/radar_ws/devel/setup.bash
roslaunch ars408_radar run_radar.launch
```

如 SPI 设备路径与默认值不同，通过参数指定：
```bash
roslaunch ars408_radar run_radar.launch spi_dev:=/dev/spidev1.0
```

### 5. 性能注意事项

- SPIO 默认 10MHz 时钟，2880 字节传输耗时约 0.3ms，对 CPU 负载极小
- 主循环 50Hz（20ms 周期），Xavier CPU 占用预计 < 1%
- 若需降低 CPU 频率以省电，可将 `spi_speed_hz` 降至 `5000000`（5MHz）

---

## 验证数据流

不启动 RViz，直接命令行验证：

```bash
# 查看前雷达目标
rostopic echo /radar_front/objects

# 查看消息频率
rostopic hz /radar_front/objects
rostopic hz /radar_rear/objects

# 查看 TF 树
rosrun tf view_frames
```
