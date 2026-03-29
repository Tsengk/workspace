ARS408 SPI Radar ROS Driver
这是一个基于 ROS 1 的大陆（Continental）ARS408 毫米波雷达驱动节点。该节点通过 SPI 接口读取底层打包好的 CAN 报文（SPI 转 CAN 桥接），严格按照 ARS408 协议（Motorola 大端序）解析目标模式数据，并在 RViz 中提供双雷达（前向与后向）的 3D 目标可视化。

✨ 核心特性
双雷达通道支持：单次 SPI 读取同步解析 CAN3（前向雷达，索引 0~359）与 CAN2（后向雷达，索引 360~719）数据。

精准协议解析：直接通过位运算提取 0x60A（帧起始）与 0x60B（目标距离、速度），规避了复杂的 DBC 依赖。

硬件与仿真无缝切换：自带 Python 模拟数据生成器，可通过 Launch 参数一键切换实车硬件读取或离线 Bin 文件读取。

RViz 直观可视化：将雷达目标转化为 visualization_msgs::MarkerArray（包含不同颜色的 Bounding Box、ID 与纵向速度），并自动完成 base_link 的 TF 坐标系转换。

📁 目录结构
Plaintext

ars408_radar/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── run_radar.launch        # 核心启动文件（包含 TF 与参数配置）
├── src/
│   └── ars408_spi_node.cpp     # C++ 节点源码
└── test_data/
    ├── generate_mock_spi.py    # 离线模拟数据生成脚本
    └── mock_radar_spi.bin      # 脚本生成的测试二进制流
🛠️ 环境依赖
操作系统：Ubuntu 18.04 (或更高版本)

ROS 版本：Melodic 或 Noetic

依赖包：roscpp, std_msgs, visualization_msgs, tf2_ros

🚀 编译与安装
将本功能包放置于 ROS 工作空间的 src 目录下
在工作空间根目录执行编译：
cd ~/radar_ws
catkin_make
source devel/setup.bash

💻 运行指南
模式 A：离线仿真模式（推荐首次验证使用）
无需连接真实硬件，读取本地生成的模拟二进制流进行算法和可视化验证。

生成模拟数据（仅需执行一次）：
cd src/ars408_radar/test_data
python3 generate_mock_spi.py

启动节点（传入 sim:=true 参数）：
roslaunch ars408_radar run_radar.launch sim:=true

模式 B：实车硬件模式
连接真实底盘与 SPI 线束，读取 /dev/spidev0.0 硬件节点。

⚠️ 权限预处理：
默认普通用户无权读取 SPI 设备。运行前需临时赋权，或将用户永久加入 spi 组：
sudo chmod 777 /dev/spidev0.0

启动节点：
roslaunch ars408_radar run_radar.launch

👁️ RViz 可视化说明
Launch 文件会自动打开 RViz。如果首次打开没有画面，请按以下步骤配置：

将 Fixed Frame 设置为 base_link。

点击 Add -> MarkerArray，Topic 选择 /radar_front/objects（绿色方块，位于车头 2.5m 处）。

再次点击 Add -> MarkerArray，Topic 选择 /radar_rear/objects（红色方块，位于车尾 -1.0m 处）。
