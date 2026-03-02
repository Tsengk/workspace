1、网络环境配置
    1.使用网线将 Helios 16 接口盒的 RJ45 网口连接至电脑
    2.打开 Ubuntu 的网络设置，找到对应的有线网卡，进行手动 IPv4 设置：
      IP 地址: 192.168.1.113 （雷达默认目标发送地址） 
      子网掩码: 255.255.255.0 
      网关: 192.168.1.1
    3.保存并重新连接网络。打开终端测试连通性：ping 192.168.1.202

2.安装抓包和基础编译依赖
    sudo apt-get update
    sudo apt-get install -y libpcap-dev

3.编译工程与可视化
    cd ~/rs-driver_ws

    # 编译整个工作空间
    catkin_make

    # 刷新环境变量
    source devel/setup.bash

    # 启动节点（SDK 自带的 launch 文件会自动调起 RViz 并加载默认配置）
    roslaunch rslidar_sdk start.launch