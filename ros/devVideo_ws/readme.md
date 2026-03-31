 ROS工作空间部署与编译步骤要运行上述节点，需要将其部署在标准的 ROS 工作空间（catkin workspace）中。工程目录层级概览在开始具体部署操作前，您可以先预览一下最终完成后的工作空间文件树结构。请在后续的放置文件步骤中，对照此结构进行核对：~/devVideo_ws/               
 # ROS 工作空间根目录
├── build/                   
├── devel/                   
└── src/                     
    ├── CMakeLists.txt       
    └── gmsl_camera_driver/  # 我们即将创建的功能包
        ├── CMakeLists.txt   
        ├── package.xml      
        ├── launch/          # 将要新增的 launch 文件夹
        │   └── gmsl_camera.launch  # 节点启动配置
        └── src/             
            └── gmsl_camera_node.cpp  
具体操作步骤如下：初始化 ROS 工作空间注意： 编译命令 catkin_make 必须在工作空间的根目录下执行！切勿在 src 目录内执行，否则会出现 The specified source space .../src/src does not exist 的错误。mkdir -p ~/devVideo_ws/src
cd ~/devVideo_ws  # 确保当前处于工作空间根目录
catkin_make
source devel/setup.bash
创建功能包 (Package)cd ~/devVideo_ws/src
catkin_create_pkg gmsl_camera_driver roscpp sensor_msgs cv_bridge image_transport
放置代码与配置文件源文件：在 ~/devVideo_ws/src/gmsl_camera_driver/src/ 目录下新建 gmsl_camera_node.cpp。Launch 文件：在 ~/devVideo_ws/src/gmsl_camera_driver/ 目录下新建 launch 文件夹，并在其中创建 gmsl_camera.launch。CMake 配置：完全覆盖原本生成的 CMakeLists.txt。包信息：检查并修改 package.xml 的依赖。解决设备权限问题 (关键)Linux 默认普通用户没有读取 /dev/video0 的权限。请将当前用户加入 video 组：sudo usermod -aG video $USER
# 注意：执行后需要注销并重新登录终端才能永久生效！
编译工作空间回到工作空间根目录进行编译：cd ~/devVideo_ws  # 再次确认处于工作空间根目录
catkin_make
source devel/setup.bash
运行节点 (使用 Launch 文件)现在不再需要手动运行 roscore 和输入一长串参数了。直接使用 roslaunch 一键启动：# 启动节点并加载参数 (会自动启动 roscore 如果它还没运行)
roslaunch gmsl_camera_driver gmsl_camera.launch
验证结果打开新的终端，查看话题和图像：rostopic hz /gmsl_image0  # 检查发布频率
rqt_image_view /gmsl_image0 # 可视化查看图像
