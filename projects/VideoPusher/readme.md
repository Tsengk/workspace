/dev/video0 视频数据采集与 RTMP/RTSP 推流方案 (C++ 极致低延迟版)

1. 方案概述本方案旨在解决从 Linux 系统本地摄像头（/dev/video0，基于 V4L2 框架）使用 C++ 采集实时视频数据，并在代码中支持二次处理（如 AI 识别、滤镜、OSD 叠加），最后转化为 RTMP 或 RTSP 视频流推送到目标服务器的需求。核心数据流向：/dev/video0 -> C++ 进程 (OpenCV 读取帧 -> 图像处理模块) -> 管道 (Pipe) -> FFmpeg 进程 (H.264 编码 + 协议封装) -> 流媒体服务器。架构优势：采用 OpenCV 读取并处理数据，通过 popen 管道将裸流输送给 FFmpeg 进程。通过模块化设计降低二次开发耦合度，并针对低延迟 (Low Latency) 进行了全链路的参数压榨。
2. 前提条件与环境准备操作系统：
    Linux (Ubuntu / CentOS / 树莓派 OS 等)
    硬件设备：已连接并被系统识别的摄像头（可通过 ls -l /dev/video0 验证）。
    推流目标：一个可用的 RTMP 或 RTSP 发布地址（如 rtmp://localhost:1935/live/stream 或 rtsp://localhost:8554/live/stream）。
    
    2.1 开发环境依赖C++ 编译器 (g++)OpenCV 库 (用于视频流读取与处理)FFmpeg (需安装在系统中，供管道调用)Ubuntu 环境安装依赖：
        sudo apt update
        sudo apt install g++ cmake libopencv-dev ffmpeg v4l-utils


    2.2 模块化工程目录层级 (推荐)VideoPusher/               # 工程根目录
    ├── CMakeLists.txt         # CMake 构建配置文件
    ├── include/               # 头文件目录
    │   ├── IImageProcessor.h  # 【接口】图像处理基类 (包含时间戳绘制)
    │   └── VideoStreamer.h    # 【封装】推流器头文件 (支持 RTMP/RTSP)
    ├── src/                   # 源代码目录
    │   ├── VideoStreamer.cpp  # 推流器实现 (支持 RTMP/RTSP)
    │   └── main.cpp           # 主程序入口 
    └── build/                 # 编译输出目录 


3. 部署mediamtx服务器

3.1下载：
wget https://github.com/bluenviron/mediamtx/releases/download/v1.6.0/mediamtx_v1.6.0_linux_amd64.tar.gz
tar -zxvf mediamtx_v1.6.0_linux_amd64.tar.gz
3.2启动mediamtx服务器：./mediamtx

4. C++ 核心代码实现
4.1 图像处理接口层 (include/IImageProcessor.h)#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

/**
 * 图像处理纯虚基类 (接口)
 */
class IImageProcessor {
public:
    virtual ~IImageProcessor() = default;

    /**
     * 处理单帧图像的接口
     * @param frame 输入的原始图像，处理结果直接修改此 frame
     */
    virtual void process(cv::Mat& frame) = 0;
};

// ==========================================
// 示例：二次开发接入 -> 实现一个“添加时间戳和警示框”的处理类
// ==========================================
class MyCustomAlgorithm : public IImageProcessor {
public:
    void process(cv::Mat& frame) override {
        if (frame.empty()) return;

        // 获取当前系统时间 (精确到毫秒)
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // 格式化时间戳字符串，例如 "2023-10-27 15:30:45.123"
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S") 
           << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
        std::string timeStr = ss.str();

        // 在画面左上角添加固定警示文字 (红色)
        cv::putText(frame, "LIVE PUSH (Low Latency)", cv::Point(20, 50), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
        
        // 在警示文字下方添加动态时间戳 (黄色)
        cv::putText(frame, timeStr, cv::Point(20, 100), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        // 画一个绿色的测试框
        cv::rectangle(frame, cv::Rect(100, 130, 200, 200), cv::Scalar(0, 255, 0), 2);
    }
};




4.2 CMake 编译配置 (CMakeLists.txt)cmake_minimum_required(VERSION 3.10)
project(VideoPusher)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(include)
file(GLOB SOURCES "src/*.cpp")
add_executable(stream_pusher ${SOURCES})
target_link_libraries(stream_pusher ${OpenCV_LIBS})


编译与运行步骤：
    进入项目根目录：mkdir build && cd build
    生成 Makefile：cmake ..
    编译工程：make

协议选择及程序运行：
    程序会根据推流地址前缀自动选择协议。
    RTMP：./stream_pusher rtmp://localhost:1935/live/stream /dev/video0
    RTSP：./stream_pusher rtsp://localhost:8554/live/stream /dev/video0
    指定摄像头：./stream_pusher rtsp://localhost:8554/live/stream /dev/video0
    注意：RTSP 推流端需要一个支持发布/推流的 RTSP 服务端，普通 RTSP 摄像头拉流地址通常不能作为推流目标。
    查看帮助：./stream_pusher --help

    
5. 性能优化与进阶说明
    5.1 全链路低延迟策略解析 (已应用)本工程已在代码层面落实了极致的低延迟调优：OpenCV 采集级 (CAP_PROP_BUFFERSIZE, 1)：避免 V4L2 驱动层面的帧积压。如果程序处理慢了半拍，系统会自动丢弃旧帧，保证画面的实时性。编码级 (-bf 0)：关闭 B 帧（双向预测帧）。B 帧需要依赖前后的帧才能解码，会强制编码器和播放器进行缓存等待，关闭它可以立即降低至少 1~2 帧的物理延迟。推流级 (-fflags nobuffer)：关闭 FFmpeg 在打包 FLV 时的内部 I/O 缓冲，生成一点数据就立刻通过 TCP 丢给 Nginx。
    5.2 C++ 层的硬件加速编码 (极度推荐)若需降低 CPU 占用，请在 src/VideoStreamer.cpp 中将 -c:v libx264 替换为：NVIDIA 显卡：-c:v h264_nvencIntel 核显：-c:v h264_qsv树莓派：-c:v h264_v4l2m2m
6. 常见问题排查OpenCV 报错：cannot open camera: 检查占用 (lsof /dev/video0) 或权限 (sudo usermod -a -G video $USER)。管道写入失败: FFmpeg 未安装或推流地址不存在。画面撕裂或卡顿: 算法 (process) 耗时超过了 1/fps (如 30fps 要求 33ms 内处理完)。考虑将 OpenCV 读取与 process 拆分到两个 C++ 线程，通过队列进行异步通讯。
