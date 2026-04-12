#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <string>
#include "../include/VideoStreamer.h"
#include "../include/IImageProcessor.h"


int main(int argc,char* argv[]) {
    if(argc > 1) {
        const std::string firstArg = argv[1];
        if(firstArg == "-h" || firstArg == "--help") {
            std::cout << "用法: " << argv[0] << " [stream_url] [camera_device]" << std::endl;
            std::cout << "RTMP 示例: " << argv[0] << " rtmp://localhost:1935/live/stream" << std::endl;
            std::cout << "RTSP 示例: " << argv[0] << " rtsp://localhost:8554/live/stream" << std::endl;
            std::cout << "摄像头示例: " << argv[0] << " rtsp://localhost:8554/live/stream /dev/video0" << std::endl;
            return 0;
        }
    }

    std::string streamUrl = argc > 1 ? argv[1] : "rtmp://localhost:1935/live/stream";
    std::string cameraDevice = argc > 2 ? argv[2] : "/dev/video0";
    int width = 1280;
    int height = 720;
    int fps = 30;

    cv::VideoCapture cap(cameraDevice,cv::CAP_V4L2);
    if(!cap.isOpened()) {
        std::cerr << "错误：无法打开摄像头 " << cameraDevice << " !" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    cap.set(cv::CAP_PROP_FPS,fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE,1);

    VideoStreamer streamer(streamUrl,width,height,fps);
    if(!streamer.start()) {
        std::cerr << "提示：请确认目标地址使用 rtmp:// 或 rtsp:// 开头，且服务端支持推流发布。" << std::endl;
        return -1;
    }

    std::unique_ptr<IImageProcessor> processor = std::make_unique<MyCustomAlgorithm>();
    std::cout << "开始采集并推送到 " << streamer.protocolName()
              << " 地址 " << streamUrl << " ，按 Ctrl+C 停止..." << std::endl;


    cv::Mat frame;

    while (true)
    {
        cap >> frame;
        if(frame.empty()) {
            std::cerr << "警告：抓取到空帧！" << std::endl;
            break;
        }

        if(processor) {
            processor->process(frame);
        }

        if(!streamer.pushFrame(frame)) {
            std::cerr << "错误：管道写入失败，FFmpeg 可能已退出！" << std::endl;
            break;
        }
        
    }
    cap.release();
    streamer.stop();
    std::cout << "推流结束。" << std::endl;
    return 0;
}
