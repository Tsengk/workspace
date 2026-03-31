#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include "../include/RtmpStreamer.h"
#include "../include/IImageProcessor.h"


int main() {
    std::string rtmp_Url = "rtmp://localhost:1935/live/stream";
    int width = 1280;
    int height = 720;
    int fps = 30;

    cv::VideoCapture cap("/dev/video0",cv::CAP_V4L2);
    if(!cap.isOpened()) {
        std::cerr << "错误：无法打开摄像头/dev/video0!" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    cap.set(cv::CAP_PROP_FPS,fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE,1);

    RtmpStreamer streamer(rtmp_Url,width,height,fps);
    if(!streamer.start()) return -1;

    std::unique_ptr<IImageProcessor> processor = std::make_unique<MyCustomAlgorithm>();
    std::cout << "开始采集推流，按 Ctrl+C 停止..." << std::endl;


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