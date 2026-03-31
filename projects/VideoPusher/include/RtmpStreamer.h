#pragma once
#include <string>
#include <opencv2/opencv.hpp>

class RtmpStreamer {
public:
    RtmpStreamer(const std::string& rtmpUrl,int width,int height,int fps);
    ~RtmpStreamer();

    bool start();
    bool pushFrame(const cv::Mat& frame);
    void stop();

private:
    std::string rtmpUrl_;
    int width_,height_,fps_;
    FILE* pipe_;
};