#pragma once
#include <cstdio>
#include <string>
#include <opencv2/opencv.hpp>

class VideoStreamer {
public:
    // 通用视频推流器，根据 URL 自动切换 RTMP / RTSP。
    VideoStreamer(const std::string& streamUrl,int width,int height,int fps);
    ~VideoStreamer();

    bool start();
    bool pushFrame(const cv::Mat& frame);
    void stop();
    std::string protocolName() const;

private:
    enum class StreamProtocol {
        Rtmp,
        Rtsp,
        Unsupported
    };

    StreamProtocol detectProtocol() const;
    std::string buildFfmpegCommand() const;
    static std::string shellQuote(const std::string& value);

    std::string streamUrl_;
    int width_,height_,fps_;
    FILE* pipe_;
};
