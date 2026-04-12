#include "../include/VideoStreamer.h"
#include <algorithm>
#include <cctype>
#include <iostream>

namespace {

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

}  // namespace

VideoStreamer::VideoStreamer(const std::string& streamUrl,int width,int height,int fps)
    : streamUrl_(streamUrl),width_(width),height_(height),fps_(fps),pipe_(nullptr){}

VideoStreamer::~VideoStreamer() {
    stop();
}

VideoStreamer::StreamProtocol VideoStreamer::detectProtocol() const {
    const std::string lowerUrl = toLowerCopy(streamUrl_);
    if(lowerUrl.rfind("rtmp://",0) == 0) {
        return StreamProtocol::Rtmp;
    }
    if(lowerUrl.rfind("rtsp://",0) == 0) {
        return StreamProtocol::Rtsp;
    }
    return StreamProtocol::Unsupported;
}

std::string VideoStreamer::protocolName() const {
    switch (detectProtocol()) {
        case StreamProtocol::Rtmp:
            return "RTMP";
        case StreamProtocol::Rtsp:
            return "RTSP";
        default:
            return "UNSUPPORTED";
    }
}

std::string VideoStreamer::shellQuote(const std::string& value) {
    std::string quoted = "'";
    for(char ch : value) {
        if(ch == '\'') {
            quoted += "'\"'\"'";
            continue;
        }
        quoted += ch;
    }
    quoted += "'";
    return quoted;
}

std::string VideoStreamer::buildFfmpegCommand() const {
    //公共参数：仅包含输入设置和 H.264 视频编码器设置
    const std::string commonArgs =
        "ffmpeg -loglevel warning -y "
        "-f rawvideo -vcodec rawvideo -pix_fmt bgr24 "
        "-s " + std::to_string(width_) + "x" + std::to_string(height_) + " "
        "-r " + std::to_string(fps_) + " "
        "-i - -an -c:v libx264 -pix_fmt yuv420p "
        "-preset ultrafast -tune zerolatency "
        "-bf 0 -g " + std::to_string(fps_) + " "
        "-keyint_min " + std::to_string(fps_) + " "
        "-sc_threshold 0 "; 

    switch (detectProtocol()) {
        case StreamProtocol::Rtmp:
            //RTMP 专属参数：添加直播流专属标记，依赖默认 chunk 机制
            return commonArgs + 
                   "-flvflags no_duration_filesize " // 关键修复：告诉FFmpeg这是直播流，不要尝试回写文件头
                   "-f flv " + shellQuote(streamUrl_);
                   
        case StreamProtocol::Rtsp:
            //RTSP 专属参数：将激进的低延迟参数移到这里
            return commonArgs +
                   "-fflags nobuffer -flags low_delay -flush_packets 1 " // 完美适配 RTSP 的低延迟策略
                   "-rtsp_transport tcp "
                   "-muxdelay 0 -muxpreload 0 "
                   "-f rtsp " + shellQuote(streamUrl_);
                   
        default:
            return "";
    }
}

bool VideoStreamer::start() {
    if(pipe_) {
        std::cerr << "警告：推流器已经启动，无需重复 start()。" << std::endl;
        return true;
    }

    const std::string ffmpegCmd = buildFfmpegCommand();
    if(ffmpegCmd.empty()) {
        std::cerr << "错误：暂不支持的推流地址协议 -> " << streamUrl_ << std::endl;
        std::cerr << "当前仅支持 RTMP (rtmp://...) 和 RTSP (rtsp://...)。" << std::endl;
        return false;
    }

    std::cout << "启动 " << protocolName() << " 推流: " << streamUrl_ << std::endl;
    pipe_ = popen(ffmpegCmd.c_str(), "w");
    if(!pipe_) {
        std::cerr << "错误：无法启动 FFmpeg 管道进程！" << std::endl;
        return false;
    }
    return true;
}

bool VideoStreamer::pushFrame(const cv::Mat& frame) {
    if(!pipe_ || frame.empty()) return false;

    cv::Mat bgrFrame;
    if(frame.type() == CV_8UC3) {
        bgrFrame = frame;
    } else if(frame.type() == CV_8UC4) {
        cv::cvtColor(frame,bgrFrame,cv::COLOR_BGRA2BGR);
    } else if(frame.type() == CV_8UC1) {
        cv::cvtColor(frame,bgrFrame,cv::COLOR_GRAY2BGR);
    } else {
        std::cerr << "错误：当前仅支持 8-bit 的 BGR/BGRA/GRAY 帧格式。" << std::endl;
        return false;
    }

    if(bgrFrame.cols != width_ || bgrFrame.rows != height_) {
        std::cerr << "错误：帧尺寸与推流器配置不匹配，期望 "
                  << width_ << "x" << height_ << "，实际 "
                  << bgrFrame.cols << "x" << bgrFrame.rows << "。" << std::endl;
        return false;
    }

    if(!bgrFrame.isContinuous()) {
        bgrFrame = bgrFrame.clone();
    }

    const size_t frameBytes = bgrFrame.total() * bgrFrame.elemSize();
    const size_t written = fwrite(bgrFrame.data,1,frameBytes,pipe_);
    if(written != frameBytes) {
        return false;
    }
    return fflush(pipe_) == 0;
}

void VideoStreamer::stop() {
    if(pipe_) {
        pclose(pipe_);
        pipe_ = nullptr;
    }
}
