#include "../include/RtmpStreamer.h"
#include <iostream>


RtmpStreamer::RtmpStreamer(const std::string& rtmpUrl,int width,int height,int fps):rtmpUrl_(rtmpUrl),width_(width),height_(height),fps_(fps){}
RtmpStreamer::~RtmpStreamer() {
    stop();
    std::cout << "推流结束。" << std::endl;
}

bool RtmpStreamer::start() {
    std::string ffmpeg_cmd = 
        "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgr24 "
        "-s " + std::to_string(width_) + "x" + std::to_string(height_) + " "
        "-r " + std::to_string(fps_) + " "
        "-i - -c:v libx264 -pix_fmt yuv420p  "
        "-preset ultrafast -tune zerolatency "
        "-bf 0 -g " + std::to_string(fps_) + " "
        "-fflags nobuffer "
        "-f flv " + rtmpUrl_;

        pipe_ = popen(ffmpeg_cmd.c_str(), "w");
        if(!pipe_) {
            std::cerr << "错误：无法启动FFmpeg管道进行！"<< std::endl;
            return false;
        }
        return true;
}

bool RtmpStreamer::pushFrame(const cv::Mat& frame) {
    if(!pipe_ || frame.empty()) return false;
    size_t written = fwrite(frame.data,1,frame.total() * frame.elemSize(),pipe_);
    return written == (frame.total() * frame.elemSize());
}

void RtmpStreamer::stop() {
    if(pipe_) {
        pclose(pipe_);
        pipe_ = nullptr;
    }
}