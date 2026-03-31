#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

class IImageProcessor {
public:
    virtual ~IImageProcessor() = default;

    virtual void process(cv::Mat& frame) = 0;
};

class MyCustomAlgorithm : public IImageProcessor {
public:
    void process(cv::Mat& frame) override {
        if(frame.empty()) return;

        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time_t),"%Y-%m-%d %H:%M:%S")
                                                << '.' <<std::setfill('0') << std::setw(3) << now_ms.count();
        std::string timeStr = ss.str();

        cv::putText(frame,"LIVE PUSH (Custom Algorithm)", cv::Point(20,50),
                                cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(0,0,255),3);

        cv::putText(frame,timeStr,cv::Point(20,100),
                                cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,255),3 );
        //cv::rectangle(frame,cv::Rect(100,100,200,200),cv::Scalar(0,255,0),2);
    }
};