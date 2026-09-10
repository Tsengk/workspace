#ifndef LAPTOP_CAM_H
#define LAPTOP_CAM_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

namespace laptop_cam {
class LaptopCam {
public:
    explicit LaptopCam(ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~LaptopCam();

    bool init();

    void run();
private:
    void captureAndPublish();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher raw_pub_;
    ros::Publisher compressed_pub_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
    std::vector<uchar> encode_buf_;
    std::vector<int> encode_params_;

    int device_id_;
    int width_;
    int height_;
    int fps_;
    std::string frame_id_;
};
}