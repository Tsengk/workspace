#include "laptop_cam/laptop_cam.h"

namespace laptop_cam {
LaptopCam::LaptopCam(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), device_id_(0), width_(640), height_(480), fps_(30), frame_id_("camera_link") {
    encode_params_ = {cv::IMWRITE_JPEG_QUALITY, 90};  // Set JPEG quality to 90
}
LaptopCam::~LaptopCam() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

bool LaptopCam::init() {
    //读取参数
    pnh_.param<int>("device_id", device_id_, 0);
    pnh_.param<int>("width", width_, 640);
    pnh_.param<int>("height", height_, 480);
    pnh_.param<int>("fps", fps_, 30);
    pnh_.param<std::string>("frame_id", frame_id_, "camera_link");

    //初始化发布者
    raw_pub_ = nh_.advertise<sensor_msgs::Image>("camera/image_raw", 10);
    compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("camera/image_compressed", 10);

    //打开摄像头硬件
    cap_.open(device_id_, cv::CAP_V4L2);  // 使用V4L2后端打开摄像头
    if (!cap_.isOpened()) {
        ROS_ERROR("无法打开视频设备: /dev/video%d", device_id_);
        return false;
    }

    //设置硬件属性
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    ROS_INFO("摄像头初始化成功: device_id=%d, width=%d, height=%d, fps=%d", device_id_, width_, height_, fps_);
    return true;
}

void LaptopCam::captureAndPublish() {
    cap_ >> frame_;
    if (frame_.empty()) {
        ROS_WARN_THROTTLE(2.0, "未能从摄像头抓取到图像帧...");
        return;
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;

    // 发布原始图像
    if (raw_pub_.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
        raw_pub_.publish(raw_msg);
    }
    // 发布压缩图像
    if (compressed_pub_.getNumSubscribers() > 0) {
        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header = header;
        compressed_msg.format = "jpeg";
        cv::imencode(".jpg", frame_, encode_buf_, encode_params_);
        compressed_msg.data = encode_buf_;
        compressed_pub_.publish(compressed_msg);
    }
}

void LaptopCam::run() {
    ros::Rate loop_rate(fps_);
    while (ros::ok()) {
        captureAndPublish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
}//namespace laptop_cam