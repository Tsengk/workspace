#include "bev_surround/bev_surround.h"

namespace bev_surround {
BevSurround::BevSurround(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh), it_(nh_) {
        loadParams();
        updateTransformMatrix();

        //订阅前视相机图像，发布BEV图像
        image_sub_ = it_.subscribe(input_topic_, 1, &BevSurround::imageCallback, this);
        bev_pub_ = it_.advertise(output_topic_, 1);
        ROS_INFO("BevSurround node initialized: [%s] -> [%s]", input_topic_.c_str(), output_topic_.c_str());
    }

void BevSurround::loadParams() {
    pnh_.param<std::string>("input_topic", input_topic_, "/camera/image_raw");
    pnh_.param<std::string>("output_topic", output_topic_, "/bev/image");
    pnh_.param<int>("bev_width", bev_width_, 640);
    pnh_.param<int>("bev_height", bev_height_, 480);

    //针对 640x480 分辨率的前视相机图像，设置梯形透视控制点坐标
    pnh_.param<double>("src_tl_x", src_tl_x_, 220.0);
    pnh_.param<double>("src_tl_y", src_tl_y_, 280.0);
    pnh_.param<double>("src_tr_x", src_tr_x_, 420.0);
    pnh_.param<double>("src_tr_y", src_tr_y_, 280.0);
    pnh_.param<double>("src_br_x", src_br_x_, 580.0);
    pnh_.param<double>("src_br_y", src_br_y_, 460.0);
    pnh_.param<double>("src_bl_x", src_bl_x_, 60.0);
    pnh_.param<double>("src_bl_y", src_bl_y_, 460.0);
}

void BevSurround::updateTransformMatrix() {
    std::vector<cv::Point2f> src_points = {
        cv::Point2f(src_tl_x_, src_tl_y_),
        cv::Point2f(src_tr_x_, src_tr_y_),
        cv::Point2f(src_br_x_, src_br_y_),
        cv::Point2f(src_bl_x_, src_bl_y_)
    };

    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(bev_width_), 0.0f),
        cv::Point2f(static_cast<float>(bev_width_), static_cast<float>(bev_height_)),
        cv::Point2f(0.0f, static_cast<float>(bev_height_))
    };

    //计算透视单位性变换矩阵
    homography_matrix_ = cv::getPerspectiveTransform(src_points, dst_pts);
}

void BevSurround::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat bev_image;
    //逆透视投影映射
    cv::warpPerspective(cv_ptr->image, bev_image, homography_matrix_,
                        cv::Size(bev_width_, bev_height_),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", bev_image).toImageMsg();
    bev_pub_.publish(out_msg);
}
}