#ifndef BEV_SURROUND_H
#define BEV_SURROUND_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace bev_surround {
class BevSurround {
public:
    BevSurround(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~BevSurround() = default;
private:
    void loadParams();
    void updateTransformMatrix();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher bev_pub_;

    std::string input_topic_;
    std::string output_topic_;
    int bev_width_;
    int bev_height_;

    //梯形透视控制点坐标(顺时针：左上、右上、右下、左下)
    double src_tl_x_, src_tl_y_;
    double src_tr_x_, src_tr_y_;
    double src_br_x_, src_br_y_;
    double src_bl_x_, src_bl_y_;

    cv::Mat homography_matrix_;
};

}// namespace bev_surround

#endif // BEV_SURROUND_H