#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "video_publisher_node");
    ros::NodeHandle nh;

    // 使用 image_transport 来创建图像发布者
    image_transport::ImageTransport it(nh);
    // 发布到 /camera/image_raw 话题，队列长度设为 10
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 10);

    // 打开本地摄像头 /dev/video0
    // 参数 0 通常对应 /dev/video0，如果你有多个摄像头，可以尝试 1, 2 等
    cv::VideoCapture cap(0); 
    // 或者也可以显式指定路径： cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera /dev/video0. Check device connection and permissions.");
        return 1;
    }

    ROS_INFO("Successfully opened the camera, publishing image data...");

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    // 设置循环频率为 30 Hz
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        cap >> frame; // 从摄像头读取一帧图像

        if (!frame.empty()) {
            // 将 OpenCV 的 cv::Mat 转换为 ROS 的 sensor_msgs::Image
            // "bgr8" 对应标准的彩色图像编码格式
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            
            // 发布图像消息
            pub.publish(msg);
        } else {
            ROS_WARN("读取到空帧！");
        }

        // 处理回调（虽然这个简单的发布者没有订阅者回调，但加上是个好习惯）
        ros::spinOnce();
        
        // 按照设定的频率休眠
        loop_rate.sleep();
    }

    // 释放资源
    cap.release();
    return 0;
}