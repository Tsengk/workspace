#include <ros/ros.h>
#include <image_transport/image_transport.h> // 提供专门针对图像传输优化的发布/订阅接口
#include <opencv2/highgui/highgui.hpp>       // 包含 OpenCV 视频捕获 (VideoCapture) 相关功能
#include <cv_bridge/cv_bridge.h>             // 提供 ROS Image 消息与 OpenCV cv::Mat 之间的转换桥梁

int main(int argc, char** argv)
{
    // ==========================================
    // 1. 初始化 ROS 节点环境
    // ==========================================
    // 注册节点名称，"gmsl_camera_node" 是在 ROS Master 中显示的默认名字
    ros::init(argc, argv, "gmsl_camera_node");
    
    // nh 用于处理全局/公共命名空间下的操作 (如发布话题)
    ros::NodeHandle nh;
    // private_nh 使用 "~" 命名空间，专门用于读取节点自身的私有参数
    ros::NodeHandle private_nh("~");

    // ==========================================
    // 2. 声明并获取配置参数
    // ==========================================
    std::string device_port;
    std::string topic_name;
    int fps;
    int image_width;
    int image_height;

    // 从参数服务器读取参数 (由 launch 文件传入)，如果未指定，则使用后面的默认值
    private_nh.param<std::string>("device_port", device_port, "/dev/video0");
    private_nh.param<std::string>("topic_name", topic_name, "/gmsl_image0");
    private_nh.param<int>("fps", fps, 30);
    private_nh.param<int>("width", image_width, 1920);
    private_nh.param<int>("height", image_height, 1080);

    // ==========================================
    // 3. 初始化 ROS 图像发布者
    // ==========================================
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic_name, 1);

    // ==========================================
    // 4. 打开并配置底层硬件设备
    // ==========================================
    cv::VideoCapture cap(device_port, cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open video device: %s (Please check connection or permissions e.g., sudo chmod 777 %s)", device_port.c_str(), device_port.c_str());
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
    cap.set(cv::CAP_PROP_FPS, fps);

    // ==========================================
    // 5. 准备循环抓取的数据结构
    // ==========================================
    cv::Mat frame;              
    sensor_msgs::ImagePtr msg;  
    
    ros::Rate loop_rate(fps);

    ROS_INFO("Camera opened successfully! Start publishing %s images to topic: %s", device_port.c_str(), topic_name.c_str());

    // ==========================================
    // 6. 主循环：采集 -> 转换 -> 发布
    // ==========================================
    while (nh.ok()) {
        cap >> frame;
        
        if (!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = "gmsl_camera_link";

            pub.publish(msg);
        } else {
            ROS_WARN("Grabbed an empty frame! (Possible frame drop, high bandwidth load, or loose cable)");
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    // ==========================================
    // 7. 退出前的资源清理
    // ==========================================
    cap.release();
    ROS_INFO("Node exited safely and camera resources released.");
    return 0;
}