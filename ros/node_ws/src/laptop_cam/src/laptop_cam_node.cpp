#include "laptop_cam/laptop_cam.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "laptop_cam_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    laptop_cam::LaptopCam cam(nh, pnh);
    if (!cam.init()) {
        ROS_ERROR("摄像头初始化失败，程序退出...");
        return -1;
    }
    cam.run();
    return 0;
}