/**
 * @brief GMSL 模拟器：生成带有强畸变的网格图像，模拟 /gmsl_image0
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class GmslSimulator {
public:
    GmslSimulator() : it_(nh_) {
        // 模拟话题
        pub_ = it_.advertise("/gmsl_image0", 1);
        width_ = 1280; height_ = 720;
        
        // 预计算畸变映射 (制造一个人造的鱼眼效果)
        setupDistortionMap();
    }

    void run() {
        ros::Rate rate(30); // 30Hz
        while (ros::ok()) {
            // 1. 生成完美网格
            cv::Mat clean_img = generateGrid();
            
            // 2. 施加畸变 (将完美图像扭曲)
            cv::Mat distorted_img;
            cv::remap(clean_img, distorted_img, map_x_, map_y_, cv::INTER_LINEAR);

            // 3. 发布
            publish(distorted_img);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    int width_, height_;
    cv::Mat map_x_, map_y_;

    void setupDistortionMap() {
        // 使用非常大的 K1 (负值) 来模拟广角/鱼眼畸变
        cv::Mat K = (cv::Mat_<double>(3, 3) << width_*0.8, 0, width_/2, 0, height_*0.8, height_/2, 0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(1, 5) << -0.35, 0.1, 0, 0, 0); // 人造畸变参数
        
        // 注意：这里我们用 initUndistortRectifyMap 的逆向思维来制造畸变
        // 让“完美图”经过这个 map 变成“畸变图”
        // 在工程上，为了模拟简单，我们直接把 K 作为新 K，产生扭曲效果
        cv::initUndistortRectifyMap(K, D, cv::Mat(), K, cv::Size(width_, height_), CV_32FC1, map_x_, map_y_);
    }

    cv::Mat generateGrid() {
        cv::Mat img = cv::Mat::zeros(height_, width_, CV_8UC3);
        img.setTo(cv::Scalar(40, 40, 40));
        // 画网格
        for(int i=0; i<width_; i+=80) cv::line(img, cv::Point(i,0), cv::Point(i,height_), cv::Scalar(200,200,200), 1);
        for(int i=0; i<height_; i+=80) cv::line(img, cv::Point(0,i), cv::Point(width_,i), cv::Scalar(200,200,200), 1);
        // 移动的球
        double t = ros::Time::now().toSec();
        cv::circle(img, cv::Point(width_/2 + sin(t)*300, height_/2 + cos(t*1.5)*200), 30, cv::Scalar(0,0,255), -1);
        cv::putText(img, "SIMULATION: /gmsl_image0", cv::Point(20,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
        return img;
    }

    void publish(const cv::Mat& img) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "sim_camera";
        pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gmsl_simulator");
    GmslSimulator sim;
    sim.run();
    return 0;
}