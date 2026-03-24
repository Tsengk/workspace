#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class FaceDetector
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::CascadeClassifier face_cascade_;

public:
    FaceDetector() : it_(nh_)
    {
        // 加载 OpenCV 官方人脸检测模型
        // 注意：根据你的 Ubuntu/ROS 版本，路径可能是 /usr/share/opencv4/... 或 /usr/share/opencv/...
        //std::string cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";
        std::string package_path = ros::package::getPath("face_detector");
        std::string cascade_path = package_path + "/data/haarcascade_frontalface_default.xml";
        if (!face_cascade_.load(cascade_path))
        {
            ROS_ERROR("无法加载人脸模型！请检查该文件是否存在: %s", cascade_path.c_str());
        }

        // 订阅你之前写的相机包发布的图像话题
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &FaceDetector::imageCallback, this);

        cv::namedWindow("Face Detection Window");
        ROS_INFO("人脸检测节点已启动，正在监听 /camera/image_raw 话题...");
    }

    ~FaceDetector()
    {
        cv::destroyWindow("Face Detection Window");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge 异常: %s", e.what());
            return;
        }

        cv::Mat gray_image;
        std::vector<cv::Rect> faces;

        // 转灰度图并提升对比度
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray_image, gray_image);

        // 检测人脸
        face_cascade_.detectMultiScale(gray_image, faces, 1.1, 4, 0, cv::Size(30, 30));

        // 画框
        for (size_t i = 0; i < faces.size(); i++)
        {
            cv::rectangle(cv_ptr->image, faces[i], cv::Scalar(255, 0, 0), 2);
        }

        // 显示图像
        cv::imshow("Face Detection Window", cv_ptr->image);
        cv::waitKey(3); 
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, ""); // 支持中文终端输出
    ros::init(argc, argv, "face_detector_node");
    FaceDetector fd;
    ros::spin();
    return 0;
}