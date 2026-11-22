#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_model/mei.h"

class CameraModelTest {
public:
    CameraModelTest(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh)
    {
        loadParams();
        initCamera();
        setupPubSub();
        ROS_INFO("[CameraModelTest] init OK");
    }

private:
    void loadParams()
    {
        std::string calib_file;
        pnh_.param<std::string>("calib_file", calib_file, "");

        if (!calib_file.empty()) {
            loadFromYaml(calib_file);
        } else {
            loadFromParams();
        }

        std::vector<int> rotation_order_vec = {2, 1, 0};
        pnh_.getParam("rotation_order", rotation_order_vec);
        if (rotation_order_vec.size() >= 3) {
            rotation_order_[0] = rotation_order_vec[0];
            rotation_order_[1] = rotation_order_vec[1];
            rotation_order_[2] = rotation_order_vec[2];
        }

        pnh_.param<std::string>("input_topic", input_topic_, "/camera/image_raw");
        pnh_.param<std::string>("output_topic", output_topic_, "/camera/undistorted");

        pnh_.param<double>("undistort_fx", undistort_fx_, K_.at<double>(0, 0));
        pnh_.param<double>("undistort_fy", undistort_fy_, K_.at<double>(1, 1));
        pnh_.param<double>("undistort_cx", undistort_cx_, K_.at<double>(0, 2));
        pnh_.param<double>("undistort_cy", undistort_cy_, K_.at<double>(1, 2));
        pnh_.param<int>("undistort_width", undistort_width_, image_width_);
        pnh_.param<int>("undistort_height", undistort_height_, image_height_);
    }

    void loadFromYaml(const std::string& path)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            ROS_FATAL("[CameraModelTest] failed to open calib file: %s", path.c_str());
            throw std::runtime_error("failed to open calib file");
        }

        fs["image_width"] >> image_width_;
        fs["image_height"] >> image_height_;

        cv::FileNode mirror = fs["mirror_parameters"];
        mirror["xi"] >> epsilon_;

        cv::FileNode projection = fs["projection_parameters"];
        K_ = cv::Mat::eye(3, 3, CV_64F);
        projection["gamma1"] >> K_.at<double>(0, 0);
        projection["gamma2"] >> K_.at<double>(1, 1);
        projection["u0"] >> K_.at<double>(0, 2);
        projection["v0"] >> K_.at<double>(1, 2);

        cv::FileNode distortion = fs["distortion_parameters"];
        D_ = cv::Mat::zeros(1, 4, CV_64F);
        distortion["k1"] >> D_.at<double>(0, 0);
        distortion["k2"] >> D_.at<double>(0, 1);
        distortion["p1"] >> D_.at<double>(0, 2);
        distortion["p2"] >> D_.at<double>(0, 3);

        fs.release();
        ROS_INFO("[CameraModelTest] loaded calib from %s: %dx%d", path.c_str(), image_width_, image_height_);
    }

    void loadFromParams()
    {
        pnh_.param<double>("epsilon", epsilon_, 0.0);
        pnh_.param<int>("image_width", image_width_, 1280);
        pnh_.param<int>("image_height", image_height_, 960);

        double fx, fy, cx, cy;
        pnh_.param<double>("fx", fx, 600.0);
        pnh_.param<double>("fy", fy, 600.0);
        pnh_.param<double>("cx", cx, 640.0);
        pnh_.param<double>("cy", cy, 480.0);

        K_ = cv::Mat::eye(3, 3, CV_64F);
        K_.at<double>(0, 0) = fx;
        K_.at<double>(1, 1) = fy;
        K_.at<double>(0, 2) = cx;
        K_.at<double>(1, 2) = cy;

        double k1, k2, p1, p2;
        pnh_.param<double>("k1", k1, 0.0);
        pnh_.param<double>("k2", k2, 0.0);
        pnh_.param<double>("p1", p1, 0.0);
        pnh_.param<double>("p2", p2, 0.0);

        D_ = cv::Mat::zeros(1, 4, CV_64F);
        D_.at<double>(0, 0) = k1;
        D_.at<double>(0, 1) = k2;
        D_.at<double>(0, 2) = p1;
        D_.at<double>(0, 3) = p2;
    }

    void initCamera()
    {
        mei_ = std::make_unique<camera::Mei>(rotation_order_);
        mei_->setIntrinsics(epsilon_, K_, D_, image_width_, image_height_);

        cv::Mat undistort_K = cv::Mat::eye(3, 3, CV_64F);
        undistort_K.at<double>(0, 0) = undistort_fx_;
        undistort_K.at<double>(1, 1) = undistort_fy_;
        undistort_K.at<double>(0, 2) = undistort_cx_;
        undistort_K.at<double>(1, 2) = undistort_cy_;
        mei_->setUndistortInfo(undistort_width_, undistort_height_, undistort_K);

        ROS_INFO("[CameraModelTest] camera initialized: %dx%d, undistort fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                 image_width_, image_height_, undistort_fx_, undistort_fy_, undistort_cx_, undistort_cy_);
    }

    void setupPubSub()
    {
        sub_ = nh_.subscribe(input_topic_, 1, &CameraModelTest::imageCallback, this);
        pub_ = it_.advertise(output_topic_, 1);
        ROS_INFO("[CameraModelTest] subscribed to %s, publishing to %s",
                 input_topic_.c_str(), output_topic_.c_str());
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
    {
        cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
                        const_cast<unsigned char*>(msg->data.data()));
        cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (image.empty()) {
            ROS_WARN("[CameraModelTest] decode failed");
            return;
        }

        cv::Mat undistorted;
        if (!mei_->undistortImage(image, undistorted)) {
            ROS_WARN("[CameraModelTest] undistort failed");
            return;
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        pub_.publish(cv_bridge::CvImage(header, "bgr8", undistorted).toImageMsg());
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    image_transport::ImageTransport it_;

    std::unique_ptr<camera::Mei> mei_;
    cv::Mat K_;
    cv::Mat D_;
    double epsilon_ = 0.0;
    int image_width_ = 1280;
    int image_height_ = 960;
    std::array<int, 3> rotation_order_ = {{2, 1, 0}};
    double undistort_fx_ = 800.0;
    double undistort_fy_ = 800.0;
    double undistort_cx_ = 960.0;
    double undistort_cy_ = 540.0;
    int undistort_width_ = 1920;
    int undistort_height_ = 1080;

    std::string input_topic_;
    std::string output_topic_;
    ros::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_model_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        CameraModelTest node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[camera_model_test_node] fatal: %s", e.what());
        return 1;
    }
    return 0;
}
