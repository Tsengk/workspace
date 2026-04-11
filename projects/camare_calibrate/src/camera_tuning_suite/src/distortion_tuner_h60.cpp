/**
 * @brief H60 专用调参器，支持 K/D 列表读取与 P 矩阵生成
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <mutex>
#include <fstream> //用于写入yaml

// 滑动条精度设置
#define S_MAX 2000
#define S_CEN 1000
#define SCALE 0.001

class TunerH60 {
public:
    TunerH60() : it_(nh_), pnh_("~") {
        loadParams();
        sub_ = it_.subscribe("/gmsl_image0", 1, &TunerH60::cb, this);
        setupGUI();
    }
    ~TunerH60() { cv::destroyWindow("Tuner H60"); }
    
    void run() {
        ros::Rate r(30);
        while(ros::ok()) {
            process();
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    cv::Mat img_;
    std::mutex mtx_;
    
    // 参数
    int w_, h_;
    double k_[9], d_[5]; // K矩阵9个，D系数5个
    
    // 滑动条引用
    int sl_fx, sl_fy, sl_cx, sl_cy;
    int sl_k1, sl_k2, sl_p1, sl_p2, sl_k3;
    int sl_zoom;

    void loadParams() {
        w_ = 1280; h_ = 720; // 默认
        pnh_.getParam("width", w_); pnh_.getParam("height", h_);
        
        std::vector<double> kv, dv;
        // 读取 YAML 中的列表
        if(pnh_.getParam("K", kv) && kv.size()>=9) std::copy(kv.begin(), kv.begin()+9, k_);
        else { k_[0]=w_*0.8; k_[2]=w_/2; k_[4]=h_*0.8; k_[5]=h_/2; k_[8]=1; } // 默认
        
        if(pnh_.getParam("D", dv) && dv.size()>=5) std::copy(dv.begin(), dv.begin()+5, d_);
        else { memset(d_, 0, sizeof(d_)); }

        // 初始化滑动条位置
        sl_fx = k_[0]; sl_fy = k_[4]; sl_cx = k_[2]; sl_cy = k_[5];
        sl_k1 = (d_[0]/SCALE) + S_CEN; sl_k2 = (d_[1]/SCALE) + S_CEN;
        sl_p1 = (d_[2]/SCALE) + S_CEN; sl_p2 = (d_[3]/SCALE) + S_CEN;
        sl_k3 = (d_[4]/SCALE) + S_CEN;
        sl_zoom = 0; // 默认裁剪黑边
    }

    void cb(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> l(mtx_);
        try { img_ = cv_bridge::toCvCopy(msg, "bgr8")->image; } catch(...) {}
    }

    void setupGUI() {
        cv::namedWindow("Tuner H60", 0);
        cv::resizeWindow("Tuner H60", 1000, 600);
        cv::createTrackbar("Fx", "Tuner H60", &sl_fx, 3000);
        cv::createTrackbar("Fy", "Tuner H60", &sl_fy, 3000);
        cv::createTrackbar("Cx", "Tuner H60", &sl_cx, w_*1.5);
        cv::createTrackbar("Cy", "Tuner H60", &sl_cy, h_*1.5);
        cv::createTrackbar("K1", "Tuner H60", &sl_k1, S_MAX);
        cv::createTrackbar("K2", "Tuner H60", &sl_k2, S_MAX);
        cv::createTrackbar("P1", "Tuner H60", &sl_p1, S_MAX);
        cv::createTrackbar("P2", "Tuner H60", &sl_p2, S_MAX);
        cv::createTrackbar("K3", "Tuner H60", &sl_k3, S_MAX);
        cv::createTrackbar("Zoom", "Tuner H60", &sl_zoom, 100);
    }

    void process() {
        cv::Mat raw;
        { std::lock_guard<std::mutex> l(mtx_); if(img_.empty()) return; raw = img_.clone(); }

        // 1. 获取当前参数
        double fx = sl_fx, fy = sl_fy, cx = sl_cx, cy = sl_cy;
        double k1 = (sl_k1 - S_CEN)*SCALE, k2 = (sl_k2 - S_CEN)*SCALE;
        double p1 = (sl_p1 - S_CEN)*SCALE, p2 = (sl_p2 - S_CEN)*SCALE;
        double k3 = (sl_k3 - S_CEN)*SCALE;

        cv::Mat K = (cv::Mat_<double>(3,3) << fx,0,cx, 0,fy,cy, 0,0,1);
        cv::Mat D = (cv::Mat_<double>(1,5) << k1,k2,p1,p2,k3);
        
        // 2. 核心去畸变
        cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, raw.size(), sl_zoom/100.0, raw.size());
        cv::Mat map1, map2;
        cv::initUndistortRectifyMap(K, D, cv::Mat(), new_K, raw.size(), CV_16SC2, map1, map2);
        
        cv::Mat undistsorted;
        cv::remap(raw, undistsorted, map1, map2, cv::INTER_LINEAR);

        // 3. 辅助线
        cv::line(undistsorted, cv::Point(w_/2,0), cv::Point(w_/2,h_), cv::Scalar(0,255,0), 1);
        cv::line(undistsorted, cv::Point(0,h_/2), cv::Point(w_,h_/2), cv::Scalar(0,255,0), 1);

        cv::imshow("Tuner H60", undistsorted);
        if(cv::waitKey(1) == 's') {
		    double fx = sl_fx, fy = sl_fy, cx = sl_cx, cy = sl_cy;
            double k1 = (sl_k1 - S_CEN)*SCALE, k2 = (sl_k2 - S_CEN)*SCALE;
            double p1 = (sl_p1 - S_CEN)*SCALE, p2 = (sl_p2 - S_CEN)*SCALE;
            double k3 = (sl_k3 - S_CEN)*SCALE;
            saveToFile(fx, fy, cx, cy, k1, k2, p1, p2, k3, new_K);
		    printToTerminal(fx,fy,cx,cy, k1,k2,p1,p2,k3, new_K);
	    }
    }

    void saveToFile(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3, cv::Mat P_mat) {
        // 你可以通过 launch 文件传入你想保存的绝对路径，默认保存在当前终端运行的目录下
        std::string save_path;
        //私有节点句柄 pnh_("~") 来读取参数
        pnh_.param<std::string>("save_path", save_path, "front_h60_intrinsics.yaml");

        std::ofstream out(save_path);
        if (!out.is_open()) {
            ROS_ERROR("Failed to open file for writing: %s", save_path.c_str());
            return;
        }

        char buf[512]; // 用于格式化浮点数和对齐

        // 写入头部和基础信息
        out << "header: \n"
            << "  seq: 0\n"
            << "  stamp: \n"
            << "    secs: 0\n"
            << "    nsecs:         0\n"
            << "  frame_id: short_camera\n"
            << "height: " << h_ << "\n"
            << "width: " << w_ << "\n"
            << "distortion_model: plumb_bob\n";

        // 写入 D 矩阵
        snprintf(buf, sizeof(buf), "D: [%.5f, %.5f, %.5f, %.5f, %.5f]\n", k1, k2, p1, p2, k3);
        out << buf;

        // 写入 K 矩阵 (严格匹配你要求的空格对齐)
        snprintf(buf, sizeof(buf), "K: [ %.5f,     0.     ,   %.5f,\n"
                                   "             0.     ,  %.5f,   %.5f,\n"
                                   "             0.     ,     0.     ,     1.     ]\n", 
                                   fx, cx, fy, cy);
        out << buf;

        // 写入 R 矩阵
        out << "R: [ 1.,  0.,  0.,\n"
            << "          0.,  1.,  0.,\n"
            << "          0.,  0.,  1.]\n";

        // 写入 P 矩阵
        snprintf(buf, sizeof(buf), "P: [ %.5f ,     0.     ,   %.5f,     0.     ,\n"
                                   "             0.     ,  %.5f,   %.5f,     0.     ,\n"
                                   "             0.     ,     0.     ,     1.     ,     0.     ]\n",
                 P_mat.at<double>(0,0), P_mat.at<double>(0,2), P_mat.at<double>(1,1), P_mat.at<double>(1,2));
        out << buf;

        // 写入结尾
        out << "binning_x: 0\n"
            << "binning_y: 0\n"
            << "roi: \n"
            << "  x_offset: 0\n"
            << "  y_offset: 0\n"
            << "  height: 0\n"
            << "  width: 0\n"
            << "  do_rectify: False\n";

        out.close();
        ROS_INFO("\033[1;32m[SUCCESS] Parameters saved to: %s\033[0m", save_path.c_str());
    }
    void printToTerminal(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3, cv::Mat P_mat) {
        printf("\n====== COPY TO front_h60_intrinsics.yaml ======\n");
        printf("height: %d\nwidth: %d\ndistortion_model: plumb_bob\n", h_, w_);
        printf("D: [%.5f, %.5f, %.5f, %.5f, %.5f]\n", k1,k2,p1,p2,k3);
        printf("K: [%.5f, 0., %.5f, 0., %.5f, %.5f, 0., 0., 1.]\n", fx, cx, fy, cy);
        printf("R: [1., 0., 0., 0., 1., 0., 0., 0., 1.]\n");
        printf("P: [%.5f, 0., %.5f, 0., 0., %.5f, %.5f, 0., 0., 0., 1., 0.]\n", 
               P_mat.at<double>(0,0), P_mat.at<double>(0,2), P_mat.at<double>(1,1), P_mat.at<double>(1,2));
        printf("===============================================\n");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distortion_tuner_h60");
    TunerH60 t; t.run(); return 0;
}
