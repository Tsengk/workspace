#include "perception_calibration.h"

namespace calibration {

  //用于控制终端循环输出的标志位
  static bool g_disable_debug_print = false;

namespace {
int getch() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
}

calib6d PerceptionCalibration::buildCalib6d(const double x, const double y, const double z,
                                  const double roll, const double pitch, const double yaw) {
  calib6d c6d;
  c6d.x = x;
  c6d.y = y;
  c6d.z = z;
  c6d.t << c6d.x, c6d.y, c6d.z;
  c6d.roll = roll; //x
  c6d.pitch = pitch; //y
  c6d.yaw = yaw; //z
  c6d.R = getRotationMat(c6d.roll, c6d.pitch, c6d.yaw);

  Eigen::Matrix3d R_test(3,3);
  R_test << c6d.R.row(0)[0],c6d.R.row(0)[1],c6d.R.row(0)[2],c6d.R.row(1)[0],c6d.R.row(1)[1],c6d.R.row(1)[2],c6d.R.row(2)[0],c6d.R.row(2)[1],c6d.R.row(2)[2];
  Eigen::Quaterniond q;
  q = R_test;


  c6d.RtLidar2Cam.row(0) <<  c6d.R(0, 0), c6d.R(0, 1), c6d.R(0, 2), c6d.t(0, 0);
  c6d.RtLidar2Cam.row(1) <<  c6d.R(1, 0), c6d.R(1, 1), c6d.R(1, 2), c6d.t(1, 0);
  c6d.RtLidar2Cam.row(2) <<  c6d.R(2, 0), c6d.R(2, 1), c6d.R(2, 2), c6d.t(2, 0);
  c6d.RtLidar2Cam.row(3) << 0, 0, 0, 1;
  return c6d;
}

Eigen::Matrix<double, 3, 3> PerceptionCalibration::getRotationMat(const float roll, const float pitch, const float yaw) {
  Eigen::Matrix<double, 3, 3> R;
  double s1, c1, s2, c2, s3, c3;
  // 1-Z-yaw, 2-Y-pitch, 3-X-roll
  s1 = sin(yaw); c1 = cos(yaw);
  s2 = sin(pitch); c2 = cos(pitch);
  s3 = sin(roll); c3 = cos(roll);
  R.row(0) << c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2;
  R.row(1) << c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3;
  R.row(2) << -s2, c2*s3, c2*c3;

  return R;
}


void PerceptionCalibration::initCameraParams() {
  Parser parser(config_.intrinsics_path);
  parser.ParseCameraInfoFromYaml(&image_width,
                                 &image_height,
                                 &camera_intrinsic_,
                                 &camera_distortion_);
  parser.setPath(config_.extrinsics_path);
  parser.ParseCamera6DofFromYaml(&camera_6DoF_);

  std::cout << "*************************************************" << std::endl;
  std::cout << "Image Size(w,h): " << image_width << "," << image_height << std::endl;
  std::cout << "Intrinsic:\n" << camera_intrinsic_ << std::endl;
  std::cout << "Distortion:\n" << camera_distortion_.transpose() << std::endl;
  std::cout << "6DoF:\n" << camera_6DoF_.transpose() << std::endl;
  std::cout << "*************************************************" << std::endl;

  R_World2RtCar << 0, -1, 0,
                0, 0, -1,
                1, 0, 0;

  // T_L2Cam << -0.068, -0.299, -0.771; // wbd
  // T_L2Cam << -0.032, -0.243, -0.603; // wxb

  T_L2Cam << camera_6DoF_(0), camera_6DoF_(1), camera_6DoF_(2);

  init_rot = buildCalib6d(0, 0, 0, 90.0 * M_PI/180, -90.0 * M_PI/180, 0.0 * M_PI/180);

  tf::Matrix3x3 R_m3;

  // // wbd
  // R_m3.setRPY(angles::from_degrees(90.0),
  //             angles::from_degrees(-90.0),
  //             angles::from_degrees(0.0));
  // // wxb
  // R_m3.setRPY(angles::from_degrees(90.0),
  //             angles::from_degrees(0.0),
  //             angles::from_degrees(0.0));

  R_m3.setRPY(angles::from_degrees(camera_6DoF_(3)),
              angles::from_degrees(camera_6DoF_(4)),
              angles::from_degrees(camera_6DoF_(5)));

  Eigen::Quaterniond q;
  Eigen::Matrix<double, 3, 3> R;
  R = getRotationMat(camera_6DoF_[3] * DEG2RAD, camera_6DoF_[4] * DEG2RAD, camera_6DoF_[5] * DEG2RAD);
  q = R;
  std::cout << "R = " << R << std::endl;
  std::cout << "Quaternion : " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;


  R_L2Cam(0, 0) = R_m3[0][0];
  R_L2Cam(0, 1) = R_m3[0][1];
  R_L2Cam(0, 2) = R_m3[0][2];
  R_L2Cam(1, 0) = R_m3[1][0];
  R_L2Cam(1, 1) = R_m3[1][1];
  R_L2Cam(1, 2) = R_m3[1][2];
  R_L2Cam(2, 0) = R_m3[2][0];
  R_L2Cam(2, 1) = R_m3[2][1];
  R_L2Cam(2, 2) = R_m3[2][2];

  lidar2camera << R_L2Cam(0, 0), R_L2Cam(0, 1), R_L2Cam(0, 2), T_L2Cam(0),
               R_L2Cam(1, 0), R_L2Cam(1, 1), R_L2Cam(1, 2), T_L2Cam(1),
               R_L2Cam(2, 0), R_L2Cam(2, 1), R_L2Cam(2, 2), T_L2Cam(2),
               0, 0, 0, 1;

  M_Mat = camera_intrinsic_ * R_L2Cam;

  p_4 = camera_intrinsic_ * T_L2Cam;

  M_Mat_inv_ = M_Mat.inverse();
  C_tilde_ = -1.0 * M_Mat_inv_ * p_4;


  //初始化滑块的值
  //映射关系：真实值=（滑块值-offset)/scale）
  //平移 (m)：精度 1mm，范围 -5m 到 +5m
  slider_x_ = static_cast<int>(camera_6DoF_[0] * 1000)+ 5000;
  slider_y_ = static_cast<int>(camera_6DoF_[1] * 1000)+ 5000;
  slider_z_ = static_cast<int>(camera_6DoF_[2] * 1000)+ 5000;
  //旋转 (deg)：精度 0.01度，范围 -180 到 +180
  slider_roll_ = static_cast<int>(camera_6DoF_[3]*100)+ 18000;
  slider_pitch_ = static_cast<int>(camera_6DoF_[4]*100)+ 18000;
  slider_yaw_ = static_cast<int>(camera_6DoF_[5]*100)+ 18000;

}

void PerceptionCalibration::CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // TIME_PROFILING_INIT(callbackcloud)
  // ++cloud_count_;
  // std::cout << "pointcloud stamp: " << std::setprecision(15)<< ros::Time(msg->header.stamp).toSec() << std::endl;
  // PointCloudPtr cloud_ = boost::make_shared<PointCloud>();
  PointCloudPtr cloud = boost::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud);

  PointCloud cloud_tmp;
  cloud_tmp.header = cloud->header;
  for (auto p : cloud->points){
    Eigen::Matrix<double, 4, 1> p4d;
    p4d << p.x, p.y, p.z, 1;
    p4d = init_rot.RtLidar2Cam * p4d;
    p.x = p4d[0];
    p.y = p4d[1];
    p.z = p4d[2];
    cloud_tmp.push_back(p);
  }

  PointCloudWithT cloud_with_t(cloud_tmp.makeShared());
  cloud_with_t.is_valid = true;

  cloud_mutex_.lock();
  if (cloud_buffer_.size() < MSG_BUFFER_SIZE) {
    cloud_buffer_.push_back(cloud_with_t);
  } else {
    cloud_buffer_.pop_front();
    cloud_buffer_.push_back(cloud_with_t);
  }
  cloud_mutex_.unlock();
  // TIME_PROFILING_OUT(callbackcloud)
}

void PerceptionCalibration::CallbackImage(const sensor_msgs::ImageConstPtr& msg) {
  // std::cout << "Image stamp: " << std::setprecision(15) << ros::Time(msg->header.stamp).toSec() << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_ptr->image = UndistortImage(cv_ptr->image);
  image_mutex_.lock();
  if (image_buffer_.size() < MSG_BUFFER_SIZE) {
    image_buffer_.push_back(cv_ptr);
  } else {
    image_buffer_.pop_front();
    image_buffer_.push_back(cv_ptr);
  }
  image_mutex_.unlock();
  // cv::Mat correctedImg;
  // cv::Mat cameraMatrix, distCoeffs;
  // eigen2cv(camera_intrinsic_, cameraMatrix);
  // eigen2cv(camera_distortion_, distCoeffs);
  // cv::undistort(cv_ptr->image, correctedImg, cameraMatrix, distCoeffs);
}

void PerceptionCalibration::KeyboardMonitor() {
  std::cout << "Thread: KeyboardMonitor start." << std::endl;
  while (ros::ok()) {
    if (getch() == 's') {
      std::cout << "\nkeyborad is pressed." << std::endl;
      capture_snapshot_ = true;
    }
    usleep(100000);
  }
  // std::cout << "Thread: KeyboardMonitor stop." << std::endl;
}

void PerceptionCalibration::DataSnapshot() {
  std::cout << "Thread: DataSnapshot start." << std::endl;
  // static ros::NodeHandle nh;
  // static ros::Publisher pub =
  //   nh.advertise<std_msgs::Int32>("camera_lidar_calib", 10);
  // TODO max_capture_num
  // const int max_capture_num = 60; 
  int capture_flag = 0;
  // std_msgs::Int32 capture_progress_;
  ros::Rate rate(5);
  while (ros::ok()) {
    static int capture_count = 0;
    if (capture_flag % 8 == 0) {
      cv_bridge::CvImagePtr image;
      PointCloudPtr cloud;
      if (SynchronizeMultiSensor(image, cloud, [this](size_t i) {return this->EnableCapture(i);})) {
        ++capture_count;
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(4) << capture_count;
        std::string image_name = config_.user_image_path + "image" + ss.str() + ".ppm";
        SaveImage(image, image_name);
        std::string cloud_name = config_.user_cloud_path + "Scan_for_MI_" + ss.str() + ".txt";
        SaveCloud(cloud, cloud_name);
      }
    }
    if (capture_count >= 60){
      break;
    }
    ++capture_flag;
    rate.sleep();
  }
}

void PerceptionCalibration::VisualizeCloudImage() {
  cv::VideoWriter output_video;
  if (enable_recorder_) {
    output_video.open(config_.user_data_path + "/Video0/mergecloud.avi",
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, //'D', 'I', 'V', 'X'
                      cv::Size(image_width, image_height));
  }

  //创建OpenCV控制窗口和滑块
  cv::namedWindow("Calibration View", cv::WINDOW_NORMAL);
  cv::resizeWindow("Calibration View", 1280, 720);

  // 滑块范围 0~10000，5000是零点
  cv::createTrackbar("X (mm) [5000=0]", "Calibration View", &slider_x_, 10000);
  cv::createTrackbar("Y (mm) [5000=0]", "Calibration View", &slider_y_, 10000);
  cv::createTrackbar("Z (mm) [5000=0]", "Calibration View", &slider_z_, 10000);
  
  // 滑块范围 0~36000，18000是零点
  cv::createTrackbar("Roll (0.01deg) [18000=0]", "Calibration View", &slider_roll_, 36000);
  cv::createTrackbar("Pitch (0.01deg) [18000=0]", "Calibration View", &slider_pitch_, 36000);
  cv::createTrackbar("Yaw (0.01deg) [18000=0]", "Calibration View", &slider_yaw_, 36000);

  ros::Rate rate(10);
  while (ros::ok()) {


    //每帧循环时，根据滑块值重新计算T_L2Cam和R_L2Cam
    T_L2Cam(0) = (slider_x_ - 5000) / 1000.0;
    T_L2Cam(1) = (slider_y_ - 5000) / 1000.0;
    T_L2Cam(2) = (slider_z_ - 5000) / 1000.0;

    double dynamic_roll  = (slider_roll_ - 18000) / 100.0;
    double dynamic_pitch = (slider_pitch_ - 18000) / 100.0;
    double dynamic_yaw   = (slider_yaw_ - 18000) / 100.0;

    tf::Matrix3x3 R_m3;
    R_m3.setRPY(angles::from_degrees(dynamic_roll),angles::from_degrees(dynamic_pitch),angles::from_degrees(dynamic_yaw));
    R_L2Cam(0,0) = R_m3[0][0];R_L2Cam(0,1) = R_m3[0][1];R_L2Cam(0,2) = R_m3[0][2];
    R_L2Cam(1,0) = R_m3[1][0];R_L2Cam(1,1) = R_m3[1][1];R_L2Cam(1,2) = R_m3[1][2];
    R_L2Cam(2,0) = R_m3[2][0];R_L2Cam(2,1) = R_m3[2][1];R_L2Cam(2,2) = R_m3[2][2];

    cv_bridge::CvImagePtr cvimage;
    PointCloudPtr cloud;


    if(SynchronizeMultiSensor(cvimage, cloud)) {

      cv::Mat merge_image = MergeCloud2Image(cloud, cvimage->image);

      // ================= 新增：在画面上打印真实的数值（带负号和小数点）=================
        char text_buf[256];
        snprintf(text_buf, sizeof(text_buf), "Real Extrinsics: X:%.3f m, Y:%.3f m, Z:%.3f m", 
                 T_L2Cam(0), T_L2Cam(1), T_L2Cam(2));
        cv::putText(merge_image, text_buf, cv::Point(20, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        snprintf(text_buf, sizeof(text_buf), "Real Rotation: R:%.2f deg, P:%.2f deg, Y:%.2f deg", 
                 dynamic_roll, dynamic_pitch, dynamic_yaw);
        cv::putText(merge_image, text_buf, cv::Point(20, 80), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        // ==============================================================================

      cv_bridge::CvImagePtr msgImg = boost::make_shared<cv_bridge::CvImage>();
      msgImg->header.frame_id = "base_link";
      msgImg->encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      msgImg->image = merge_image;

      static ros::Publisher pub = nh_.advertise<sensor_msgs::Image>("pointcloud_image", 10);
      pub.publish(msgImg);

      if (enable_recorder_ && output_video.isOpened()) {
        output_video << merge_image;
      }

      //Opencv窗口中实时渲染融合图像
      cv::imshow("Calibration View",merge_image);

      char key = (char)cv::waitKey(10);
      // 当在 OpenCV 窗口按下 's' 或 'S' 时触发保存
       if (key == 's' || key == 'S') {

            //关闭时间戳循环打印
            g_disable_debug_print = true;
           // 1. 使用 tf 库将 Roll, Pitch, Yaw 转换为四元数 (Quaternion)
           tf::Quaternion q;
           q.setRPY(angles::from_degrees(dynamic_roll),
                    angles::from_degrees(dynamic_pitch),
                    angles::from_degrees(dynamic_yaw));
           
           // 2. 终端打印输出结果
           std::cout << "\n================ Extrinsics/Intrinsics Saved ================" << std::endl;
           std::cout << "6DoF (x,y,z,r,p,y): [" 
                     << T_L2Cam(0) << ", " << T_L2Cam(1) << ", " << T_L2Cam(2) << ", "
                     << dynamic_roll << ", " << dynamic_pitch << ", " << dynamic_yaw << "]" 
                     << std::endl;
           std::cout << "Quaternion (x,y,z,w): [" 
                     << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]" 
                     << std::endl;
           std::cout << "==================================================" << std::endl;

          // 文件读写参数配置
          std::ifstream file_ex_in(config_.extrinsics_path);
          std::ifstream file_in_in(config_.intrinsics_path);
          std::vector<std::string> lines_ex;
          std::vector<std::string> lines_in;
          std::string line_ex;
          std::string line_in;
          bool in_quaternion = false;
          bool has_quaternion = false;
          // 3. 6DoF自动填写到 camera_extrinsics.yaml 文件
          if (file_ex_in.is_open()) {
            while (std::getline(file_ex_in, line_ex)) {
                // 1. 修改 6DoF 字段，并尝试保留原有的 # 注释
                if (line_ex.find("6DoF:") != std::string::npos) {
                    std::stringstream ss;
                    // 保留原有的行首缩进
                    size_t indent = line_ex.find_first_not_of(" \t");
                    if (indent != std::string::npos) ss << line_ex.substr(0, indent);
                    
                    ss << "6DoF: [" << T_L2Cam(0) << ", " << T_L2Cam(1) << ", " << T_L2Cam(2) << ", "
                       << dynamic_roll << ", " << dynamic_pitch << ", " << dynamic_yaw << "]";
                    
                    // 寻找并保留该行的注释
                    size_t comment_pos = line_ex.find("#");
                    if (comment_pos != std::string::npos) {
                        ss << "  " << line_ex.substr(comment_pos);
                    }
                    lines_ex.push_back(ss.str());
                } 
                // 2. 其他行原样保留
                else {
                    lines_ex.push_back(line_ex);
                }
            }
            file_ex_in.close();
            // 3. 覆盖写回原文件
            std::ofstream file_ex_out(config_.extrinsics_path);
            for (const auto& l : lines_ex) {
                file_ex_out << l << "\n";
            }
            file_ex_out.close();
               std::cout << "-> Successfully updated 6DoF_YAML file at:\n   " << config_.extrinsics_path << std::endl;
           } else {
               std::cerr << "-> ERROR: Failed to open file for writing at:\n   " << config_.extrinsics_path << std::endl;
           }
           // 4. Quaternion自动填写到 camera_intrinsics.yaml 文件
           if (file_in_in.is_open()) {
            while (std::getline(file_in_in, line_in)) {
                // 1. 检测是否已经有 Quaternion 字段
              if (line_in.find("Quaternion:") != std::string::npos) {
                    has_quaternion = true;
                    in_quaternion = true;
                    lines_in.push_back(line_in);
                } 
                // 2. 修改 Quaternion 下的 data 字段
                else if (in_quaternion && line_in.find("data:") != std::string::npos) {
                    std::stringstream ss;
                    size_t indent = line_in.find_first_not_of(" \t");
                    if (indent != std::string::npos) ss << line_in.substr(0, indent);
                    
                    ss << "data: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]";
                    lines_in.push_back(ss.str());
                    in_quaternion = false; // 修改完 data 后重置标记
                } 
                // 3. 其他行原样保留
                else {
                    lines_in.push_back(line_in);
                }
            }
            file_in_in.close();

            // 4. 如果原文件中根本没有 Quaternion 字段，则追加到最后
            if (!has_quaternion) {
                lines_in.push_back("  Quaternion:");
                lines_in.push_back("    rows: 1");
                lines_in.push_back("    cols: 4");
                std::stringstream ss;
                ss << "    data: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]";
                lines_in.push_back(ss.str());
            }

            // 5. 覆盖写回原文件
            std::ofstream file_in_out(config_.intrinsics_path);
            for (const auto& l : lines_in) {
                file_in_out << l << "\n";
            }
            file_in_out.close();
               std::cout << "-> Successfully updated Quaternion_YAML file at:\n   " << config_.intrinsics_path << std::endl;
           } else {
               std::cerr << "-> ERROR: Failed to open file for writing at:\n   " << config_.intrinsics_path << std::endl;
           }
        }
    }
    rate.sleep();
  }
}

bool PerceptionCalibration::SynchronizeMultiSensor(cv_bridge::CvImagePtr& cvimage,
    PointCloudPtr& cloud, std::function<bool(size_t)> cloud_is_capturable) {
  if (cloud == nullptr) {
    cloud = boost::make_shared<PointCloud>();
  }
  if (cvimage == nullptr) {
    cvimage = boost::make_shared<cv_bridge::CvImage>();
  }
  std::lock_guard<std::mutex> image_lock_guard(image_mutex_);
  std::lock_guard<std::mutex> cloud_lock_guard(cloud_mutex_);
  if (image_buffer_.size() != MSG_BUFFER_SIZE ||
      cloud_buffer_.size() != MSG_BUFFER_SIZE) {
    return false;
  }

  for (size_t i = MSG_BUFFER_SIZE - 1; i > 0; --i) {
    double cloud_time = cloud_buffer_[i].GetTime();
    for (size_t j = MSG_BUFFER_SIZE - 1; j > 0; --j) {
      double image_time = ros::Time(image_buffer_[j]->header.stamp).toSec();
      /// add for timestamp not correct
      cloud_time = int(cloud_time) % 3600 + cloud_time - int(cloud_time);
      image_time = int(image_time) % 3600 + image_time - int(image_time);
      // if (fabs(cloud_time - image_time) < 0.009 && cloud_buffer_[i].is_valid) {
      // if (fabs(cloud_time - image_time) < 0.5 && cloud_buffer_[i].is_valid) {
     // if (fabs(cloud_time - image_time) < 0.06 && cloud_buffer_[i].is_valid) {
        

        //受标志位控制输出
        if(!g_disable_debug_print) {
          std::cout << std::setprecision(15) << "cloud_time" << cloud_time << std::endl;
          std::cout << "image_time" << image_time << std::endl;
          std::cout << std::endl;
        }

        if (!cloud_is_capturable(i)) {
          return false;
        }
        // *cvimage = *image_buffer_[j-1];
        *cvimage = *image_buffer_[j];
        *cloud = *cloud_buffer_[i].cloud;
        return true;
       //}
    }
  }

  // for (size_t i = MSG_BUFFER_SIZE - 1; i > 0; --i) {
  //   double image_time = ros::Time(image_buffer_[i]->header.stamp).toSec();
  //   for (size_t j = MSG_BUFFER_SIZE - 1; j > 0; --j) {
  //     double pre_cloud_time = cloud_buffer_[j - 1].GetTime();
  //     double cloud_time = cloud_buffer_[j].GetTime();
  //     if (image_time <= cloud_time && image_time >= pre_cloud_time) {
  //       if (cloud_buffer_[j].is_valid && cloud_buffer_[j - 1].is_valid) {
  //         if (!cloud_is_capturable(j)) {
  //           return false;
  //         }
  //         *cvimage = *image_buffer_[i];
  //         // no interpolate
  //         if (fabs(image_time - pre_cloud_time) > fabs(cloud_time - image_time)) {
  //           *cloud = *cloud_buffer_[j].cloud;
  //         } else {
  //           *cloud = *cloud_buffer_[j - 1].cloud;
  //         }
  //         return true;
  //       }
  //     }
  //   }
  // }
  return false;
}

bool PerceptionCalibration::EnableCapture(size_t index) {
  // index/=2;
  if (static_model_){
    // ros::param::get("enable_capture", enable_capture_);
    // if (enable_capture_){
    //   enable_capture_ = false;
    //   ros::param::set("enable_capture", false);
    //   return true;
    // }
    // return false;
    return true;
  }
  else{
    static ros::Time pre_capture = ros::Time::now();
    static std::pair<float, float> pre_position;
    std::pair<float, float> position =
      std::make_pair(cloud_buffer_[index].t(0, 3), cloud_buffer_[index].t(1, 3));
    bool capture_is_valid =
      CheckCaptureCloud(index) &&
      (ros::Time::now() - pre_capture).toSec() > 5.0 &&
      std::hypot(position.first - pre_position.first,
                 position.second - pre_position.second) > 1.5;
    if (capture_is_valid) {
      pre_capture = ros::Time::now();
      pre_position = position;
    }
    return capture_is_valid;
  }
}

bool PerceptionCalibration::CheckCaptureCloud(size_t index) {
  const int check_size = 8;
  if (index <= check_size || index >= cloud_buffer_.size()) {
    return false;
  }
  bool check_t = true;
  for (size_t i = index - check_size; i <= index; ++i) {
    check_t = check_t && cloud_buffer_[i].is_valid;
  }
  if (!check_t) {
    return false;
  }
  std::vector<float> headings;
  for (size_t i = index - check_size; i <= index; ++i) {
    const auto& cloud = cloud_buffer_[i];
    headings.push_back(cloud.rotation[2]);
  }
  std::sort(headings.begin(), headings.end());
  float max_heading_diff = fabs(headings.front() - headings.back());
  float heading_dev;
  GetMeanStdDev(headings, (float*)nullptr, &heading_dev);

  return (max_heading_diff < 0.5 && heading_dev < 0.2);
}

void PerceptionCalibration::SaveImage(const cv_bridge::CvImagePtr& cvimage,
                                      const std::string& filename) {
  std::cout << "image saved in " << filename << std::endl;
  std::cout << "image time " << std::fixed << std::setprecision(6) <<
            ros::Time(cvimage->header.stamp).toSec() << std::flush << std::endl;

  cv::imwrite(filename, cvimage->image);
}

void PerceptionCalibration::SaveCloud(const PointCloudPtr& cloud,
                                      const std::string& filename) {
  std::cout << "cloud saved in " << filename << std::endl;
  std::cout << "cloud time " << std::fixed << std::setprecision(6)
            << cloud->header.stamp * 1e-6 <<  std::flush << std::endl;

  PointCloud no_nan_cloud;
  no_nan_cloud.reserve(cloud->size());
  for (const auto& p : cloud->points) {
    if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
      avos::perception_calib::pcl_util::Point pt = p;
      no_nan_cloud.push_back(pt);
    }
  }
  std::ofstream ofs;
  ofs.open(filename);
  ofs << no_nan_cloud.size() << std::endl;
  for (const auto& p : no_nan_cloud.points) {
    ofs << std::fixed << std::setprecision(6)
        << p.x << " "
        << p.y << " "
        << p.z << " "
        << static_cast<int>(p.intensity) << " "
        << static_cast<int>(p.ring) << std::endl;
  }
  ofs.close();
}

PointCloudWithT PerceptionCalibration::Interpolate(PointCloudWithT start,
    PointCloudWithT end, const double time) const {
  if (fabs(time - start.GetTime()) > fabs(end.GetTime() - time)) {
    std::swap(start, end);
  }
  const double duration = end.GetTime() - start.GetTime();
  const double factor = (time - start.GetTime()) / duration;
  const Eigen::Matrix<float, 3, 1> delta_trans =
    (end.t.block<3, 1>(0, 3) - start.t.block<3, 1>(0, 3)) * factor;
  const Eigen::Matrix<float, 3, 1> translation =
    start.t.block<3, 1>(0, 3) + delta_trans;;
  const Eigen::Matrix<float, 3, 3> rotation(
    Eigen::Quaternionf(start.t.block<3, 3>(0, 0))
    .slerp(factor, Eigen::Quaternionf(end.t.block<3, 3>(0, 0))));
  const Eigen::Matrix<float, 3, 3> delta_rot =
    rotation * start.t.block<3, 3>(0, 0).inverse();
  Eigen::Matrix<float, 4, 4> delta_t;
  delta_t.block<3, 3>(0, 0) = delta_rot;
  delta_t.block<3, 1>(0, 3) = delta_trans;
  delta_t.block<1, 4>(3, 0) << 0, 0, 0, 1;
  PointCloudPtr cloud = boost::make_shared<PointCloud>();
  cloud->resize(start.cloud->size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    auto& p = cloud->points[i];
    p = start.cloud->points[i];
    Eigen::Matrix<float, 4, 1> v4;
    v4 << p.x, p.y, p.z, 1.0;
    v4 = delta_t.inverse() * v4;
    p.x = v4[0];
    p.y = v4[1];
    p.z = v4[2];
  }
  cloud->header = start.cloud->header;
  cloud->height = 1;
  cloud->width = cloud->size();
  Eigen::Matrix4f t;
  t.block<3, 3>(0, 0) = rotation;
  t.block<3, 1>(0, 3) = translation;
  t.row(3) << 0, 0, 0, 1;
  return PointCloudWithT(cloud, t);
}

cv::Mat PerceptionCalibration::MergeCloud2Image(const PointCloudPtr& cloud,
    const cv::Mat& image) {
  cv::Mat merge_image = image.clone();
  Eigen::Matrix<double, 3, 1> point;
  for (const auto& p : cloud->points) {
    point << p.x, p.y, p.z;
    if (point[1] > 1.75){
      continue;
    }
    // point = R_L2Cam * point + T_L2Cam;
    point = R_L2Cam * (point + T_L2Cam);
    if (point[2] > 0) {
      point = camera_intrinsic_ * point;
      cv::Point uv(point[0] / point[2], point[1] / point[2]);
      if (uv.x >= 0 && uv.x < merge_image.cols &&
          uv.y >= 0 && uv.y < merge_image.rows) {
        // image.at<cv::Vec3b>(uv) = ConvertIntensity2Scalar<cv::Vec3b>(p.intensity);
        cv::circle(merge_image, uv, 1.5, ConvertDistance2Scalar<cv::Scalar>(point[2]), -1, 8);
      }
    }
  }
  return merge_image;
}


void PerceptionCalibration::ConvertCloud2XYZI(
  const PointCloudPtr& input_cloud,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& matching_cloud,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& original_cloud) {
  if (matching_cloud == nullptr) {
    matching_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  } else {
    matching_cloud->clear();
  }
  matching_cloud->reserve(input_cloud->size());
  matching_cloud->header = input_cloud->header;
  if (original_cloud == nullptr) {
    original_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  } else {
    original_cloud->clear();
  }
  original_cloud->reserve(input_cloud->size());
  original_cloud->header = input_cloud->header;
  for (const auto& p : input_cloud->points) {
    if (!std::isnan(p.x)) {
      pcl::PointXYZI pi;
      pi.x = p.x;
      pi.y = p.y;
      pi.z = p.z;
      pi.intensity = static_cast<float>(p.intensity);
      matching_cloud->push_back(pi);
      if (pi.x > 0) {
        original_cloud->push_back(pi);
      }
    }
  }
  // std::vector<int> indices; 
  // pcl::removeNaNFromPointCloud(*matching_cloud, *matching_cloud, indices);
  // std::vector<int> indices_; 
  // pcl::removeNaNFromPointCloud(*original_cloud, *original_cloud, indices_);
}

void PerceptionCalibration::ParseCloudFromeScan() {
  std::string scans_path_base =
    config_.user_data_path + "/scans/Scan_for_MI_";
  int scans_num = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds;
  while (++scans_num <= 200) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(4) << scans_num;
    std::string scan_path = scans_path_base + ss.str() + ".txt";
    Parser parser(scan_path);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (!parser.ParseFromCSV(cloud, ' ', true)) {
      break;
    }
    clouds.push_back(cloud);
  }
  static ros::Publisher pub =
    nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 10);
  while (ros::ok()) {
    for (const auto& cloud : clouds) {
      pub.publish(cloud);
      ros::Duration(0.1).sleep();
    }
  }
}

void PerceptionCalibration::fixScanPrecision() {
  std::string scans_path_base =
    config_.user_data_path + "/scans/Scan_for_MI_";
  std::string fixed_scans_path_base =
    config_.user_data_path + "/scans0/Scan_for_MI_";
  std::ifstream iscans;
  std::ofstream oscans;
  int scans_num = 0;
  while (++scans_num <= 200) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(4) << scans_num;
    std::string scan_path = scans_path_base + ss.str() + ".txt";
    std::string fixed_scan_path = fixed_scans_path_base + ss.str() + ".txt";
    iscans.open(scan_path);
    if (!iscans.is_open()) {
      return;
    }
    oscans.open(fixed_scan_path);
    std::string line;
    std::getline(iscans, line);
    oscans << line << std::endl;
    while (std::getline(iscans, line)) {
      std::vector<float> line_data;
      std::stringstream ss(line);
      std::string subline;
      while (std::getline(ss, subline, ' ')) {
        float t = 0;
        std::stringstream(subline) >> t;
        line_data.push_back(t);
      }
      float fixed_line_data[3];
      // xc = -y; yc = -z; zc = x
      // fixed_line_data[0] = -line_data[1];
      // fixed_line_data[1] = -line_data[2];
      // fixed_line_data[2] = line_data[0];
      // xc = x; yc = -z; zc = y
      fixed_line_data[0] = line_data[0];
      fixed_line_data[1] = line_data[1];
      fixed_line_data[2] = line_data[2];
      oscans << std::fixed << std::setprecision(6)
             << fixed_line_data[0] << " "
             << fixed_line_data[1] << " "
             << fixed_line_data[2] << " "
             << static_cast<int>(line_data[3]) << std::endl;
    }
    iscans.close();
    oscans.close();
  }
}

cv::Mat PerceptionCalibration::UndistortImage(const cv::Mat& image) const {
  cv::Mat undistort_image;
  cv::Mat cameraMatrix, distCoeffs;
  eigen2cv(camera_intrinsic_, cameraMatrix);
  eigen2cv(camera_distortion_, distCoeffs);
  cv::undistort(image, undistort_image, cameraMatrix, distCoeffs);
  

  // cv::Size imageSize = image.size();
  // cv::Mat map1, map2;
  // cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), 
  //   cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize, 0), imageSize, CV_16SC2, map1, map2);
  // // initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_16SC2, map1, map2);
  // cv::Mat image1;
  // cv::remap(image, undistort_image, map1, map2, CV_INTER_LINEAR);
  // // //-----------------------end ------------------------------
  return undistort_image;
}

template <typename Matrix>
void PerceptionCalibration::GetRPY(const Matrix& t,
                                   double* r, double* p, double* y) const {
  tf::Matrix3x3 rot(t(0, 0), t(0, 1), t(0, 2),
                    t(1, 0), t(1, 1), t(1, 2),
                    t(2, 0), t(2, 1), t(2, 2));
  rot.getRPY(*r, *p, *y);
  *r = (*r) * 180 / M_PI;
  *p = (*p) * 180 / M_PI;
  *y = (*y) * 180 / M_PI;
}

template <typename T>
void PerceptionCalibration::GetMeanStdDev(const std::vector<T>& data,
    T* input_mean, T* input_dev) const {
  T sum = std::accumulate(std::begin(data), std::end(data), 0.0);
  T mean =  sum / data.size();
  if (input_mean) {
    *input_mean = mean;
  }
  T accum  = 0.0;
  std::for_each (std::begin(data), std::end(data), [&](const T d) {
    accum  += (d - mean) * (d - mean);
  });

  *input_dev = sqrt(accum / (data.size() - 1));
}

void PerceptionCalibration::initMask() {
  cv::Mat mask(image_height, image_width, CV_8UC1, cv::Scalar::all(0));
  cv::rectangle(mask,
                cv::Point(30, 30),
                cv::Point(image_width - 30, image_height - 30),
                cv::Scalar::all(255),
                cv::FILLED);
  cv::imwrite(config_.user_data_path + "/Mask/Cam0.png", mask);
}

template <typename Scalar>
auto PerceptionCalibration::ConvertDistance2Scalar(const float dis) const -> Scalar {
  int i = dis * 5;
  if (i < 0) i = 0;
  if (i > 255) i = 255;
  return GetRainbowColor<Scalar>(i);
}

template <typename Scalar>
auto PerceptionCalibration::ConvertIntensity2Scalar(const int i) const -> Scalar {
  return GetRainbowColor<Scalar>(i);
}

template <typename Scalar>
auto PerceptionCalibration::GetRainbowColor(const int i) const -> Scalar {
  static std::vector<Scalar> i_color;
  static bool init_flag = false;
  if (!init_flag) {
    for (int i = 0; i < 256; i++) {
      Scalar p;
      p[0] = 0;
      p[1] = i;
      p[2] = 255;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      Scalar p;
      p[0] = 0;
      p[1] = 255;
      p[2] = 255 - i;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      Scalar p;
      p[0] = i;
      p[1] = 255;
      p[2] = 0;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      Scalar p;
      p[0] = 255;
      p[1] = 255 - i;
      p[2] = 0;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      Scalar p;
      p[0] = 255;
      p[1] = 0;
      p[2] = i;
      i_color.push_back(p);
    }
    init_flag = true;
  }
  return i_color[i * 5];
}

void PerceptionCalibration::initEgoCarMarker() {
  visualization_msgs::Marker ego_car_marker;
  ego_car_marker.header.frame_id = "/base_link";
  ego_car_marker.header.stamp = ros::Time::now();
  ego_car_marker.ns = "lane_mask";
  ego_car_marker.id = 1;
  ego_car_marker.type = visualization_msgs::Marker::CUBE;
  ego_car_marker.lifetime = ros::Duration();
  ego_car_marker.action = visualization_msgs::Marker::ADD;
  ego_car_marker.scale.x = 4;
  ego_car_marker.scale.y = 2;
  ego_car_marker.scale.z = 1;
  ego_car_marker.pose.position.x = 2;
  ego_car_marker.pose.position.y = 0;
  ego_car_marker.pose.position.z = 0;
  ego_car_marker.color.r = 0.9;
  ego_car_marker.color.b = 0.0;
  ego_car_marker.color.g = 0.9;
  ego_car_marker.color.a = 1.0;

  static ros::Publisher pub_marker_ =
    nh_.advertise<visualization_msgs::Marker> ("ego_car", 10);
  pub_marker_.publish(ego_car_marker);
}

void PerceptionCalibration::CoordinateTransform_inv(const PointCloudPtr& inputPcPointer, 
    PointCloudPtr& outputPcPointer, const double Roll, const double Pitch, const double Yaw, const double x_offset, const double y_offset, const double z_offset) {

  // int inputPcPointer_size = inputPcPointer->size();
  outputPcPointer->header = inputPcPointer->header;
  for (auto &p : inputPcPointer->points) {
    // Point p = inputPcPointer->points[i];
    if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
      double x = p.x + x_offset;
      double y = p.y + y_offset;
      double z = p.z + z_offset;
      p.x = cos(Yaw * M_PI /180) * cos(Pitch * M_PI/180) * x + sin(Yaw * M_PI /180) * cos(Pitch * M_PI/180) * y - sin(Pitch * M_PI/180) * z;
      
      p.y = (-sin(Yaw * M_PI /180) * cos(Roll * M_PI/180) + cos(Yaw * M_PI/180) * sin(Pitch * M_PI/180) * sin(Roll * M_PI/180)) * x +
          (cos(Yaw * M_PI/180) * cos(Roll * M_PI/180) + sin(Yaw * M_PI/180) * sin(Pitch * M_PI/180) * sin(Roll * M_PI/180)) * y +
          cos(Pitch * M_PI /180) * sin(Roll * M_PI/180) * z;
          
      p.z = (sin(Yaw * M_PI/180) * sin(Roll * M_PI/180) + cos(Yaw * M_PI/180) * sin(Pitch * M_PI/180) * cos(Roll * M_PI/180)) * x + 
          (-cos(Yaw * M_PI/180) * sin(Roll * M_PI/180) + sin(Yaw * M_PI/180) * sin(Pitch * M_PI/180) * cos(Roll * M_PI/180)) * y +
            cos(Pitch * M_PI/180) * cos(Roll * M_PI/180) * z; 
    }

    outputPcPointer->push_back(p);
  }
}


}  // namespace calibration
