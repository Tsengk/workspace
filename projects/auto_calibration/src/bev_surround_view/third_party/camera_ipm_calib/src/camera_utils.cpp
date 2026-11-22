#include "include/camera_utils.h"

namespace avos
{
  namespace perception
  {

    CameraUtils::CameraUtils() {}

    CameraUtils::~CameraUtils() {}

    bool CameraUtils::GetT(std::vector<float> &params_t)
    {
      params_t.resize(3);
      params_t[0] = extr_t_wheel_.col(0)(0);
      params_t[1] = extr_t_wheel_.col(0)(1);
      params_t[2] = extr_t_wheel_.col(0)(2);
      return true;
    }

    bool CameraUtils::GetRoi(std::vector<int> &roi)
    {
      roi = bev_roi_;
      return true;
    }

    bool CameraUtils::GetR(std::vector<float> &params_r)
    {
      params_r.clear();
      params_r.push_back(extr_pitch_original_ * 180.0 / M_PI);
      params_r.push_back(extr_yaw_original_ * 180.0 / M_PI);
      params_r.push_back(extr_roll_original_ * 180.0 / M_PI);
      return true;
    }

    bool CameraUtils::SetT(const std::vector<float> &params_t)
    {
      if (params_t.size() != 3)
      {
        return false;
      }
      extr_t_.col(0) << params_t[0], params_t[1], params_t[2];
      extr_t_original_ = extr_t_;
      extr_t_wheel_.col(0) << params_t[0], params_t[1], params_t[2];
      return true;
    }

    bool CameraUtils::SetZOffset(const float z)
    {
      z_offset_ = z;
      return true;
    }

    bool CameraUtils::GetZOffset(float &z)
    {
      z = z_offset_;
      return true;
    }

    bool CameraUtils::SetR(const std::vector<float> &R_euler)
    {
      if (R_euler.size() != 3)
      {
        return false;
      }
      Eigen::VectorXf params_euler(3);
      for (size_t i = 0; i < 3; i++)
      {
        params_euler[i] = R_euler[i] / 180.0 * M_PI;
      }
      Eigen::Quaternionf rotation =
          (Eigen::AngleAxisf(params_euler.z(), // pitch
                             Eigen::Vector3f::UnitZ()) *
           Eigen::AngleAxisf(params_euler.x(),
                             Eigen::Vector3f::UnitX()) *
           Eigen::AngleAxisf(params_euler.y(),
                             Eigen::Vector3f::UnitY()));
      extr_R_ = rotation.matrix();
      extr_R_original_ = extr_R_;
      extr_pitch_original_ = params_euler.x();
      extr_yaw_original_ = params_euler.y();
      extr_roll_original_ = params_euler.z();
      return true;
    }

    bool CameraUtils::LoadCameraParams(const int &cam_idx, const YAML::Node &node)
    {
      cam_idx_ = cam_idx;
      bool success1 = LoadCameraIntrinsics(node);
      bool success2 = LoadCameraExtrinsics(node);
      bool success3 = LoadCameraRoi(node);
      extr_R_comp_ = Eigen::Matrix3f::Identity();
      extr_t_comp_ = Eigen::Vector3f::Zero();
      if (node["vcs_to_ground_height"])
      {
        z_offset_ = node["vcs_to_ground_height"].as<float>();
      }
      if (success1 && success2)
      {
        std::cout << "load " << CamIdx2Str[cam_idx_] << " id:" << cam_idx << " camera params ok." << std::endl;
        return true;
      }
      return false;
    }

    bool CameraUtils::LoadCameraRoi(const YAML::Node &node)
    {
      try
      {
        const YAML::Node &roi_node = node["bev_roi"];
        if (!roi_node || roi_node.size() != 4)
        {
          std::cout << "[CameraUtils] bev_roi is missing or size is not 4." << std::endl;
          return false;
        }

        bev_roi_.resize(4);
        for (int i = 0; i < 4; ++i)
        {
          bev_roi_[i] = roi_node[i].as<int>();
        }
      }
      catch (const YAML::Exception &e)
      {
        std::cout << "[CameraUtils] YAML parse error: " << e.what() << std::endl;
        return false;
      }

      return true;
    }

    bool CameraUtils::LoadCameraIntrinsics(const YAML::Node &node)
    {
      try
      {
        // // 不知道为什么，x86非得这样搞一下才能跑通
        // YAML::Node node;
        // node = _node;
        // node["key"] = "value";

        camera_width_ = node["Size"][0].as<int>();  // 表示标定时图像的大小
        camera_height_ = node["Size"][1].as<int>(); // 表示标定时图像的大小
        // camera_width_ = original_width_;
        // camera_height_ = original_height_;

        // scaled_width_ = node["scaled_Size"][0].as<int>(); // 表示去完畸变后图像大小，与原图尺寸比例保持一致，由3d任务离线使用
        // scaled_height_ = node["scaled_Size"][1].as<int>(); // 表示去完畸变后图像大小，与原图尺寸比例保持一致，由3d任务离线使用
        new_width_ = node["new_Size"][0].as<int>();  // 表示去完畸变后图像大小，经过GDC去畸变，resize之后输出的图像大小
        new_height_ = node["new_Size"][1].as<int>(); // 表示去完畸变后图像大小，经过GDC去畸变，resize之后输出的图像大小

        Eigen::VectorXf params_k(9);
        for (size_t i = 0; i < 9; ++i)
        {
          params_k(i) = node["K"][i].as<float>();
        }

        // K&D 值均为在Size: original_width_ & original_height_ 尺寸下标定的参数
        intr_K_.row(0) << params_k(0), params_k(1), params_k(2);
        intr_K_.row(1) << params_k(3), params_k(4), params_k(5);
        intr_K_.row(2) << params_k(6), params_k(7), params_k(8);

        // 下面代码只在图像经过去畸变后的工程中是有效的，在带畸变的功能中所有全局变量无效
        Eigen::VectorXf params_new_k(9);
        for (size_t i = 0; i < 9; ++i)
        {
          params_new_k(i) = node["new_K"][i].as<float>();
        }
        new_intr_K_.row(0) << params_new_k(0), params_new_k(1), params_new_k(2);
        new_intr_K_.row(1) << params_new_k(3), params_new_k(4), params_new_k(5);
        new_intr_K_.row(2) << params_new_k(6), params_new_k(7), params_new_k(8);

        // if(node["crop_rect"]) {
        //   crop_rect = cv::Rect2f(node["crop_rect"][0].as<float>(), node["crop_rect"][1].as<float>(),
        //                          node["crop_rect"][2].as<float>(), node["crop_rect"][3].as<float>());
        //   crop_sx = (float)crop_rect.width / scaled_width_;
        //   crop_sy = (float)crop_rect.height / scaled_height_;
        //   crop_tx = crop_rect.x;
        //   crop_ty = crop_rect.y;
        // }

        // 需要区分pinhole和fisheye
        if (node["Epsilon"])
        {
          intr_D_.camera_model = MODEL_FISHEYE;
          intr_D_.epsilon = node["Epsilon"].as<float>();
          Eigen::VectorXf params_d(4);
          for (size_t i = 0; i < 4; ++i)
          {
            intr_D_.params_d[i] = node["D"][i].as<float>();
          }
        }
        else
        {
          intr_D_.camera_model = MODEL_PINHOLE;
          Eigen::VectorXf params_d(5);
          for (size_t i = 0; i < 5; ++i)
          {
            intr_D_.params_d[i] = node["D"][i].as<float>();
          }
        }
      }
      catch (YAML::Exception &e)
      {
        std::cout << "load camera intrisic with error,"
                  << " YAML exception: " << e.what() << std::endl;
        return false;
      }
      return true;
    }

    // bool CameraUtils::LoadCameraExtrinsics(const YAML::Node &node)
    // {
    //   Eigen::VectorXf params_t(3), params_euler(3);
    //   try
    //   {
    //     for (int i = 0; i < 3; ++i)
    //     {
    //       params_t(i) = node["t"][i].as<float>();
    //       params_euler(i) = node["R_euler"][i].as<float>();
    //     }
    //     params_euler = params_euler / 180.0 * M_PI;
    //   }
    //   catch (const YAML::Exception &e)
    //   {
    //     std::cout << "load camera extrinsics with error,"
    //               << " YAML exception: " << e.what() << std::endl;
    //     return false;
    //   }

    //   // fill rotation and translation
    //   // 初始旋转矩阵，需要根据相机进行调整？
    //   float roll = 90 * M_PI / 180;   // 3-x-roll
    //   float pitch = -90 * M_PI / 180; // 2-y-pitch
    //   float yaw = 0 * M_PI / 180;     // 1-z-yaw
    //   extr_R_init_ = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    //   Eigen::Quaternionf rotation =
    //       (Eigen::AngleAxisf(params_euler.z(), // pitch
    //                          Eigen::Vector3f::UnitZ()) *
    //        Eigen::AngleAxisf(params_euler.y(),
    //                          Eigen::Vector3f::UnitY()) *
    //        Eigen::AngleAxisf(params_euler.x(),
    //                          Eigen::Vector3f::UnitX()));
    //   extr_R_ = rotation.matrix();
    //   extr_R_original_ = extr_R_;
    //   extr_pitch_original_ = params_euler.x();
    //   extr_yaw_original_ = params_euler.y();
    //   extr_roll_original_ = params_euler.z();

    //   extr_t_.col(0) << params_t(0), params_t(1) - z_offset_, params_t(2);
    //   extr_t_original_ = extr_t_;
    //   extr_t_wheel_.col(0) << params_t(0), params_t(1), params_t(2);
    //   return true;
    // }

    // // # The distortion parameters, size depending on the distortion model.
    // // # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    // void CameraUtils::ProjectInImagePinhole(cv::Point3f &pt)
    // {
    //   Eigen::Matrix<double, 2, 1> pt2d(pt.x, pt.y);
    //   double r_sq = pt2d[0] * pt2d[0] + pt2d[1] * pt2d[1];

    //   auto params_d = intr_D_.params_d;
    //   Eigen::Matrix<double, 2, 1> pt2d_radial =
    //       pt2d * (1 + params_d[0] * r_sq + params_d[1] * r_sq * r_sq +
    //               params_d[4] * r_sq * r_sq * r_sq);
    //   Eigen::Matrix<double, 2, 1> dpt2d;
    //   dpt2d[0] = 2 * params_d[2] * pt2d[0] * pt2d[1] +
    //              params_d[3] * (r_sq + 2 * pt2d[0] * pt2d[0]);
    //   dpt2d[1] = params_d[2] * (r_sq + 2 * pt2d[1] * pt2d[1]) +
    //              2 * params_d[3] * pt2d[0] * pt2d[1];

    //   Eigen::Matrix<double, 2, 1> pt2d_distort;
    //   pt2d_distort[0] = pt2d_radial[0] + dpt2d[0];
    //   pt2d_distort[1] = pt2d_radial[1] + dpt2d[1];

    //   // to image pixel
    //   // transform to image coordinates
    //   const double fx = intr_K_(0, 0);
    //   const double fy = intr_K_(1, 1);
    //   const double cx = intr_K_(0, 2);
    //   const double cy = intr_K_(1, 2);
    //   pt.x = pt2d_distort[0] * fx + cx;
    //   pt.y = pt2d_distort[1] * fy + cy;
    // }

    bool CameraUtils::LoadCameraExtrinsics(const YAML::Node &node)
    {
      std::vector<float> params_t(3), params_euler(3);
      try
      {
        for (int i = 0; i < 3; ++i)
        {
          params_t[i] = node["t"][i].as<float>();
          params_euler[i] = node["R_euler"][i].as<float>();
        }
        // params_euler = params_euler / 180.0 * M_PI;
      }
      catch (const YAML::Exception &e)
      {
        std::cout << "load camera extrinsics with error,"
                  << " YAML exception: " << e.what() << std::endl;
        return false;
      }

      // fill rotation and translation
      // 初始旋转矩阵，需要根据相机进行调整？
      float roll = 90 * M_PI / 180;   // 3-x-roll
      float pitch = -90 * M_PI / 180; // 2-y-pitch
      float yaw = 0 * M_PI / 180;     // 1-z-yaw
      extr_R_init_ = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

      SetR(params_euler);

      SetT(params_t);
      return true;
    }

    // # The distortion parameters, size depending on the distortion model.
    // # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    void CameraUtils::ProjectInImagePinhole(cv::Point3f &pt)
    {
      Eigen::Matrix<double, 2, 1> pt2d(pt.x, pt.y);
      double r_sq = pt2d[0] * pt2d[0] + pt2d[1] * pt2d[1];

      auto params_d = intr_D_.params_d;
      Eigen::Matrix<double, 2, 1> pt2d_radial =
          pt2d * (1 + params_d[0] * r_sq + params_d[1] * r_sq * r_sq +
                  params_d[4] * r_sq * r_sq * r_sq);
      Eigen::Matrix<double, 2, 1> dpt2d;
      dpt2d[0] = 2 * params_d[2] * pt2d[0] * pt2d[1] +
                 params_d[3] * (r_sq + 2 * pt2d[0] * pt2d[0]);
      dpt2d[1] = params_d[2] * (r_sq + 2 * pt2d[1] * pt2d[1]) +
                 2 * params_d[3] * pt2d[0] * pt2d[1];

      Eigen::Matrix<double, 2, 1> pt2d_distort;
      pt2d_distort[0] = pt2d_radial[0] + dpt2d[0];
      pt2d_distort[1] = pt2d_radial[1] + dpt2d[1];

      // to image pixel
      // transform to image coordinates
      const double fx = intr_K_(0, 0);
      const double fy = intr_K_(1, 1);
      const double cx = intr_K_(0, 2);
      const double cy = intr_K_(1, 2);
      pt.x = pt2d_distort[0] * fx + cx;
      pt.y = pt2d_distort[1] * fy + cy;
    }

    void CameraUtils::ProjectInImageFisheye(cv::Point3f &pt)
    {
      // float x[3] = {pt.x, pt.y, pt.z};
      // const float norm = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
      // Eigen::Vector3f norm_dir(pt.x, pt.y, pt.z);
      Eigen::Vector3f norm_dir(pt.x, pt.y, 1.0);
      norm_dir.normalize();

      auto params_d = intr_D_.params_d;
      auto epsilon = intr_D_.epsilon;
      // project to image plane
      float xu = norm_dir(0) / (norm_dir(2) + epsilon);
      float yu = norm_dir(1) / (norm_dir(2) + epsilon);
      // add distortion
      float r2 = xu * xu + yu * yu;
      float r4 = r2 * r2;

      Eigen::Vector2f pt2d_img;
      pt2d_img[0] = (1 + params_d[0] * r2 + params_d[1] * r4) * xu + 2 * params_d[2] * xu * yu + params_d[3] * (r2 + 2 * xu * xu);
      pt2d_img[1] = (1 + params_d[0] * r2 + params_d[1] * r4) * yu + params_d[2] * (r2 + 2 * yu * yu) + 2 * params_d[3] * xu * yu;

      // to image pixel
      // transform to image coordinates
      const float fx = intr_K_(0, 0);
      const float fy = intr_K_(1, 1);
      const float cx = intr_K_(0, 2);
      const float cy = intr_K_(1, 2);
      const float s = intr_K_(0, 1);
      pt.x = fx * pt2d_img[0] + s * pt2d_img[1] + cx;
      pt.y = fy * pt2d_img[1] + cy;
    }

    bool CameraUtils::ConvertPointVCS2IMG(const cv::Point3f &p3fVcs, cv::Point3f &p3fImg)
    {
      Eigen::Matrix<float, 3, 1> point_vcs;
      Eigen::Matrix<float, 3, 1> point_camera;
      point_vcs << p3fVcs.x, p3fVcs.y, p3fVcs.z;
      auto extr_t_with_z_offset = extr_t_;
      extr_t_with_z_offset(1, 0) += z_offset_;
      // std::cout<< extr_t_with_z_offset << std::endl;
      point_camera = (extr_R_ * extr_R_comp_) * (extr_R_init_ * point_vcs + (extr_t_with_z_offset + extr_t_comp_));
      if (point_camera[2] <= 0)
      {
        p3fImg.z = -1;
        return false;
      }
      float x = point_camera[0] / point_camera[2];
      float y = point_camera[1] / point_camera[2];
      float depth = point_camera[2];
      // float depth = 1.0;
      p3fImg = cv::Point3f(x, y, depth);

      // add distortion first, then project to img pixel
      if (intr_D_.camera_model == MODEL_PINHOLE)
      {
        ProjectInImagePinhole(p3fImg);
      }
      else if (intr_D_.camera_model == MODEL_FISHEYE)
      {
        ProjectInImageFisheye(p3fImg);
      }

      if (p3fImg.x >= 0 && p3fImg.x < camera_width_ && p3fImg.y >= 0 && p3fImg.y < camera_height_)
      {
        return true;
      }
      // p3fImg.z = -1;
      return false;
    }

    // 自车坐标转相机坐标：注意是相机坐标系。不是图像坐标系
    bool CameraUtils::ConvertPointVCS2CAM(const Eigen::Vector3f &p3fVcs, Eigen::Vector3f &p3fImg)
    {
      // Eigen::Matrix<float, 3, 1> point_vcs;
      // Eigen::Matrix<float, 3, 1> point_camera;
      // point_vcs << p3fVcs.x, p3fVcs.y, p3fVcs.z;
      p3fImg = (extr_R_ * extr_R_comp_) * (extr_R_init_ * p3fVcs + (extr_t_ + extr_t_comp_));
      if (p3fImg[2] <= 0)
      {
        p3fImg[2] = -1;
        return false;
      }
      return true;
    }

    // 自车坐标转相机坐标：注意是相机坐标系。不是图像坐标系
    bool CameraUtils::ConvertPointCAM2VCS(const Eigen::Vector3f &p3fImg, Eigen::Vector3f &p3fVcs)
    {
      // Eigen::Matrix<float, 3, 1> point_vcs;
      // Eigen::Matrix<float, 3, 1> point_camera;
      // point_vcs << p3fVcs.x, p3fVcs.y, p3fVcs.z;

      // p3fImg = (extr_R_ * extr_R_comp_) * (extr_R_init_ * p3fVcs + (extr_t_ + extr_t_comp_));
      p3fVcs = extr_R_init_.inverse() * ((extr_R_ * extr_R_comp_).inverse() * p3fImg - (extr_t_ + extr_t_comp_));
      // if (p3fImg[2] <= 0) {
      //   p3fImg[2] = -1;
      //   return false;
      // }
      // if(p3fVcs[0]<0)return false;
      return true;
    }

    bool CameraUtils::ConvertPointWheelVCS2IMG(const cv::Point3f &p3fVcs, cv::Point3f &p3fImg)
    {
      Eigen::Matrix<float, 3, 1> point_vcs;
      Eigen::Matrix<float, 3, 1> point_camera;
      point_vcs << p3fVcs.x, p3fVcs.y, p3fVcs.z;
      point_camera = extr_R_ * (extr_R_init_ * point_vcs + extr_t_wheel_);
      if (point_camera[2] <= 0)
      {
        p3fImg.z = -1;
        return false;
      }
      float x = point_camera[0] / point_camera[2];
      float y = point_camera[1] / point_camera[2];
      float depth = point_camera[2];
      p3fImg = cv::Point3f(x, y, depth);

      // add distortion first, then project to img pixel
      if (intr_D_.camera_model == MODEL_PINHOLE)
      {
        ProjectInImagePinhole(p3fImg);
      }
      else if (intr_D_.camera_model == MODEL_FISHEYE)
      {
        ProjectInImageFisheye(p3fImg);
      }

      if (p3fImg.x >= 0 && p3fImg.x < camera_width_ && p3fImg.y >= 0 && p3fImg.y < camera_height_)
      {
        return true;
      }
      return false;
    }

    bool CameraUtils::ConvertPointIMG2GND(const cv::Point3f &p3fImg, cv::Point3f &p3fVcs)
    {
      cv::Point2f pt, upoint;
      pt.x = p3fImg.x;
      pt.y = p3fImg.y;
      UndistortPoint(pt, upoint);
      // if (upoint.x <= 0 || upoint.x >= scaled_width_ || upoint.y <= 0 || upoint.y >= scaled_height_) {
      //   return false;
      // }
      Eigen::Matrix<float, 3, 3> M_Mat;
      M_Mat = intr_K_ * (extr_R_ * extr_R_comp_) * extr_R_init_;
      Eigen::Matrix<float, 3, 1> p_4;
      p_4 = intr_K_ * (extr_R_ * extr_R_comp_) * (extr_t_ + extr_t_comp_);
      auto M_Mat_inv_ = M_Mat.inverse();
      auto C_tilde_ = M_Mat_inv_ * p_4;

      Eigen::Matrix<float, 3, 1> p_in_camera;
      p_in_camera(0, 0) = upoint.x;
      p_in_camera(1, 0) = upoint.y;
      p_in_camera(2, 0) = 1;
      Eigen::Matrix<float, 3, 1> X_Tilde = M_Mat_inv_ * p_in_camera;

      // 地平线以上的点会投影到相机后面,统一把这些点输出000
      if (X_Tilde(2) > 0)
      {
        p3fVcs = cv::Point3f(0, 0, 0);
        return false;
      }

      float mue_N = C_tilde_(2, 0) / X_Tilde(2, 0);
      // float mue_N = (z_offset_ + C_tilde_(2, 0)) / X_Tilde(2, 0);
      Eigen::Matrix<float, 3, 1> point = mue_N * X_Tilde - C_tilde_;

      p3fVcs.x = point[0];
      p3fVcs.y = point[1];
      p3fVcs.z = point[2];
      return true;
    }

    bool CameraUtils::UndistortPointPinhole(const cv::Point2f &pt, cv::Point2f &upoint)
    {
      const cv::Mat K = (cv::Mat_<double>(3, 3) << intr_K_(0, 0), intr_K_(0, 1), intr_K_(0, 2),
                         intr_K_(1, 0), intr_K_(1, 1), intr_K_(1, 2),
                         intr_K_(2, 0), intr_K_(2, 1), intr_K_(2, 2));

      const cv::Mat D = (cv::Mat_<double>(5, 1) << intr_D_.params_d[0], intr_D_.params_d[1], intr_D_.params_d[2], intr_D_.params_d[3], intr_D_.params_d[4]);

      cv::Mat mat(1, 2, CV_32F);
      mat.at<float>(0, 0) = pt.x;
      mat.at<float>(0, 1) = pt.y;
      mat = mat.reshape(2);
      cv::undistortPoints(mat, mat, K, D, cv::Mat(), K);
      mat = mat.reshape(1);
      upoint.x = mat.at<float>(0, 0);
      upoint.y = mat.at<float>(0, 1);
      return true;
    }

    bool CameraUtils::UndistortPointFisheye(const cv::Point2f &pt, cv::Point2f &upoint)
    {
      Eigen::Vector2f pp((pt.x * intr_K_(1, 1) - intr_K_(0, 2) * intr_K_(1, 1) - intr_K_(0, 1) * (pt.y - intr_K_(1, 2))) / (intr_K_(0, 0) * intr_K_(1, 1)), (pt.y - intr_K_(1, 2)) / intr_K_(1, 1)); // plane
      Eigen::Vector2f pu = pp;
      for (int j = 0; j < 4; j++)
      {
        float r2 = pu[0] * pu[0] + pu[1] * pu[1];
        float r4 = r2 * r2;
        pu[0] = (pp[0] - 2 * intr_D_.params_d[2] * pu[0] * pu[1] - intr_D_.params_d[3] * (r2 + 2 * pu[0] * pu[0])) / (1 + intr_D_.params_d[0] * r2 + intr_D_.params_d[1] * r4);
        pu[1] = (pp[1] - 2 * intr_D_.params_d[3] * pu[0] * pu[1] - intr_D_.params_d[2] * (r2 + 2 * pu[1] * pu[1])) / (1 + intr_D_.params_d[0] * r2 + intr_D_.params_d[1] * r4);
      }

      auto epsilon = intr_D_.epsilon;
      // project to unit sphere
      float r2 = pu[0] * pu[0] + pu[1] * pu[1];
      float a = r2 + 1;
      float b = 2 * epsilon * r2;
      float c = r2 * epsilon * epsilon - 1;

      if (b * b - 4 * a * c < 0)
      {
        upoint.x = 1e5;
        upoint.y = 1e5;
        return false;
      }

      float Zs = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      Eigen::Vector3f Xw(pu[0] * (Zs + epsilon), pu[1] * (Zs + epsilon), Zs);

      if (Xw[2] < 0)
      {
        upoint.x = 1e5;
        upoint.y = 1e5;
        return false;
      }
      upoint.x = Xw[0] / Xw[2] * intr_K_(0, 0) + intr_K_(0, 2);
      upoint.y = Xw[1] / Xw[2] * intr_K_(1, 1) + intr_K_(1, 2);
      return true;
    }

    bool CameraUtils::UndistortPoint(const cv::Point2f &pt, cv::Point2f &upoint)
    {
      if (intr_D_.camera_model == MODEL_PINHOLE)
      {
        return UndistortPointPinhole(pt, upoint);
      }
      else if (intr_D_.camera_model == MODEL_FISHEYE)
      {
        return UndistortPointFisheye(pt, upoint);
      }
      else
        return false;
    }

    // 计算每个校正图点对应的原图点，并保存成map
    void CameraUtils::generateUndistortMap(cv::Matx33f newK, cv::Size newSize)
    {
      umapx.create(newSize.height, newSize.width, CV_32F);
      umapy.create(newSize.height, newSize.width, CV_32F);
      cv::Matx33f M(intr_K_(0, 0), intr_K_(0, 1), intr_K_(0, 2), intr_K_(1, 0), intr_K_(1, 1), intr_K_(1, 2), intr_K_(2, 0), intr_K_(2, 1), intr_K_(2, 2));
      cv::Vec4f D(intr_D_.params_d[0], intr_D_.params_d[1], intr_D_.params_d[2], intr_D_.params_d[3]);
      float xi = 0;
      if (intr_D_.camera_model == MODEL_FISHEYE)
        xi = intr_D_.epsilon;
      for (int y = 0; y < umapx.rows; y++)
      {
        float *pumapx = umapx.ptr<float>(y);
        float *pumapy = umapy.ptr<float>(y);
        for (int x = 0; x < umapx.cols; x++)
        {
          // convert to camera coordinate
          cv::Vec3f Xc(x - newK(0, 2), y - newK(1, 2), newK(0, 0));
          // convert to unit sphere
          cv::Vec3f Xs = Xc / cv::norm(Xc);
          // rotate
          // if(hasR) Xs = R * Xs;
          // convert to normalized image plane
          float xu = Xs[0] / (Xs[2] + xi);
          float yu = Xs[1] / (Xs[2] + xi);
          // add distortion
          float r2 = xu * xu + yu * yu;
          float r4 = r2 * r2;
          float r = 1 + D[0] * r2 + D[1] * r4;
          float xd = xu * r + 2 * D[2] * xu * yu + D[3] * (r2 + 2 * xu * xu);
          float yd = yu * r + D[2] * (r2 + 2 * yu * yu) + 2 * D[3] * xu * yu;
          // convert to pixel coordinate
          pumapx[x] = M(0, 0) * xd + M(0, 1) * yd + M(0, 2);
          pumapy[x] = M(1, 1) * yd + M(1, 2);
        }
      }
    }

  } // namespace perception
} // namespace avos
