#ifndef _CAMERA_SERVICE_CAMERA_UTILS_H___
#define _CAMERA_SERVICE_CAMERA_UTILS_H___

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "camera_type_define.h"

namespace avos
{
  namespace perception
  {

    class CameraUtils
    {
    public:
      CameraUtils();
      ~CameraUtils();

      bool SetT(const std::vector<float> &);

      bool SetR(const std::vector<float> &);

      bool SetZOffset(const float);
      bool GetZOffset(float &);

      bool GetT(std::vector<float> &);

      bool GetR(std::vector<float> &);

      bool GetRoi(std::vector<int> &);

      bool LoadCameraParams(const int &cam_idx, const YAML::Node &node);

      bool ConvertPointVCS2IMG(const cv::Point3f &p3fVcs, cv::Point3f &p3fImg);

      bool ConvertPointVCS2CAM(const Eigen::Vector3f &p3fVcs, Eigen::Vector3f &p3fImg);

      bool ConvertPointCAM2VCS(const Eigen::Vector3f &p3fImg, Eigen::Vector3f &p3fVcs);

      bool ConvertPointWheelVCS2IMG(const cv::Point3f &p3fVcs, cv::Point3f &p3fImg);

      bool UndistortPoint(const cv::Point2f &pt, cv::Point2f &upoint);

      bool ConvertPointIMG2GND(const cv::Point3f &p3fImg, cv::Point3f &p3fVcs);

      // 根据自车俯仰颠簸在线对外参进行补偿
      void SetExtrCompensate(const Eigen::Vector3f euler)
      {
        Eigen::Quaternionf rotation =
            (Eigen::AngleAxisf(euler.z(), Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(euler.y(), Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(euler.x(), Eigen::Vector3f::UnitX()));
        extr_R_comp_ = rotation.matrix();
        extr_t_comp_ = extr_R_comp_ * extr_t_ - extr_t_;
      }

      void ResetExtrR()
      {
        extr_R_ = extr_R_original_;
      }

      void SetExtrR(const Eigen::Vector3f euler)
      {
        Eigen::Quaternionf rotation =
            (Eigen::AngleAxisf(euler.z(), Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(euler.y(), Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(euler.x(), Eigen::Vector3f::UnitX()));
        extr_R_ = rotation.matrix();
      }

      void ResetExtrTrans()
      {
        extr_t_ = extr_t_original_;
      }

      void SetExtrTrans(const Eigen::Matrix<float, 3, 1> trans)
      {
        extr_t_ = trans;
      }

      int camera_width() const
      {
        return camera_width_;
      }

      int camera_height() const
      {
        return camera_height_;
      }

      // int original_camera_width() const {
      //   return original_width_;
      // }

      // int original_camera_height() const {
      //   return original_height_;
      // }

      // int scaled_camera_width() const {
      //   return scaled_width_;
      // }

      // int scaled_camera_height() const {
      //   return scaled_height_;
      // }

      Eigen::Matrix<float, 3, 3> intr_K() const
      {
        return intr_K_;
      }

      DistortionParams intr_D() const
      {
        return intr_D_;
      }

      Eigen::Matrix<float, 3, 3> new_intr_K() const
      {
        return new_intr_K_;
      }

      int new_width() const
      {
        return new_width_;
      }

      int new_height() const
      {
        return new_height_;
      }

      Eigen::Matrix<float, 3, 3> extr_R_init() const
      {
        return extr_R_init_;
      }

      Eigen::Matrix<float, 3, 3> extr_R() const
      {
        return extr_R_;
      }

      Eigen::Matrix<float, 3, 1> extr_t() const
      {
        return extr_t_;
      }

      // private:
      bool LoadCameraRoi(const YAML::Node &node);
      bool LoadCameraIntrinsics(const YAML::Node &node);
      bool LoadCameraExtrinsics(const YAML::Node &node);

      void ProjectInImagePinhole(cv::Point3f &pt);
      void ProjectInImageFisheye(cv::Point3f &pt);

      bool UndistortPointFisheye(const cv::Point2f &pt, cv::Point2f &upoint);
      bool UndistortPointPinhole(const cv::Point2f &pt, cv::Point2f &upoint);
      Eigen::Vector3f solveDirect(const Eigen::Matrix3f &A, const Eigen::Vector3f &b);

      void generateUndistortMap(cv::Matx33f newK, cv::Size newSize);
      void undistortImage(cv::Mat &dist, cv::Mat &undist)
      {
        assert(!umapx.empty() && "call generateUndistortMap() before call me.");
        remap(dist, undist, umapx, umapy, cv::INTER_AREA, cv::BORDER_CONSTANT);
      }
      cv::Mat umapx, umapy;

      int cam_idx_ = 0;
      // int original_width_ = 0;
      // int original_height_ = 0;
      int camera_width_ = 0.0;
      int camera_height_ = 0.0;
      // int scaled_width_=0.0;
      // int scaled_height_=0.0;
      int new_width_ = 0.0;
      int new_height_ = 0.0;
      Eigen::Matrix<float, 3, 3> intr_K_;
      DistortionParams intr_D_;

      Eigen::Matrix<float, 3, 3> new_intr_K_;

      Eigen::Matrix<float, 3, 3> extr_R_init_;
      Eigen::Matrix<float, 3, 3> extr_R_;
      Eigen::Matrix<float, 3, 3> extr_R_original_;
      double extr_roll_original_;
      double extr_pitch_original_;
      double extr_yaw_original_;

      Eigen::Matrix<float, 3, 1> extr_t_original_;
      Eigen::Matrix<float, 3, 1> extr_t_;       // to ground
      Eigen::Matrix<float, 3, 1> extr_t_wheel_; // to wheel center

      Eigen::Matrix<float, 3, 3> extr_R_comp_;
      Eigen::Matrix<float, 3, 1> extr_t_comp_;
      std::vector<int> bev_roi_;

      cv::Rect2f crop_rect;
      float crop_sx{0}, crop_sy{0}, crop_tx{0}, crop_ty{0}; // 分割图像素转换到原图像素：x原 = x分 * sx + tx

      // TODO: load from sensor_calib file
      float z_offset_ = -0.33;
    };

  } // namespace perception
} // namespace avos

#endif // _CAMERA_SERVICE_CAMERA_UTILS_H___
