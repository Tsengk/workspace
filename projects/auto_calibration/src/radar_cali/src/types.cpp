#include "radar_cali/types.h"
#include <Eigen/Geometry>

namespace radar_cali {

RadarCalibResult::RadarCalibResult()
    : R(Eigen::Matrix3d::Identity()),
      t(Eigen::Vector3d::Zero()),
      T(Eigen::Matrix4d::Identity()),
      roll_deg(0), pitch_deg(0), yaw_deg(0),
      x_m(0), y_m(0), z_m(0),
      valid(false),
      message("Not calibrated"),
      quality_grade("POOR"),
      confidence(0) {}

InstallParams::InstallParams()
    : radar_name(""),
      x(0), y(0), z(0),
      roll_deg(0), pitch_deg(0), yaw_deg(0) {}

CalibOutput::CalibOutput() {}

} // namespace radar_cali