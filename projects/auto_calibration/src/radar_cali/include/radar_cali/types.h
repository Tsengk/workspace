#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace radar_cali {

/// Radar calibration result
struct RadarCalibResult {
    Eigen::Matrix3d R;           ///< Rotation matrix (radar to VCS)
    Eigen::Vector3d t;           ///< Translation vector (radar to VCS) in meters
    Eigen::Matrix4d T;           ///< 4x4 homogeneous transformation matrix
    double roll_deg, pitch_deg, yaw_deg;  ///< Euler angles in degrees
    double x_m, y_m, z_m;        ///< Translation in meters
    bool valid;                  ///< Whether calibration is valid
    std::string message;         ///< Status message
    std::string quality_grade;   ///< EXCELLENT/GOOD/ACCEPTABLE/POOR
    double confidence;           ///< Confidence metric [0,1]

    RadarCalibResult();
};

/// Installation parameters
struct InstallParams {
    std::string radar_name;      ///< Radar name
    double x, y, z;              ///< Installation position (m)
    double roll_deg, pitch_deg, yaw_deg;  ///< Installation orientation (deg)

    InstallParams();
};

/// Calibration output structure
struct CalibOutput {
    InstallParams installation;  ///< Installation parameters
    RadarCalibResult result;     ///< Calibration result
    std::string save_path;       ///< Path to save calibration
    std::string timestamp;       ///< Timestamp

    CalibOutput();
};

} // namespace radar_cali