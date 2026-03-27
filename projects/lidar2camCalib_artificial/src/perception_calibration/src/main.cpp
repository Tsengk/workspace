#include "perception_calibration.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "perception_calibration");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  calibration::PerceptionCalibration pc(nh, private_nh);
  ros::spin();
  return 0;
}

