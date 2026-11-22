#include "vehicle_config/types.h"

namespace vehicle_config {

VehicleInfo::VehicleInfo()
    : id(""), car_length(0), car_width(0), car_height(0),
      front_overhang(0), wheelbase(0), rear_overhang(0), z_offset(0) {}

CameraIntrinsics::CameraIntrinsics()
    : Epsilon(0) {}

CameraExtrinsics::CameraExtrinsics()
    : t(3, 0.0), R_euler(3, 0.0),
      vcs_to_ground_height(0), base_euler(0) {}

CameraInfo::CameraInfo() : name(""), topic("") {}

LidarInfo::LidarInfo()
    : name(""), topic(""), t(3, 0.0), R(3, 0.0) {}

RadarInfo::RadarInfo()
    : name(""), topic(""), t(3, 0.0), R(3, 0.0) {}

GnssInfo::GnssInfo() : name(""), topic("") {}

BevSurroundViewConfig::BevSurroundViewConfig()
    : node_ns("/bev_surround_view_node"),
      bev_image_topic("/bev_surround_view_node/bev_image"),
      detection_vis_front_topic("/bev_surround_view_node/detection_vis_front"),
      detection_vis_rear_topic("/bev_surround_view_node/detection_vis_rear"),
      detection_vis_left_topic("/bev_surround_view_node/detection_vis_left"),
      detection_vis_right_topic("/bev_surround_view_node/detection_vis_right"),
      output_width(1280),
      output_height(1280),
      resolution(0.02),
      range_front(250.0),
      range_rear(150.0),
      range_left(125.0),
      range_right(125.0),
      origin_offset_x(0.0),
      origin_offset_y(0.0) {}

VehicleConfig::VehicleConfig() {}

} // namespace vehicle_config