#ifndef _CAMERA_SERVICE_TYPE_DEFINE_H___
#define _CAMERA_SERVICE_TYPE_DEFINE_H___

// #include "modules/common/common_define.h"


#include "common_define.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>

namespace avos {
namespace perception {

#define MODEL_PINHOLE 0
#define MODEL_FISHEYE 1

static std::map<int, std::string> CamIdx2Str = {
    // {CAM_FRONT_H100, "front_h120"},
    {CAM_REAR, "back"},
    {CAM_LEFT_FRONT, "left_front"},
    {CAM_LEFT_REAR, "left_back"},
    {CAM_RIGHT_FRONT, "right_front"},
    {CAM_RIGHT_REAR, "right_back"},
    {CAM_FRONT_H30, "front_h30"},
    {CAM_FRONT_H100, "front_h100"}
    // {CAM_FRONT_H120_CROP_NEW, "front_h120_crop_new"}
    // {CAM_FRONT_H100, "front_h100"}
};

static std::map<int, int> GetCameraModel = {
    // {CAM_FRONT_H100, MODEL_FISHEYE},
    {CAM_FRONT_H100, MODEL_FISHEYE},
    {CAM_REAR, MODEL_FISHEYE},
    {CAM_LEFT_FRONT, MODEL_FISHEYE},
    {CAM_LEFT_REAR, MODEL_FISHEYE},
    {CAM_RIGHT_FRONT, MODEL_FISHEYE},
    {CAM_RIGHT_REAR, MODEL_FISHEYE},
    {CAM_FRONT_H30, MODEL_PINHOLE}
    // {CAM_FRONT_H120_CROP, MODEL_FISHEYE}
    // {CAM_FRONT_H120_CROP_NEW,   MODEL_FISHEYE}
    // {CAM_FRONT_H100,   MODEL_FISHEYE}
};

typedef struct DistortionParams {
    int camera_model = 0;
    float params_d[5] = {0, 0, 0, 0, 0};
    float epsilon = 0;
} DataType;

} // namespace perception
} // namespace avos

#endif // _CAMERA_SERVICE_TYPE_DEFINE_H___
