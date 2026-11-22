# camera::Mei 使用说明

基于 Kannala-Brandt 鱼眼相机模型，支持内参、外参、去畸变和 IPM（逆透视映射）。

## 基本构造

```cpp
#include "camera_model/mei.h"

// 默认旋转顺序 {2,1,0} = ZYX
camera::Mei mei;

// 自定义旋转顺序
std::array<int, 3> rotation_order = {{0, 1, 2}};  // XYZ
camera::Mei mei(rotation_order);
```

## 完整构造

```cpp
camera::Mei mei(epsilon, K, D, image_width, image_height, ground_height,
               t_world_to_camera, r_world_to_camera_euler, rotation_order);
```

参数：
- `epsilon`: 鱼眼参数
- `K`: 内参矩阵 (3x3)
- `D`: 畸变系数 (1x4): k1, k2, p1, p2
- `image_width/height`: 图像尺寸
- `ground_height`: 地面高度
- `t_world_to_camera`: 平移向量
- `r_world_to_camera_euler`: 欧拉角（弧度）
- `rotation_order`: 旋转顺序，默认 {2,1,0}

## 内参设置

```cpp
mei.setIntrinsics(epsilon, K, D, image_width, image_height);
mei.setEpsilon(epsilon);
mei.setK(K);
mei.setD(D);
mei.setImageWidth(width);
mei.setImageHeight(height);
```

## 外参设置

```cpp
// 平移 - 自有相机标准
mei.setT(Eigen::Vector3d(1.0, 2.0, 3.0));

// 旋转 - 自有相机标准
mei.setR(Eigen::Vector3d(rx, ry, rz));  // 欧拉角（弧度）
mei.setR(Eigen::Matrix3d(R));            // 旋转矩阵

// 世界坐标系转换 - 自动应用坐标系转换
mei.setWorldToCameraR(R_w_c);  // 输入 world->camera 旋转矩阵
Eigen::Matrix3d R = mei.getWorldToCameraR();  // 获取 world->camera 旋转矩阵
```

## 去畸变

```cpp
// 设置去畸变目标参数
cv::Mat undistort_K = cv::Mat::eye(3, 3, CV_64F);
undistort_K.at<double>(0,0) = 800; undistort_K.at<double>(1,1) = 800;
undistort_K.at<double>(0,2) = 960; undistort_K.at<double>(1,2) = 540;
mei.setUndistortInfo(width, height, undistort_K);

// 单点去畸变
cv::Point2d distorted_point(100, 200);
cv::Point2d undistorted_point;
mei.undistort(distorted_point, undistorted_point, undistort_K);

// 图像去畸变
cv::Mat distorted_image = cv::imread("fish_eye.jpg");
cv::Mat undistorted_image;
mei.undistortImage(distorted_image, undistorted_image);

// 生成去畸变映射表
cv::Mat map1, map2;
mei.generateUndistortMap(&map1, &map2);
cv::remap(distorted_image, undistorted_image, map1, map2, cv::INTER_LINEAR);
```

## IPM（逆透视映射）

```cpp
mei.setGroundHeight(ground_height);
// 平移 - 自有相机标准
mei.setT(Eigen::Vector3d(1.0, 2.0, 3.0));

// 旋转 - 自有相机标准
mei.setR(Eigen::Vector3d(rx, ry, rz));  // 欧拉角（弧度）
mei.setR(Eigen::Matrix3d(R));            // 旋转矩阵

// 设置 IPM 参数
// bev_width/bev_height: BEV图像尺寸（像素）
// scale: 像素/米，表示1米对应多少像素
// origin_offset_x/y: BEV图像原点在像素坐标系中的偏移（用于调整BEV位置）
// roi_*: 感兴趣区域，用于裁剪BEV图像
mei.setIpmInfo(bev_width, bev_height, scale,
               origin_offset_x, origin_offset_y,
               roi_top, roi_bottom, roi_left, roi_right);

// 生成 IPM 映射表
cv::Mat map_x, map_y;
mei.generateIpmMap(&map_x, &map_y);

// 生成 IPM 图像
cv::Mat fish_eye_image;
cv::Mat bev_image;
mei.generateIpmImage(fish_eye_image, bev_image);
```

参数详解：
- `bev_width/bev_height`: 输出BEV图像的宽高（像素）
- `scale`: 像素到米的转换比例，如 scale=100 表示 100像素=1米
- `origin_offset_x`: BEV原点X方向偏移（像素），正值向左偏移
- `origin_offset_y`: BEV原点Y方向偏移（像素），正值向上偏移
- `roi_top/bottom/left/right`: ROI区域边界（像素），用于自动裁剪BEV

## 投影

```cpp
// 世界坐标 -> 相机坐标 -> 图像坐标
Eigen::Vector3d point_world(1.0, 2.0, 0.0);
Eigen::Vector3d point_camera;
cv::Point2d point_2d;

mei.world3dToCamera3d(point_world, point_camera);        // 世界->相机
mei.projectCamera3dTo2d(point_camera, point_2d);           // 相机->图像
mei.projectWorld3dTo2d(point_world, point_2d);          // 直接世界->图像
```

## 旋转顺序

`rotation_order` 定义欧拉角的旋转轴顺序：

| 旋转顺序 | 含义 |
|---------|------|
| {2,1,0} | Z-Y-X (默认) |
| {0,1,2} | X-Y-Z |
| {0,2,1} | X-Z-Y |

```cpp
// 使用 ZYX 顺序（默认）
std::array<int, 3> rot_zyx = {{2, 1, 0}};
camera::Mei mei(rot_zyx);

// 使用 XYZ 顺序
std::array<int, 3> rot_xyz = {{0, 1, 2}};
camera::Mei mei(rot_xyz);
```

## 坐标系转换

`setWorldToCameraR` 和 `getWorldToCameraR` 提供世界坐标系转换：

- `R_w_c` = world → camera（世界坐标转相机坐标）
- 自动应用内部坐标系转换矩阵

```cpp
// 设置世界->相机旋转
mei.setWorldToCameraR(R_w_c);

// 获取世界->相机旋转
Eigen::Matrix3d R = mei.getWorldToCameraR();

// 对比：自有标准旋转（不经过坐标系转换）
mei.setR(R_raw);  // 直接设置自有标准
Eigen::Matrix3d R_raw = mei.getR();  // 获取自有标准
```

## 完整示例

```cpp
#include "camera_model/mei.h"

int main() {
    // 1. 构造相机模型
    std::array<int, 3> rotation_order = {{2, 1, 0}};
    camera::Mei mei(rotation_order);

    // 2. 设置内参
    double epsilon = 1.9;
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0,0) = 1511.7; K.at<double>(1,1) = 1511.0;
    K.at<double>(0,2) = 961.6; K.at<double>(1,2) = 540.5;
    cv::Mat D = cv::Mat::zeros(1, 4, CV_64F);
    D.at<double>(0,0) = 0.76; D.at<double>(0,1) = -0.93;
    mei.setIntrinsics(epsilon, K, D, 1920, 1080);

    // 3. 设置外参
    Eigen::Vector3d t(3.6, 0.0, 1.2);
    Eigen::Vector3d r_euler(0.0, -0.26, 0.0);  // 弧度
    mei.setExtrinsics(t, r_euler);

    // 4. 去畸变
    cv::Mat undistort_K = K.clone();  // 使用相同内参
    mei.setUndistortInfo(1920, 1080, undistort_K);

    // 5. 去畸变图像
    cv::Mat distorted = cv::imread("fisheye.jpg");
    cv::Mat undistorted;
    mei.undistortImage(distorted, undistorted);

    return 0;
}
```
