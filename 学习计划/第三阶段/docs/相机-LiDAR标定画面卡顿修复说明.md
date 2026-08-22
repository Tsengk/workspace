# 相机-LiDAR 标定画面卡顿修复说明

## 问题现象

启动软件、导入车辆配置文件后，在"相机-LiDAR标定" Tab 中选择 `front_middle_lidar` 和 `front_120`，画面卡顿（叠加图像不更新），而选择 `front_60` 等其他相机流畅。

## 根因分析

通过日志定位到根本原因：

```
[WARN] [LidarCameraProjection] image/cloud time diff too large: 6.700 sec
```

投影节点 `lidar_camera_projection_node` 的 `onTimer` 回调中有一道时间戳同步检查，要求相机图像与激光雷达点云的时间戳差值不超过 `sync_tolerance_sec_`（默认 0.2 秒）。但由于 ROS bag 回放时 front_120 相机与 `front_middle_lidar` 激光雷达的数据来源不同，时间戳相差 6~7 秒，导致**所有帧都被同步检查跳过**，叠加图像无法更新，GUI 表现为画面卡死。

其他相机（如 front_60）的 bag 数据与激光雷达时间戳相近，恰好能通过同步检查，因此表现流畅。

## 修改内容

**文件**: `src/lidar_camera_projection/src/lidar_camera_projection_node.cpp`  
**函数**: `onTimer`

- 移除时间戳同步检查（`image_stamp`、`cloud_stamp` 相关变量及差值判断逻辑）
- 发布时直接使用点云的 `header.stamp` 作为输出图像时间戳

**理由**: 相机-LiDAR 标定是手动可视化校准工具，用户通过观察点云投影到图像上的贴合程度来手动微调外参，不需要精确的时间戳同步。取最新图像 + 最新点云直接叠加即可满足使用需求。

### 代码变更

```diff
     void onTimer(const ros::TimerEvent&)
     {
         cv::Mat image;
         sensor_msgs::PointCloud2::ConstPtr cloud;
-        ros::Time image_stamp;
-        ros::Time cloud_stamp;
 
         {
             std::lock_guard<std::mutex> lock(mutex_);
@@ -335,19 +333,7 @@ private:
                 return;
             }
             image = latest_image_.clone();
-            image_stamp = latest_image_stamp_;
             cloud = latest_cloud_;
-            cloud_stamp = latest_cloud_->header.stamp;
-        }
-
-        if (!image_stamp.isZero() && !cloud_stamp.isZero()) {
-            const double dt = std::fabs((image_stamp - cloud_stamp).toSec());
-            if (dt > sync_tolerance_sec_) {
-                ROS_WARN_THROTTLE(1.0,
-                                  "[LidarCameraProjection] image/cloud time diff too large: %.3f sec",
-                                  dt);
-                return;
-            }
         }
 
         cv::Mat overlay = renderOverlay(image, cloud);
@@ -355,11 +341,7 @@ private:
             return;
         }
 
-        std_msgs::Header header = cloud->header;
-        if (!image_stamp.isZero()) {
-            header.stamp = image_stamp;
-        }
-        pub_overlay_.publish(cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg());
+        pub_overlay_.publish(cv_bridge::CvImage(cloud->header, "bgr8", overlay).toImageMsg());
     }
```

## 涉及文件

```
src/lidar_camera_projection/src/lidar_camera_projection_node.cpp
```

## 重新编译

```bash
catkin build lidar_camera_projection
```

## 验证方法

编译后重新启动软件，加载配置，选择 `front_middle_lidar` + `front_120`，开启可视化。叠加图像应持续更新，不再卡顿。

---
