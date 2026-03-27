/**************************************************************
  * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
  * NodeName: perception_calibration
  * FileName: plane_segmentation.h
  * Description:
  * 1.
  * 2.
  * 3.
  * History:
  * <author>    <time>    <version>    <description>
  * chendong   2018/04/22   1.0.0      bulid this module
************************************************************/

#ifndef PLANE_SEGMENTATION_H_
#define PLANE_SEGMENTATION_H_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace calibration {

struct ROI {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
};

struct PlaneEquation {
  PlaneEquation()
    : a(0)
    , b(0)
    , c(0)
    , isValid(false) {}

  float a;
  float b;
  float c;
  bool isValid;
};

template <typename PointT>
class PlaneSegmentation {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

  explicit PlaneSegmentation();
  ~PlaneSegmentation();

  //  name
  std::string Name() const {return "PlaneSegmentation";}

  //  input method
  inline void setInputCloud(const PointCloudPtr& cloud) {
    input_cloud_ = cloud;

    if (1) {
      output_cloud_.reset(new PointCloud());
      output_cloud_->reserve(50000);
      suspected_ground_point_ptrs_.clear();
      suspected_ground_point_ptrs_.resize(2);
      for (auto& cloud_ptr : suspected_ground_point_ptrs_) {
        cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_ptr->reserve(50000);
      }
      ground_planes_.clear();
      ground_planes_.resize(2);
    }
  }

  inline PointCloudPtr getOutputCloud() const {
    return (output_cloud_);
  }

  void Segment() {
    if (input_cloud_ == nullptr) {
      ROS_ERROR_STREAM("plane segmentation input cloud is nullptr.");
      return;
    }
    GenerateSuspectedGroundPoint();
    GroundPlaneFitting();
    RemoveGroundPoint();
  }

private:
  void GenerateSuspectedGroundPoint();
  void GenerateSuspectedGroundSegment(
    const ROI& roi, pcl::PointCloud<pcl::PointXYZI>::Ptr* cloud_ptr_ptr);
  bool IsGroundSegment(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                       const float threshold) const;
  void GroundPlaneFitting();
  void RemoveGroundPoint();

  inline int FastRand() {
    static int fast_rand_seed = static_cast<int>(std::time(NULL));
    fast_rand_seed = (214013 * fast_rand_seed + 2531011);
    return (fast_rand_seed >> 16) & 0x7FFF;
  }

  PlaneEquation PlaneFitting(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
                             const float inlier_threshold);
  float DistancePoint2Plane(const PointT& p, const PlaneEquation& pl,
                            const int mode = 1);

private:
  PointCloudPtr input_cloud_, output_cloud_;
  //  data
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> suspected_ground_point_ptrs_;
  std::vector<PlaneEquation> ground_planes_;
};

}  // namespace calibration

#include "impl/plane_segmentation.hpp"

#endif  // PLANE_SEGMENTATION_H_
