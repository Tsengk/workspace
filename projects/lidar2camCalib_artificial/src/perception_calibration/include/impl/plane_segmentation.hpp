/**************************************************************
  * Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
  * NodeName: lidarseg
  * FileName: plane_segmentatioh.hpp
  * Description:
  * 1.
  * 2.
  * 3.
  * History:
  * <author>    <time>    <version>    <description>
  * chendong   2018/05/04   1.0.0      bulid this module
************************************************************/
#ifndef IMPL_PLANE_SEGMENTATION_H_
#define IMPL_PLANE_SEGMENTATION_H_

#include <cmath>
#include <random>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl_types.h"

namespace calibration {

template <typename PointT>
PlaneSegmentation<PointT>::PlaneSegmentation()
  : input_cloud_(new PointCloud()), output_cloud_(new PointCloud()) {
  suspected_ground_point_ptrs_.resize(2);
  ground_planes_.resize(2);
  for (auto& cloud_ptr : suspected_ground_point_ptrs_) {
    cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud_ptr->reserve(50000);
  }
}

template <typename PointT>
PlaneSegmentation<PointT>::~PlaneSegmentation() {}

template <typename PointT> void
PlaneSegmentation<PointT>::GenerateSuspectedGroundPoint() {
  ROI roi{ -20, 20, -8, 8};
  for (size_t i = 0; i < input_cloud_->size(); i++) {
    const PointT& _p = input_cloud_->points[i];
    pcl::PointXYZI p(static_cast<float>(_p.intensity));
    p.x = _p.x; p.y = _p.y; p.z = _p.z;
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    bool sample_flag = input_cloud_->size() < 30000 ? true : false;
    if (p.x > 0 && p.x < roi.max_x && fabs(p.y) < roi.max_y) {
      if (sample_flag || (i % (FastRand() % 4 + 1)) == 0) {
        suspected_ground_point_ptrs_[0]->push_back(p);
      }
    } else if (p.x < 0 && p.x > roi.min_x && fabs(p.y) < roi.max_y) {
      if (sample_flag || (i % (FastRand() % 4 + 1)) == 0) {
        suspected_ground_point_ptrs_[1]->push_back(p);
      }
    }
  }

  std::vector<ROI> rois(2);
  rois[0] = roi;
  rois[0].min_x = 0.0f;
  rois[1] = roi;
  rois[1].max_x = 0.0f;
  GenerateSuspectedGroundSegment(rois[0], &suspected_ground_point_ptrs_[0]);
  GenerateSuspectedGroundSegment(rois[1], &suspected_ground_point_ptrs_[1]);

  // Debug
  if (0) {
    static ros::NodeHandle nh_debug;
    static ros::Publisher pub_debug;
    pub_debug = nh_debug.advertise<sensor_msgs::PointCloud2>("/plane_debug", 10);
    pcl::PointCloud<pcl::PointXYZI> pc;
    pc += *suspected_ground_point_ptrs_[0];
    pc += *suspected_ground_point_ptrs_[1];
    pc.header.frame_id = input_cloud_->header.frame_id;
    pc.height = 1;
    pc.width = pc.size();
    if (!pc.empty()) {
      pub_debug.publish(pc);
    }
  }
}

template <typename PointT> void
PlaneSegmentation<PointT>::GenerateSuspectedGroundSegment(const ROI& roi,
    pcl::PointCloud<pcl::PointXYZI>::Ptr* cloud_ptr_ptr) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr = *cloud_ptr_ptr;
  if (cloud_ptr == nullptr || cloud_ptr->empty()) {
    return;
  }
  const size_t x_block_size =
    (roi.max_x - roi.min_x) / 0.7;
  const size_t y_block_size =
    (roi.max_y - roi.min_y) / 0.7;
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>>> ground_segment(x_block_size);
  for (size_t i = 0; i < ground_segment.size(); i++) {
    ground_segment[i].resize(y_block_size);
  }
  size_t cloud_points_size = cloud_ptr->points.size();
  for (size_t i = 0; i < cloud_points_size; i++) {
    const auto& p = cloud_ptr->points[i];
    if (p.x < roi.max_x && p.x > roi.min_x &&
        p.y < roi.max_y && p.y > roi.min_y) {
      size_t xpos = (p.x - roi.min_x) / 0.7;
      size_t ypos = (p.y - roi.min_y) / 0.7;
      if (xpos >= 0 && xpos < x_block_size && ypos >= 0 && ypos < y_block_size) {
        ground_segment[xpos][ypos].points.push_back(p);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  ground_cloud_ptr->reserve(cloud_ptr->size());
  for (size_t i = 0; i < ground_segment.size(); i++) {
    for (size_t j = 0; j < ground_segment[i].size(); j++) {
      if (IsGroundSegment(ground_segment[i][j], 0.05)) {
        ground_cloud_ptr->points.insert(ground_cloud_ptr->points.end(),
                                        ground_segment[i][j].points.begin(),
                                        ground_segment[i][j].points.end());
      }
    }
  }
  cloud_ptr = ground_cloud_ptr;
}

template <typename PointT> bool
PlaneSegmentation<PointT>::IsGroundSegment(
  const pcl::PointCloud<pcl::PointXYZI>& cloud,
  const float threshold) const {
  std::vector<float> zVec;
  zVec.reserve(100);
  size_t cloud_points_size = cloud.points.size();
  if (cloud_points_size < 5) {
    return false;
  }
  for (size_t i = 0; i < cloud_points_size; i++) {
    zVec.push_back(cloud.points[i].z);
  }
  //
  std::sort(zVec.begin(), zVec.end());
  if (cloud_points_size > 20) {
    size_t noise = static_cast<int>(cloud_points_size * 0.05);
    zVec.erase(zVec.end() - noise, zVec.end());
    zVec.erase(zVec.begin(), zVec.begin() + noise);
  }
  float diff_z = zVec[zVec.size() - 1] - zVec[0];

  return diff_z < threshold;
}

template <typename PointT> void
PlaneSegmentation<PointT>::GroundPlaneFitting() {
  for (size_t i = 0; i < 2; i++) {
    if (suspected_ground_point_ptrs_[i]->size() > 200) {
      ground_planes_[i] = PlaneFitting(suspected_ground_point_ptrs_[i], 0.1);
    }
  }
  if (ground_planes_[0].isValid && !ground_planes_[1].isValid) {
    ground_planes_[1] = ground_planes_[0];
  } else if (ground_planes_[1].isValid && !ground_planes_[0].isValid) {
    ground_planes_[0] = ground_planes_[1];
  }
}

template <typename PointT> void
PlaneSegmentation<PointT>::RemoveGroundPoint() {
  if (!ground_planes_[0].isValid && !ground_planes_[1].isValid) {
    output_cloud_ = input_cloud_;
    return;
  }
  for (size_t i = 0; i < input_cloud_->size(); i++) {
    const PointT& p = input_cloud_->points[i];
    float dis = 0;
    int ground_distance_mode =
      input_cloud_->size() > 20000 ? 0 : 1;
    if (p.x >= 0 && ground_planes_[0].isValid) {
      dis = DistancePoint2Plane(p, ground_planes_[0], ground_distance_mode);
    } else if (p.x < 0 && ground_planes_[1].isValid) {
      dis = DistancePoint2Plane(p, ground_planes_[1], ground_distance_mode);
    } else if (p.x < 0 && ground_planes_[0].isValid) {
      dis = DistancePoint2Plane(p, ground_planes_[0], ground_distance_mode);
    } else if (p.x >= 0 && ground_planes_[1].isValid) {
      dis = DistancePoint2Plane(p, ground_planes_[0], ground_distance_mode);
    }

    if (fabs(dis) > 0.35) {
      output_cloud_->push_back(p);
    }
  }
  output_cloud_->header = input_cloud_->header;
}

template <typename PointT> PlaneEquation
PlaneSegmentation<PointT>::PlaneFitting(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_ptr,
  const float inlier_threshold) {
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(inlier_threshold);
  seg.setInputCloud(cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (0 == inliers->indices.size()) {
    return PlaneEquation();
  }
  PlaneEquation sp;
  //ax+by+cz+1=0
  sp.a = coefficients->values[0] / coefficients->values[3];
  sp.b = coefficients->values[1] / coefficients->values[3];
  sp.c = coefficients->values[2] / coefficients->values[3];
  if (std::isfinite(sp.a) &&
      std::isfinite(sp.b) &&
      std::isfinite(sp.c) &&
      sp.c != 0) {
    sp.isValid = true;
  }
  return sp;
}

template <typename PointT> float
PlaneSegmentation<PointT>::DistancePoint2Plane(const PointT& p,
    const PlaneEquation& pl, const int mode) {
  if (!pl.isValid ||
      !std::isfinite(p.x) ||
      !std::isfinite(p.y) ||
      !std::isfinite(p.z)) {
    return p.z;
  }
  if (0 == mode) {
    float z_offset = -1 * (pl.a * p.x + pl.b * p.y + 1) / pl.c;
    return (p.z - z_offset);
  } else if (1 == mode) {
    float dis1 = pl.a * p.x + pl.b * p.y + pl.c * p.z + 1;
    dis1 = pl.c * 10e10 + 1 > 0 ? dis1 : -dis1;
    float dis2 = sqrt(pl.a * pl.a + pl.b * pl.b + pl.c * pl.c);
    return (dis1 / dis2);
  }
  return p.z;
}

}  // namespace calibration

#endif  // IMPL_PLANE_SEGMENTATION_H_
