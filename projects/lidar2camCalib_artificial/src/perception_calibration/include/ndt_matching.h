#ifndef PERCPTION_CALIBRATION_NDTMATCHING_H_
#define PERCPTION_CALIBRATION_NDTMATCHING_H_
// c++ lib
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <boost/thread/thread.hpp>
// ros lib
#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/tf.h>
// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
// Eigen
#include <Eigen/Dense>

namespace calibration {
template<typename PointT>
class NdtMatching {
  template<typename T>
  friend std::ostream& operator<<(std::ostream& os,
                                  const NdtMatching<T>& ndt);
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  NdtMatching();
  ~NdtMatching();
  void setInputCloud(const PointCloudPtr& input_target_cloud_ptr,
                     const PointCloudPtr& input_source_cloud_ptr);
  void setTransformationEpsilon(const double trans_epsilon);
  void setStepSize(const double step_size);
  void setResolution(const double resolution);
  void setMaximumIterations(const int max_iter);
  Eigen::Matrix4f getFinalTransformation() const;
  double getFitnessScore() const;
  int getFinalNumIteration() const;
  PointCloudPtr getTransformedCloud() const;
  double getTransformationProbability() const;
  void reset();

private:
  void AlignProcess(const PointCloudPtr& target_cloud_ptr,
                    const PointCloudPtr& source_cloud_ptr);
  void ConvertTf2Eigen(const tf::Transform& tf, Eigen::Matrix4f& m4f);
  void InitPose();

private:
  PointCloudPtr target_cloud_ptr_;
  PointCloudPtr source_cloud_ptr_;
  PointCloudPtr transformed_cloud_ptr_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    
  // trans
  tf::Transform pose_, pre_pose_, diff_pose_;
  Eigen::Matrix4f t_;
  // time
  std::chrono::steady_clock::time_point ndt_current_time;
  std::chrono::steady_clock::time_point ndt_previous_time;
  // param
  double fitness_score = 0.0;
  int num_iteration = 0;
  bool convergence = false;
  double trans_probability = 0.0;
  // const
  const double voxel_leaf_size_ = 0.2;
  const double trans_epsilon_ = 0.01;
  const double step_size_ = 0.1;
  const double ndt_resolution_ = 1.0;
  const int max_iter_ = 15;
};
}  // namespace calibration
#include "impl/ndt_matching.hpp"
#endif  // PERCPTION_CALIBRATION_NDTMATCHING_H_
