#ifndef PERCPTION_CALIBRATION_DENSE_CLOUD_GENERATOR_H_
#define PERCPTION_CALIBRATION_DENSE_CLOUD_GENERATOR_H_

#include <deque>

#include "ndt_matching.h"



namespace calibration {
template<typename PointT>
class DenseCloudGenerator {
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

public:
  DenseCloudGenerator();
  ~DenseCloudGenerator();

  void setMatchingCloudPtr(const PointCloudPtr&);
  void setOriginalCloudPtr(const PointCloudPtr&);
  PointCloudPtr getDensedCloudPtr() const;
  Eigen::Matrix4f getTransformation() const;

  bool Start();
  bool GenerateDenseCloud();

  void reset();

private:
  void VoxelGridFilter(PointCloudPtr& filtered_cloud_ptr,
                       const PointCloudPtr& input_cloud_ptr,
                       double voxel_leaf_size) const;

private:
  PointCloudPtr matching_cloud_ptr_;
  PointCloudPtr source_cloud_ptr_;
  PointCloudPtr target_cloud_ptr_;
  PointCloudPtr original_cloud_ptr_;
  PointCloudPtr densed_cloud_ptr_;
  PointCloudPtr ego_densed_cloud_ptr_;
  std::deque<PointCloudPtr> target_cloud_buffer_;
  std::deque<PointCloudPtr> densed_cloud_buffer_;
  NdtMatching<PointT> ndt_;

  Eigen::Matrix4f t_;
  bool target_cloud_init_;
};
}  // namespace calibration
#endif // PERCPTION_CALIBRATION_DENSE_CLOUD_GENERATOR_H_