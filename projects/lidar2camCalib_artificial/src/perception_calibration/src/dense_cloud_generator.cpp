#include "dense_cloud_generator.h"

namespace calibration {

template <typename PointT>
DenseCloudGenerator<PointT>::DenseCloudGenerator()
  : matching_cloud_ptr_(boost::make_shared<PointCloud>()),
    original_cloud_ptr_(boost::make_shared<PointCloud>()),
    source_cloud_ptr_(boost::make_shared<PointCloud>()),
    target_cloud_ptr_(boost::make_shared<PointCloud>()),
    t_(Eigen::Matrix4f::Identity()),
    target_cloud_init_(false) {}

template <typename PointT>
DenseCloudGenerator<PointT>::~DenseCloudGenerator() {}

template <typename PointT>
void DenseCloudGenerator<PointT>::setMatchingCloudPtr(
  const PointCloudPtr& input_cloud_ptr) {
  matching_cloud_ptr_ = input_cloud_ptr;
  VoxelGridFilter(source_cloud_ptr_, matching_cloud_ptr_, 0.5);
  VoxelGridFilter(target_cloud_ptr_, matching_cloud_ptr_, 0.2);
}

template <typename PointT>
void DenseCloudGenerator<PointT>::setOriginalCloudPtr(
  const PointCloudPtr& input_cloud_ptr) {
  original_cloud_ptr_ = input_cloud_ptr;
}

template <typename PointT>
auto DenseCloudGenerator<PointT>::getDensedCloudPtr() const -> PointCloudPtr {
  return densed_cloud_ptr_;
}

template <typename PointT>
auto DenseCloudGenerator<PointT>::getTransformation() const -> Eigen::Matrix4f {
  return t_;
}

template <typename PointT>
bool DenseCloudGenerator<PointT>::Start() {
  if (target_cloud_init_ == false) {
    t_ = Eigen::Matrix4f::Identity();
    target_cloud_buffer_.push_back(matching_cloud_ptr_);
    target_cloud_init_ = true;
    return false;
  }
  PointCloudPtr total_target_cloud_ptr = boost::make_shared<PointCloud>();
  for (auto cloud : target_cloud_buffer_) {
    *total_target_cloud_ptr += *cloud;
  }
  ndt_.setInputCloud(total_target_cloud_ptr, source_cloud_ptr_);
  if (ndt_.getTransformationProbability() < 1.5 || ndt_.getFinalNumIteration() > 10) {
    this->reset();
    return false;
  }
  t_ = ndt_.getFinalTransformation();
  PointCloudPtr trans_target_cloud_ptr = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*target_cloud_ptr_, *trans_target_cloud_ptr, t_);
  if (target_cloud_buffer_.size() < 3) {
    target_cloud_buffer_.push_back(trans_target_cloud_ptr);
  } else {
    target_cloud_buffer_.pop_front();
    target_cloud_buffer_.push_back(trans_target_cloud_ptr);
  }
  // for debug
  // densed_cloud_ptr_.reset(new PointCloud());
  // for (const auto& cloud : target_cloud_buffer_) {
  //   *densed_cloud_ptr_ += *cloud;
  // }
  // static ros::NodeHandle private_nh("~");
  // static ros::Publisher pub =
  //   private_nh.advertise<sensor_msgs::PointCloud2>("densed_cloud", 10);
  // densed_cloud_ptr_->header.frame_id = "base_link";
  // densed_cloud_ptr_->height = 1;
  // densed_cloud_ptr_->width = densed_cloud_ptr_->size();
  // pub.publish(densed_cloud_ptr_);
  return true;
}

template <typename PointT>
bool DenseCloudGenerator<PointT>::GenerateDenseCloud() {
  PointCloudPtr trans_original_cloud_ptr = boost::make_shared<PointCloud>();
  pcl::transformPointCloud(*original_cloud_ptr_, *trans_original_cloud_ptr, t_);
  if (densed_cloud_buffer_.size() < 20) {
    densed_cloud_buffer_.push_back(trans_original_cloud_ptr);
    return false;
  } else {
    densed_cloud_buffer_.pop_front();
    auto it = t_.inverse() * Eigen::Matrix4f::Identity();
    ego_densed_cloud_ptr_.reset(new PointCloud());
    for (size_t i = 0; i < densed_cloud_buffer_.size(); ++i) {
      PointCloudPtr ego_cloud_ptr = boost::make_shared<PointCloud>();
      pcl::transformPointCloud(*(densed_cloud_buffer_[i]), *ego_cloud_ptr, it);
      *ego_densed_cloud_ptr_ += *ego_cloud_ptr;
    }
    *ego_densed_cloud_ptr_ += *original_cloud_ptr_;
    ego_densed_cloud_ptr_->header = original_cloud_ptr_->header;
    ego_densed_cloud_ptr_->height = 1;
    ego_densed_cloud_ptr_->width = ego_densed_cloud_ptr_->size();
    densed_cloud_buffer_.push_back(trans_original_cloud_ptr);
  }
  return true;
}

template <typename PointT>
void DenseCloudGenerator<PointT>::VoxelGridFilter(
  PointCloudPtr& filtered_cloud_ptr,
  const PointCloudPtr& input_cloud_ptr,
  double voxel_leaf_size) const {
  if (nullptr == input_cloud_ptr || input_cloud_ptr->empty()) {
    return;
  }
  if (nullptr == filtered_cloud_ptr) {
    filtered_cloud_ptr = boost::make_shared<PointCloud>();
    filtered_cloud_ptr->header = input_cloud_ptr->header;
  } else {
    filtered_cloud_ptr->points.clear();
    filtered_cloud_ptr->header = input_cloud_ptr->header;
  }
  pcl::VoxelGrid<PointT> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(input_cloud_ptr);
  voxel_grid_filter.filter(*filtered_cloud_ptr);
}

template <typename PointT>
void DenseCloudGenerator<PointT>::reset() {
  ndt_.reset();
  target_cloud_buffer_.clear();
  densed_cloud_buffer_.clear();
  t_ = Eigen::Matrix4f::Identity();
  target_cloud_init_ = false;
}

template class DenseCloudGenerator<pcl::PointXYZ>;
template class DenseCloudGenerator<pcl::PointXYZI>;

}  // namespace calibration