namespace calibration {
template<typename PointT>
NdtMatching<PointT>::NdtMatching()
  : target_cloud_ptr_(new PointCloud()),
    source_cloud_ptr_(new PointCloud()),
    transformed_cloud_ptr_(new PointCloud()),
    t_(Eigen::Matrix4f::Identity()) {
  ndt_.setTransformationEpsilon(trans_epsilon_);
  ndt_.setStepSize(step_size_);
  ndt_.setResolution(ndt_resolution_);
  ndt_.setMaximumIterations(max_iter_);
  InitPose();
  ndt_previous_time = std::chrono::steady_clock::now();
  ndt_current_time = std::chrono::steady_clock::now();
}

template<typename PointT>
NdtMatching<PointT>::~NdtMatching() {}

template<typename PointT>
void NdtMatching<PointT>::setInputCloud(
  const PointCloudPtr& input_target_cloud_ptr,
  const PointCloudPtr& input_source_cloud_ptr) {
  ndt_previous_time = ndt_current_time;
  ndt_current_time = std::chrono::steady_clock::now();
  AlignProcess(input_target_cloud_ptr, input_source_cloud_ptr);
  target_cloud_ptr_ = input_target_cloud_ptr;
  source_cloud_ptr_ = input_source_cloud_ptr;
  // debug
  // std::cout << *this << std::endl;
}

template<typename PointT>
void NdtMatching<PointT>::AlignProcess(const PointCloudPtr& target_cloud_ptr,
                                       const PointCloudPtr& source_cloud_ptr) {
  ndt_.setInputTarget(target_cloud_ptr);
  ndt_.setInputSource(source_cloud_ptr);
  tf::Transform init_guess_tf = diff_pose_ * pre_pose_;
  Eigen::Matrix4f init_guess_eigen;
  ConvertTf2Eigen(init_guess_tf, init_guess_eigen);
  transformed_cloud_ptr_.reset(new PointCloud());
  #ifndef USING_GPU_NDT_
    ndt_.align(*transformed_cloud_ptr_, init_guess_eigen);
  #else
    ndt_.align(init_guess_eigen);
  #endif
  // ndt_.align(*transformed_cloud_ptr_, init_guess_eigen); ///////////////////////////
  t_ = ndt_.getFinalTransformation();
  fitness_score = ndt_.getFitnessScore();
  num_iteration = ndt_.getFinalNumIteration();
  convergence = ndt_.hasConverged();
  trans_probability = ndt_.getTransformationProbability();
  pose_.setBasis(tf::Matrix3x3(
                   t_(0, 0), t_(0, 1), t_(0, 2),
                   t_(1, 0), t_(1, 1), t_(1, 2),
                   t_(2, 0), t_(2, 1), t_(2, 2)));
  pose_.setOrigin(tf::Vector3(
                    t_(0, 3), t_(1, 3), t_(2, 3)));
  diff_pose_ = pose_ * pre_pose_.inverse();
  pre_pose_ = pose_;
}

template<typename PointT>
void NdtMatching<PointT>::ConvertTf2Eigen(const tf::Transform& tf, Eigen::Matrix4f& m4f) {
  m4f(0, 0) = tf.getBasis()[0][0];
  m4f(0, 1) = tf.getBasis()[0][1];
  m4f(0, 2) = tf.getBasis()[0][2];
  m4f(0, 3) = tf.getOrigin()[0];
  m4f(1, 0) = tf.getBasis()[1][0];
  m4f(1, 1) = tf.getBasis()[1][1];
  m4f(1, 2) = tf.getBasis()[1][2];
  m4f(1, 3) = tf.getOrigin()[1];
  m4f(2, 0) = tf.getBasis()[2][0];
  m4f(2, 1) = tf.getBasis()[2][1];
  m4f(2, 2) = tf.getBasis()[2][2];
  m4f(2, 3) = tf.getOrigin()[2];
  m4f(3, 0) = 0;
  m4f(3, 1) = 0;
  m4f(3, 2) = 0;
  m4f(3, 3) = 1;
}

template<typename PointT>
void NdtMatching<PointT>::setTransformationEpsilon(const double trans_epsilon) {
  ndt_.setTransformationEpsilon(trans_epsilon);
}

template<typename PointT>
void NdtMatching<PointT>::setStepSize(const double step_size) {
  ndt_.setStepSize(step_size);
}

template<typename PointT>
void NdtMatching<PointT>::setResolution(const double resolution) {
  ndt_.setResolution(resolution);
}

template<typename PointT>
void NdtMatching<PointT>::setMaximumIterations(const int max_iter) {
  ndt_.setMaximumIterations(max_iter);
}

template<typename PointT>
auto NdtMatching<PointT>::getTransformedCloud() const -> PointCloudPtr {
  return transformed_cloud_ptr_;
}

template<typename PointT>
Eigen::Matrix4f NdtMatching<PointT>::getFinalTransformation() const {
  return t_;
}

template<typename PointT>
double NdtMatching<PointT>::getFitnessScore() const {
  return fitness_score;
}

template<typename PointT>
int NdtMatching<PointT>::getFinalNumIteration() const {
  return num_iteration;
}

template<typename PointT>
double NdtMatching<PointT>::getTransformationProbability() const {
  return trans_probability;
}

template<typename PointT>
void NdtMatching<PointT>::reset() {
  InitPose();
  target_cloud_ptr_.reset(new PointCloud());
  source_cloud_ptr_.reset(new PointCloud());
  ndt_previous_time = std::chrono::steady_clock::now();
  ndt_current_time = std::chrono::steady_clock::now();
}

template<typename PointT>
void NdtMatching<PointT>::InitPose() {
  pose_.getBasis().setIdentity();
  pose_.getOrigin().setZero();
  pre_pose_.getBasis().setIdentity();
  pre_pose_.getOrigin().setZero();
  diff_pose_.getBasis().setIdentity();
  diff_pose_.getOrigin().setZero();
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const NdtMatching<T>& ndt) {
  double roll, pitch, yaw;
  ndt.pose_.getBasis().getRPY(roll, pitch, yaw);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Number of Target Points: " << ndt.target_cloud_ptr_->size() << " points." << std::endl;
  std::cout << "Number of Source Points: " << ndt.source_cloud_ptr_->size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt.convergence << std::endl;
  std::cout << "Fitness Score: " << ndt.fitness_score << std::endl;
  std::cout << "Transformation Probability: " << ndt.trans_probability << std::endl;
  std::cout << "Number of Iterations: " << ndt.num_iteration << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
  std::cout << "(" << ndt.pose_.getOrigin()[0] << ", " << ndt.pose_.getOrigin()[1] << ", " << ndt.pose_.getOrigin()[2] << ", "
            << angles::to_degrees(roll) << ", " << angles::to_degrees(pitch) << ", " << angles::to_degrees(yaw) << ")" << std::endl;
  std::cout << "Transformation Matrix: " << std::endl;
  std::cout << ndt.t_ << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}
} // namespace calibration
