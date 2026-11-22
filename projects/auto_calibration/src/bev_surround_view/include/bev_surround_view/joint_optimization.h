#pragma once

#include <vector>
#include <limits>

#include <opencv2/core.hpp>

#include "bev_surround_view/fisheye_auto_calibrator.h"
#include "bev_surround_view/pose_utils.h"

namespace bev_surround_view {

// Observation per shared board edge
struct JointBoardObservation {
    int rows = 0;
    int cols = 0;
    int col_sign = 1;
    std::vector<cv::Point2f> image_points_a;
    std::vector<cv::Point2f> image_points_b;
    double initial_origin_x = 0.0;
    double initial_origin_y = 0.0;
    double initial_yaw = 0.0;
};

// Fixed per-camera parameters (not optimized)
struct JointCameraFixedState {
    double center_z = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
};

// The complete joint optimization problem definition
struct JointOptimizationProblem {
    FisheyeAutoCameraConfig front_camera;
    FisheyeAutoCameraConfig left_camera;
    FisheyeAutoCameraConfig right_camera;
    FisheyeAutoCameraConfig back_camera;
    std::vector<cv::Point3f> left_object_points;
    std::vector<cv::Point2f> left_image_points;
    std::vector<cv::Point3f> right_object_points;
    std::vector<cv::Point2f> right_image_points;
    std::vector<cv::Point2f> left_rear_image_points;
    std::vector<cv::Point2f> right_rear_image_points;
    std::vector<cv::Point2f> back_image_points;
    double ground_z = 0.0;
    double sort_threshold = 0.15;
    JointBoardObservation front_left_board;
    JointBoardObservation front_right_board;
    JointBoardObservation back_right_board;
    JointBoardObservation back_left_board;
    JointBoardObservation front_middle_board;
    JointBoardObservation left_middle_board;
    JointBoardObservation right_middle_board;
    JointBoardObservation back_middle_board;
    cv::Mat front_T_c_w = cv::Mat::eye(4, 4, CV_64F);
    cv::Vec3d front_center_w = cv::Vec3d(0.0, 0.0, 0.0);
    JointCameraFixedState left_fixed_state;
    JointCameraFixedState right_fixed_state;
    JointCameraFixedState back_fixed_state;
    std::vector<double> initial_params;
    double grid_spacing_m = 0.0;
    double rotation_prior_weight = 0.0;
    double translation_prior_weight = 0.0;
    double middle_board_metric_weight = 0.0001;
    double middle_board_metric_sigma_m = 0.01;
    double board_position_prior_weight = 0.0;
    double board_yaw_prior_weight = 0.0;
};

// Optimization result with per-camera metrics
struct JointOptimizationEvalResult {
    bool valid = false;
    cv::Mat front_T_c_w = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat left_T_c_w = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat right_T_c_w = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat back_T_c_w = cv::Mat::eye(4, 4, CV_64F);
    std::vector<cv::Point3f> front_left_object_points;
    std::vector<cv::Point3f> front_right_object_points;
    std::vector<cv::Point3f> back_right_object_points;
    std::vector<cv::Point3f> back_left_object_points;
    std::vector<cv::Point3f> front_middle_object_points;
    std::vector<cv::Point3f> left_middle_object_points;
    std::vector<cv::Point3f> right_middle_object_points;
    std::vector<cv::Point3f> back_middle_object_points;
    std::vector<cv::Point3f> back_object_points;
    double total_cost = std::numeric_limits<double>::max();
    double front_rmse = -1.0;
    double left_rmse = -1.0;
    double right_rmse = -1.0;
    double back_rmse = -1.0;
};

// Parameter count
size_t expectedJointParamCount(const JointOptimizationProblem& problem);

// Board object points
std::vector<cv::Point3f> buildCanonicalBoardObjectPoints(int rows, int cols);
std::vector<cv::Point2f> applyMiddleBoardTopology(const std::vector<cv::Point2f>& ordered_image_points,
                                                  int rows, int cols, const std::string& topology_name);
std::vector<cv::Point3f> buildJointBoardObjectPoints(const JointBoardObservation& board,
                                                     double origin_x,
                                                     double origin_y,
                                                     double yaw_rad,
                                                     double spacing_m,
                                                     double ground_z);
bool setInitialJointBoardPoseFromObjectPoints(JointBoardObservation& board,
                                              const std::vector<cv::Point3f>& object_points);
bool initJointBoardObservation(int rows, int cols,
                               const std::vector<cv::Point2f>& image_points_a,
                               const std::vector<cv::Point2f>& image_points_b,
                               JointBoardObservation& board);
std::vector<cv::Point3f> buildJointBoardObjectPointsFromParams(const JointBoardObservation& board,
                                                               const std::vector<double>& params,
                                                               size_t param_offset,
                                                               double spacing_m,
                                                               double ground_z);

// Parameter packing
void appendJointBoardParams(std::vector<double>& params, const JointBoardObservation& board);
std::vector<double> packJointPoseParams(const cv::Mat& front_T_c_w,
                                        const cv::Mat& left_T_c_w,
                                        const cv::Mat& right_T_c_w,
                                        const cv::Mat& back_T_c_w,
                                        const JointOptimizationProblem& problem);
bool unpackJointPoseParams(const JointOptimizationProblem& problem,
                           const std::vector<double>& params,
                           JointOptimizationEvalResult& result);

// Ray utilities
bool imagePointToUnitRay(const FisheyeAutoCameraConfig& cam, const cv::Point2f& image_point, cv::Vec3d& ray);
bool worldPointToUnitRay(const cv::Mat& T_c_w, const cv::Point3f& world_point, cv::Vec3d& ray);
bool distortedImagePointToUnitRay(const camera::Mei& camera_model, const cv::Point2f& image_point, cv::Vec3d& ray);
bool distortedImagePointToUnitRay(const FisheyeAutoCameraConfig& cam, const cv::Point2f& image_point, cv::Vec3d& ray);

// Residual building
bool appendRayBundleResiduals(const std::vector<cv::Point3f>& object_points,
                              const std::vector<cv::Point2f>& image_points,
                              const FisheyeAutoCameraConfig& cam,
                              const cv::Mat& T_c_w,
                              std::vector<double>& residuals,
                              double& angular_sq_sum_deg, int& angular_count);
bool appendEpipolar2d2dResiduals(const std::vector<cv::Point2f>& image_points_a,
                                 const FisheyeAutoCameraConfig& camera_a,
                                 const cv::Mat& T_a_w,
                                 const std::vector<cv::Point2f>& image_points_b,
                                 const FisheyeAutoCameraConfig& camera_b,
                                 const cv::Mat& T_b_w,
                                 std::vector<double>& residuals,
                                 double& angular_sq_sum_a_deg, int& angular_count_a,
                                 double& angular_sq_sum_b_deg, int& angular_count_b);
bool appendGround3d3dResiduals(const std::vector<cv::Point2f>& image_points_a,
                               const FisheyeAutoCameraConfig& camera_a,
                               const cv::Mat& T_a_w,
                               const std::vector<cv::Point2f>& image_points_b,
                               const FisheyeAutoCameraConfig& camera_b,
                               const cv::Mat& T_b_w,
                               double ground_z,
                               std::vector<double>& residuals,
                               double& metric_sq_sum_a_m, int& metric_count_a,
                               double& metric_sq_sum_b_m, int& metric_count_b);
bool appendRayGround3d3dResiduals(const std::vector<cv::Point2f>& image_points_a,
                                  const FisheyeAutoCameraConfig& camera_a,
                                  const cv::Mat& T_a_w,
                                  const std::vector<cv::Point2f>& image_points_b,
                                  const FisheyeAutoCameraConfig& camera_b,
                                  const cv::Mat& T_b_w,
                                  double ground_z,
                                  std::vector<double>& residuals,
                                  double& metric_sq_sum_a_m, int& metric_count_a,
                                  double& metric_sq_sum_b_m, int& metric_count_b,
                                  std::vector<cv::Point3f>* object_points_out);
bool appendSharedBoardRawReprojectionResiduals(const JointBoardObservation& board,
                                               const FisheyeAutoCameraConfig& camera_a,
                                               const cv::Mat& T_a_w,
                                               const FisheyeAutoCameraConfig& camera_b,
                                               const cv::Mat& T_b_w,
                                               const std::vector<double>& params,
                                               size_t board_param_base,
                                               double spacing_m,
                                               double ground_z,
                                               std::vector<double>& residuals,
                                               double& sq_sum_a_px, int& count_a,
                                               double& sq_sum_b_px, int& count_b,
                                               std::vector<cv::Point3f>* object_points_out);

// Middle board & prior residuals
bool appendMiddleBoardMetricResiduals(const JointBoardObservation& board,
                                      const FisheyeAutoCameraConfig& camera,
                                      const cv::Mat& T_c_w,
                                      double ground_z,
                                      double cell_size_m,
                                      double weight,
                                      double sigma_m,
                                      std::vector<double>& residuals);
bool appendMiddleBoardMetricResiduals(const JointOptimizationProblem& problem,
                                      const JointOptimizationEvalResult& eval,
                                      std::vector<double>& residuals);
void appendJointPriorResiduals(const JointOptimizationProblem& problem,
                               const std::vector<double>& params,
                               std::vector<double>& residuals);

// Cost/error
double residualCost(const std::vector<double>& residuals);
cv::Vec3d point3fToVec3d(const cv::Point3f& point);
void appendMetricResidual(double value_m, double sigma_m, double weight_sqrt, std::vector<double>& residuals);
bool computePoseMetrics(const std::vector<cv::Point3f>& object_points,
                        const std::vector<cv::Point2f>& image_points,
                        const FisheyeAutoCameraConfig& cam,
                        const cv::Mat& T_c_w,
                        double& rmse_px,
                        double& mse_px);
bool accumulateReprojectionError(const std::vector<cv::Point3f>& object_points,
                                 const std::vector<cv::Point2f>& image_points,
                                 const FisheyeAutoCameraConfig& cam,
                                 const cv::Mat& T_c_w,
                                 double& total_sq_px, size_t& total_count,
                                 double* camera_sq_px, size_t* camera_count);
bool accumulateSharedBoardReprojectionError(const JointBoardObservation& board,
                                            const FisheyeAutoCameraConfig& camera_a,
                                            const cv::Mat& T_a_w,
                                            const FisheyeAutoCameraConfig& camera_b,
                                            const cv::Mat& T_b_w,
                                            double ground_z,
                                            double& total_sq_px, size_t& total_count,
                                            double* camera_a_sq_px, size_t* camera_a_count,
                                            double* camera_b_sq_px, size_t* camera_b_count);

// RMSE computation
bool computeSharedBoardReprojectionRmse(const JointOptimizationProblem& problem,
                                        const cv::Mat& front_T_c_w,
                                        const cv::Mat& left_T_c_w,
                                        const cv::Mat& right_T_c_w,
                                        const cv::Mat& back_T_c_w,
                                        double& rmse_px,
                                        double* front_rmse_px, double* left_rmse_px,
                                        double* right_rmse_px, double* back_rmse_px);
bool computeSharedBoardRawReprojectionRmse(const JointOptimizationProblem& problem,
                                           const JointOptimizationEvalResult& eval,
                                           double& rmse_px,
                                           double* front_rmse_px, double* left_rmse_px,
                                           double* right_rmse_px, double* back_rmse_px);
bool computeFrontSharedBoardReprojectionRmse(const JointOptimizationProblem& problem,
                                             const cv::Mat& front_T_c_w,
                                             const cv::Mat& left_T_c_w,
                                             const cv::Mat& right_T_c_w,
                                             double& rmse_px,
                                             double* front_rmse_px, double* left_rmse_px,
                                             double* right_rmse_px);
std::vector<cv::Point3f> buildBackSharedObjectPoints(const JointOptimizationProblem& problem,
                                                     const cv::Mat& left_T_c_w,
                                                     const cv::Mat& right_T_c_w);

// Core optimization
bool evaluateJointOptimization(const JointOptimizationProblem& problem,
                               const std::vector<double>& params,
                               JointOptimizationEvalResult& result);
bool buildJointBundleResiduals(const JointOptimizationProblem& problem,
                               const std::vector<double>& params,
                               JointOptimizationEvalResult& eval,
                               std::vector<double>& residuals);
bool optimizeJointLeftRightBack(const JointOptimizationProblem& problem,
                                cv::Mat& front_T_c_w,
                                cv::Mat& left_T_c_w,
                                cv::Mat& right_T_c_w,
                                cv::Mat& back_T_c_w,
                                double& front_rmse,
                                double& left_rmse,
                                double& right_rmse,
                                double& back_rmse,
                                double& total_cost,
                                JointOptimizationEvalResult* final_eval);

}  // namespace bev_surround_view
