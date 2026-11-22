#include "camera_model/mei.h"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>

namespace camera {

namespace {

#define MEI_LOCK_GUARD() std::lock_guard<std::recursive_mutex> lock(mutex_)

constexpr double kMinDenom = 1e-9;
constexpr double kMinRadial = 1e-12;
constexpr int kUndistortIterations = 8;
constexpr double kCompareTolerance = 1e-12;

struct DistortionCoeffs {
    double k1;
    double k2;
    double p1;
    double p2;
};

cv::Mat defaultK()
{
    return cv::Mat::eye(3, 3, CV_64F);
}

cv::Mat defaultD()
{
    return cv::Mat::zeros(1, 4, CV_64F);
}

cv::Mat normalizeCameraMatrixImpl(const cv::Mat& K)
{
    cv::Mat out = defaultK();
    if (K.empty()) {
        return out;
    }

    if (K.rows == 3 && K.cols == 3) {
        K.convertTo(out, CV_64F);
        return out;
    }

    const cv::Mat flat = K.reshape(1, 1);
    if (flat.total() < 9) {
        return out;
    }

    cv::Mat flat64;
    flat.convertTo(flat64, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.at<double>(r, c) = flat64.at<double>(0, r * 3 + c);
        }
    }
    return out;
}

cv::Mat normalizeDistCoeffsImpl(const cv::Mat& D)
{
    cv::Mat out = defaultD();
    if (D.empty()) {
        return out;
    }

    const cv::Mat flat = D.reshape(1, 1);
    cv::Mat flat64;
    flat.convertTo(flat64, CV_64F);
    const int count = static_cast<int>(std::min<std::size_t>(4, flat64.total()));
    for (int i = 0; i < count; ++i) {
        out.at<double>(0, i) = flat64.at<double>(0, i);
    }
    return out;
}

void releaseMaps(cv::Mat* map1, cv::Mat* map2)
{
    if (map1 != nullptr) {
        map1->release();
    }
    if (map2 != nullptr) {
        map2->release();
    }
}

bool isFinitePoint3(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x()) &&
           std::isfinite(point.y()) &&
           std::isfinite(point.z());
}

bool isFinitePoint2(const cv::Point2d& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool isSameDouble(double lhs, double rhs)
{
    return std::abs(lhs - rhs) <= kCompareTolerance;
}

bool isSameMat(const cv::Mat& lhs, const cv::Mat& rhs)
{
    if (lhs.empty() && rhs.empty()) {
        return true;
    }
    if (lhs.empty() || rhs.empty()) {
        return false;
    }
    if (lhs.rows != rhs.rows || lhs.cols != rhs.cols || lhs.type() != rhs.type()) {
        return false;
    }
    return cv::norm(lhs, rhs, cv::NORM_INF) <= kCompareTolerance;
}

bool isSameVector3(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
    return (lhs - rhs).cwiseAbs().maxCoeff() <= kCompareTolerance;
}

bool isSameMatrix3(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs)
{
    return (lhs - rhs).cwiseAbs().maxCoeff() <= kCompareTolerance;
}

bool isValidBevSourcePixel(const cv::Point2d& point, int image_width, int image_height)
{
    if (!isFinitePoint2(point)) {
        return false;
    }
    if (image_width <= 0 || image_height <= 0) {
        return point.x > 0.0 && point.y > 0.0;
    }
    return point.x > 0.0 && point.x < static_cast<double>(image_width) &&
           point.y > 0.0 && point.y < static_cast<double>(image_height);
}

bool isInIpmRoi(int x,
                int y,
                int roi_top,
                int roi_bottom,
                int roi_left,
                int roi_right)
{
    if (roi_top > 0 && y < roi_top) {
        return false;
    }
    if (roi_bottom > 0 && y >= roi_bottom) {
        return false;
    }
    if (roi_left > 0 && x < roi_left) {
        return false;
    }
    if (roi_right > 0 && x >= roi_right) {
        return false;
    }
    return true;
}

Eigen::Matrix3d initRotWorldToCamera()
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    rotation(0, 1) = -1.0;
    rotation(1, 2) = -1.0;
    rotation(2, 0) = 1.0;
    return rotation;
}

DistortionCoeffs readDistortionCoeffs(const cv::Mat& D)
{
    DistortionCoeffs coeffs;
    coeffs.k1 = D.at<double>(0, 0);
    coeffs.k2 = D.at<double>(0, 1);
    coeffs.p1 = D.at<double>(0, 2);
    coeffs.p2 = D.at<double>(0, 3);
    return coeffs;
}

void readIntrinsics(const cv::Mat& K,
                    double& fx,
                    double& fy,
                    double& cx,
                    double& cy)
{
    fx = K.at<double>(0, 0);
    fy = K.at<double>(1, 1);
    cx = K.at<double>(0, 2);
    cy = K.at<double>(1, 2);
}

bool projectUndistortedRay(double epsilon,
                           double xu,
                           double yu,
                           double& xp,
                           double& yp)
{
    if (epsilon < 1e-3) {
        return false;
    }

    const double rho2 = xu * xu + yu * yu;
    const double sqrt_arg = 1.0 + (1.0 - epsilon * epsilon) * rho2;
    if (sqrt_arg <= 0.0) {
        return false;
    }

    const double lambda = (epsilon + std::sqrt(sqrt_arg)) / (rho2 + 1.0);
    const double zs = lambda - epsilon;
    if (std::abs(zs) < kMinRadial) {
        return false;
    }

    xp = lambda * xu / zs;
    yp = lambda * yu / zs;
    return std::isfinite(xp) && std::isfinite(yp);
}

Eigen::Vector3d makeIpmWorldPoint(int x,
                                  int y,
                                  int width,
                                  int height,
                                  double scale,
                                  double ground_height,
                                  double origin_offset_x,
                                  double origin_offset_y)
{
    const double origin_y_px = 0.5 * static_cast<double>(height) - origin_offset_x;
    const double origin_x_px = 0.5 * static_cast<double>(width) - origin_offset_y;
    const double world_x = (origin_y_px - static_cast<double>(y)) / scale;
    const double world_y = (origin_x_px - static_cast<double>(x)) / scale;
    return Eigen::Vector3d(world_x, world_y, ground_height);
}

}  // namespace

UndistortInfo::UndistortInfo()
    : width_(0),
      height_(0),
      K_(defaultK())
{}

void UndistortInfo::setInfo(int width,
                            int height,
                            const cv::Mat& undistort_K)
{
    const cv::Mat normalized_K = normalizeCameraMatrixImpl(undistort_K);
    if (sameInfo(width, height, normalized_K)) {
        return;
    }

    width_ = width;
    height_ = height;
    K_ = normalized_K;
    clearMap();
}

int UndistortInfo::getWidth() const
{
    return width_;
}

int UndistortInfo::getHeight() const
{
    return height_;
}

cv::Mat UndistortInfo::getK() const
{
    return K_.clone();
}

bool UndistortInfo::isReady() const
{
    return width_ > 0 && height_ > 0 && !K_.empty();
}

void UndistortInfo::clearMap()
{
    map1_.release();
    map2_.release();
}

bool UndistortInfo::sameInfo(int width,
                             int height,
                             const cv::Mat& undistort_K) const
{
    return width_ == width &&
           height_ == height &&
           isSameMat(K_, undistort_K);
}

bool UndistortInfo::hasMap() const
{
    return !map1_.empty() && !map2_.empty();
}

void UndistortInfo::updateMap(const cv::Mat& map1,
                              const cv::Mat& map2)
{
    map1_ = map1.clone();
    map2_ = map2.clone();
}

void UndistortInfo::copyMap(cv::Mat* map1,
                            cv::Mat* map2) const
{
    if (map1 != nullptr) {
        *map1 = map1_.clone();
    }
    if (map2 != nullptr) {
        *map2 = map2_.clone();
    }
}

IpmInfo::IpmInfo()
    : width_(0),
      height_(0),
      scale_(0.0),
      origin_offset_x_(0.0),
      origin_offset_y_(0.0),
      roi_top_(0),
      roi_bottom_(0),
      roi_left_(0),
      roi_right_(0)
{}

void IpmInfo::setInfo(int width,
                      int height,
                      double scale,
                      double origin_offset_x,
                      double origin_offset_y,
                      int roi_top,
                      int roi_bottom,
                      int roi_left,
                      int roi_right)
{
    if (sameInfo(width,
                 height,
                 scale,
                 origin_offset_x,
                 origin_offset_y,
                 roi_top,
                 roi_bottom,
                 roi_left,
                 roi_right)) {
        return;
    }

    width_ = width;
    height_ = height;
    scale_ = scale;
    origin_offset_x_ = origin_offset_x;
    origin_offset_y_ = origin_offset_y;
    roi_top_ = roi_top;
    roi_bottom_ = roi_bottom;
    roi_left_ = roi_left;
    roi_right_ = roi_right;
    clearMap();
}

int IpmInfo::getWidth() const
{
    return width_;
}

int IpmInfo::getHeight() const
{
    return height_;
}

double IpmInfo::getScale() const
{
    return scale_;
}

double IpmInfo::getOriginOffsetX() const
{
    return origin_offset_x_;
}

double IpmInfo::getOriginOffsetY() const
{
    return origin_offset_y_;
}

int IpmInfo::getRoiTop() const
{
    return roi_top_;
}

int IpmInfo::getRoiBottom() const
{
    return roi_bottom_;
}

int IpmInfo::getRoiLeft() const
{
    return roi_left_;
}

int IpmInfo::getRoiRight() const
{
    return roi_right_;
}

bool IpmInfo::isReady() const
{
    return width_ > 0 && height_ > 0 && scale_ > 0.0;
}

void IpmInfo::clearMap()
{
    map_x_.release();
    map_y_.release();
}

bool IpmInfo::sameInfo(int width,
                       int height,
                       double scale,
                       double origin_offset_x,
                       double origin_offset_y,
                       int roi_top,
                       int roi_bottom,
                       int roi_left,
                       int roi_right) const
{
    return width_ == width &&
           height_ == height &&
           isSameDouble(scale_, scale) &&
           isSameDouble(origin_offset_x_, origin_offset_x) &&
           isSameDouble(origin_offset_y_, origin_offset_y) &&
           roi_top_ == roi_top &&
           roi_bottom_ == roi_bottom &&
           roi_left_ == roi_left &&
           roi_right_ == roi_right;
}

bool IpmInfo::hasMap() const
{
    return !map_x_.empty() && !map_y_.empty();
}

void IpmInfo::updateMap(const cv::Mat& map_x,
                        const cv::Mat& map_y)
{
    map_x_ = map_x.clone();
    map_y_ = map_y.clone();
}

void IpmInfo::copyMap(cv::Mat* map_x,
                      cv::Mat* map_y) const
{
    if (map_x != nullptr) {
        *map_x = map_x_.clone();
    }
    if (map_y != nullptr) {
        *map_y = map_y_.clone();
    }
}

Mei::Mei(const std::array<int, 3>& rotation_order)
    : epsilon_(0.0),
      K_(defaultK()),
      D_(defaultD()),
      t_world_to_camera_(Eigen::Vector3d::Zero()),
      r_world_to_camera_euler_(Eigen::Vector3d::Zero()),
      R_world_to_camera_(Eigen::Matrix3d::Identity()),
      image_width_(0),
      image_height_(0),
      ground_height_(0.0),
      rotation_order_(normalizeRotationOrder(rotation_order))
{}

Mei::Mei(const Mei& other)
    : epsilon_(0.0),
      K_(defaultK()),
      D_(defaultD()),
      t_world_to_camera_(Eigen::Vector3d::Zero()),
      r_world_to_camera_euler_(Eigen::Vector3d::Zero()),
      R_world_to_camera_(Eigen::Matrix3d::Identity()),
      image_width_(0),
      image_height_(0),
      ground_height_(0.0),
      rotation_order_(std::array<int, 3>{{2, 1, 0}})
{
    std::lock_guard<std::recursive_mutex> other_lock(other.mutex_);
    epsilon_ = other.epsilon_;
    K_ = other.K_.clone();
    D_ = other.D_.clone();
    t_world_to_camera_ = other.t_world_to_camera_;
    r_world_to_camera_euler_ = other.r_world_to_camera_euler_;
    R_world_to_camera_ = other.R_world_to_camera_;
    image_width_ = other.image_width_;
    image_height_ = other.image_height_;
    ground_height_ = other.ground_height_;
    rotation_order_ = other.rotation_order_;
    undistort_info_ = other.undistort_info_;
    ipm_info_ = other.ipm_info_;
}

Mei::Mei(Mei&& other)
    : epsilon_(0.0),
      K_(defaultK()),
      D_(defaultD()),
      t_world_to_camera_(Eigen::Vector3d::Zero()),
      r_world_to_camera_euler_(Eigen::Vector3d::Zero()),
      R_world_to_camera_(Eigen::Matrix3d::Identity()),
      image_width_(0),
      image_height_(0),
      ground_height_(0.0),
      rotation_order_(std::array<int, 3>{{2, 1, 0}})
{
    std::lock_guard<std::recursive_mutex> other_lock(other.mutex_);
    epsilon_ = other.epsilon_;
    K_ = std::move(other.K_);
    D_ = std::move(other.D_);
    t_world_to_camera_ = other.t_world_to_camera_;
    r_world_to_camera_euler_ = other.r_world_to_camera_euler_;
    R_world_to_camera_ = other.R_world_to_camera_;
    image_width_ = other.image_width_;
    image_height_ = other.image_height_;
    ground_height_ = other.ground_height_;
    rotation_order_ = other.rotation_order_;
    undistort_info_ = std::move(other.undistort_info_);
    ipm_info_ = std::move(other.ipm_info_);
}

Mei& Mei::operator=(const Mei& other)
{
    if (this == &other) {
        return *this;
    }

    std::lock(mutex_, other.mutex_);
    std::lock_guard<std::recursive_mutex> self_lock(mutex_, std::adopt_lock);
    std::lock_guard<std::recursive_mutex> other_lock(other.mutex_, std::adopt_lock);
    epsilon_ = other.epsilon_;
    K_ = other.K_.clone();
    D_ = other.D_.clone();
    t_world_to_camera_ = other.t_world_to_camera_;
    r_world_to_camera_euler_ = other.r_world_to_camera_euler_;
    R_world_to_camera_ = other.R_world_to_camera_;
    image_width_ = other.image_width_;
    image_height_ = other.image_height_;
    ground_height_ = other.ground_height_;
    rotation_order_ = other.rotation_order_;
    undistort_info_ = other.undistort_info_;
    ipm_info_ = other.ipm_info_;
    return *this;
}

Mei& Mei::operator=(Mei&& other)
{
    if (this == &other) {
        return *this;
    }

    std::lock(mutex_, other.mutex_);
    std::lock_guard<std::recursive_mutex> self_lock(mutex_, std::adopt_lock);
    std::lock_guard<std::recursive_mutex> other_lock(other.mutex_, std::adopt_lock);
    epsilon_ = other.epsilon_;
    K_ = std::move(other.K_);
    D_ = std::move(other.D_);
    t_world_to_camera_ = other.t_world_to_camera_;
    r_world_to_camera_euler_ = other.r_world_to_camera_euler_;
    R_world_to_camera_ = other.R_world_to_camera_;
    image_width_ = other.image_width_;
    image_height_ = other.image_height_;
    ground_height_ = other.ground_height_;
    rotation_order_ = other.rotation_order_;
    undistort_info_ = std::move(other.undistort_info_);
    ipm_info_ = std::move(other.ipm_info_);
    return *this;
}

Mei::Mei(double epsilon,
         const cv::Mat& K,
         const cv::Mat& D,
         int image_width,
         int image_height,
         double ground_height,
         const Eigen::Vector3d& t_world_to_camera,
         const Eigen::Vector3d& r_world_to_camera_euler,
         const std::array<int, 3>& rotation_order)
    : epsilon_(epsilon),
      K_(normalizeK(K)),
      D_(normalizeD(D)),
      t_world_to_camera_(t_world_to_camera),
      r_world_to_camera_euler_(r_world_to_camera_euler),
      R_world_to_camera_(Eigen::Matrix3d::Identity()),
      image_width_(image_width),
      image_height_(image_height),
      ground_height_(ground_height),
      rotation_order_(normalizeRotationOrder(rotation_order))
{
    R_world_to_camera_ = eulerToRotation(r_world_to_camera_euler_);
}

Mei::Mei(double epsilon,
         const cv::Mat& K,
         const cv::Mat& D,
         int image_width,
         int image_height,
         double ground_height,
         const Eigen::Vector3d& t_world_to_camera,
         const Eigen::Matrix3d& R_world_to_camera,
         const std::array<int, 3>& rotation_order)
    : epsilon_(epsilon),
      K_(normalizeK(K)),
      D_(normalizeD(D)),
      t_world_to_camera_(t_world_to_camera),
      r_world_to_camera_euler_(Eigen::Vector3d::Zero()),
      R_world_to_camera_(R_world_to_camera),
      image_width_(image_width),
      image_height_(image_height),
      ground_height_(ground_height),
      rotation_order_(normalizeRotationOrder(rotation_order))
{
    r_world_to_camera_euler_ = rotationToEuler(R_world_to_camera_);
}

void Mei::setEpsilon(double epsilon)
{
    MEI_LOCK_GUARD();
    if (isSameDouble(epsilon_, epsilon)) {
        return;
    }
    epsilon_ = epsilon;
    undistort_info_.clearMap();
    ipm_info_.clearMap();
}

double Mei::getEpsilon() const
{
    MEI_LOCK_GUARD();
    return epsilon_;
}

void Mei::setImageWidth(int image_width)
{
    MEI_LOCK_GUARD();
    if (image_width_ == image_width) {
        return;
    }
    image_width_ = image_width;
    ipm_info_.clearMap();
}

int Mei::getImageWidth() const
{
    MEI_LOCK_GUARD();
    return image_width_;
}

void Mei::setImageHeight(int image_height)
{
    MEI_LOCK_GUARD();
    if (image_height_ == image_height) {
        return;
    }
    image_height_ = image_height;
    ipm_info_.clearMap();
}

int Mei::getImageHeight() const
{
    MEI_LOCK_GUARD();
    return image_height_;
}

void Mei::setGroundHeight(double ground_height)
{
    MEI_LOCK_GUARD();
    if (isSameDouble(ground_height_, ground_height)) {
        return;
    }
    ground_height_ = ground_height;
    ipm_info_.clearMap();
}

double Mei::getGroundHeight() const
{
    MEI_LOCK_GUARD();
    return ground_height_;
}

void Mei::setK(const cv::Mat& K)
{
    MEI_LOCK_GUARD();
    const cv::Mat normalized_K = normalizeK(K);
    if (isSameMat(K_, normalized_K)) {
        return;
    }
    K_ = normalized_K;
    undistort_info_.clearMap();
    ipm_info_.clearMap();
}

cv::Mat Mei::getK() const
{
    MEI_LOCK_GUARD();
    return K_.clone();
}

void Mei::setD(const cv::Mat& D)
{
    MEI_LOCK_GUARD();
    const cv::Mat normalized_D = normalizeD(D);
    if (isSameMat(D_, normalized_D)) {
        return;
    }
    D_ = normalized_D;
    undistort_info_.clearMap();
    ipm_info_.clearMap();
}

cv::Mat Mei::getD() const
{
    MEI_LOCK_GUARD();
    return D_.clone();
}

void Mei::setT(const Eigen::Vector3d& t_world_to_camera)
{
    MEI_LOCK_GUARD();
    if (isSameVector3(t_world_to_camera_, t_world_to_camera)) {
        return;
    }
    t_world_to_camera_ = t_world_to_camera;
    ipm_info_.clearMap();
}

Eigen::Vector3d Mei::getT() const
{
    MEI_LOCK_GUARD();
    return t_world_to_camera_;
}

void Mei::setR(const Eigen::Vector3d& r_world_to_camera_euler)
{
    MEI_LOCK_GUARD();
    if (isSameVector3(r_world_to_camera_euler_, r_world_to_camera_euler)) {
        return;
    }
    r_world_to_camera_euler_ = r_world_to_camera_euler;
    R_world_to_camera_ = eulerToRotation(r_world_to_camera_euler_);
    ipm_info_.clearMap();
}

void Mei::setR(const Eigen::Matrix3d& R_world_to_camera)
{
    MEI_LOCK_GUARD();
    if (isSameMatrix3(R_world_to_camera_, R_world_to_camera)) {
        return;
    }
    R_world_to_camera_ = R_world_to_camera;
    r_world_to_camera_euler_ = rotationToEuler(R_world_to_camera_);
    ipm_info_.clearMap();
}

void Mei::setWorldToCameraR(const Eigen::Matrix3d& R_world_to_camera)
{
    MEI_LOCK_GUARD();
    static const Eigen::Matrix3d kResolveT = initRotWorldToCamera().transpose();
    setR(static_cast<Eigen::Matrix3d>(R_world_to_camera * kResolveT));
}

Eigen::Matrix3d Mei::getWorldToCameraR() const
{
    MEI_LOCK_GUARD();
    static const Eigen::Matrix3d kResolve = initRotWorldToCamera();
    return R_world_to_camera_ * kResolve;
}

Eigen::Matrix3d Mei::getR() const
{
    MEI_LOCK_GUARD();
    return R_world_to_camera_;
}

Eigen::Vector3d Mei::getREuler() const
{
    MEI_LOCK_GUARD();
    return r_world_to_camera_euler_;
}

void Mei::setIntrinsics(double epsilon,
                        const cv::Mat& K,
                        const cv::Mat& D,
                        int image_width,
                        int image_height)
{
    MEI_LOCK_GUARD();
    const cv::Mat normalized_K = normalizeK(K);
    const cv::Mat normalized_D = normalizeD(D);
    if (isSameDouble(epsilon_, epsilon) &&
        isSameMat(K_, normalized_K) &&
        isSameMat(D_, normalized_D) &&
        image_width_ == image_width &&
        image_height_ == image_height) {
        return;
    }
    epsilon_ = epsilon;
    K_ = normalized_K;
    D_ = normalized_D;
    image_width_ = image_width;
    image_height_ = image_height;
    undistort_info_.clearMap();
    ipm_info_.clearMap();
}

void Mei::setExtrinsics(const Eigen::Vector3d& t_world_to_camera,
                        const Eigen::Vector3d& r_world_to_camera_euler)
{
    MEI_LOCK_GUARD();
    if (isSameVector3(t_world_to_camera_, t_world_to_camera) &&
        isSameVector3(r_world_to_camera_euler_, r_world_to_camera_euler)) {
        return;
    }
    t_world_to_camera_ = t_world_to_camera;
    r_world_to_camera_euler_ = r_world_to_camera_euler;
    R_world_to_camera_ = eulerToRotation(r_world_to_camera_euler_);
    ipm_info_.clearMap();
}

void Mei::setExtrinsics(const Eigen::Vector3d& t_world_to_camera,
                        const Eigen::Matrix3d& R_world_to_camera)
{
    MEI_LOCK_GUARD();
    if (isSameVector3(t_world_to_camera_, t_world_to_camera) &&
        isSameMatrix3(R_world_to_camera_, R_world_to_camera)) {
        return;
    }
    t_world_to_camera_ = t_world_to_camera;
    R_world_to_camera_ = R_world_to_camera;
    r_world_to_camera_euler_ = rotationToEuler(R_world_to_camera_);
    ipm_info_.clearMap();
}
void Mei::setUndistortInfo(int width,
                           int height,
                           const cv::Mat& undistort_K)
{
    MEI_LOCK_GUARD();
    undistort_info_.setInfo(width, height, undistort_K);
}

UndistortInfo Mei::getUndistortInfo() const
{
    MEI_LOCK_GUARD();
    return undistort_info_;
}

void Mei::setIpmInfo(int width,
                     int height,
                     double scale,
                     double origin_offset_x,
                     double origin_offset_y,
                     int roi_top,
                     int roi_bottom,
                     int roi_left,
                     int roi_right)
{
    MEI_LOCK_GUARD();
    const int normalized_roi_top = std::max(0, std::min(roi_top, height));
    const int normalized_roi_bottom = std::max(0, std::min(roi_bottom, height));
    const int normalized_roi_left = std::max(0, std::min(roi_left, width));
    const int normalized_roi_right = std::max(0, std::min(roi_right, width));
    ipm_info_.setInfo(width,
                      height,
                      scale,
                      origin_offset_x,
                      origin_offset_y,
                      normalized_roi_top,
                      normalized_roi_bottom,
                      normalized_roi_left,
                      normalized_roi_right);
}

IpmInfo Mei::getIpmInfo() const
{
    MEI_LOCK_GUARD();
    return ipm_info_;
}

bool Mei::world3dToCamera3d(const Eigen::Vector3d& point_world,
                            Eigen::Vector3d& point_camera) const
{
    MEI_LOCK_GUARD();
    static const Eigen::Matrix3d kInitRotWorldToCamera = initRotWorldToCamera();
    point_camera = R_world_to_camera_ * (kInitRotWorldToCamera * point_world + t_world_to_camera_);
    return isFinitePoint3(point_camera);
}

bool Mei::camera3dToWorld3d(const Eigen::Vector3d& point_camera,
                            Eigen::Vector3d& point_world) const
{
    MEI_LOCK_GUARD();
    static const Eigen::Matrix3d kInitRotCameraToWorld = initRotWorldToCamera().transpose();
    point_world = kInitRotCameraToWorld * (R_world_to_camera_.transpose() * point_camera - t_world_to_camera_);
    return isFinitePoint3(point_world);
}

bool Mei::projectCamera3dTo2d(const Eigen::Vector3d& point_camera,
                              cv::Point2d& point_2d) const
{
    MEI_LOCK_GUARD();
    const double x = point_camera.x();
    const double y = point_camera.y();
    const double z = point_camera.z();
    if (z <= kMinDenom) {
        return false;
    }

    const double norm = point_camera.norm();
    if (!std::isfinite(norm)) {
        return false;
    }

    const double denom = z + epsilon_ * norm;
    if (std::abs(denom) <= kMinDenom) {
        return false;
    }

    const double xu = x / denom;
    const double yu = y / denom;
    if (!std::isfinite(xu) || !std::isfinite(yu)) {
        return false;
    }

    const DistortionCoeffs distortion = readDistortionCoeffs(D_);
    const double r2 = xu * xu + yu * yu;
    const double r4 = r2 * r2;
    const double radial = 1.0 + distortion.k1 * r2 + distortion.k2 * r4;
    const double xd = xu * radial + 2.0 * distortion.p1 * xu * yu +
                      distortion.p2 * (r2 + 2.0 * xu * xu);
    const double yd = yu * radial + distortion.p1 * (r2 + 2.0 * yu * yu) +
                      2.0 * distortion.p2 * xu * yu;
    if (!std::isfinite(xd) || !std::isfinite(yd)) {
        return false;
    }

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    readIntrinsics(K_, fx, fy, cx, cy);

    point_2d.x = fx * xd + cx;
    point_2d.y = fy * yd + cy;
    return isFinitePoint2(point_2d);
}

bool Mei::projectCamera2dToCamera3d(const cv::Point2d& point_2d,
                                    const double z,
                                    Eigen::Vector3d& point_camera) const
{
    MEI_LOCK_GUARD();
    const double invalid = std::nan("");
    point_camera = Eigen::Vector3d(invalid, invalid, invalid);
    if (!std::isfinite(z) || z <= kMinDenom) {
        return false;
    }

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    readIntrinsics(K_, fx, fy, cx, cy);
    if (std::abs(fx) < kMinDenom || std::abs(fy) < kMinDenom) {
        return false;
    }

    const double xd = (point_2d.x - cx) / fx;
    const double yd = (point_2d.y - cy) / fy;
    if (!std::isfinite(xd) || !std::isfinite(yd)) {
        return false;
    }

    double xu = xd;
    double yu = yd;
    const DistortionCoeffs distortion = readDistortionCoeffs(D_);
    for (int iter = 0; iter < kUndistortIterations; ++iter) {
        const double r2 = xu * xu + yu * yu;
        const double radial = 1.0 + distortion.k1 * r2 + distortion.k2 * r2 * r2;
        if (std::abs(radial) < kMinRadial) {
            return false;
        }

        const double dx = 2.0 * distortion.p1 * xu * yu +
                          distortion.p2 * (r2 + 2.0 * xu * xu);
        const double dy = distortion.p1 * (r2 + 2.0 * yu * yu) +
                          2.0 * distortion.p2 * xu * yu;
        xu = (xd - dx) / radial;
        yu = (yd - dy) / radial;
        if (!std::isfinite(xu) || !std::isfinite(yu)) {
            return false;
        }
    }

    double xp = 0.0;
    double yp = 0.0;
    if (!projectUndistortedRay(epsilon_, xu, yu, xp, yp)) {
        return false;
    }

    point_camera.x() = xp * z;
    point_camera.y() = yp * z;
    point_camera.z() = z;
    return isFinitePoint3(point_camera);
}

bool Mei::projectCamera2dToWorld3d(const cv::Point2d& point_2d,
                                   const double camera_z,
                                   Eigen::Vector3d& point_world) const
{
    MEI_LOCK_GUARD();
    const double invalid = std::nan("");
    point_world = Eigen::Vector3d(invalid, invalid, invalid);

    Eigen::Vector3d point_camera;
    if (!projectCamera2dToCamera3d(point_2d, camera_z, point_camera)) {
        return false;
    }

    return camera3dToWorld3d(point_camera, point_world);
}

bool Mei::projectWorld3dTo2d(const Eigen::Vector3d& point_world,
                             cv::Point2d& point_2d) const
{
    MEI_LOCK_GUARD();
    Eigen::Vector3d point_camera;
    if (!world3dToCamera3d(point_world, point_camera)) {
        return false;
    }
    return projectCamera3dTo2d(point_camera, point_2d);
}

bool Mei::undistort(const cv::Point2d& distorted_point,
                    cv::Point2d& undistorted_point,
                    const cv::Mat& undistort_k) const
{
    MEI_LOCK_GUARD();
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    readIntrinsics(K_, fx, fy, cx, cy);
    if (std::abs(fx) < kMinDenom || std::abs(fy) < kMinDenom) {
        return false;
    }

    const double xd = (distorted_point.x - cx) / fx;
    const double yd = (distorted_point.y - cy) / fy;
    if (!std::isfinite(xd) || !std::isfinite(yd)) {
        return false;
    }

    double xu = xd;
    double yu = yd;
    const DistortionCoeffs distortion = readDistortionCoeffs(D_);
    for (int iter = 0; iter < kUndistortIterations; ++iter) {
        const double r2 = xu * xu + yu * yu;
        const double radial = 1.0 + distortion.k1 * r2 + distortion.k2 * r2 * r2;
        if (std::abs(radial) < kMinRadial) {
            return false;
        }

        const double dx = 2.0 * distortion.p1 * xu * yu +
                          distortion.p2 * (r2 + 2.0 * xu * xu);
        const double dy = distortion.p1 * (r2 + 2.0 * yu * yu) +
                          2.0 * distortion.p2 * xu * yu;
        xu = (xd - dx) / radial;
        yu = (yd - dy) / radial;
        if (!std::isfinite(xu) || !std::isfinite(yu)) {
            return false;
        }
    }

    double xp = 0.0;
    double yp = 0.0;
    if (!projectUndistortedRay(epsilon_, xu, yu, xp, yp)) {
        return false;
    }

    cv::Mat effective_K;
    if (!undistort_k.empty()) {
        effective_K = normalizeCameraMatrixImpl(undistort_k);
    } else if (undistort_info_.isReady()) {
        effective_K = undistort_info_.K_.clone();
    } else {
        effective_K = K_.clone();
    }

    double out_fx = 0.0;
    double out_fy = 0.0;
    double out_cx = 0.0;
    double out_cy = 0.0;
    readIntrinsics(effective_K, out_fx, out_fy, out_cx, out_cy);

    undistorted_point.x = out_fx * xp + out_cx;
    undistorted_point.y = out_fy * yp + out_cy;
    return isFinitePoint2(undistorted_point);
}

bool Mei::generateUndistortMap(cv::Mat* map1,
                               cv::Mat* map2)
{
    MEI_LOCK_GUARD();

    if (epsilon_ < 1e-3) {
        releaseMaps(map1, map2);
        return false;
    }

    if(!undistort_info_.isReady()){
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 10, 0, 960, 0, 10, 540, 0, 0, 1);
        setUndistortInfo(image_width_, image_height_, camera_matrix);
    }


    if (!undistort_info_.hasMap()) {
        cv::Mat generated_map1;
        cv::Mat generated_map2;
        if (!generateOmniUndistortMap(K_,
                                      D_,
                                      epsilon_,
                                      undistort_info_.K_,
                                      cv::Size(undistort_info_.width_, undistort_info_.height_),
                                      generated_map1,
                                      generated_map2)) {
            releaseMaps(map1, map2);
            return false;
        }
        undistort_info_.updateMap(generated_map1, generated_map2);
    }

    undistort_info_.copyMap(map1, map2);
    return true;
}

bool Mei::undistortImage(const cv::Mat& image,
                         cv::Mat& undistorted_image)
{
    MEI_LOCK_GUARD();
    if (image.empty()) {
        undistorted_image.release();
        return false;
    }

    cv::Mat map1;
    cv::Mat map2;
    if (!generateUndistortMap(&map1, &map2)) {
        undistorted_image.release();
        return false;
    }

    cv::remap(image,
              undistorted_image,
              map1,
              map2,
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
    return true;
}

bool Mei::generateIpmMap(cv::Mat* map_x,
                         cv::Mat* map_y)
{
    MEI_LOCK_GUARD();
    if (!ipm_info_.isReady()) {
        releaseMaps(map_x, map_y);
        return false;
    }

    if (!ipm_info_.hasMap()) {
        cv::Mat generated_map_x(cv::Size(ipm_info_.width_, ipm_info_.height_), CV_32F, cv::Scalar(-1.0f));
        cv::Mat generated_map_y(cv::Size(ipm_info_.width_, ipm_info_.height_), CV_32F, cv::Scalar(-1.0f));

        for (int y = 0; y < ipm_info_.height_; ++y) {
            float* map_x_row = generated_map_x.ptr<float>(y);
            float* map_y_row = generated_map_y.ptr<float>(y);
            for (int x = 0; x < ipm_info_.width_; ++x) {
                if (!isInIpmRoi(x,
                                y,
                                ipm_info_.roi_top_,
                                ipm_info_.roi_bottom_,
                                ipm_info_.roi_left_,
                                ipm_info_.roi_right_)) {
                    continue;
                }

                const Eigen::Vector3d point_world = makeIpmWorldPoint(x,
                                                                      y,
                                                                      ipm_info_.width_,
                                                                      ipm_info_.height_,
                                                                      ipm_info_.scale_,
                                                                      ground_height_,
                                                                      ipm_info_.origin_offset_x_,
                                                                      ipm_info_.origin_offset_y_);
                cv::Point2d point_image;
                if (!projectWorld3dTo2d(point_world, point_image) ||
                    !isValidBevSourcePixel(point_image, image_width_, image_height_)) {
                    continue;
                }

                map_x_row[x] = static_cast<float>(point_image.x);
                map_y_row[x] = static_cast<float>(point_image.y);
            }
        }

        ipm_info_.updateMap(generated_map_x, generated_map_y);
    }

    ipm_info_.copyMap(map_x, map_y);
    return true;
}

bool Mei::generateIpmImage(const cv::Mat& image,
                           cv::Mat& bev_image)
{
    MEI_LOCK_GUARD();
    if (image.empty()) {
        bev_image.release();
        return false;
    }

    cv::Mat map_x;
    cv::Mat map_y;
    if (!generateIpmMap(&map_x, &map_y)) {
        bev_image.release();
        return false;
    }

    cv::remap(image,
              bev_image,
              map_x,
              map_y,
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
    return true;
}

cv::Mat Mei::normalizeK(const cv::Mat& K)
{
    return normalizeCameraMatrixImpl(K);
}

cv::Mat Mei::normalizeD(const cv::Mat& D)
{
    return normalizeDistCoeffsImpl(D);
}
std::array<int, 3> Mei::normalizeRotationOrder(const std::array<int, 3>& rotation_order)
{
    std::array<int, 3> normalized = rotation_order;
    bool seen[3] = {false, false, false};
    for (size_t i = 0; i < normalized.size(); ++i) {
        if (normalized[i] < 0 || normalized[i] > 2 || seen[normalized[i]]) {
            return std::array<int, 3>{{2, 1, 0}};
        }
        seen[normalized[i]] = true;
    }
    return normalized;
}

Eigen::Matrix3d Mei::eulerToRotation(const Eigen::Vector3d& r_euler) const
{
    static const Eigen::Vector3d kAxes[3] = {
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    for (size_t i = 0; i < rotation_order_.size(); ++i) {
        const int axis = rotation_order_[i];
        rotation *= Eigen::AngleAxisd(r_euler[axis], kAxes[axis]).toRotationMatrix();
    }
    return rotation;
}

Eigen::Vector3d Mei::rotationToEuler(const Eigen::Matrix3d& rotation) const
{
    const Eigen::Vector3d ordered_euler = rotation.eulerAngles(rotation_order_[0],
                                                               rotation_order_[1],
                                                               rotation_order_[2]);
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < rotation_order_.size(); ++i) {
        euler[rotation_order_[i]] = ordered_euler[static_cast<Eigen::Index>(i)];
    }
    return euler;
}

bool Mei::generateOmniUndistortMap(const cv::Mat& K,
                                   const cv::Mat& D,
                                   double epsilon,
                                   const cv::Mat& new_K,
                                   const cv::Size& size,
                                   cv::Mat& map1,
                                   cv::Mat& map2)
{
    map1.create(size.height, size.width, CV_32F);
    map2.create(size.height, size.width, CV_32F);

    const float xi = static_cast<float>(epsilon);
    cv::Matx33f src_K = cv::Matx33f::eye();
    cv::Matx33f dst_K = cv::Matx33f::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            src_K(r, c) = static_cast<float>(K.at<double>(r, c));
            dst_K(r, c) = static_cast<float>(new_K.at<double>(r, c));
        }
    }

    const cv::Vec4f src_D(
        static_cast<float>(D.at<double>(0, 0)),
        static_cast<float>(D.at<double>(0, 1)),
        static_cast<float>(D.at<double>(0, 2)),
        static_cast<float>(D.at<double>(0, 3)));

    for (int y = 0; y < size.height; ++y) {
        float* map1_row = map1.ptr<float>(y);
        float* map2_row = map2.ptr<float>(y);
        for (int x = 0; x < size.width; ++x) {
            const cv::Vec3f Xc(
                static_cast<float>(x) - dst_K(0, 2),
                static_cast<float>(y) - dst_K(1, 2),
                dst_K(0, 0));
            const float Xc_norm = cv::norm(Xc);
            if (!std::isfinite(Xc_norm) || Xc_norm <= static_cast<float>(kMinDenom)) {
                map1.release();
                map2.release();
                return false;
            }

            const cv::Vec3f Xs = Xc / Xc_norm;
            const float denom = Xs[2] + xi;
            if (!std::isfinite(denom) || std::abs(denom) <= static_cast<float>(kMinDenom)) {
                map1.release();
                map2.release();
                return false;
            }

            const float xu = Xs[0] / denom;
            const float yu = Xs[1] / denom;
            if (!std::isfinite(xu) || !std::isfinite(yu)) {
                map1.release();
                map2.release();
                return false;
            }

            const float r2 = xu * xu + yu * yu;
            const float r4 = r2 * r2;
            const float radial = 1.0F + src_D[0] * r2 + src_D[1] * r4;
            const float xd = xu * radial + 2.0F * src_D[2] * xu * yu +
                             src_D[3] * (r2 + 2.0F * xu * xu);
            const float yd = yu * radial + src_D[2] * (r2 + 2.0F * yu * yu) +
                             2.0F * src_D[3] * xu * yu;
            if (!std::isfinite(xd) || !std::isfinite(yd)) {
                map1.release();
                map2.release();
                return false;
            }

            map1_row[x] = src_K(0, 0) * xd + src_K(0, 1) * yd + src_K(0, 2);
            map2_row[x] = src_K(1, 1) * yd + src_K(1, 2);
            if (!std::isfinite(map1_row[x]) || !std::isfinite(map2_row[x])) {
                map1.release();
                map2.release();
                return false;
            }
        }
    }

    return true;
}

}  // namespace camera
