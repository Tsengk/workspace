#include "image_lidar_calibration/fastlivo2_detector.h"

#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace image_lidar_calibration {

// ────────────────────────────────────────────────────────────────────────────
// 内部：几何一致性检查（等价于 common_lib.h 中的 Square::is_valid()）
// 检查4个点是否构成 w×h 矩形（允许 8% 相对误差）
// ────────────────────────────────────────────────────────────────────────────
static bool squareIsValid(const std::vector<Eigen::Vector3d>& pts,
                           double w, double h)
{
    if (pts.size() != 4) return false;

    const double GEOM_TOL = 0.08;
    const double diag     = std::sqrt(w * w + h * h);

    // 1. 计算质心
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const auto& p : pts) c += p;
    c /= 4.0;

    // 2. 检查每个点到质心的距离 ≈ 对角线/2
    for (const auto& p : pts) {
        double d = (p - c).norm();
        if (std::fabs(d - diag / 2.0) / (diag / 2.0) > GEOM_TOL * 2.0)
            return false;
    }

    // 3. 按 XY 平面角度排序（与 demo 的 sortPatternCenters "camera" 等价）
    std::vector<std::pair<double, int>> angles;
    for (int i = 0; i < 4; ++i) {
        double dx = pts[i].x() - c.x();
        double dy = pts[i].y() - c.y();
        angles.emplace_back(std::atan2(dy, dx), i);
    }
    std::sort(angles.begin(), angles.end());

    std::vector<Eigen::Vector3d> sorted;
    for (const auto& a : angles) sorted.push_back(pts[a.second]);

    double s01 = (sorted[1] - sorted[0]).norm();
    double s12 = (sorted[2] - sorted[1]).norm();
    double s23 = (sorted[3] - sorted[2]).norm();
    double s30 = (sorted[0] - sorted[3]).norm();

    // 检查两种边长组合（宽高或高宽）
    bool ok1 = (std::fabs(s01 - w) / w < GEOM_TOL) &&
               (std::fabs(s12 - h) / h < GEOM_TOL) &&
               (std::fabs(s23 - w) / w < GEOM_TOL) &&
               (std::fabs(s30 - h) / h < GEOM_TOL);
    bool ok2 = (std::fabs(s01 - h) / h < GEOM_TOL) &&
               (std::fabs(s12 - w) / w < GEOM_TOL) &&
               (std::fabs(s23 - h) / h < GEOM_TOL) &&
               (std::fabs(s30 - w) / w < GEOM_TOL);
    if (!ok1 && !ok2) return false;

    // 4. 检查周长
    double perim = s01 + s12 + s23 + s30;
    double ideal = 2.0 * (w + h);
    return std::fabs(perim - ideal) / ideal <= GEOM_TOL;
}

// ────────────────────────────────────────────────────────────────────────────
static bool undistortPinholePixel(const CameraIntrinsics& cam,
                                  const cv::Point2f& distorted_pixel,
                                  double& xu,
                                  double& yu)
{
    if (std::fabs(cam.fx) < 1e-9 || std::fabs(cam.fy) < 1e-9) {
        return false;
    }

    const double xd = (static_cast<double>(distorted_pixel.x) - cam.cx) / cam.fx;
    const double yd = (static_cast<double>(distorted_pixel.y) - cam.cy) / cam.fy;

    xu = xd;
    yu = yd;
    for (int iter = 0; iter < 8; ++iter) {
        const double r2 = xu * xu + yu * yu;
        const double radial = 1.0 + cam.k1 * r2 + cam.k2 * r2 * r2;
        if (std::fabs(radial) < 1e-12) {
            return false;
        }
        const double dx = 2.0 * cam.p1 * xu * yu + cam.p2 * (r2 + 2.0 * xu * xu);
        const double dy = cam.p1 * (r2 + 2.0 * yu * yu) + 2.0 * cam.p2 * xu * yu;
        xu = (xd - dx) / radial;
        yu = (yd - dy) / radial;
    }
    return std::isfinite(xu) && std::isfinite(yu);
}

static bool undistortMEIPixelToVirtual(const CameraIntrinsics& cam,
                                       const cv::Mat& virtual_K,
                                       const cv::Point2f& distorted_pixel,
                                       cv::Point2f& undistorted_pixel)
{
    double xu = 0.0;
    double yu = 0.0;
    if (!undistortPinholePixel(cam, distorted_pixel, xu, yu)) {
        return false;
    }

    const double rho2 = xu * xu + yu * yu;
    const double sqrt_arg = 1.0 + (1.0 - cam.xi * cam.xi) * rho2;
    if (sqrt_arg <= 0.0) {
        return false;
    }

    const double lambda = (cam.xi + std::sqrt(sqrt_arg)) / (rho2 + 1.0);
    const double zs = lambda - cam.xi;
    if (std::fabs(zs) < 1e-12) {
        return false;
    }

    const double xp = lambda * xu / zs;
    const double yp = lambda * yu / zs;
    const double fx = virtual_K.at<double>(0, 0);
    const double fy = virtual_K.at<double>(1, 1);
    const double cx = virtual_K.at<double>(0, 2);
    const double cy = virtual_K.at<double>(1, 2);

    undistorted_pixel.x = static_cast<float>(fx * xp + cx);
    undistorted_pixel.y = static_cast<float>(fy * yp + cy);
    return std::isfinite(undistorted_pixel.x) && std::isfinite(undistorted_pixel.y);
}

static bool undistortMEICornersToVirtual(const CameraIntrinsics& cam,
                                         const cv::Mat& virtual_K,
                                         const std::vector<std::vector<cv::Point2f> >& distorted_corners,
                                         std::vector<std::vector<cv::Point2f> >& undistorted_corners)
{
    undistorted_corners.clear();
    undistorted_corners.reserve(distorted_corners.size());
    for (size_t i = 0; i < distorted_corners.size(); ++i) {
        std::vector<cv::Point2f> marker_corners;
        marker_corners.reserve(distorted_corners[i].size());
        for (size_t j = 0; j < distorted_corners[i].size(); ++j) {
            cv::Point2f undistorted_pixel;
            if (!undistortMEIPixelToVirtual(cam, virtual_K, distorted_corners[i][j], undistorted_pixel)) {
                return false;
            }
            marker_corners.push_back(undistorted_pixel);
        }
        undistorted_corners.push_back(marker_corners);
    }
    return true;
}

static bool projectCameraPointToRawPixel(const CameraIntrinsics& cam,
                                         const Eigen::Vector3d& point_cam,
                                         cv::Point2f& pixel)
{
    const double X = point_cam.x();
    const double Y = point_cam.y();
    const double Z = point_cam.z();
    if (Z <= 1e-9) {
        return false;
    }

    double xu = 0.0;
    double yu = 0.0;
    if (cam.camera_model == MEI) {
        const double norm = point_cam.norm();
        const double denom = Z + cam.xi * norm;
        if (denom <= 1e-9) {
            return false;
        }
        xu = X / denom;
        yu = Y / denom;
    } else {
        xu = X / Z;
        yu = Y / Z;
    }

    const double r2 = xu * xu + yu * yu;
    const double radial = 1.0 + cam.k1 * r2 + cam.k2 * r2 * r2;
    const double xd = xu * radial + 2.0 * cam.p1 * xu * yu + cam.p2 * (r2 + 2.0 * xu * xu);
    const double yd = yu * radial + cam.p1 * (r2 + 2.0 * yu * yu) + 2.0 * cam.p2 * xu * yu;

    pixel.x = static_cast<float>(cam.fx * xd + cam.cx);
    pixel.y = static_cast<float>(cam.fy * yd + cam.cy);
    return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

static void drawMEIRawAxes(const CameraIntrinsics& cam,
                           const cv::Vec3d& rvec,
                           const cv::Vec3d& tvec,
                           cv::Mat& image,
                           double axis_length)
{
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    std::vector<Eigen::Vector3d> board_points;
    board_points.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    board_points.push_back(Eigen::Vector3d(axis_length, 0.0, 0.0));
    board_points.push_back(Eigen::Vector3d(0.0, axis_length, 0.0));
    board_points.push_back(Eigen::Vector3d(0.0, 0.0, axis_length));

    std::vector<cv::Point2f> image_points;
    image_points.reserve(board_points.size());
    for (size_t i = 0; i < board_points.size(); ++i) {
        const cv::Mat p_board = (cv::Mat_<double>(3, 1) << board_points[i].x(), board_points[i].y(), board_points[i].z());
        const cv::Mat p_cam = R_cv * p_board + (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
        cv::Point2f uv;
        if (!projectCameraPointToRawPixel(cam,
                                          Eigen::Vector3d(p_cam.at<double>(0), p_cam.at<double>(1), p_cam.at<double>(2)),
                                          uv)) {
            return;
        }
        image_points.push_back(uv);
    }

    cv::line(image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 2);
    cv::line(image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 2);
    cv::line(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 2);
}

// FastLivo2Detector 实现

FastLivo2Detector::FastLivo2Detector()
{
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

void FastLivo2Detector::setTarget(const TargetParams& target)
{
    target_ = target;
}

void FastLivo2Detector::setCamera(const CameraIntrinsics& cam)
{
    cam_ = cam;
    mei_maps_valid_ = false;
    mei_virtual_K_.release();

    if (cam_.camera_model == MEI) {
        buildMEIUndistortMaps();
    }
}

// ────────────────────────────────────────────────────────────────────────────
// MEI 模型去畸变映射表构建
// 完全参考 GenerateOmniUndistortMap 的逻辑实现
//
// GenerateOmniUndistortMap 原始代码逻辑：
//   Xc = (u - K(0,2), v - K(1,2), K(0,0))
//   Xs = Xc / ||Xc||
//   xu = Xs[0] / (Xs[2] + xi)
//   yu = Xs[1] / (Xs[2] + xi)
//   r2 = xu² + yu²
//   r4 = r2²
//   radial = 1 + D[0]*r2 + D[1]*r4
//   xd = xu*radial + 2*D[2]*xu*yu + D[3]*(r² + 2*xu²)
//   yd = yu*radial + D[2]*(r² + 2*yu²) + 2*D[3]*xu*yu
//   umapx = M(0,0)*xd + M(0,1)*yd + M(0,2)
//   umapy = M(1,1)*yd + M(1,2)
//
// 这里 M 是单位矩阵，直接输出 xd, yd 作为畸变后的归一化坐标
// 再乘以内参得到像素坐标
// ────────────────────────────────────────────────────────────────────────────
inline	void GenerateOmniUndistortMap(cv::Matx33f M, cv::Vec4f D, float xi, cv::Matx33f K,
							  cv::Size size, cv::Mat &umapx, cv::Mat &umapy) 
	{
		umapx.create(size.height, size.width, CV_32F);
		umapy.create(size.height, size.width, CV_32F);
		for (int y = 0; y < umapx.rows; y++)
		{
			float *pumapx = umapx.ptr<float>(y);
			float *pumapy = umapy.ptr<float>(y);
			for (int x = 0; x < umapx.cols; x++)
			{
				cv::Vec3f Xc(x - K(0, 2), y - K(1, 2), K(0, 0));
				cv::Vec3f Xs = Xc / norm(Xc);
				float xu = Xs[0] / (Xs[2] + xi);
				float yu = Xs[1] / (Xs[2] + xi);
				float r2 = xu * xu + yu * yu;
				float r4 = r2 * r2;
				float r = 1 + D[0] * r2 + D[1] * r4;
				float xd = xu * r + 2 * D[2] * xu * yu + D[3] * (r2 + 2 * xu * xu);
				float yd = yu * r + D[2] * (r2 + 2 * yu * yu) + 2 * D[3] * xu * yu;
				pumapx[x] = M(0, 0) * xd + M(0, 1) * yd + M(0, 2);
				pumapy[x] = M(1, 1) * yd + M(1, 2);
			}
		}
	}

void FastLivo2Detector::buildMEIUndistortMaps()
{
    const int W  = cam_.width;
    const int H  = cam_.height;
    const float xi = static_cast<float>(cam_.xi);
    
    // 畸变参数 D = (k1, k2, p1, p2)
    cv::Vec4f D(static_cast<float>(cam_.k1),
                static_cast<float>(cam_.k2),
                static_cast<float>(cam_.p1),
                static_cast<float>(cam_.p2));
    
    const float new_fx = 1000.0f;
    const float new_fy = 1000.0f;
    const float new_cx = 0.5f * static_cast<float>(W);
    const float new_cy = 0.5f * static_cast<float>(H);
    cv::Matx33f K(new_fx, 0.0f,  new_cx,
                  0.0f,  new_fy, new_cy,
                  0.0f,  0.0f,   1.0f);
    
    // M 是单位矩阵，直接输出畸变后的归一化坐标
    cv::Matx33f M(cam_.fx, 0.0f, cam_.cx,
                  0.0f, cam_.fy, cam_.cy,
                  0.0f, 0.0f, 1.0f);
    
    // 调用 GenerateOmniUndistortMap 生成去畸变映射表
    GenerateOmniUndistortMap(M, D, xi, K, cv::Size(W, H), mei_map1_, mei_map2_);

    mei_virtual_K_ = (cv::Mat_<double>(3, 3) <<
        static_cast<double>(new_fx), 0.0, static_cast<double>(new_cx),
        0.0, static_cast<double>(new_fy), static_cast<double>(new_cy),
        0.0, 0.0, 1.0);
    mei_maps_valid_ = true;
    ROS_DEBUG("[FastLivo2Detector] MEI undistort maps built (xi=%.4f, new_k=(%.1f,%.1f,%.1f,%.1f), %dx%d).",
             cam_.xi, new_fx, new_fy, new_cx, new_cy, W, H);
}

bool FastLivo2Detector::verifyGeometry(const std::vector<Eigen::Vector3d>& centers) const
{
    return squareIsValid(centers,
                         target_.delta_width_circles,
                         target_.delta_height_circles);
}

// ────────────────────────────────────────────────────────────────────────────
// 检测并计算标定板位姿
// 从 rvec/tvec 提取平移和旋转（欧拉角）
// board 坐标系：Z轴垂直于标定板平面（朝相机方向）
// 相机坐标系到标定板的旋转：rvec = [roll, pitch, yaw] (ZYX顺序，度)
// ────────────────────────────────────────────────────────────────────────────
bool FastLivo2Detector::detectWithPose(const cv::Mat& image,
                                       std::vector<Eigen::Vector3d>& circle_centers_cam,
                                       std::vector<cv::Point2f>& circle_pixels,
                                       BoardPose& board_pose,
                                       cv::Mat& debug_image)
{
    if (image.empty()) {
        ROS_ERROR("[FastLivo2Detector] Input image is empty.");
        return false;
    }

    const cv::Mat& working_image = image;
    cv::Mat K = cam_.getPnPCameraMatrix();
    cv::Mat D = cam_.getPnPDistCoeffs();
    debug_image = image.clone();

    const cv::Mat& K_pnp = K;
    const cv::Mat& D_pnp = D;
    ROS_DEBUG("[FastLivo2Detector] PnP K=(%.1f, %.1f, %.1f, %.1f) model=%s",
             K_pnp.at<double>(0, 0), K_pnp.at<double>(1, 1),
             K_pnp.at<double>(0, 2), K_pnp.at<double>(1, 2),
             cam_.camera_model == MEI ? "MEI-marker_raw_point_undistort" : "PINHOLE");

    const double mw = target_.marker_size / 2.0;
    const double dw = target_.delta_width_qr_center;
    const double dh = target_.delta_height_qr_center;
    const double cw = target_.delta_width_circles  / 2.0;
    const double ch = target_.delta_height_circles / 2.0;

    std::vector<std::vector<cv::Point3f> > boardCorners(4);
    std::vector<cv::Point3f> boardCircleCenters;

    for (int i = 0; i < 4; ++i) {
        int xs = (i == 0 || i == 3) ? -1 : 1;
        int ys = (i < 2) ? 1 : -1;

        boardCircleCenters.push_back(cv::Point3f(static_cast<float>(xs * cw),
                                                 static_cast<float>(ys * ch),
                                                 0.f));

        const double xc = xs * dw;
        const double yc = ys * dh;
        for (int j = 0; j < 4; ++j) {
            int xq = (j == 0 || j == 3) ? -1 : 1;
            int yq = (j < 2) ? 1 : -1;
            boardCorners[i].push_back(cv::Point3f(static_cast<float>(xc + xq * mw),
                                                  static_cast<float>(yc + yq * mw),
                                                  0.f));
        }
    }

    std::vector<int> boardIds{1, 2, 4, 3};
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(boardCorners, dictionary_, boardIds);

    cv::Ptr<cv::aruco::DetectorParameters> det_params = cv::aruco::DetectorParameters::create();
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    det_params->doCornerRefinement = true;
#else
    det_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#endif

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(working_image, dictionary_, corners, ids, det_params);
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
    }

    const int nmarks = static_cast<int>(ids.size());
    if (nmarks < target_.min_detected_markers) {
        ROS_WARN("[FastLivo2Detector] %d markers found, need >= %d.", nmarks, target_.min_detected_markers);
        return false;
    }
    if (nmarks > 4) {
        ROS_WARN("[FastLivo2Detector] %d markers found (expected <= 4).", nmarks);
        return false;
    }

    std::vector<std::vector<cv::Point2f> > pose_corners = corners;
    if (cam_.camera_model == MEI) {
        if (!mei_maps_valid_) {
            ROS_ERROR("[FastLivo2Detector] MEI maps not built. Call setCamera() first.");
            return false;
        }
        if (!undistortMEICornersToVirtual(cam_, K_pnp, corners, pose_corners)) {
            ROS_WARN("[FastLivo2Detector] Failed to undistort MEI marker corners to virtual image.");
            return false;
        }
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(pose_corners, target_.marker_size, K_pnp, D_pnp, rvecs, tvecs);

    cv::Vec3d tvec_avg(0, 0, 0);
    cv::Vec3f sin_r(0, 0, 0), cos_r(0, 0, 0);
    for (int i = 0; i < nmarks; ++i) {
        tvec_avg += tvecs[i];
        for (int k = 0; k < 3; ++k) {
            sin_r[k] += static_cast<float>(std::sin(rvecs[i][k]));
            cos_r[k] += static_cast<float>(std::cos(rvecs[i][k]));
        }
    }
    tvec_avg /= nmarks;
    cv::Vec3d rvec_avg;
    for (int k = 0; k < 3; ++k) {
        rvec_avg[k] = std::atan2(sin_r[k] / nmarks, cos_r[k] / nmarks);
    }

    cv::Vec3d rvec = rvec_avg;
    cv::Vec3d tvec = tvec_avg;
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    int n_valid = cv::aruco::estimatePoseBoard(pose_corners, ids, board, K_pnp, D_pnp, rvec, tvec);
#else
    int n_valid = cv::aruco::estimatePoseBoard(pose_corners, ids, board, K_pnp, D_pnp, rvec, tvec, true);
#endif
    if (n_valid == 0) {
        ROS_WARN("[FastLivo2Detector] estimatePoseBoard failed.");
        return false;
    }

    if (cam_.camera_model == MEI) {
        drawMEIRawAxes(cam_, rvec, tvec, debug_image, 0.2);
    } else {
#if CV_MAJOR_VERSION >= 4
        cv::drawFrameAxes(debug_image, K_pnp, D_pnp, rvec, tvec, 0.2f);
#else
        cv::aruco::drawAxis(debug_image, K_pnp, D_pnp, rvec, tvec, 0.2);
#endif
    }

    board_pose.x = tvec[0];
    board_pose.y = tvec[1];
    board_pose.z = tvec[2];

    cv::Mat R_cv(3, 3, CV_64F);
    cv::Rodrigues(rvec, R_cv);
    const double sy = std::sqrt(R_cv.at<double>(0, 2) * R_cv.at<double>(0, 2) +
                                R_cv.at<double>(1, 2) * R_cv.at<double>(1, 2));
    const bool singular = sy < 1e-6;
    if (!singular) {
        board_pose.yaw_deg = std::atan2(R_cv.at<double>(1, 0), R_cv.at<double>(0, 0)) * 180.0 / M_PI;
        board_pose.pitch_deg = std::atan2(-R_cv.at<double>(2, 0), sy) * 180.0 / M_PI;
        board_pose.roll_deg = std::atan2(R_cv.at<double>(2, 1), R_cv.at<double>(2, 2)) * 180.0 / M_PI;
    } else {
        board_pose.yaw_deg = std::atan2(-R_cv.at<double>(1, 0), R_cv.at<double>(0, 0)) * 180.0 / M_PI;
        board_pose.pitch_deg = std::atan2(-R_cv.at<double>(2, 0), sy) * 180.0 / M_PI;
        board_pose.roll_deg = 0.0;
    }

    cv::Mat R_board(3, 3, CV_32F);
    {
        cv::Mat rv32;
        cv::Mat(rvec).convertTo(rv32, CV_32F);
        cv::Rodrigues(rv32, R_board);
    }
    cv::Mat t_board = (cv::Mat_<float>(3, 1) <<
        static_cast<float>(tvec[0]),
        static_cast<float>(tvec[1]),
        static_cast<float>(tvec[2]));

    cv::Mat board_T = cv::Mat::eye(3, 4, CV_32F);
    R_board.copyTo(board_T.rowRange(0, 3).colRange(0, 3));
    t_board.copyTo(board_T.rowRange(0, 3).col(3));

    std::vector<Eigen::Vector3d> candidates;
    std::vector<cv::Point2f> candidate_pixels;
    candidates.reserve(boardCircleCenters.size());
    candidate_pixels.reserve(boardCircleCenters.size());
    for (size_t i = 0; i < boardCircleCenters.size(); ++i) {
        const cv::Point3f& bc = boardCircleCenters[i];
        cv::Mat m = (cv::Mat_<float>(4, 1) << bc.x, bc.y, bc.z, 1.f);
        cv::Mat mc = board_T * m;
        const Eigen::Vector3d camera_point(static_cast<double>(mc.at<float>(0)),
                                           static_cast<double>(mc.at<float>(1)),
                                           static_cast<double>(mc.at<float>(2)));
        candidates.push_back(camera_point);

        std::vector<cv::Point3f> pts3d(1, cv::Point3f(mc.at<float>(0), mc.at<float>(1), mc.at<float>(2)));
        std::vector<cv::Point2f> pts2d;
        cv::projectPoints(pts3d,
                          cv::Mat::zeros(3, 1, CV_64F),
                          cv::Mat::zeros(3, 1, CV_64F),
                          K_pnp,
                          D_pnp,
                          pts2d);
        if (pts2d.empty()) {
            ROS_WARN("[FastLivo2Detector] projectPoints failed for pseudo circle center %zu.", i);
            return false;
        }
        candidate_pixels.push_back(pts2d[0]);

        cv::Point2f debug_pixel = pts2d[0];
        if (cam_.camera_model == MEI) {
            if (!projectCameraPointToRawPixel(cam_, camera_point, debug_pixel)) {
                ROS_WARN("[FastLivo2Detector] Failed to project pseudo circle center %zu back to raw MEI image.", i);
                return false;
            }
        }
        cv::circle(debug_image, debug_pixel, 6, cv::Scalar(0, 255, 0), -1);
    }

    if (!verifyGeometry(candidates)) {
        ROS_WARN("[FastLivo2Detector] Geometry check failed on circle centers.");
        return false;
    }

    circle_centers_cam = candidates;
    circle_pixels = candidate_pixels;
    ROS_DEBUG("[FastLivo2Detector] OK: %d markers -> 4 circle centers, pose=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)",
             nmarks,
             board_pose.x,
             board_pose.y,
             board_pose.z,
             board_pose.roll_deg,
             board_pose.pitch_deg,
             board_pose.yaw_deg);
    return true;
}
} // namespace image_lidar_calibration
