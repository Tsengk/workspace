#pragma once

#include <string>
#include <vector>
#include <array>
#include <opencv2/core.hpp>
#include "bev_surround_view/types.h"

namespace bev_surround_view {

// -------------------------------------------------------
// 单组棋盘格在标定布中的定义（从 chasiss_board_V2.yaml 加载）
// -------------------------------------------------------
struct BoardGroupDef {
    // 棋盘格角点的板本地坐标起点：corner(0,0) = (start_x, start_y)
    // 按 findChessboardCorners 约定：row/col 均从 0 开始，步长 = square_size
    // 3D 对象点：corner(r,c) = [start_x + c*sq, start_y + r*sq, 0]
    double start_x;     // 板本地 X 起始坐标（m）
    double start_y;     // 板本地 Y 起始坐标（m）
    int    corner_rows; // 角点行数
    int    corner_cols; // 角点列数
    double square_size; // 格子边长（m）
};

// -------------------------------------------------------
// 相机安装参数（位置精确已知，旋转为初始猜测）
// -------------------------------------------------------
struct CameraInstallation {
    std::string name;
    double x_vcs, y_vcs, z_vcs;    // VCS 中的安装位置（m，精确已知）
    double roll0, pitch0, yaw0;     // 初始旋转猜测（rad，来自安装参数）
};

// -------------------------------------------------------
// ExtrinsicSolver
//
// 功能：已知相机位置 (x,y,z)，通过棋盘格图像角点，
//       使用 solvePnP 初始化 + Levenberg-Marquardt 细化，
//       求解相机旋转外参 (roll, pitch, yaw)。
//
// 地平面约束：所有标定布均平放在地面上（z_vcs = ground_z），
//             标定布位姿自由参数降为 (x_b, y_b, psi_b)，无需手动测量。
//
// 旋转约定（ZYX Euler，VCS→Camera）：
//   R_vc = Rz(yaw) · Ry(pitch) · Rx(roll)
//   P_cam = R_vc · (P_vcs - p_c)
//
// 使用方式：
//   1. setGroundZ(-0.37)
//   2. 对每个相机调用 solve()
// -------------------------------------------------------
class ExtrinsicSolver {
public:
    ExtrinsicSolver();

    // 设置地面平面 VCS z 坐标（通常为 -z_offset，如 -0.37）
    void setGroundZ(double gz);

    // 求解单个相机的旋转外参
    //
    // intr        : 相机内参（用于投影计算；corners 须来自去畸变图像）
    // install     : 相机安装位置 + 初始 RPY 猜测
    // groups      : 该相机对应的棋盘格组定义列表
    // detected    : 每组检测到的角点（groups[i] 未检测到时 detected[i] 为空）
    // result      : 输出的相机外参（R_vc 和 t_vc = -R_vc * p_c）
    //
    // 返回值：平均重投影误差（像素），< 0 表示失败
    double solve(const CameraIntrinsics& intr,
                 const CameraInstallation& install,
                 const std::vector<BoardGroupDef>& groups,
                 const std::vector<std::vector<cv::Point2f>>& detected,
                 CameraExtrinsic& result);

private:
    double ground_z_;   // 地面 VCS z（负值）

    // 生成单组棋盘格的 3D 对象点（组本地坐标系）
    // corner(r,c) = [start_x + c*sq, start_y + r*sq, 0]
    std::vector<cv::Point3f> buildObjectPoints(const BoardGroupDef& g) const;

    // ZYX Euler → 旋转矩阵（3×3, CV_64F）
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cv::Mat rpy2R(double roll, double pitch, double yaw) const;

    // 旋转矩阵 → ZYX Euler（返回 [roll, pitch, yaw]）
    void R2rpy(const cv::Mat& R, double& roll, double& pitch, double& yaw) const;

    // solvePnP 初始化：对每个有效组独立 PnP，平均得到初始 RPY + 各组板位姿
    // params 输出：[roll, pitch, yaw, x_b0, y_b0, psi_b0, x_b1, ...]
    bool initFromPnP(const CameraIntrinsics& intr,
                     const CameraInstallation& install,
                     const std::vector<BoardGroupDef>& groups,
                     const std::vector<std::vector<cv::Point2f>>& detected,
                     std::vector<double>& params);

    // 计算所有有效组的重投影残差向量
    // residuals: [u_err0, v_err0, u_err1, v_err1, ...] per corner
    void computeResiduals(const CameraIntrinsics& intr,
                          const CameraInstallation& install,
                          const std::vector<BoardGroupDef>& groups,
                          const std::vector<std::vector<cv::Point2f>>& detected,
                          const std::vector<int>& valid_idx,
                          const std::vector<double>& params,
                          std::vector<double>& residuals) const;

    // 单角点投影（组本地坐标 → 图像像素，使用当前 params）
    // 返回 false 表示点在相机后方
    bool projectCorner(const CameraIntrinsics& intr,
                       const CameraInstallation& install,
                       double roll, double pitch, double yaw,
                       double x_b, double y_b, double psi_b,
                       double bx, double by,
                       double& u, double& v) const;

    // Levenberg-Marquardt 优化
    // params: 初始值（原地更新）
    // 返回最终均方根误差（像素）
    double optimizeLM(const CameraIntrinsics& intr,
                      const CameraInstallation& install,
                      const std::vector<BoardGroupDef>& groups,
                      const std::vector<std::vector<cv::Point2f>>& detected,
                      const std::vector<int>& valid_idx,
                      std::vector<double>& params);
};

} // namespace bev_surround_view
