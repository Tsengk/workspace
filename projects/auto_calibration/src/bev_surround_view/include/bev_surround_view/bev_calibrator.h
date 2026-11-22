#pragma once

#include <vector>
#include <string>
#include <array>
#include <opencv2/core.hpp>

#include "bev_surround_view/types.h"

namespace bev_surround_view {

// -------------------------------------------------------
// BevCalibrator: 环视 BEV 标定核心类
//
// 工作流程（有真实棋盘格图像时）:
//   1. setParams()        设置 BEV 分辨率/视野范围等参数
//   2. calibrate()        从图像中检测棋盘格 + 已知外参，计算各相机单应矩阵
//
// 工作流程（纯外参合成时，无真实图像）:
//   1. setParams()
//   2. calibrateFromExtrinsics()  仅用外参 + 内参数学推导单应矩阵
//
// 通用:
//   3. generateBEV()      用标定好的单应矩阵合成 BEV 图像
//   4. evaluateStitching() 评估拼接误差
// -------------------------------------------------------
class BevCalibrator {
public:
    BevCalibrator();
    ~BevCalibrator() = default;

    // 设置 BEV 参数（分辨率、视野范围等）
    void setParams(const BevParams& params);

    // 从棋盘格图像 + 已知摆放位置直接计算各相机单应矩阵
    //
    // 原理：
    //   1. 检测各相机图像中的棋盘格角点 → 图像坐标 (u, v)
    //   2. 根据 placements[i] 计算每个角点的 VCS 坐标 (x, y)
    //   3. cv::findHomography(vcs_pts, img_pts) → H (VCS→图像)
    //
    // images[4]     : 4 个相机的原始图像（前/后/左/右）
    // intrinsics[4] : 4 个相机的内参（用于去畸变）
    // placements[4] : 4 个标定板的摆放参数（角点 VCS 坐标描述）
    // cb_params     : 棋盘格参数（角点数、格子尺寸）
    // result        : 标定结果（输出）
    bool calibrate(const std::array<cv::Mat, 4>& images,
                   const std::array<CameraIntrinsics, 4>& intrinsics,
                   const std::array<BoardPlacement, 4>& placements,
                   const CheckerboardParams& cb_params,
                   CalibResult& result);

    // 纯从外参 + 内参数学计算单应矩阵（用于合成自测，无需真实图像）
    // 采样地面上的网格点，通过针孔投影模型构造单应矩阵
    // intrinsics[4]  : 4 个相机的内参
    // extrinsics[4]  : 4 个相机的外参（VCS -> CAM）
    // result         : 标定结果（输出）
    // 返回值          : true=成功，false=失败
    bool calibrateFromExtrinsics(const std::array<CameraIntrinsics, 4>& intrinsics,
                                 const std::array<CameraExtrinsic, 4>& extrinsics,
                                 CalibResult& result);

    // 使用标定好的单应矩阵合成 BEV 全景图像
    // images[4]       : 4 个相机的原始图像（已去畸变或原始均可，与标定时保持一致）
    // homographies[4] : calibrate/calibrateFromExtrinsics 输出的单应矩阵
    // output_bev      : 输出的 BEV 图像（CV_8UC3）
    // 返回值           : true=成功，false=失败
    bool generateBEV(const std::array<cv::Mat, 4>& images,
                     const std::vector<CameraHomography>& homographies,
                     cv::Mat& output_bev);

    // 评估 BEV 图像中的拼接误差
    // bev              : generateBEV 输出的 BEV 图像
    // result           : 更新 stitch_error_m 和 quality_grade 字段
    // 说明：通过检测 BEV 图像中相邻相机重叠区域的特征点匹配误差来评估
    void evaluateStitching(const cv::Mat& bev, CalibResult& result);

    // 可视化检测结果（在原图上绘制角点和黑块角点）
    // placements[4] 可选：若非空则同时可视化黑块角点（仅前后板）
    void visualizeDetection(const std::array<cv::Mat, 4>& images,
                           const CheckerboardParams& cb_params,
                           std::array<DetectionVis, 4>& vis_results,
                           const std::array<BoardPlacement, 4>* placements = nullptr);

    // 将 VCS 地面坐标 (x, y) 转换为 BEV 像素坐标 (u, v)
    // x: VCS X（前方为正），y: VCS Y（左方为正）
    // 返回像素坐标（列，行）
    cv::Point2d vcsToPixel(double x, double y) const;

    // 将 BEV 像素坐标 (u, v) 转换为 VCS 地面坐标 (x, y)
    cv::Point2d pixelToVcs(double u, double v) const;

    // 单点投影：将 VCS 地面点 (x, y, z=-z_offset) 投影到相机图像
    // 返回 true 表示在相机前方（z_cam > 0），像素坐标写入 uv
    bool projectVcsToImage(const cv::Point3d& pt_vcs,
                           const CameraIntrinsics& intr,
                           const CameraExtrinsic& extr,
                           cv::Point2d& uv) const;

    // 计算单相机的单应矩阵（从地面采样点 -> 图像点 DLT）
    // 内部使用，也可外部调用（用于自定义流程）
    bool computeHomographyFromExtrinsic(const CameraIntrinsics& intr,
                                        const CameraExtrinsic& extr,
                                        CameraHomography& hom) const;

    // 常量：VCS 坐标系中地面 z 坐标（相机外参约定地面在 VCS z = -z_offset）
    static constexpr double kGroundZ = -0.37;  // m，后轴到地面高度

private:
    BevParams params_;
    bool params_set_;

    // 从图像中检测棋盘格角点（已去畸变图像），返回图像角点
    bool detectCheckerboard(const cv::Mat& image,
                            const CheckerboardParams& cb_params,
                            int camera_idx,
                            std::vector<cv::Point2f>& img_corners);

    // 在图像中检测单个黑块角点
    // H          : 当前相机初始单应矩阵（VCS→图像），用于将期望 VCS 点投影到 ROI 中心
    // vcs_pt     : 该角点的 VCS 坐标
    // img_size   : 图像尺寸（用于边界检查）
    // img_pt     : 检测到的图像角点（输出）
    // 返回 true 表示检测成功
    bool detectBlackBlockCorner(const cv::Mat& undist_image,
                                const cv::Mat& H,
                                const cv::Point2f& vcs_pt,
                                cv::Size img_size,
                                cv::Point2f& img_pt);

    // 计算前后板 8 个黑块角点的 VCS 坐标
    // （左块4个 + 右块4个，顺序：左块底左/底右/顶左/顶右，右块同）
    std::vector<cv::Point2f> computeBlackBlockVcsCorners(
            const BoardPlacement& placement,
            const CheckerboardParams& cb_params);

    // 为给定相机生成地面采样网格点及其对应图像投影点
    // 采样策略：在 VCS 地面上均匀采样 N x N 网格点
    bool sampleGroundPoints(const CameraIntrinsics& intr,
                            const CameraExtrinsic& extr,
                            std::vector<cv::Point2f>& ground_pts_2d,
                            std::vector<cv::Point2f>& image_pts) const;

    // BEV 图像中各相机的绘制顺序权重（重叠区域的混合策略）
    // 返回 [0,1] 的权重，1 表示该相机完全覆盖该像素
    double blendWeight(int camera_idx, double x_vcs, double y_vcs) const;
};

} // namespace bev_surround_view
