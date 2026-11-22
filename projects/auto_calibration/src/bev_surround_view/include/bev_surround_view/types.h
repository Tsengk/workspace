#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace bev_surround_view {

// -------------------------------------------------------
// 单相机单应矩阵：描述地面平面 (z=0 in VCS) 与图像的映射
// -------------------------------------------------------
struct CameraHomography {
    std::string camera_name;  // 相机名称，如 "front", "rear", "left", "right"
    cv::Mat H;                // 3x3 地面->图像 单应矩阵（VCS xy -> 像素 uv）
    cv::Mat H_inv;            // 3x3 图像->地面 单应矩阵（像素 uv -> VCS xy）
    bool valid;               // 是否已完成标定

    CameraHomography();
};

// -------------------------------------------------------
// BEV（Bird's Eye View）图像参数
// -------------------------------------------------------
struct BevParams {
    double resolution;    // BEV 分辨率，单位 m/pixel，默认 0.02
    double range_front;   // 前视范围，单位 px，默认 250（VCS +X 方向）
    double range_rear;    // 后视范围，单位 px，默认 150（VCS -X 方向）
    double range_left;    // 左视范围，单位 px，默认 125（VCS +Y 方向）
    double range_right;   // 右视范围，单位 px，默认 125（VCS -Y 方向）

    // 可视中心偏移（px），正值=向前/向左偏移（多看前方/左方，少看后方/右方）
    // 图像尺寸不变，仅移动车辆原点在图中的位置
    double origin_offset_x;  // 前后方向偏移（px），正值=向前偏移
    double origin_offset_y;  // 左右方向偏移（px），正值=向左偏移

    // 以下由上面参数派生，构造函数中计算
    int bev_width;        // BEV 图像宽度（像素）= range_left + range_right
    int bev_height;       // BEV 图像高度（像素）= range_front + range_rear
    double origin_x;      // BEV 原点（VCS原点）在图像中的像素 x（列）
    double origin_y;      // BEV 原点（VCS原点）在图像中的像素 y（行）

    BevParams();
    // 根据当前 resolution/range_* 重新计算像素尺寸和原点
    void computeImageSize();
};

// -------------------------------------------------------
// 标定结果
// -------------------------------------------------------
struct CalibResult {
    std::vector<CameraHomography> homographies;  // 4 个相机的单应矩阵（顺序：前/后/左/右）
    BevParams bev_params;
    double stitch_error_m;     // 拼接误差（m），评估棋盘格格点在 BEV 中的重投影残差
    bool valid;                // 标定是否成功
    std::string message;       // 结果描述信息
    // 质量等级：EXCELLENT(<0.01m) / GOOD(<0.02m) / ACCEPTABLE(<0.05m) / POOR
    std::string quality_grade;

    CalibResult();
};

// -------------------------------------------------------
// 相机内参
// -------------------------------------------------------
struct CameraIntrinsics {
    cv::Mat K;       // 3x3 内参矩阵
    cv::Mat D;       // 畸变系数：针孔(k1,k2,p1,p2[,k3]) 或 MEI鱼眼(k1,k2,k3,k4)
    bool is_fisheye; // true=MEI鱼眼模型(D.cols==4)，false=针孔模型
    int image_width;
    int image_height;

    CameraIntrinsics();
    CameraIntrinsics(double fx, double fy, double cx, double cy,
                     int width, int height);
};

// -------------------------------------------------------
// 相机外参（相机在 VCS 中的位姿）
// extrinsic: T_vcs_to_cam, 即将 VCS 坐标变换到相机坐标系
// P_cam = R * P_vcs + t
// -------------------------------------------------------
struct CameraExtrinsic {
    std::string camera_name;
    cv::Mat R;    // 3x3 旋转矩阵
    cv::Mat t;    // 3x1 平移向量（相机原点在 VCS 中的坐标，取负得平移到相机系）
    // 注：P_cam = R * P_vcs + t_cam, 其中 t_cam = -R * t_vcs
    // 本结构存储 R_vc (从VCS到CAM的旋转) 和 t_vc (从VCS到CAM的平移)
    // 即 P_cam = R_vc * P_vcs + t_vc
    bool valid;

    CameraExtrinsic();
};

// -------------------------------------------------------
// 标定板摆放位置：描述 findChessboardCorners 输出角点在 VCS 中的坐标
//
// 单相机标定时，角点 (row=r, col=c) 的 VCS 坐标为：
//   x = corner0_x + r * row_dx + c * col_dx
//   y = corner0_y + r * row_dy + c * col_dy
//
// corner0 是 findChessboardCorners 返回的第 0 个角点在 VCS 中的位置。
// row/col 步长方向取决于相机朝向，需与实际相机摆放一致。
//
// board_origin + board_lx/ly 额外描述板本地坐标系到 VCS 的仿射变换，
// 用于计算黑块等辅助特征在 VCS 中的坐标：
//   VCS = board_origin + bx * board_lx + by * board_ly
// （bx, by 为 chasiss_board.yaml 中的板本地坐标，原点=板左下角，X向右，Y向上）
// -------------------------------------------------------
struct BoardPlacement {
    double corner0_x;   // 角点(0,0) 的 VCS X 坐标（m）
    double corner0_y;   // 角点(0,0) 的 VCS Y 坐标（m）
    double row_dx;      // 行方向 VCS X 步长（m/row）
    double row_dy;      // 行方向 VCS Y 步长（m/row）
    double col_dx;      // 列方向 VCS X 步长（m/col）
    double col_dy;      // 列方向 VCS Y 步长（m/col）

    // 板本地坐标系到 VCS 的仿射变换（用于黑块角点等辅助特征）
    double board_origin_x;  // 板本地(0,0) 在 VCS 的 X 坐标
    double board_origin_y;  // 板本地(0,0) 在 VCS 的 Y 坐标
    double board_lx_dx;     // 板本地 X 方向（+1m）在 VCS 的 X 分量
    double board_lx_dy;     // 板本地 X 方向（+1m）在 VCS 的 Y 分量
    double board_ly_dx;     // 板本地 Y 方向（+1m）在 VCS 的 X 分量
    double board_ly_dy;     // 板本地 Y 方向（+1m）在 VCS 的 Y 分量

    BoardPlacement();

    // 计算棋盘格角点 (row, col) 的 VCS 坐标
    cv::Point2f cornerVcs(int row, int col) const;

    // 将板本地坐标 (bx, by) 转换为 VCS 坐标
    // bx/by：chasiss_board.yaml 中的板坐标（原点=板左下角，X向右，Y向上/深）
    cv::Point2f boardLocalToVcs(double bx, double by) const;
};

// -------------------------------------------------------
// 棋盘格标定板参数（支持前后大板+左右小板布局）
// -------------------------------------------------------
struct CheckerboardParams {
    // 前后板（大板）：两边黑块 + 中间棋盘格
    int front_rear_corner_rows;       // 前后板角点行数（默认4）
    int front_rear_corner_cols;       // 前后板角点列数（默认6）
    double front_rear_square_size;    // 前后板格子尺寸（m，默认0.2）

    // 左右板（小板）：纯棋盘格
    int left_right_corner_rows;       // 左右板角点行数（默认6）
    int left_right_corner_cols;       // 左右板角点列数（默认4）
    double left_right_square_size;    // 左右板格子尺寸（m，默认0.2）

    // 前后板黑块几何（板本地坐标系，从 chasiss_board.yaml 读取）
    // 左黑块：板本地 X 范围 [black_left_x1, black_left_x2]
    // 右黑块：板本地 X 范围 [black_right_x1, black_right_x2]
    // 两块共用 Y 范围 [black_y1, black_y2]
    double black_left_x1;    // 左黑块左边 X（板本地）= x_start = 0.1
    double black_left_x2;    // 左黑块右边 X（板本地）= x_start + width = 1.3
    double black_right_x1;   // 右黑块左边 X（板本地）= x_start = 3.1
    double black_right_x2;   // 右黑块右边 X（板本地）= x_start + width = 4.3
    double black_y1;          // 黑块 Y 下边（板本地）= y_start = 0.1
    double black_y2;          // 黑块 Y 上边（板本地）= y_start + height = 1.1

    CheckerboardParams();
};

// -------------------------------------------------------
// 检测结果可视化数据
// -------------------------------------------------------
struct DetectionVis {
    std::vector<cv::Point2f> corners;  // 检测到的角点
    bool detected;                      // 是否检测成功
    std::string camera_name;            // 相机名称
    cv::Mat vis_image;                  // 可视化图像（绘制了角点）
};

} // namespace bev_surround_view
