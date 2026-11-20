
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>

// =====================================================
// 1. 参数配置
// =====================================================

// 棋盘格内部角点数量：9列、6行
const cv::Size CHESSBOARD_SIZE(9, 6);

// 棋盘格单格实际尺寸，单位 mm
const float SQUARE_SIZE_MM = 100.0f;

// 棋盘格实际外边界尺寸：10列、7行
const int BOARD_COLS = 10;
const int BOARD_ROWS = 7;

// BEV 横向范围：车辆左侧为负，右侧为正
const double X_MIN = -3000.0;
const double X_MAX =  3000.0;

// BEV 前方范围
const double Y_MIN = 0.0;
const double Y_MAX = 6000.0;

// 10mm 对应 1 像素
const double PIXELS_PER_MM = 0.1;

// =====================================================
// 2. 棋盘格在车辆坐标系中的实际位置
// =====================================================

// 这里定义棋盘格左侧、远端的外边界坐标。
// 假设棋盘格沿车辆横向和纵向摆正。
// 需要根据实际测量修改这两个值。

const double BOARD_LEFT_X_MM = -500.0;
const double BOARD_FAR_Y_MM  = 1700.0;

// =====================================================
// 3. 生成 BEV 目标坐标
// =====================================================

cv::Point2f groundToBEV(const cv::Point2f& p)
{
    // X 向右为正，BEV 图像 u 向右增加
    float u = static_cast<float>(
        (p.x - X_MIN) * PIXELS_PER_MM
    );

    // Y 向前为正，BEV 图像 v 向下增加
    // 所以前方对应图像上方
    float v = static_cast<float>(
        (Y_MAX - p.y) * PIXELS_PER_MM
    );

    return cv::Point2f(u, v);
}

// =====================================================
// 4. 检测棋盘格角点
// =====================================================

bool detectChessboard(
    const cv::Mat& image,
    std::vector<cv::Point2f>& imagePoints)
{
    if (image.empty())
        return false;

    cv::Mat gray;

    cv::cvtColor(
        image,
        gray,
        cv::COLOR_BGR2GRAY
    );

    bool found = cv::findChessboardCorners(
        gray,
        CHESSBOARD_SIZE,
        imagePoints,
        cv::CALIB_CB_ADAPTIVE_THRESH |
        cv::CALIB_CB_NORMALIZE_IMAGE
    );

    if (!found)
    {
        std::cerr << "棋盘格检测失败！" << std::endl;
        return false;
    }

    // 亚像素优化
    cv::cornerSubPix(
        gray,
        imagePoints,
        cv::Size(5, 5),
        cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::EPS |
            cv::TermCriteria::COUNT,
            30,
            0.001
        )
    );

    return true;
}

// =====================================================
// 5. 建立棋盘格物理坐标
// =====================================================

void createGroundPoints(
    std::vector<cv::Point2f>& groundPoints)
{
    groundPoints.clear();

    for (int row = 0; row < CHESSBOARD_SIZE.height; ++row)
    {
        for (int col = 0; col < CHESSBOARD_SIZE.width; ++col)
        {
            // 图像中从左向右：
            // 对应车辆坐标系 X 从左向右增加

            float x = static_cast<float>(
                BOARD_LEFT_X_MM + (col + 1) * SQUARE_SIZE_MM
            );

            // 图像中从远端向近端：
            // 对应车辆坐标系 Y 逐渐减小

            float y = static_cast<float>(
                BOARD_FAR_Y_MM - (row + 1) * SQUARE_SIZE_MM
            );

            groundPoints.emplace_back(x, y);
        }
    }
}

// =====================================================
// 6. 计算 Homography
// =====================================================

bool calculateHomography(
    const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point2f>& groundPoints,
    cv::Mat& H,
    double& meanError)
{
    if (imagePoints.size() != groundPoints.size() ||
        imagePoints.size() < 4)
    {
        return false;
    }

    std::vector<cv::Point2f> bevPoints;

    for (const auto& p : groundPoints)
    {
        bevPoints.push_back(groundToBEV(p));
    }

    // 原图像素坐标 -> BEV 像素坐标
    H = cv::findHomography(
        imagePoints,
        bevPoints,
        cv::RANSAC,
        3.0
    );

    if (H.empty())
        return false;

    // 计算重投影误差
    std::vector<cv::Point2f> projectedPoints;

    cv::perspectiveTransform(
        imagePoints,
        projectedPoints,
        H
    );

    double totalError = 0.0;

    for (size_t i = 0; i < bevPoints.size(); ++i)
    {
        double error = cv::norm(
            projectedPoints[i] - bevPoints[i]
        );

        totalError += error;
    }

    meanError = totalError / bevPoints.size();

    return true;
}

// =====================================================
// 7. 保存标定参数
// =====================================================

bool saveCalibration(
    const cv::Mat& H,
    int imageWidth,
    int imageHeight)
{
    cv::FileStorage fs(
        "output/calibration.yml",
        cv::FileStorage::WRITE
    );

    if (!fs.isOpened())
        return false;

    fs << "homography" << H;

    fs << "image_width" << imageWidth;
    fs << "image_height" << imageHeight;

    fs << "x_min_mm" << X_MIN;
    fs << "x_max_mm" << X_MAX;

    fs << "y_min_mm" << Y_MIN;
    fs << "y_max_mm" << Y_MAX;

    fs << "pixels_per_mm" << PIXELS_PER_MM;

    fs << "square_size_mm" << SQUARE_SIZE_MM;

    fs.release();

    return true;
}

// =====================================================
// 8. 主函数
// =====================================================

int main()
{
    // 创建输出目录
    std::filesystem::create_directories("output");

    // -------------------------------------------------
    // 第一步：读取棋盘格标定图片
    // -------------------------------------------------

    cv::Mat calibrationImage = cv::imread(
        "data/chessboard.png",
        cv::IMREAD_COLOR
    );

    if (calibrationImage.empty())
    {
        std::cerr << "棋盘格图片读取失败！" << std::endl;
        return -1;
    }

    // -------------------------------------------------
    // 第二步：检测棋盘格角点
    // -------------------------------------------------

    std::vector<cv::Point2f> imagePoints;

    if (!detectChessboard(
        calibrationImage,
        imagePoints))
    {
        return -1;
    }

    std::cout << "检测到角点数量："
              << imagePoints.size()
              << std::endl;

    cv::Mat cornerImage = calibrationImage.clone();

    cv::drawChessboardCorners(
        cornerImage,
        CHESSBOARD_SIZE,
        imagePoints,
        true
    );

    cv::imwrite(
        "output/corners.png",
        cornerImage
    );

    // -------------------------------------------------
    // 第三步：生成棋盘格地面物理坐标
    // -------------------------------------------------

    std::vector<cv::Point2f> groundPoints;

    createGroundPoints(groundPoints);

    // -------------------------------------------------
    // 第四步：计算 Homography
    // -------------------------------------------------

    cv::Mat H;
    double meanError = 0.0;

    if (!calculateHomography(
        imagePoints,
        groundPoints,
        H,
        meanError))
    {
        std::cerr << "Homography 计算失败！" << std::endl;
        return -1;
    }

    std::cout << "\nHomography 矩阵：\n"
              << H << std::endl;

    std::cout << "\n平均重投影误差："
              << meanError << " pixel"
              << std::endl;

    // -------------------------------------------------
    // 第五步：计算 BEV 输出尺寸
    // -------------------------------------------------

    int bevWidth = static_cast<int>(
        std::ceil((X_MAX - X_MIN) * PIXELS_PER_MM)
    );

    int bevHeight = static_cast<int>(
        std::ceil((Y_MAX - Y_MIN) * PIXELS_PER_MM)
    );

    std::cout << "\nBEV 输出尺寸："
              << bevWidth << " x "
              << bevHeight << std::endl;

    // -------------------------------------------------
    // 第六步：读取完整道路原始图片
    // -------------------------------------------------

    cv::Mat roadImage = cv::imread(
        "data/road.png",
        cv::IMREAD_COLOR
    );

    if (roadImage.empty())
    {
        std::cerr << "道路图片读取失败！" << std::endl;
        return -1;
    }

    // 必须与标定图片具有相同的图像尺寸
    if (roadImage.size() != calibrationImage.size())
    {
        std::cerr << "标定图片和道路图片尺寸不一致！"
                  << std::endl;
        return -1;
    }

    // -------------------------------------------------
    // 第七步：整张道路图像执行 BEV 变换
    // -------------------------------------------------

    cv::Mat bevImage;

    cv::warpPerspective(
        roadImage,
        bevImage,
        H,
        cv::Size(bevWidth, bevHeight),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0, 0, 0)
    );

    // -------------------------------------------------
    // 第八步：保存 BEV 结果
    // -------------------------------------------------

    cv::imwrite(
        "output/bev_result.png",
        bevImage
    );

    // -------------------------------------------------
    // 第九步：保存标定参数
    // -------------------------------------------------

    if (!saveCalibration(
        H,
        calibrationImage.cols,
        calibrationImage.rows))
    {
        std::cerr << "标定参数保存失败！" << std::endl;
        return -1;
    }

    // -------------------------------------------------
    // 第十步：显示结果
    // -------------------------------------------------

    cv::imshow("Original Road Image", roadImage);

    cv::imshow("Chessboard Corners", cornerImage);

    cv::imshow("Road BEV Result", bevImage);

    std::cout << "\nBEV 处理完成！" << std::endl;

    cv::waitKey(0);

    return 0;
}

