#pragma once
/**
 * @file mei.h
 * @brief Mei相机模型 - 广角/鱼眼镜头标定与畸变校正
 *
 * 该模型基于 Kannala-Brandt 相机模型，适用于广角镜头和鱼眼镜头。
 * 支持: 内参/外参设置、畸变校正、去畸变映射、IPM(逆透视映射)等功能
 */

#include <array>
#include <mutex>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace camera {

/**
 * @brief 去畸变信息类
 *
 * 存储图像去畸变所需的参数和预计算的映射表
 */
class UndistortInfo {
public:
    UndistortInfo();

    /**
     * @brief 设置去畸变信息
     * @param width 去畸变后图像宽度
     * @param height 去畸变后图像高度
     * @param undistort_K 去畸变目标相机内参矩阵 (3x3)
     */
    void setInfo(int width,
                 int height,
                 const cv::Mat& undistort_K);

    /** @brief 获取去畸变后图像宽度 */
    int getWidth() const;

    /** @brief 获取去畸变后图像高度 */
    int getHeight() const;

    /** @brief 获取去畸变目标相机内参矩阵 */
    cv::Mat getK() const;

    /** @brief 检查去畸变信息是否已准备就绪 */
    bool isReady() const;

    /** @brief 清除预计算的映射表 */
    void clearMap();

private:
    friend class Mei;  ///< Mei类访问私有成员

    bool sameInfo(int width,
                  int height,
                  const cv::Mat& undistort_K) const;  ///< 检查参数是否相同
    bool hasMap() const;  ///< 检查映射表是否存在
    void updateMap(const cv::Mat& map1,
                   const cv::Mat& map2);  ///< 更新映射表
    void copyMap(cv::Mat* map1,
                 cv::Mat* map2) const;  ///< 复制映射表

    int width_;          ///< 去畸变后图像宽度
    int height_;         ///< 去畸变后图像高度
    cv::Mat K_;          ///< 去畸变目标相机内参矩阵
    cv::Mat map1_;       ///< 去畸变映射表1 (x方向)
    cv::Mat map2_;       ///< 去畸变映射表2 (y方向)
};

/**
 * @brief IPM(逆透视映射)信息类
 *
 * 存储俯视图(BEV)生成所需的参数和预计算的映射表
 */
class IpmInfo {
public:
    IpmInfo();

    /**
     * @brief 设置IPM信息
     * @param width IPM图像宽度
     * @param height IPM图像高度
     * @param scale 像素到世界的缩放比例 (像素/米)
     * @param origin_offset_x 原点X方向偏移(像素)
     * @param origin_offset_y 原点Y方向偏移(像素)
     * @param roi_top ROI顶部边界
     * @param roi_bottom ROI底部边界
     * @param roi_left ROI左边界
     * @param roi_right ROI右边界
     */
    void setInfo(int width,
                 int height,
                 double scale,
                 double origin_offset_x,
                 double origin_offset_y,
                 int roi_top = 0,
                 int roi_bottom = 0,
                 int roi_left = 0,
                 int roi_right = 0);

    /** @brief 获取IPM图像宽度 */
    int getWidth() const;

    /** @brief 获取IPM图像高度 */
    int getHeight() const;

    /** @brief 获取缩放比例 */
    double getScale() const;

    /** @brief 获取原点X偏移 */
    double getOriginOffsetX() const;

    /** @brief 获取原点Y偏移 */
    double getOriginOffsetY() const;

    /** @brief 获取ROI顶部边界 */
    int getRoiTop() const;

    /** @brief 获取ROI底部边界 */
    int getRoiBottom() const;

    /** @brief 获取ROI左边界 */
    int getRoiLeft() const;

    /** @brief 获取ROI右边界 */
    int getRoiRight() const;

    /** @brief 检查IPM信息是否已准备就绪 */
    bool isReady() const;

    /** @brief 清除预计算的映射表 */
    void clearMap();

private:
    friend class Mei;  ///< Mei类访问私有成员

    bool sameInfo(int width,
                  int height,
                  double scale,
                  double origin_offset_x,
                  double origin_offset_y,
                  int roi_top,
                  int roi_bottom,
                  int roi_left,
                  int roi_right) const;  ///< 检查参数是否相同
    bool hasMap() const;  ///< 检查映射表是否存在
    void updateMap(const cv::Mat& map_x,
                   const cv::Mat& map_y);  ///< 更新映射表
    void copyMap(cv::Mat* map_x,
                 cv::Mat* map_y) const;  ///< 复制映射表

    int width_;              ///< IPM图像宽度
    int height_;             ///< IPM图像高度
    double scale_;          ///< 像素到世界坐标的缩放比例
    double origin_offset_x_; ///< 原点X方向偏移
    double origin_offset_y_; ///< 原点Y方向偏移
    int roi_top_;           ///< ROI顶部边界
    int roi_bottom_;        ///< ROI底部边界
    int roi_left_;          ///< ROI左边界
    int roi_right_;         ///< ROI右边界
    cv::Mat map_x_;         ///< IPM映射表X方向
    cv::Mat map_y_;         ///< IPM映射表Y方向
};

/**
 * @brief Mei相机模型类
 *
 * 基于 Kannala-Brandt 广角/鱼眼相机模型
 * 支持: 内参(K, D, epsilon)、外参(R, T)设置、畸变校正、IPM生成
 */
class Mei {
public:
    /**
     * @brief 默认构造函数
     * @param rotation_order 旋转顺序, 默认 {2,1,0} 表示 Z-Y-X 顺序
     */
    explicit Mei(const std::array<int, 3>& rotation_order = std::array<int, 3>{{2, 1, 0}});
    Mei(const Mei& other);
    Mei(Mei&& other);
    Mei& operator=(const Mei& other);
    Mei& operator=(Mei&& other);

    /**
     * @brief 全参数构造函数(欧拉角外参)
     * @param epsilon 鱼眼镜头参数 epsilon
     * @param K 相机内参矩阵 (3x3)
     * @param D 畸变系数 (1x4): k1, k2, p1, p2
     * @param image_width 图像宽度
     * @param image_height 图像高度
     * @param ground_height 地面高度
     * @param t_world_to_camera 世界到相机平移向量
     * @param r_world_to_camera_euler 世界到相机旋转欧拉角
     * @param rotation_order 旋转顺序
     */
    Mei(double epsilon,
        const cv::Mat& K,
        const cv::Mat& D,
        int image_width,
        int image_height,
        double ground_height,
        const Eigen::Vector3d& t_world_to_camera,
        const Eigen::Vector3d& r_world_to_camera_euler,
        const std::array<int, 3>& rotation_order = std::array<int, 3>{{2, 1, 0}});

    /**
     * @brief 全参数构造函数(旋转矩阵外参)
     * @param epsilon 鱼眼镜头参数 epsilon
     * @param K 相机内参矩阵 (3x3)
     * @param D 畸变系数 (1x4): k1, k2, p1, p2
     * @param image_width 图像宽度
     * @param image_height 图像高度
     * @param ground_height 地面高度
     * @param t_world_to_camera 世界到相机平移向量
     * @param R_world_to_camera 世界到相机旋转矩阵
     * @param rotation_order 旋转顺序
     */
    Mei(double epsilon,
        const cv::Mat& K,
        const cv::Mat& D,
        int image_width,
        int image_height,
        double ground_height,
        const Eigen::Vector3d& t_world_to_camera,
        const Eigen::Matrix3d& R_world_to_camera,
        const std::array<int, 3>& rotation_order = std::array<int, 3>{{2, 1, 0}});

    // ========== Epsilon (鱼眼参数) ==========

    /** @brief 设置鱼眼镜头参数 epsilon */
    void setEpsilon(double epsilon);

    /** @brief 获取鱼眼镜头参数 epsilon */
    double getEpsilon() const;

    // ========== 图像尺寸 ==========

    /** @brief 设置图像宽度 */
    void setImageWidth(int image_width);

    /** @brief 获取图像宽度 */
    int getImageWidth() const;

    /** @brief 设置图像高度 */
    void setImageHeight(int image_height);

    /** @brief 获取图像高度 */
    int getImageHeight() const;

    // ========== 地面高度 ==========

    /** @brief 设置地面高度(用于IPM) */
    void setGroundHeight(double ground_height);

    /** @brief 获取地面高度 */
    double getGroundHeight() const;

    // ========== 内参 K ==========

    /** @brief 设置相机内参矩阵 */
    void setK(const cv::Mat& K);

    /** @brief 获取相机内参矩阵 */
    cv::Mat getK() const;

    // ========== 畸变系数 D ==========

    /** @brief 设置畸变系数 (k1, k2, p1, p2) */
    void setD(const cv::Mat& D);

    /** @brief 获取畸变系数 */
    cv::Mat getD() const;

    // ========== 外参 T ==========

    /** @brief 设置世界到相机平移向量 */
    void setT(const Eigen::Vector3d& t_world_to_camera);

    /** @brief 获取世界到相机平移向量 */
    Eigen::Vector3d getT() const;

    // ========== 外参 R ==========

    /** @brief 设置自有相机定义标准旋转量(欧拉角) */
    void setR(const Eigen::Vector3d& r_euler);

    /** @brief 设置自有相机定义标准旋转量(旋转矩阵) */
    void setR(const Eigen::Matrix3d& R);

    /** @brief 设置世界到相机旋转(旋转矩阵)，自动应用坐标系转换 */
    void setWorldToCameraR(const Eigen::Matrix3d& R_world_to_camera);

    /** @brief 获取世界到相机旋转(旋转矩阵)，自动应用逆坐标系转换 */
    Eigen::Matrix3d getWorldToCameraR() const;

    /** @brief 获取自有相机定义标准旋转量(旋转矩阵) */
    Eigen::Matrix3d getR() const;

    /** @brief 获取自有相机定义标准旋转量(欧拉角) */
    Eigen::Vector3d getREuler() const;

    // ========== 综合设置 ==========

    /**
     * @brief 批量设置内参
     * @param epsilon 鱼眼参数
     * @param K 内参矩阵
     * @param D 畸变系数
     * @param image_width 图像宽度
     * @param image_height 图像高度
     */
    void setIntrinsics(double epsilon,
                       const cv::Mat& K,
                       const cv::Mat& D,
                       int image_width,
                       int image_height);

    /**
     * @brief 批量设置外参(欧拉角)
     * @param t_world_to_camera 平移向量
     * @param r_world_to_camera_euler 旋转欧拉角
     */
    void setExtrinsics(const Eigen::Vector3d& t_world_to_camera,
                       const Eigen::Vector3d& r_world_to_camera_euler);

    /**
     * @brief 批量设置外参(旋转矩阵)
     * @param t_world_to_camera 平移向量
     * @param R_world_to_camera 旋转矩阵
     */
    void setExtrinsics(const Eigen::Vector3d& t_world_to_camera,
                       const Eigen::Matrix3d& R_world_to_camera);

    /** @brief 设置旋转顺序 */
    void setRotationOrder(const std::array<int, 3>& rotation_order);

    // ========== 去畸变信息 ==========

    /**
     * @brief 设置去畸变参数
     * @param width 去畸变后图像宽度
     * @param height 去畸变后图像高度
     * @param undistort_K 去畸变目标内参矩阵
     */
    void setUndistortInfo(int width,
                          int height,
                          const cv::Mat& undistort_K);

    /** @brief 获取去畸变信息快照 */
    UndistortInfo getUndistortInfo() const;

    // ========== IPM信息 ==========

    /**
     * @brief 设置IPM参数
     * @param width IPM图像宽度
     * @param height IPM图像高度
     * @param scale 缩放比例
     * @param origin_offset_x 原点X偏移
     * @param origin_offset_y 原点Y偏移
     * @param roi_top ROI顶部
     * @param roi_bottom ROI底部
     * @param roi_left ROI左边
     * @param roi_right ROI右边
     */
    void setIpmInfo(int width,
                    int height,
                    double scale,
                    double origin_offset_x,
                    double origin_offset_y,
                    int roi_top = 0,
                    int roi_bottom = 0,
                    int roi_left = 0,
                    int roi_right = 0);

    /** @brief 获取IPM信息快照 */
    IpmInfo getIpmInfo() const;

    // ========== 坐标变换 ==========

    /**
     * @brief 世界坐标转相机坐标
     * @param point_world 世界坐标点
     * @param point_camera 输出: 相机坐标点
     * @return 转换是否成功
     */
    bool world3dToCamera3d(const Eigen::Vector3d& point_world,
                           Eigen::Vector3d& point_camera) const;


    /**
     * @brief 相机坐标转世界坐标
     * @param point_camera 相机坐标点
     * @param point_world 输出: 世界坐标点
     * @return 转换是否成功
     */                       
    bool camera3dToWorld3d(const Eigen::Vector3d& point_camera,
                           Eigen::Vector3d& point_world) const;

    /**
     * @brief 相机3D坐标投影到2D图像点
     * @param point_camera 相机坐标点
     * @param point_2d 输出: 图像2D点
     * @return 投影是否成功
     */
    bool projectCamera3dTo2d(const Eigen::Vector3d& point_camera,
                             cv::Point2d& point_2d) const;

    /**
     * @brief 图像2D点反投影到相机3D点
     * @param point_2d 图像2D点
     * @param z 指定相机坐标系深度
     * @param point_camera 输出: 相机坐标点
     * @return 反投影是否成功
     */
    bool projectCamera2dToCamera3d(const cv::Point2d& point_2d,
                                   const double z,
                                   Eigen::Vector3d& point_camera) const;

    /**
     * @brief 图像2D点反投影到世界3D点
     * @param point_2d 图像2D点
     * @param camera_z 指定相机坐标系深度
     * @param point_world 输出: 世界坐标点
     * @return 反投影是否成功
     */
    bool projectCamera2dToWorld3d(const cv::Point2d& point_2d,
                                  const double camera_z,
                                  Eigen::Vector3d& point_world) const;
    /**
     * @brief 世界3D坐标直接投影到2D图像点
     * @param point_world 世界坐标点
     * @param point_2d 输出: 图像2D点
     * @return 投影是否成功
     */
    bool projectWorld3dTo2d(const Eigen::Vector3d& point_world,
                            cv::Point2d& point_2d) const;

    // ========== 去畸变 ==========

    /**
     * @brief 单点去畸变
     * @param distorted_point 原始畸变点
     * @param undistorted_point 输出: 去畸变后点
     * @param undistort_k 可选: 指定去畸变目标内参, 为空则使用undistort_K
     * @return 去畸变是否成功
     */
    bool undistort(const cv::Point2d& distorted_point,
                   cv::Point2d& undistorted_point,
                   const cv::Mat& undistort_k = cv::Mat()) const;

    /**
     * @brief 生成去畸变映射表
     * @param map1 输出: 映射表1 (可为空)
     * @param map2 输出: 映射表2 (可为空)
     * @return 映射表生成是否成功
     */
    bool generateUndistortMap(cv::Mat* map1 = nullptr,
                              cv::Mat* map2 = nullptr);

    /**
     * @brief 对整张图像去畸变
     * @param image 输入畸变图像
     * @param undistorted_image 输出去畸变图像
     * @return 去畸变是否成功
     */
    bool undistortImage(const cv::Mat& image,
                        cv::Mat& undistorted_image);

    // ========== IPM (逆透视映射) ==========

    /**
     * @brief 生成IPM映射表
     * @param map_x 输出: X方向映射表 (可为空)
     * @param map_y 输出: Y方向映射表 (可为空)
     * @return 映射表生成是否成功
     */
    bool generateIpmMap(cv::Mat* map_x = nullptr,
                        cv::Mat* map_y = nullptr);

    /**
     * @brief 生成IPM俯视图
     * @param image 输入原始图像
     * @param bev_image 输出俯视图
     * @return IPM生成是否成功
     */
    bool generateIpmImage(const cv::Mat& image,
                          cv::Mat& bev_image);

private:
    double epsilon_;                      ///< 鱼眼镜头参数
    cv::Mat K_;                           ///< 相机内参矩阵
    cv::Mat D_;                           ///< 畸变系数
    Eigen::Vector3d t_world_to_camera_;  ///< 世界到相机平移
    Eigen::Vector3d r_world_to_camera_euler_;  ///< 世界到相机旋转欧拉角
    Eigen::Matrix3d R_world_to_camera_;  ///< 世界到相机旋转矩阵
    int image_width_;                    ///< 图像宽度
    int image_height_;                   ///< 图像高度
    double ground_height_;               ///< 地面高度
    std::array<int, 3> rotation_order_; ///< 旋转顺序

    UndistortInfo undistort_info_;       ///< 去畸变信息
    IpmInfo ipm_info_;                   ///< IPM信息
    mutable std::recursive_mutex mutex_; ///< Mei对象级锁

    static cv::Mat normalizeK(const cv::Mat& K);  ///< 标准化内参矩阵
    static cv::Mat normalizeD(const cv::Mat& D);  ///< 标准化畸变系数
    static std::array<int, 3> normalizeRotationOrder(const std::array<int, 3>& rotation_order);  ///< 标准化旋转顺序
    Eigen::Matrix3d eulerToRotation(const Eigen::Vector3d& r_euler) const;  ///< 欧拉角转旋转矩阵
    Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& rotation) const;  ///< 旋转矩阵转欧拉角

    /** @brief 生成全向去畸变映射表(鱼眼镜头) */
    static bool generateOmniUndistortMap(const cv::Mat& K,
                                         const cv::Mat& D,
                                         double epsilon,
                                         const cv::Mat& new_K,
                                         const cv::Size& size,
                                         cv::Mat& map1,
                                         cv::Mat& map2);
};

}  // namespace camera
