#pragma once
#include <QWidget>
#include <QLabel>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#if AUTO_CALIB_VTK9_OR_NEWER
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#else
#include <QVTKWidget.h>
#endif
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkLookupTable.h>
#include <vtkCallbackCommand.h>
#include <vtkPointPicker.h>
#include <vector>
#include <Eigen/Dense>

namespace calibration_gui {

/// 基于 QVTKWidget 的点云俯视图查看器
class PointCloudViewer : public QWidget {
    Q_OBJECT
public:
    explicit PointCloudViewer(QWidget* parent = nullptr);
    ~PointCloudViewer();

    /// 显示单个点云（默认白色）
    void showCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                   const std::string& name = "cloud",
                   double r = 1.0, double g = 1.0, double b = 1.0,
                   double point_size = 2.0);

    /// 显示逐点真彩色点云（PointXYZRGB，直接使用每个点的 RGB）
    void showCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                      double point_size = 2.0);

    /// 显示两个点云（主雷达红色，辅雷达绿色）
    void showDualCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2);

    /// 显示原始点云（小点，灰白色）+ 检测目标点云（大点，高亮橙色）叠加
    /// raw_cloud: 原始传感器数据，点小(1.5px)，灰白 #aaaaaa
    /// target_cloud: 算法检测目标（地面/锥桶），点大(4px)，橙色/绿色
    void showRawAndTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
                          double tr, double tg, double tb);

    /// 显示点云 + 锥桶标记（黄色球）
    void showCloudWithCones(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            const std::vector<Eigen::Vector3f>& cone_centers);

    /// 显示锥桶连线（用于 yaw 标定可视化）
    void showConeLine(const std::vector<Eigen::Vector3f>& cone_centers,
                      double angle_deg);

    /// 追加一个点云 actor（不清空已有内容，不触发渲染，配合 clear()/renderNow() 使用）
    void addCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                  double r, double g, double b,
                  double point_size = 2.0);

    /// 触发渲染（配合 addCloud 使用）
    void renderNow();

    /// 清空显示
    void clear(bool render = true);

    /// 显示"正在加载..."覆盖提示
    void showLoading(bool loading);

    /// 重置相机视角（俯视图）
    void resetView();

    /// 开启/关闭选点模式：开启后鼠标左键点击会 emit pointPicked 信号
    void setPickMode(bool enabled);
    bool isPickMode() const { return pick_mode_; }

    QSize sizeHint() const override { return QSize(640, 480); }

signals:
    void pointPicked(float x, float y, float z);

private:
#if AUTO_CALIB_VTK9_OR_NEWER
    QVTKOpenGLNativeWidget*      vtk_widget_;
#else
    QVTKWidget*                  vtk_widget_;
#endif
    vtkSmartPointer<vtkRenderer> renderer_;
    QLabel*                      loading_label_{nullptr};  // "正在加载..."覆盖层

    // ---- 选点模式 ----
    bool                              pick_mode_{false};
    vtkSmartPointer<vtkPointPicker>   picker_;
    unsigned long                     pick_observer_tag_{0};
    static void onVtkLeftClick(vtkObject* caller, unsigned long eventId,
                               void* clientData, void* callData);

    vtkSmartPointer<vtkActor> makeCloudActor(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        double r, double g, double b,
        double point_size = 2.0);
    vtkSmartPointer<vtkActor> makeCloudActorRGB(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        double point_size = 2.0);
    vtkSmartPointer<vtkActor> makeSphereActor(
        const Eigen::Vector3f& center, double radius,
        double r, double g, double b);
    vtkSmartPointer<vtkActor> makeLineActor(
        const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
        double r, double g, double b);
};

} // namespace calibration_gui
