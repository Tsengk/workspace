#include "calibration_gui/point_cloud_viewer.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QStackedLayout>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkUnsignedCharArray.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>

namespace calibration_gui {

PointCloudViewer::PointCloudViewer(QWidget* parent) : QWidget(parent)
{
#if AUTO_CALIB_VTK9_OR_NEWER
    vtk_widget_ = new QVTKOpenGLNativeWidget(this);
    auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtk_widget_->SetRenderWindow(render_window);
#else
    vtk_widget_ = new QVTKWidget(this);
#endif
    renderer_   = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetBackground(0.08, 0.08, 0.15);
    vtk_widget_->GetRenderWindow()->AddRenderer(renderer_);

    picker_ = vtkSmartPointer<vtkPointPicker>::New();
    picker_->SetTolerance(0.01);

    // "正在加载..." 覆盖层（叠加在 VTK 上方）
    loading_label_ = new QLabel("⏳  正在加载，等待第一帧数据...", this);
    loading_label_->setAlignment(Qt::AlignCenter);
    loading_label_->setStyleSheet(
        "QLabel { color: #ffcc00; font-size: 14px; font-weight: bold; "
        "         background: rgba(13,13,26,180); border-radius: 6px; padding: 10px 20px; }");
    loading_label_->hide();

    // 用 QStackedLayout 将 loading_label_ 绝对叠加在 vtk_widget_ 上
    auto* stacked = new QStackedLayout(this);
    stacked->setStackingMode(QStackedLayout::StackAll);
    stacked->setContentsMargins(0, 0, 0, 0);
    stacked->addWidget(vtk_widget_);
    stacked->addWidget(loading_label_);
    stacked->setCurrentIndex(0);  // vtk 在底层，loading 在顶层但隐藏

    resetView();
}

PointCloudViewer::~PointCloudViewer() {}

void PointCloudViewer::showLoading(bool loading)
{
    loading_label_->setVisible(loading);
    // 确保 loading label 居中显示（绝对定位在 vtk 中央）
    if (loading) {
        loading_label_->resize(width() * 2 / 3, 50);
        loading_label_->move((width() - loading_label_->width()) / 2,
                              (height() - loading_label_->height()) / 2);
        loading_label_->raise();
    }
}

void PointCloudViewer::showCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                  const std::string& /*name*/,
                                  double r, double g, double b,
                                  double point_size)
{
    showLoading(false);
    renderer_->RemoveAllViewProps();
    if (cloud && !cloud->empty()) {
        renderer_->AddActor(makeCloudActor(cloud, r, g, b, point_size));
    }
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::showCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                     double point_size)
{
    showLoading(false);
    renderer_->RemoveAllViewProps();
    if (cloud && !cloud->empty()) {
        renderer_->AddActor(makeCloudActorRGB(cloud, point_size));
    }
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::showDualCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                                      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2)
{
    showLoading(false);
    renderer_->RemoveAllViewProps();
    // 主雷达：亮红色，点稍大(3px)
    if (cloud1 && !cloud1->empty())
        renderer_->AddActor(makeCloudActor(cloud1, 1.0, 0.25, 0.25, 3.0));
    // 副雷达：亮绿色，点稍大(3px)
    if (cloud2 && !cloud2->empty())
        renderer_->AddActor(makeCloudActor(cloud2, 0.25, 1.0, 0.25, 3.0));
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::showRawAndTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud,
                                         double tr, double tg, double tb)
{
    showLoading(false);
    renderer_->RemoveAllViewProps();
    // 原始点云：小点(1.5px)，暗灰色，透明感
    if (raw_cloud && !raw_cloud->empty())
        renderer_->AddActor(makeCloudActor(raw_cloud, 0.45, 0.45, 0.45, 1.5));
    // 目标点云：大点(8px)，高亮颜色，视觉突出
    if (target_cloud && !target_cloud->empty())
        renderer_->AddActor(makeCloudActor(target_cloud, tr, tg, tb, 8.0));
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::showCloudWithCones(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           const std::vector<Eigen::Vector3f>& cone_centers)
{
    showLoading(false);
    renderer_->RemoveAllViewProps();
    // 背景点云：灰色小点
    if (cloud && !cloud->empty())
        renderer_->AddActor(makeCloudActor(cloud, 0.5, 0.5, 0.5, 1.5));
    // 锥桶：橙黄色球，大且醒目
    for (const auto& c : cone_centers)
        renderer_->AddActor(makeSphereActor(c, 0.18, 1.0, 0.75, 0.0));
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::showConeLine(const std::vector<Eigen::Vector3f>& cone_centers,
                                     double /*angle_deg*/)
{
    // 连接相邻锥桶
    for (size_t i = 0; i + 1 < cone_centers.size(); ++i) {
        renderer_->AddActor(makeLineActor(
            cone_centers[i], cone_centers[i + 1], 0.0, 1.0, 1.0));
    }
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::addCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                 double r, double g, double b,
                                 double point_size)
{
    if (cloud && !cloud->empty())
        renderer_->AddActor(makeCloudActor(cloud, r, g, b, point_size));
}

void PointCloudViewer::renderNow()
{
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::clear(bool render)
{
    showLoading(false);
    renderer_->RemoveAllViewProps();
    if (render)
        vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::resetView()
{
    vtkCamera* cam = renderer_->GetActiveCamera();
    cam->SetPosition(0, 0, 50);
    cam->SetFocalPoint(0, 0, 0);
    cam->SetViewUp(1, 0, 0);
    cam->SetParallelProjection(1);
    cam->SetParallelScale(20.0);
    vtk_widget_->GetRenderWindow()->Render();
}

void PointCloudViewer::setPickMode(bool enabled)
{
    pick_mode_ = enabled;
    auto* interactor = vtk_widget_->GetRenderWindow()->GetInteractor();
    if (!interactor) return;
    if (enabled && pick_observer_tag_ == 0) {
        auto* cmd = vtkCallbackCommand::New();
        cmd->SetCallback(&PointCloudViewer::onVtkLeftClick);
        cmd->SetClientData(this);
        pick_observer_tag_ = interactor->AddObserver(
            vtkCommand::LeftButtonPressEvent, cmd);
        cmd->Delete();
    } else if (!enabled && pick_observer_tag_ != 0) {
        interactor->RemoveObserver(pick_observer_tag_);
        pick_observer_tag_ = 0;
    }
}

void PointCloudViewer::onVtkLeftClick(vtkObject* caller, unsigned long,
                                       void* clientData, void*)
{
    auto* self = static_cast<PointCloudViewer*>(clientData);
    if (!self->pick_mode_) return;
    auto* interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    int* pos = interactor->GetEventPosition();
    self->picker_->Pick(pos[0], pos[1], 0, self->renderer_);
    if (self->picker_->GetPointId() >= 0) {
        double* p = self->picker_->GetPickPosition();
        emit self->pointPicked(static_cast<float>(p[0]),
                               static_cast<float>(p[1]),
                               static_cast<float>(p[2]));
    }
}

vtkSmartPointer<vtkActor> PointCloudViewer::makeCloudActor(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    double r, double g, double b,
    double point_size)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = cloud->points[i];
        points->SetPoint(static_cast<vtkIdType>(i), p.x, p.y, p.z);
    }

    auto polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);

    auto filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    filter->SetInputData(polydata);
    filter->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(filter->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, b);
    actor->GetProperty()->SetPointSize(point_size);
    return actor;
}

vtkSmartPointer<vtkActor> PointCloudViewer::makeCloudActorRGB(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    double point_size)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(cloud->size());

    auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");
    colors->SetNumberOfTuples(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = cloud->points[i];
        points->SetPoint(static_cast<vtkIdType>(i), p.x, p.y, p.z);
        unsigned char rgb[3] = {p.r, p.g, p.b};
        colors->SetTypedTuple(static_cast<vtkIdType>(i), rgb);
    }

    auto polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    polydata->GetPointData()->SetScalars(colors);

    auto filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    filter->SetInputData(polydata);
    filter->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(filter->GetOutputPort());
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->ScalarVisibilityOn();

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(point_size);
    return actor;
}

vtkSmartPointer<vtkActor> PointCloudViewer::makeSphereActor(
    const Eigen::Vector3f& center, double radius,
    double r, double g, double b)
{
    auto src = vtkSmartPointer<vtkSphereSource>::New();
    src->SetCenter(center.x(), center.y(), center.z());
    src->SetRadius(radius);
    src->SetPhiResolution(16);
    src->SetThetaResolution(16);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(src->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, b);
    return actor;
}

vtkSmartPointer<vtkActor> PointCloudViewer::makeLineActor(
    const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
    double r, double g, double b)
{
    auto src = vtkSmartPointer<vtkLineSource>::New();
    src->SetPoint1(p1.x(), p1.y(), p1.z());
    src->SetPoint2(p2.x(), p2.y(), p2.z());

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(src->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, b);
    actor->GetProperty()->SetLineWidth(3.0);
    return actor;
}

} // namespace calibration_gui
