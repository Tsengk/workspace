#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <memory>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/image_viewer.h"

namespace calibration_gui {

/// Tab 6：相机间联合标定（6DOF）
/// 对应 camera_cali 节点
class TabCameraCali : public QWidget {
    Q_OBJECT
public:
    explicit TabCameraCali(std::shared_ptr<ConfigManager> cfg,
                           std::shared_ptr<RosWorker> ros,
                           QWidget* parent = nullptr);
    void applyConfig();

private slots:
    void onRefCameraSelected(const QString& name);
    void onTargetCameraSelected(const QString& name);
    void onStartCalib();
    void onSaveCalib();
    void onResetCalib();
    void onParamChanged(double v);
    void onVizToggle();

private:
    void setupUI();
    void onRefImageReceived(cv::Mat img);
    void onTargetImageReceived(cv::Mat img);
    void onStatusReceived(const std::string& msg);
    void onResultReceived(double x, double y, double z);

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    QComboBox*       combo_ref_;
    QComboBox*       combo_target_;
    QLabel*          label_install_;
    QLabel*          label_data_status_;
    QLabel*          label_cali_status_;
    // 6 个可编辑参数
    QDoubleSpinBox*  spin_x_;
    QDoubleSpinBox*  spin_y_;
    QDoubleSpinBox*  spin_z_;
    QDoubleSpinBox*  spin_roll_;
    QDoubleSpinBox*  spin_pitch_;
    QDoubleSpinBox*  spin_yaw_;
    QPushButton*     btn_start_;
    QPushButton*     btn_save_;
    QPushButton*     btn_reset_;
    QLabel*          label_result_;
    ImageViewer*     viewer_ref_;
    ImageViewer*     viewer_target_;

    // ---- 可视化控制 ----
    QPushButton*     btn_viz_toggle_{nullptr};
    bool             viz_enabled_{false};

    std::string node_ns_;
};

} // namespace calibration_gui
