#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/point_cloud_viewer.h"

namespace calibration_gui {

/// Tab 3：GNSS + 激光雷达联合 Yaw 标定
/// 对应 lidar_gnss_yaw 节点
class TabLidarGnssYaw : public QWidget {
    Q_OBJECT
public:
    explicit TabLidarGnssYaw(std::shared_ptr<ConfigManager> cfg,
                              std::shared_ptr<RosWorker> ros,
                              QWidget* parent = nullptr);
    void applyConfig();
    void deactivate();

private slots:
    void onLidarSelected(const QString& name);
    void onStartCollect();
    void onStopAndCalib();
    void onSaveCalib();
    void onResetCalib();
    void onYawChanged(double v);
    void onVizToggle();
    void onRawCloudToggle(int state);

private:
    void setupUI();
    void onRawCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onStatusReceived(const std::string& msg);
    void onResultReceived(double x, double y, double z);

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    QComboBox*       combo_lidar_;
    QLabel*          label_install_;
    QLabel*          label_param_source_{nullptr};
    QLabel*          label_data_status_;
    QLabel*          label_cali_status_;
    QDoubleSpinBox*  spin_yaw_;
    QPushButton*     btn_start_;
    QPushButton*     btn_stop_;
    QPushButton*     btn_save_;
    QPushButton*     btn_reset_;
    QLabel*          label_result_;
    PointCloudViewer* viewer_;

    // ---- 可视化控制 ----
    QPushButton*     btn_viz_toggle_{nullptr};
    QCheckBox*       chk_raw_cloud_{nullptr};
    bool             viz_enabled_{false};
    bool             show_raw_cloud_{false};
    std::string      raw_cloud_topic_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_buf_;  // 原始点云缓存

    std::string current_lidar_;
    std::string prev_lidar_;
    std::string node_ns_;
    bool collecting_{false};
    double last_yaw_calib_{0.0};
};

} // namespace calibration_gui
