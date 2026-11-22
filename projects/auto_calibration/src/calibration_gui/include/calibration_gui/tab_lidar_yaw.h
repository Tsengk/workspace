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

/// Tab 2：激光雷达静态 Yaw 标定
/// 对应 lidar_yaw1 节点
class TabLidarYaw : public QWidget {
    Q_OBJECT
public:
    explicit TabLidarYaw(std::shared_ptr<ConfigManager> cfg,
                         std::shared_ptr<RosWorker> ros,
                         QWidget* parent = nullptr);
    void applyConfig();
    void deactivate();

private slots:
    void onLidarSelected(const QString& name);
    void onStartCalib();
    void onSaveCalib();
    void onResetCalib();
    void onYawChanged(double v);
    void onVizToggle();
    void onRawCloudToggle(int state);
    void onStartNode();
    void onStopNode();
    void onNodeStarted(const QString& key);
    void onNodeStopped(const QString& key);
    void onPickModeToggle(bool checked);
    void onPointPicked(float x, float y, float z);

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
    QPushButton*     btn_save_;
    QPushButton*     btn_reset_;
    QLabel*          label_result_;
    PointCloudViewer* viewer_;

    // ---- 节点管理 ----
    QPushButton*     btn_start_node_{nullptr};
    QPushButton*     btn_stop_node_{nullptr};
    QLabel*          label_node_status_{nullptr};

    // ---- 可视化控制 ----
    QPushButton*     btn_viz_toggle_{nullptr};
    QCheckBox*       chk_raw_cloud_{nullptr};
    QPushButton*     btn_pick_mode_{nullptr};
    bool             viz_enabled_{false};
    bool             show_raw_cloud_{false};
    std::string      raw_cloud_topic_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_buf_;     // 原始点云缓存
    pcl::PointCloud<pcl::PointXYZI>::Ptr clicked_cloud_buf_; // 已选锥桶点缓存

    std::string current_lidar_;
    std::string prev_lidar_;
    std::string node_ns_;
    double last_yaw_delta_{0.0};
};

} // namespace calibration_gui
