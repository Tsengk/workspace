#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QGroupBox>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/point_cloud_viewer.h"

namespace calibration_gui {

/// Tab: LiDAR-LiDAR 联合标定（x/y/yaw），两阶段手动选锥桶
/// 对应 lidar_lidar_cali_clicked 节点
class TabLidarLidar : public QWidget {
    Q_OBJECT
public:
    explicit TabLidarLidar(std::shared_ptr<ConfigManager> cfg,
                           std::shared_ptr<RosWorker> ros,
                           QWidget* parent = nullptr);
    void applyConfig();
    void deactivate();

private slots:
    void onPrimarySelected(const QString& name);
    void onSecondarySelected(const QString& name);
    void onConfirmPrimary();
    void onSaveCalib();
    void onResetCalib();
    void onXChanged(double v);
    void onYChanged(double v);
    void onYawChanged(double v);
    void onVizToggle();
    void onRawCloudToggle(int state);
    void onPickModeToggle(bool checked);
    void onPointPicked(float x, float y, float z);
    void onStartNode();
    void onStopNode();
    void onNodeStarted(const QString& key);
    void onNodeStopped(const QString& key);

private:
    void setupUI();
    void resubscribe();
    void refreshViewer();
    void onPrimaryCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onSecondaryCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onStatusReceived(const std::string& msg);
    void onResultReceived(double x, double y, double z);

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 节点管理 ----
    QPushButton*  btn_start_node_{nullptr};
    QPushButton*  btn_stop_node_{nullptr};
    QLabel*       label_node_status_{nullptr};

    // ---- 传感器选择 ----
    QComboBox*       combo_primary_;
    QComboBox*       combo_secondary_;
    QLabel*          label_install_;
    QLabel*          label_param_source_{nullptr};

    // ---- 安装参数微调 ----
    QDoubleSpinBox*  spin_x_;
    QDoubleSpinBox*  spin_y_;
    QDoubleSpinBox*  spin_yaw_;

    // ---- 阶段 ----
    QLabel*       label_phase_{nullptr};
    QPushButton*  btn_confirm_primary_{nullptr};
    int           current_phase_{1};

    // ---- 状态 ----
    QLabel*          label_data_status_;
    QLabel*          label_cali_status_;
    QLabel*          label_result_;

    // ---- 操作按钮 ----
    QPushButton*     btn_save_;
    QPushButton*     btn_reset_;

    // ---- 可视化控制 ----
    QPushButton*  btn_viz_toggle_{nullptr};
    QCheckBox*    chk_show_primary_{nullptr};
    QCheckBox*    chk_show_secondary_{nullptr};
    QPushButton*  btn_pick_mode_{nullptr};
    bool          viz_enabled_{false};
    bool          show_primary_cloud_{false};
    bool          show_secondary_cloud_{false};

    PointCloudViewer* viewer_;

    // ---- 点云缓存 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_primary_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_secondary_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_primary_buf_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_secondary_buf_;

    std::string node_ns_;
    std::string prev_primary_;        // 上一次选中的主雷达名（用于切换时停止旧 driver）
    std::string prev_secondary_ll_;   // 上一次选中的辅雷达名（用于切换时停止旧 driver）
    double last_delta_x_{0.0};
    double last_delta_y_{0.0};
    double last_delta_yaw_{0.0};
};

} // namespace calibration_gui
