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

/// Tab: 毫米波雷达-LiDAR 联合标定（手动调整6DOF参数对齐）
/// 激光雷达已标定到车体坐标系，手动调整毫米波雷达安装参数使两者对齐
class TabRadarLidar : public QWidget {
    Q_OBJECT
public:
    explicit TabRadarLidar(std::shared_ptr<ConfigManager> cfg,
                           std::shared_ptr<RosWorker> ros,
                           QWidget* parent = nullptr);
    void applyConfig();
    void deactivate();

private slots:
    void onLidarSelected(const QString& name);
    void onRadarSelected(const QString& name);
    void onXChanged(double v);
    void onYChanged(double v);
    void onZChanged(double v);
    void onRollChanged(double v);
    void onPitchChanged(double v);
    void onYawChanged(double v);
    void onVizToggle();
    void onSaveCalib();
    void onResetParams();

private:
    void setupUI();
    void resubscribe();
    void refreshViewer();
    void onLidarCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onRadarCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void writeRadarShm();

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 传感器选择 ----
    QComboBox*  combo_lidar_{nullptr};
    QComboBox*  combo_radar_{nullptr};
    QLabel*     label_lidar_install_{nullptr};
    QLabel*     label_radar_install_{nullptr};

    // ---- 6DOF 参数微调 ----
    QDoubleSpinBox* spin_x_{nullptr};
    QDoubleSpinBox* spin_y_{nullptr};
    QDoubleSpinBox* spin_z_{nullptr};
    QDoubleSpinBox* spin_roll_{nullptr};
    QDoubleSpinBox* spin_pitch_{nullptr};
    QDoubleSpinBox* spin_yaw_{nullptr};
    QLabel*         label_params_source_{nullptr};

    // ---- 状态 ----
    QLabel* label_data_status_{nullptr};

    // ---- 操作按钮 ----
    QPushButton* btn_viz_toggle_{nullptr};
    QPushButton* btn_save_{nullptr};
    QPushButton* btn_reset_{nullptr};

    bool viz_enabled_{false};

    PointCloudViewer* viewer_{nullptr};

    // ---- 点云缓存 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_radar_;

    std::string current_lidar_;
    std::string current_radar_;
    std::string prev_lidar_rl_;   // 上一次选中的激光雷达名（用于切换时停止旧 driver）
    std::string prev_radar_;      // 上一次选中的毫米波雷达名（用于切换时停止旧 driver）
};

} // namespace calibration_gui
