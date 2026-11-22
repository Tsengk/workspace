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

/// Tab: 辅激光雷达 Roll/Pitch/Z 手动选点标定
/// 对应 lidar_secondary_rp_cali 节点
/// 两阶段流程：
///   阶段1 — 在 RViz 中点击主雷达地面点 → [确认主雷达]
///   阶段2 — 在 RViz 中点击辅雷达地面点 → [保存标定]
class TabSecondaryRp : public QWidget {
    Q_OBJECT
public:
    explicit TabSecondaryRp(std::shared_ptr<ConfigManager> cfg,
                            std::shared_ptr<RosWorker> ros,
                            QWidget* parent = nullptr);
    void applyConfig();
    void deactivate();

private slots:
    void onPrimaryLidarSelected(const QString& name);
    void onSecondaryLidarSelected(const QString& name);
    void onConfirmPrimary();
    void onSaveCalib();
    void onResetCalib();
    void onRollChanged(double v);
    void onPitchChanged(double v);
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
    void updateInstallLabel();
    void resubscribe();
    void onPrimaryCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onSecondaryCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onStatusReceived(const std::string& msg);
    void onResultReceived(double x, double y, double z);
    void refreshViewer();

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 节点管理 ----
    QPushButton*     btn_start_node_{nullptr};
    QPushButton*     btn_stop_node_{nullptr};
    QLabel*          label_node_status_{nullptr};

    // ---- UI 控件 ----
    QComboBox*       combo_primary_;
    QComboBox*       combo_secondary_;
    QLabel*          label_install_;
    QLabel*          label_param_source_{nullptr};
    QLabel*          label_phase_;       // 当前阶段指示
    QLabel*          label_data_status_;
    QLabel*          label_cali_status_;
    QDoubleSpinBox*  spin_roll_;
    QDoubleSpinBox*  spin_pitch_;
    QPushButton*     btn_confirm_primary_;
    QPushButton*     btn_save_;
    QPushButton*     btn_reset_;
    QLabel*          label_result_;
    PointCloudViewer* viewer_;

    // ---- 可视化控制 ----
    QPushButton*     btn_viz_toggle_{nullptr};
    QCheckBox*       chk_show_primary_{nullptr};
    QCheckBox*       chk_show_secondary_{nullptr};
    QPushButton*     btn_pick_mode_{nullptr};
    bool             viz_enabled_{false};
    bool             show_primary_cloud_{false};
    bool             show_secondary_cloud_{false};
    std::string      raw_cloud_topic_;   // 辅雷达原始点云 topic
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_primary_buf_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_secondary_buf_;

    // ---- 点云缓存（节点发布的已选点） ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr primary_cloud_buf_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr secondary_cloud_buf_;

    std::string current_primary_;
    std::string prev_primary_;
    std::string current_secondary_;
    std::string prev_secondary_;   // 上一次选中的辅雷达名（用于切换时停止旧 driver）
    std::string node_ns_;

    double last_roll_{0.0}, last_pitch_{0.0};
    int    current_phase_{1};  // 1=主雷达阶段, 2=辅雷达阶段
};

} // namespace calibration_gui
