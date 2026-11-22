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

/// Tab 1：激光雷达 Roll/Pitch/Z 标定
/// 对应 lidar_rp_cali 节点
class TabLidarRp : public QWidget {
    Q_OBJECT
public:
    explicit TabLidarRp(std::shared_ptr<ConfigManager> cfg,
                        std::shared_ptr<RosWorker> ros,
                        QWidget* parent = nullptr);

    /// 配置加载后更新雷达下拉框与安装参数显示
    void applyConfig();
    void deactivate();

private slots:
    void onLidarSelected(const QString& name);
    void onStartCalib();
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
    void loadLidarParamsToUi(const std::string& lidar_name);
    void resolveLidarParams(const std::string& lidar_name,
                            double& x, double& y, double& z,
                            double& roll, double& pitch, double& yaw,
                            QString* source_text = nullptr) const;
    void updateStatusBar(const std::string& msg);
    void onCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onRawCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void onStatusReceived(const std::string& msg);
    void onResultReceived(double x, double y, double z);

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- UI 控件 ----
    QComboBox*       combo_lidar_;
    QLabel*          label_install_;    // 安装参数（只读）
    QLabel*          label_param_source_{nullptr};
    QLabel*          label_data_status_;
    QLabel*          label_cali_status_;
    QDoubleSpinBox*  spin_roll_;
    QDoubleSpinBox*  spin_pitch_;
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_buf_;   // 原始点云缓存
    pcl::PointCloud<pcl::PointXYZI>::Ptr clicked_cloud_buf_; // 已选点缓存

    // ---- 当前选中雷达名 ----
    std::string current_lidar_;
    std::string prev_lidar_;   // 上一次选中的雷达名（用于切换时停止旧 driver）
    std::string node_ns_;  // ROS 节点命名空间前缀

    // ---- 最新标定结果 ----
    double last_z_{0.0};
    double last_roll_{0.0}, last_pitch_{0.0};  // 最新标定结果
};

} // namespace calibration_gui
