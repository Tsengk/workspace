#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/image_viewer.h"

namespace calibration_gui {

/// 相机-LiDAR 标定：封装 lidar_camera_projection_node
/// - LiDAR 要求已经标定完毕（process/ 下存在对应标定文件）
/// - 相机安装参数作为初始外参，用户通过 6 个 spinbox 实时微调
/// - 保存时 GUI 侧写 process/image_lidar_<camera>.yaml
class TabImageLidar : public QWidget {
    Q_OBJECT
public:
    explicit TabImageLidar(std::shared_ptr<ConfigManager> cfg,
                           std::shared_ptr<RosWorker> ros,
                           QWidget* parent = nullptr);
    void applyConfig();
    void deactivate();

private slots:
    void onLidarSelected(const QString& name);
    void onCameraSelected(const QString& name);
    void onStartNode();
    void onStopNode();
    void onNodeStarted(const QString& key);
    void onNodeStopped(const QString& key);
    void onSaveCalib();
    void onResetParams();
    void onParamChanged(double v);
    void onVizToggle();

private:
    void setupUI();
    void onImageReceived(cv::Mat img);
    void refreshLidarStatus();
    void updateStartButtonState();
    void loadInitialParamsToSpins(const std::string& camera_name);

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 节点管理 ----
    QPushButton* btn_start_node_{nullptr};
    QPushButton* btn_stop_node_{nullptr};
    QLabel*      label_node_status_{nullptr};

    // ---- 可视化控制 ----
    QPushButton* btn_viz_toggle_{nullptr};
    bool         viz_enabled_{false};

    // ---- 传感器选择 ----
    QComboBox* combo_lidar_{nullptr};
    QComboBox* combo_camera_{nullptr};
    QLabel*    label_lidar_status_{nullptr};
    QLabel*    label_install_{nullptr};

    // ---- 6DOF 参数微调 ----
    QDoubleSpinBox* spin_x_{nullptr};
    QDoubleSpinBox* spin_y_{nullptr};
    QDoubleSpinBox* spin_z_{nullptr};
    QDoubleSpinBox* spin_roll_{nullptr};
    QDoubleSpinBox* spin_pitch_{nullptr};
    QDoubleSpinBox* spin_yaw_{nullptr};

    // ---- 状态 ----
    QLabel* label_data_status_{nullptr};

    // ---- 操作按钮 ----
    QPushButton* btn_save_{nullptr};
    QPushButton* btn_reset_{nullptr};

    // ---- 右侧调试视图 ----
    ImageViewer* viewer_{nullptr};

    // ---- 当前选择与上一次 driver 雷达名 ----
    std::string current_lidar_;
    std::string current_camera_;
    std::string prev_lidar_;

    // ---- 当前节点输出 topic（每次启动时根据相机名生成） ----
    std::string output_topic_;
    bool lidar_is_calibrated_{false};
};

} // namespace calibration_gui
