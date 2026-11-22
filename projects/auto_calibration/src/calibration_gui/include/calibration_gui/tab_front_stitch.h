#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <memory>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/image_viewer.h"

namespace calibration_gui {

/// Tab：前视图像拼接
/// 对应 multi_view_stitch 节点
/// 三相机选择后 topic/内参自动确定，支持深度/锚点参数调整
class TabFrontStitch : public QWidget {
    Q_OBJECT
public:
    explicit TabFrontStitch(std::shared_ptr<ConfigManager> cfg,
                            std::shared_ptr<RosWorker> ros,
                            QWidget* parent = nullptr);
    void applyConfig();

private slots:
    void onLeftCameraSelected(const QString& name);
    void onFrontCameraSelected(const QString& name);
    void onRightCameraSelected(const QString& name);
    void onStartNode();
    void onStopNode();
    void onVizToggle();
    void onParamChanged(double v);

private:
    void setupUI();
    void updateNodeParams();
    void onStitchImageReceived(cv::Mat img);
    std::string getCameraTopicByName(const std::string& name) const;
    std::string getCameraIntrinsicsByName(const std::string& name) const;

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 相机选择 ----
    QComboBox*  combo_left_{nullptr};
    QComboBox*  combo_front_{nullptr};
    QComboBox*  combo_right_{nullptr};
    QLabel*     label_left_topic_{nullptr};
    QLabel*     label_front_topic_{nullptr};
    QLabel*     label_right_topic_{nullptr};

    // ---- 拼接参数 ----
    QDoubleSpinBox* spin_align_depth_z_{nullptr};   // 0~50 m
    QSpinBox*       spin_anchor_left_x_{nullptr};
    QSpinBox*       spin_anchor_left_y_{nullptr};
    QSpinBox*       spin_anchor_right_x_{nullptr};
    QSpinBox*       spin_anchor_right_y_{nullptr};
    QLineEdit*      edit_stitched_topic_{nullptr};

    // ---- 节点管理 ----
    QPushButton* btn_start_{nullptr};
    QPushButton* btn_stop_{nullptr};
    QLabel*      label_node_status_{nullptr};

    // ---- 可视化 ----
    QPushButton* btn_viz_toggle_{nullptr};
    bool         viz_enabled_{false};
    QLabel*      label_data_status_{nullptr};
    ImageViewer* viewer_{nullptr};

    std::string current_left_;
    std::string current_front_;
    std::string current_right_;

    static constexpr const char* kNodeKey = "multi_view_stitch";
    static constexpr const char* kNodeNs  = "/multi_view_stitch_node";
};

} // namespace calibration_gui
