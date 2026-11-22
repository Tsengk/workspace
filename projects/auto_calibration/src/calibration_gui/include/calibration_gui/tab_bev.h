#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QSplitter>
#include <memory>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/image_viewer.h"

namespace calibration_gui {

/// Tab: 环视 BEV 标定
/// 对应 bev_surround_view_node，支持节点管理、5路图像显示、ROI线叠加
class TabBev : public QWidget {
    Q_OBJECT
public:
    explicit TabBev(std::shared_ptr<ConfigManager> cfg,
                    std::shared_ptr<RosWorker> ros,
                    QWidget* parent = nullptr);
    void applyConfig();

private slots:
    void onCameraSelected(int cam_idx, const QString& name);
    void onStartCalib();
    void onSaveCalib();
    void onResetCalib();
    void onBevParamChanged(double v);
    void onVizToggle();
    void onStartNode();
    void onStopNode();
    void onNodeStarted(const QString& key);
    void onNodeStopped(const QString& key);

private:
    void setupUI();
    void onBevImageReceived(cv::Mat img);
    void onDetVisReceived(int cam_idx, cv::Mat img);
    void onStatusReceived(const std::string& msg);
    void drawRoiLines(cv::Mat& img);

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 节点管理 ----
    QPushButton* btn_start_node_{nullptr};
    QPushButton* btn_stop_node_{nullptr};
    QLabel*      label_node_status_{nullptr};

    // ---- 可视化控制 ----
    QPushButton* btn_viz_toggle_{nullptr};
    bool         viz_enabled_{false};

    // ---- 相机绑定 (front/rear/left/right) ----
    QComboBox* combo_front_{nullptr};
    QComboBox* combo_rear_{nullptr};
    QComboBox* combo_left_{nullptr};
    QComboBox* combo_right_{nullptr};

    // ---- BEV 参数 ----
    QDoubleSpinBox* spin_resolution_{nullptr};   // m/px
    QLabel*         label_px_per_m_{nullptr};    // 派生显示: px/m
    QDoubleSpinBox* spin_range_front_{nullptr};
    QDoubleSpinBox* spin_range_rear_{nullptr};
    QDoubleSpinBox* spin_range_left_{nullptr};
    QDoubleSpinBox* spin_range_right_{nullptr};
    QDoubleSpinBox* spin_origin_offset_x_{nullptr};  // 前后中心偏移 (m)
    QDoubleSpinBox* spin_origin_offset_y_{nullptr};  // 左右中心偏移 (m)

    // ---- 4路相机安装参数（只读）----
    struct CamParams {
        QLabel* install_label{nullptr};
    };
    CamParams cam_params_[4];  // 0=front 1=rear 2=left 3=right

    // ---- 状态 / 操作 ----
    QLabel*      label_data_status_{nullptr};
    QLabel*      label_cali_status_{nullptr};
    QLabel*      label_result_{nullptr};
    QLabel*      label_bev_info_{nullptr};    // 输出分辨率 + topic 信息
    QPushButton* btn_start_{nullptr};
    QPushButton* btn_save_{nullptr};
    QPushButton* btn_reset_{nullptr};

    // ---- 右侧图像显示 ----
    ImageViewer* viewer_front_{nullptr};   // detection_vis_front
    ImageViewer* viewer_rear_{nullptr};    // detection_vis_rear
    ImageViewer* viewer_left_{nullptr};    // detection_vis_left
    ImageViewer* viewer_right_{nullptr};   // detection_vis_right
    ImageViewer* viewer_bev_{nullptr};     // bev_image (with ROI lines)

    std::string node_ns_;

    // ---- 订阅 topic（从配置文件读取）----
    std::string topic_bev_image_;
    std::string topic_vis_front_;
    std::string topic_vis_rear_;
    std::string topic_vis_left_;
    std::string topic_vis_right_;
};

} // namespace calibration_gui
