#pragma once
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QStackedWidget>
#include <QGroupBox>
#include <QLabel>
#include <memory>
#include <opencv2/opencv.hpp>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"
#include "calibration_gui/image_viewer.h"
#include "calibration_gui/point_cloud_viewer.h"
#include "perception_fusion_common/camera_service/camera_service_api.h"

// 摄像头数量
#define CAM_MAX_NUM 8
#define CAM_FRONT 0
#define CAM_LEFT_FRONT 1
#define CAM_RIGHT_FRONT 2
#define CAM_LEFT_REAR 3
#define CAM_RIGHT_REAR 4
#define CAM_REAR 5
#define CAM_H30 6
#define CAM_INFRARED 7
namespace calibration_gui {
using namespace cv;
/// 相机-LiDAR 标定：封装 lidar_camera_projection_node
/// - LiDAR 要求已经标定完毕（process/ 下存在对应标定文件）
/// - 相机安装参数作为初始外参，用户通过 6 个 spinbox 实时微调
/// - 保存时 GUI 侧写 process/image_lidar_<camera>.yaml
class TabImageLidarBev : public QWidget {
    Q_OBJECT
public:
    explicit TabImageLidarBev(std::shared_ptr<ConfigManager> cfg,
                           std::shared_ptr<RosWorker> ros,
                           QWidget* parent = nullptr);
    // bool initBEVMap();
    // void InitCameraBevMaps(); //初始化相机投影
    // void  bevDrawIPMImage(cv::Mat &canvas, cv::Mat& img,bool draw_seg);
     void applyConfig();
    void deactivate();

    /// 标定方式：前视标定（lidar_camera_projection）或 BEV 标定（lidar_camera_projection_bev）
    enum class CalibMode { Front, Bev };

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
    void onSelectModeFront();
    void onSelectModeBev();

private:
    void setupUI();
    void onImageReceived(cv::Mat img);
    void onCloudReceived(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void startVizSubscription();
    void refreshLidarStatus();
    void updateStartButtonState();
    void loadInitialParamsToSpins(const std::string& camera_name);
    void setCalibMode(CalibMode mode);
    void updateModeUI();

    // 当前标定方式对应的节点/包信息（随 mode_ 变化）
    std::string nodeKey() const;
    std::string nodeNs() const;
    std::string projectionPkg() const;
    std::string projectionLaunch() const;

// 	//---------------------------------图像BEV视角处理-----------------------------------
//     std::shared_ptr<::perception::PerCameraService> cam_service_ptr;
//     cv::Mat bevcanvas;
//     cv::Size bevsize;
//     float scale = 7; // 几个像素是1米
// cv::Point2f cct;
//     // ipm maps
// 	cv::Mat mapx_f120, mapy_f120;
// 	// cv::Mat mapx_f30, mapy_f30;
// 	cv::Mat mapx_lf, mapy_lf;
// 	cv::Mat mapx_rf, mapy_rf;
// 	cv::Mat mapx_lr, mapy_lr;
// 	cv::Mat mapx_rr, mapy_rr;
// 	cv::Mat mapx_rear, mapy_rear;
// 	double prevtime = 0;
//     int camviewmode = 0;   //相机视角模式

//     //---------------------------------图像BEV视角处理-----------------------------------

    std::shared_ptr<ConfigManager> cfg_;
    std::shared_ptr<RosWorker>     ros_;

    // ---- 节点管理 ----
    QPushButton* btn_start_node_{nullptr};
    QPushButton* btn_stop_node_{nullptr};
    QLabel*      label_node_status_{nullptr};

    // ---- 标定方式选择 ----
    QRadioButton* btn_mode_front_{nullptr};
    QRadioButton* btn_mode_bev_{nullptr};
    QButtonGroup* mode_group_{nullptr};
    CalibMode    mode_{CalibMode::Front};
    // bool         bev_map_ready_{false};   // BEV 相机服务/投影表是否初始化成功

    // ---- 右侧标题（随标定方式切换文案） ----
    QGroupBox*   right_box_{nullptr};

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
    // 前视：图像叠加；BEV：点云（相机真彩色地面点 + 主雷达强度彩色点）
    QStackedWidget*   view_stack_{nullptr};
    ImageViewer*      viewer_{nullptr};        // 前视标定视图
    PointCloudViewer* cloud_viewer_{nullptr};  // BEV 标定视图

    // ---- 当前选择与上一次 driver 雷达名 ----
    std::string current_lidar_;
    std::string current_camera_;
    std::string cameras_dist_file_;
    std::string prev_lidar_;

    // ---- 当前节点输出 topic（每次启动时根据相机名生成） ----
    std::string output_topic_;        // 前视：图像话题
    std::string output_cloud_topic_;  // BEV：点云话题
    bool lidar_is_calibrated_{false};
};

} // namespace calibration_gui
