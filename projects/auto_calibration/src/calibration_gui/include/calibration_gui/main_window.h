#pragma once
#include <QMainWindow>
#include <QTabWidget>
#include <QLabel>
#include <QStatusBar>
#include <QTimer>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include "calibration_gui/config_manager.h"
#include "calibration_gui/ros_worker.h"

// 前向声明各 Tab
namespace calibration_gui {
class TabLidarRp;
class TabLidarYaw;
class TabLidarGnssYaw;
class TabLidarLidar;
class TabImageLidarBev;
class TabCameraCali;
class TabBev;
class TabFrontStitch;
class TabSecondaryRp;
class TabRadarLidar;
}

namespace calibration_gui {

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(int argc, char** argv, QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onLoadConfig();
    void onRosTimer();
    void onTabChanged(int index);
    void onExportParams();

private:
    void setupUI();
    void setupMenuBar();
    void setupStatusBar();
    void updateConfigLabel();
    void applyConfig();
    void checkIntrinsicsAndNotify();
    void showIntrinsicsDialog();
    bool isTabEnabled(const std::string& name) const;
    void loadTabsConfig();

    // ---- 核心组件 ----
    std::shared_ptr<ConfigManager> config_mgr_;
    std::shared_ptr<RosWorker>     ros_worker_;

    // ---- UI ----
    QTabWidget* tab_widget_;
    QLabel*     config_label_;
    QTimer*     ros_timer_;

    // ---- Tab 显示控制 ----
    std::map<std::string, bool> tabs_enabled_;
    // YAML 中声明的 Tab 顺序；未声明的 Tab 会退回到内置默认顺序追加到末尾
    std::vector<std::string> tabs_order_;

    // ---- 各标定 Tab ----
    TabLidarRp*      tab_lidar_rp_{nullptr};
    TabLidarYaw*     tab_lidar_yaw_{nullptr};
    TabLidarGnssYaw* tab_lidar_gnss_yaw_{nullptr};
    TabLidarLidar*   tab_lidar_lidar_{nullptr};
    TabImageLidarBev* tab_image_lidar_bev_{nullptr};
    TabCameraCali*   tab_camera_cali_{nullptr};
    TabBev*          tab_bev_{nullptr};
    TabFrontStitch*  tab_front_stitch_{nullptr};
    TabSecondaryRp*  tab_secondary_rp_{nullptr};
    TabRadarLidar*   tab_radar_lidar_{nullptr};
};

} // namespace calibration_gui
