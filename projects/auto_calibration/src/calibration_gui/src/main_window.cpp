#include "calibration_gui/main_window.h"
#include "calibration_gui/wheel_guard.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "calibration_gui/tab_lidar_rp.h"
#include "calibration_gui/tab_lidar_yaw.h"
#include "calibration_gui/tab_lidar_gnss_yaw.h"
#include "calibration_gui/tab_lidar_lidar.h"
#include "calibration_gui/tab_image_lidar_bev.h"
#include "calibration_gui/tab_camera_cali.h"
#include "calibration_gui/tab_bev.h"
#include "calibration_gui/tab_front_stitch.h"
#include "calibration_gui/tab_secondary_rp.h"
#include "calibration_gui/tab_radar_lidar.h"

#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QStatusBar>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QDialog>
#include <QDialogButtonBox>
#include <QScrollArea>
#include <QSpacerItem>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace calibration_gui {

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
    : QMainWindow(parent)
{
    // 保留默认窗口标志，保证系统自带的最小化/最大化/关闭/拉伸都可用
    resize(1600, 900);
    setMinimumSize(800, 600);

    config_mgr_ = std::make_shared<ConfigManager>();
    ros_worker_ = std::make_shared<RosWorker>(this);
    ros_worker_->init(argc, argv);

    loadTabsConfig();
    setupUI();
    setupMenuBar();
    setupStatusBar();

    // ROS 轮询定时器，50ms 间隔
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &MainWindow::onRosTimer);
    ros_timer_->start(50);

    connect(ros_worker_.get(), &RosWorker::rosShutdown,
            this, &QMainWindow::close);

    // 若 rosparam vehicle_config 非空，自动加载
    std::string vc_path;
    bool param_ok = ros::param::get("/vehicle_config", vc_path);
    if (!param_ok) param_ok = ros::param::get("~vehicle_config", vc_path);
    if (param_ok && !vc_path.empty()) {
        if (config_mgr_->loadVehicleConfig(vc_path)) {
            updateConfigLabel();
            applyConfig();
            checkIntrinsicsAndNotify();
            statusBar()->showMessage("已自动加载：" + QString::fromStdString(vc_path));
        }
    }

    // 全局禁用 ComboBox/SpinBox 滚轮：滚轮只滚动面板，不改变控件值。
    installGlobalWheelGuard();
}

MainWindow::~MainWindow()
{
    ros_timer_->stop();
}

void MainWindow::setupUI()
{
    auto* central = new QWidget(this);
    auto* vlayout = new QVBoxLayout(central);
    vlayout->setContentsMargins(10, 8, 10, 8);
    vlayout->setSpacing(8);

    // 顶部配置文件信息栏
    auto* top_bar  = new QHBoxLayout();
    auto* lbl_hint = new QLabel("车辆配置：", this);
    config_label_  = new QLabel("未加载", this);
    config_label_->setStyleSheet("QLabel { color: #e88; font-weight: bold; }");
    auto* btn_load = new QPushButton("导入车辆配置文件", this);
    btn_load->setFixedWidth(160);
    connect(btn_load, &QPushButton::clicked, this, &MainWindow::onLoadConfig);
    top_bar->addWidget(lbl_hint);
    top_bar->addWidget(config_label_, 1);
    top_bar->addWidget(btn_load);
    vlayout->addLayout(top_bar);

    // 标签页
    tab_widget_ = new QTabWidget(this);
    tab_widget_->setTabPosition(QTabWidget::North);

    // 创建所有 Tab
    tab_lidar_rp_       = new TabLidarRp(config_mgr_, ros_worker_, this);
    tab_lidar_yaw_      = new TabLidarYaw(config_mgr_, ros_worker_, this);
    tab_lidar_gnss_yaw_ = new TabLidarGnssYaw(config_mgr_, ros_worker_, this);
    tab_lidar_lidar_    = new TabLidarLidar(config_mgr_, ros_worker_, this);
    // 合并后的相机-LiDAR标定：内含“前视标定/BEV标定”方式选择
    tab_image_lidar_bev_ = new TabImageLidarBev(config_mgr_, ros_worker_, this);
    tab_camera_cali_    = new TabCameraCali(config_mgr_, ros_worker_, this);
    tab_bev_            = new TabBev(config_mgr_, ros_worker_, this);
    tab_front_stitch_   = new TabFrontStitch(config_mgr_, ros_worker_, this);
    tab_secondary_rp_   = new TabSecondaryRp(config_mgr_, ros_worker_, this);
    tab_radar_lidar_    = new TabRadarLidar(config_mgr_, ros_worker_, this);

    struct TabEntry {
        std::string name;
        QWidget*    widget;
        const char* label;
    };
    const std::vector<TabEntry> entries = {
        {"lidar_rp",       tab_lidar_rp_,       "LiDAR RP标定"},
        {"secondary_rp",   tab_secondary_rp_,   "辅雷达RP标定"},
        {"lidar_yaw",      tab_lidar_yaw_,      "LiDAR Yaw标定"},
        {"lidar_gnss_yaw", tab_lidar_gnss_yaw_, "GNSS+LiDAR Yaw标定"},
        {"lidar_lidar",    tab_lidar_lidar_,    "LiDAR-LiDAR联合标定"},
        {"image_lidar",    tab_image_lidar_bev_, "相机-LiDAR标定"},
        {"camera_cali",    tab_camera_cali_,    "相机-相机标定"},
        {"bev",            tab_bev_,            "环视BEV标定"},
        {"front_stitch",   tab_front_stitch_,   "前视图像拼接"},
        {"radar_lidar",    tab_radar_lidar_,    "毫米波-LiDAR标定"},
    };
    std::map<std::string, const TabEntry*> by_name;
    for (const auto& e : entries) by_name[e.name] = &e;

    for (const auto& name : tabs_order_) {
        auto it = by_name.find(name);
        if (it == by_name.end()) continue;
        if (!isTabEnabled(name)) continue;
        tab_widget_->addTab(it->second->widget, it->second->label);
    }

    connect(tab_widget_, &QTabWidget::currentChanged,
            this, &MainWindow::onTabChanged);

    vlayout->addWidget(tab_widget_, 1);
    setCentralWidget(central);

    // 深色主题
    setStyleSheet(
        "QMainWindow { background: #0d0d1a; }"
        "QWidget { background: #0d0d1a; color: #ccc; }"  // 全局默认白色文字
        "QTabWidget::pane { border: 1px solid #333; background: #0d0d1a; }"
        "QTabBar::tab { background: #1a1a2e; color: #aaa; padding: 8px 20px; min-width: 100px; }"
        "QTabBar::tab:selected { background: #16213e; color: #fff; font-weight: bold; }"
        "QGroupBox { color: #ccc; border: 1px solid #333; margin-top: 12px; padding-top: 6px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #ddd; }"
        "QLabel { color: #ccc; }"
        "QCheckBox { color: #ccc; spacing: 6px; }"
        "QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #666; "
        "                       background: #16213e; border-radius: 2px; }"
        "QCheckBox::indicator:checked { background: #0f3460; border-color: #4af; }"
        "QCheckBox::indicator:checked::after { color: #4af; }"
        "QPushButton { background: #16213e; color: #ccc; border: 1px solid #444; "
        "              padding: 7px 16px; border-radius: 4px; min-height: 28px; }"
        "QPushButton:hover { background: #0f3460; color: #fff; }"
        "QPushButton:pressed { background: #533483; color: #fff; }"
        "QPushButton:disabled { color: #555; background: #0d0d1a; }"
        "QDoubleSpinBox, QSpinBox, QComboBox { background: #16213e; color: #ccc; "
        "                                      border: 1px solid #444; padding: 4px 6px; "
        "                                      min-height: 24px; border-radius: 3px; }"
        "QComboBox QAbstractItemView { background: #16213e; color: #ccc; "
        "                              selection-background-color: #0f3460; }"
        "QScrollArea { border: none; }"
        "QScrollBar:vertical { background: #0d0d1a; width: 10px; }"
        "QScrollBar::handle:vertical { background: #333; border-radius: 4px; }"
        "QStatusBar { background: #0a0a14; color: #888; }"
    );
}

void MainWindow::setupMenuBar()
{
    auto* file_menu = menuBar()->addMenu("文件(&F)");
    auto* act_load  = file_menu->addAction("导入车辆配置文件(&O)...");
    auto* act_export = file_menu->addAction("导出参数(&E)...");
    file_menu->addSeparator();
    auto* act_quit  = file_menu->addAction("退出(&Q)");
    connect(act_load, &QAction::triggered, this, &MainWindow::onLoadConfig);
    connect(act_export, &QAction::triggered, this, &MainWindow::onExportParams);
    connect(act_quit, &QAction::triggered, this, &QMainWindow::close);

    auto* sensor_menu = menuBar()->addMenu("传感器(&S)");
    auto* act_intrinsics = sensor_menu->addAction("管理相机内参(&I)...");
    connect(act_intrinsics, &QAction::triggered, this, &MainWindow::showIntrinsicsDialog);

    menuBar()->setStyleSheet(
        "QMenuBar { background: #0a0a14; color: #aaa; }"
        "QMenuBar::item:selected { background: #16213e; }"
        "QMenu { background: #0d0d1a; color: #ccc; border: 1px solid #333; }"
        "QMenu::item:selected { background: #16213e; }"
    );
}

void MainWindow::setupStatusBar()
{
    statusBar()->showMessage("就绪");
}

void MainWindow::onLoadConfig()
{
    QString path = QFileDialog::getOpenFileName(
        this, "选择车辆配置文件",
        QString::fromStdString(ros::package::getPath("calibration_gui") + "/config"),
        "YAML 文件 (*.yaml *.yml);;所有文件 (*)");
    if (path.isEmpty()) return;

    if (!config_mgr_->loadVehicleConfig(path.toStdString())) {
        QMessageBox::critical(this, "错误", "加载配置文件失败：\n" + path);
        return;
    }

    updateConfigLabel();
    applyConfig();
    checkIntrinsicsAndNotify();
    statusBar()->showMessage("已加载：" + path);
    //tab_image_lidar_bev_->initBEVMap();
}

void MainWindow::updateConfigLabel()
{
    if (!config_mgr_->isLoaded()) {
        config_label_->setText("未加载");
        config_label_->setStyleSheet("QLabel { color: #e88; font-weight: bold; }");
        return;
    }
    const auto& v = config_mgr_->getConfig().vehicle;
    QString info = QString("车辆ID: %1  | LiDAR: %2个 | 相机: %3个 | 雷达: %4个")
        .arg(QString::fromStdString(v.id))
        .arg(config_mgr_->lidarNames().size())
        .arg(config_mgr_->cameraNames().size())
        .arg(config_mgr_->radarNames().size());
    config_label_->setText(info);
    config_label_->setStyleSheet("QLabel { color: #8e8; font-weight: bold; }");
}

void MainWindow::applyConfig()
{
    if (!config_mgr_->isLoaded()) return;
    tab_lidar_rp_->applyConfig();
    tab_lidar_yaw_->applyConfig();
    tab_secondary_rp_->applyConfig();
    tab_lidar_gnss_yaw_->applyConfig();
    tab_lidar_lidar_->applyConfig();
    tab_image_lidar_bev_->applyConfig();
    tab_camera_cali_->applyConfig();
    tab_bev_->applyConfig();
    tab_front_stitch_->applyConfig();
    tab_radar_lidar_->applyConfig();
}

void MainWindow::onRosTimer()
{
    ros_worker_->spinOnce();
}

void MainWindow::onTabChanged(int index)
{
    static int previous_index = -1;
    if (previous_index == index) return;

    QWidget* previous = previous_index >= 0 ? tab_widget_->widget(previous_index) : nullptr;
    if (previous == tab_lidar_rp_) tab_lidar_rp_->deactivate();
    else if (previous == tab_lidar_yaw_) tab_lidar_yaw_->deactivate();
    else if (previous == tab_lidar_gnss_yaw_) tab_lidar_gnss_yaw_->deactivate();
    else if (previous == tab_secondary_rp_) tab_secondary_rp_->deactivate();
    else if (previous == tab_lidar_lidar_) tab_lidar_lidar_->deactivate();
    else if (previous == tab_image_lidar_bev_) tab_image_lidar_bev_->deactivate();
    else if (previous == tab_radar_lidar_) tab_radar_lidar_->deactivate();

    previous_index = index;
}

void MainWindow::showIntrinsicsDialog()
{
    if (!config_mgr_->isLoaded()) {
        QMessageBox::information(this, "提示", "请先加载车辆配置文件。");
        return;
    }
    const auto& cameras = config_mgr_->getConfig().cameras;
    if (cameras.empty()) return;

    const QString intrinsics_dir = QString::fromStdString(
        config_mgr_->configDir() + "/intrinsics");
    QDir().mkpath(intrinsics_dir);

    QDialog dlg(this);
    dlg.setWindowTitle("相机内参管理");
    dlg.setMinimumWidth(640);

    auto* vl = new QVBoxLayout(&dlg);
    auto* hint = new QLabel("为每个相机指定内参文件（缺失项以红色标注）：", &dlg);
    hint->setWordWrap(true);
    vl->addWidget(hint);

    auto* scroll = new QScrollArea(&dlg);
    scroll->setWidgetResizable(true);
    auto* content = new QWidget(scroll);
    auto* grid = new QGridLayout(content);
    grid->setColumnStretch(1, 1);
    grid->setSpacing(8);
    grid->setContentsMargins(8, 8, 8, 8);

    grid->addWidget(new QLabel("<b>相机名称</b>", content), 0, 0);
    grid->addWidget(new QLabel("<b>内参文件</b>", content), 0, 1);
    grid->addWidget(new QLabel("<b>操作</b>", content), 0, 2);

    for (int i = 0; i < (int)cameras.size(); ++i) {
        const auto& cam = cameras[i];
        const QString qname = QString::fromStdString(cam.name);
        const bool has_k = !cam.intrinsics.K.empty();

        auto* lbl_name = new QLabel(qname, content);
        if (!has_k) lbl_name->setStyleSheet("QLabel{color:#f66;font-weight:bold;}");

        QString cur_path;
        if (!cam.intrinsics_file.empty())
            cur_path = QString::fromStdString(
                config_mgr_->configDir() + "/" + cam.intrinsics_file);
        auto* lbl_path = new QLabel(cur_path.isEmpty() ? "（未指定）" : cur_path, content);
        lbl_path->setWordWrap(false);
        lbl_path->setStyleSheet(has_k ? "QLabel{color:#8e8;}" : "QLabel{color:#f66;}");

        auto* btn = new QPushButton("选择文件...", content);
        const std::string cam_name_copy = cam.name;
        connect(btn, &QPushButton::clicked, [=, &dlg]() mutable {
            QString src = QFileDialog::getOpenFileName(
                &dlg, QString("选择 %1 的内参文件").arg(qname),
                intrinsics_dir, "YAML 文件 (*.yaml *.yml);;所有文件 (*)");
            if (src.isEmpty()) return;
            const QString dst = intrinsics_dir + "/" + qname + ".yaml";
            if (QFile::exists(dst) && dst != src) QFile::remove(dst);
            if (dst != src && !QFile::copy(src, dst)) {
                QMessageBox::warning(&dlg, "复制失败",
                    QString("无法将文件复制到：\n%1").arg(dst));
                return;
            }
            const QString final_path = (dst == src) ? src : dst;
            if (config_mgr_->setCameraIntrinsicsFile(cam_name_copy, final_path.toStdString())) {
                lbl_path->setText(final_path);
                lbl_path->setStyleSheet("QLabel{color:#8e8;}");
                lbl_name->setStyleSheet("QLabel{color:#ccc;}");
            } else {
                QMessageBox::warning(&dlg, "设置失败",
                    QString("无法设置内参路径：\n%1").arg(final_path));
            }
        });

        grid->addWidget(lbl_name, i + 1, 0);
        grid->addWidget(lbl_path, i + 1, 1);
        grid->addWidget(btn,      i + 1, 2);
    }

    content->setLayout(grid);
    scroll->setWidget(content);
    vl->addWidget(scroll, 1);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok, &dlg);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    vl->addWidget(btns);
    dlg.exec();
}

void MainWindow::checkIntrinsicsAndNotify()
{
    if (!config_mgr_->isLoaded()) return;
    if (config_mgr_->missingIntrinsics().empty()) return;
    showIntrinsicsDialog();
}

namespace {

// ---- 导出辅助结构 ----

struct CamIntrinsicsFlat {
    bool has_new_K{false},  has_new_Size{false};
    bool has_scaled_K{false}, has_scaled_Size{false};
    bool has_K{false},      has_Size{false};
    bool has_D{false},      has_Epsilon{false};
    std::vector<double> new_K;
    std::vector<int>    new_Size;
    std::vector<double> scaled_K;
    std::vector<int>    scaled_Size;
    std::vector<double> K;
    std::vector<int>    Size;
    std::vector<double> D;
    double              Epsilon{0.0};
    bool has_bev_roi{false};
    bool has_ipmscale{false};
    std::vector<int>    bev_roi;
    int                 ipmscale{0};
};

// 从独立 intrinsics 文件读取数值。支持两种格式：
// 1) OpenCV MEI 格式：model_type / image_width / image_height / mirror_parameters.xi /
//    distortion_parameters.{k1,k2,p1,p2} / projection_parameters.{gamma1,gamma2,u0,v0}
// 2) 扁平 YAML：顶层或相机名下包含 K/new_K/D 等键
// 映射规则：Size <- [image_width,image_height]；Epsilon <- xi；
// D <- [k1,k2,p1,p2]；K <- [gamma1,0,u0, 0,gamma2,v0, 0,0,1]。
bool loadIntrinsicsYaml(const std::string& path, const std::string& cam_name,
                        CamIntrinsicsFlat& out)
{
    std::ifstream f(path);
    if (!f.good()) return false;
    // OpenCV 写出的 "%YAML:1.0" 首行不是标准 YAML 指令，yaml-cpp 会报错，
    // 所以整文件读进来后把所有以 '%' 开头的行清除再解析。
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    {
        std::string cleaned;
        cleaned.reserve(content.size());
        size_t i = 0;
        while (i < content.size()) {
            size_t eol = content.find('\n', i);
            const std::string line = content.substr(i, eol - i);
            size_t first = line.find_first_not_of(" \t");
            if (first == std::string::npos || line[first] != '%')
                cleaned.append(line).append("\n");
            if (eol == std::string::npos) break;
            i = eol + 1;
        }
        content.swap(cleaned);
    }

    try {
        YAML::Node root = YAML::Load(content);

        // ---- OpenCV MEI 格式 ----
        const bool is_mei = root.IsMap() &&
            (root["projection_parameters"] || root["distortion_parameters"] ||
             root["mirror_parameters"] || root["model_type"] || root["image_width"]);
        if (is_mei) {
            if (root["image_width"] && root["image_height"]) {
                out.Size = { root["image_width"].as<int>(),
                             root["image_height"].as<int>() };
                out.has_Size = true;
            }
            if (root["projection_parameters"]) {
                const auto& p = root["projection_parameters"];
                const double gamma1 = p["gamma1"] ? p["gamma1"].as<double>() : 0.0;
                const double gamma2 = p["gamma2"] ? p["gamma2"].as<double>() : 0.0;
                const double u0     = p["u0"]     ? p["u0"].as<double>()     : 0.0;
                const double v0     = p["v0"]     ? p["v0"].as<double>()     : 0.0;
                out.K = { gamma1, 0.0, u0,
                          0.0,    gamma2, v0,
                          0.0,    0.0,    1.0 };
                out.has_K = true;
            }
            if (root["distortion_parameters"]) {
                const auto& d = root["distortion_parameters"];
                const double k1 = d["k1"] ? d["k1"].as<double>() : 0.0;
                const double k2 = d["k2"] ? d["k2"].as<double>() : 0.0;
                const double p1 = d["p1"] ? d["p1"].as<double>() : 0.0;
                const double p2 = d["p2"] ? d["p2"].as<double>() : 0.0;
                out.D = { k1, k2, p1, p2 };
                out.has_D = true;
            }
            if (root["mirror_parameters"] && root["mirror_parameters"]["xi"]) {
                out.Epsilon = root["mirror_parameters"]["xi"].as<double>();
                out.has_Epsilon = true;
            }
            // 模板里的默认 new_Size/scaled_Size/new_K（畸变矫正后的虚拟相机）
            out.new_Size    = {640, 320};               out.has_new_Size    = true;
            out.scaled_Size = {1920, 1080};             out.has_scaled_Size = true;
            out.new_K       = {320.0, 0.0, 320.0,
                               0.0, 320.0, 160.0,
                               0.0, 0.0, 1.0};          out.has_new_K       = true;
            return out.has_K && out.has_D;
        }

        // ---- 扁平 YAML（顶层或单个子 map 含 K/new_K/D） ----
        YAML::Node node = root;
        auto has_flat = [](const YAML::Node& n) {
            return n.IsMap() && (n["K"] || n["new_K"] || n["D"]);
        };
        if (!has_flat(node) && node.IsMap()) {
            if (!cam_name.empty() && node[cam_name] && node[cam_name].IsMap()) {
                node = node[cam_name];
            } else {
                for (auto it = node.begin(); it != node.end(); ++it) {
                    if (it->second.IsMap() && has_flat(it->second)) {
                        node = it->second;
                        break;
                    }
                }
            }
        }
        if (!has_flat(node)) return false;

        if (node["new_K"])    { out.new_K    = node["new_K"].as<std::vector<double>>();    out.has_new_K    = true; }
        if (node["new_Size"]) { out.new_Size = node["new_Size"].as<std::vector<int>>();    out.has_new_Size = true; }
        if (node["scaled_K"]) { out.scaled_K = node["scaled_K"].as<std::vector<double>>(); out.has_scaled_K = true; }
        if (node["scaled_Size"]) { out.scaled_Size = node["scaled_Size"].as<std::vector<int>>(); out.has_scaled_Size = true; }
        if (node["K"])        { out.K        = node["K"].as<std::vector<double>>();        out.has_K        = true; }
        if (node["Size"])     { out.Size     = node["Size"].as<std::vector<int>>();        out.has_Size     = true; }
        if (node["D"])        { out.D        = node["D"].as<std::vector<double>>();        out.has_D        = true; }
        if (node["Epsilon"])  { out.Epsilon  = node["Epsilon"].as<double>();               out.has_Epsilon  = true; }
        if (node["bev_roi"])  { out.bev_roi  = node["bev_roi"].as<std::vector<int>>();     out.has_bev_roi  = true; }
        if (node["ipmscale"]) { out.ipmscale = node["ipmscale"].as<int>();                 out.has_ipmscale = true; }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// 读取 image_lidar_<camera>.yaml 中 calibrated 段（返回是否都齐）
bool readImageLidarCalibrated(const std::string& proc_dir, const std::string& camera,
                              double& x, double& y, double& z,
                              double& roll, double& pitch, double& yaw)
{
    if (proc_dir.empty() || camera.empty()) return false;
    const std::string file = proc_dir + "/image_lidar_" + camera + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;
    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node c = root["calibrated"];
        if (!c || !c["x"] || !c["y"] || !c["z"] ||
            !c["roll_deg"] || !c["pitch_deg"] || !c["yaw_deg"]) {
            return false;
        }
        x         = c["x"].as<double>();
        y         = c["y"].as<double>();
        z         = c["z"].as<double>();
        roll      = c["roll_deg"].as<double>();
        pitch     = c["pitch_deg"].as<double>();
        yaw       = c["yaw_deg"].as<double>();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// 读取 radar_lidar_<radar>.yaml 中 calibrated 段（返回是否都齐）
bool readRadarLidarCalibrated(const std::string& proc_dir, const std::string& radar,
                              double& x, double& y, double& z,
                              double& roll, double& pitch, double& yaw)
{
    if (proc_dir.empty() || radar.empty()) return false;
    const std::string file = proc_dir + "/radar_lidar_" + radar + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;
    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node c = root["calibrated"];
        if (!c || !c["x"] || !c["y"] || !c["z"] ||
            !c["roll_deg"] || !c["pitch_deg"] || !c["yaw_deg"]) {
            return false;
        }
        x     = c["x"].as<double>();
        y     = c["y"].as<double>();
        z     = c["z"].as<double>();
        roll  = c["roll_deg"].as<double>();
        pitch = c["pitch_deg"].as<double>();
        yaw   = c["yaw_deg"].as<double>();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// 读取鱼眼相机标定文件 process/<cam>.yaml（格式不同：顶层是相机名，外参在 t/R_euler）
bool readFishEyeCalibrated(const std::string& proc_dir, const std::string& camera,
                           double& x, double& y, double& z,
                           double& roll, double& pitch, double& yaw)
{
    if (proc_dir.empty() || camera.empty()) return false;
    const std::string file = proc_dir + "/" + camera + ".yaml";
    std::ifstream f(file);
    if (!f.good()) return false;
    try {
        YAML::Node root = YAML::LoadFile(file);
        YAML::Node cam = root[camera];
        if (!cam || !cam["t"] || !cam["R_euler"]) return false;
        const auto& t = cam["t"];
        const auto& r = cam["R_euler"];
        if (t.size() < 3 || r.size() < 3) return false;
        x     = t[0].as<double>();
        y     = t[1].as<double>();
        z     = t[2].as<double>();
        roll  = r[0].as<double>();
        pitch = r[1].as<double>();
        yaw   = r[2].as<double>();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// --- YAML 写入辅助（保持与模板一致的格式） ---

std::string fmtDouble(double v, int prec = 6)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

template <typename T>
std::string joinVec(const std::vector<T>& v)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ", ";
        os << v[i];
    }
    os << "]";
    return os.str();
}

std::string joinVecDouble(const std::vector<double>& v, int prec = 6)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ", ";
        os << std::fixed << std::setprecision(prec) << v[i];
    }
    os << "]";
    return os.str();
}

std::string joinNames(const std::vector<std::string>& names)
{
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) os << ", ";
        os << "\"" << names[i] << "\"";
    }
    os << "]";
    return os.str();
}

} // anonymous namespace

void MainWindow::onExportParams()
{
    if (!config_mgr_->isLoaded()) {
        QMessageBox::information(this, "提示", "请先加载车辆配置文件。");
        return;
    }

    const auto& cfg = config_mgr_->getConfig();
    const auto& cams   = cfg.cameras;
    const auto& lidars = cfg.lidars;
    const auto& radars = cfg.radars;
    const std::string proc_dir = config_mgr_->processDir();

    // ---- 收集每个传感器的参数（来源 + 是否已标定） ----
    struct CamExport {
        std::string name;
        // extrinsics
        double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
        bool ext_from_calib{false};   // 来自 image_lidar_<cam>.yaml
        // intrinsics
        CamIntrinsicsFlat intr;
        bool has_intrinsics{false};
        std::string intrinsics_path;  // 绝对路径
    };
    struct LidarExport {
        std::string name;
        double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
        bool has_install{false};
        bool x_from_process{false}, y_from_process{false}, z_from_process{false};
        bool roll_from_process{false}, pitch_from_process{false}, yaw_from_process{false};
    };
    struct RadarExport {
        std::string name;
        double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
        bool has_install{false};
        bool calibrated{false};   // radar_lidar_<radar>.yaml 存在并齐全
    };

    std::vector<CamExport>   cam_list;
    std::vector<LidarExport> lidar_list;
    std::vector<RadarExport> radar_list;

    QStringList missing_warnings;  // 用于弹窗提示

    for (const auto& cam : cams) {
        CamExport e;
        e.name = cam.name;
        // 先用安装参数兜底
        double x, y, z, roll, pitch, yaw;
        if (config_mgr_->getCameraInstall(cam.name, x, y, z, roll, pitch, yaw)) {
            e.x = x; e.y = y; e.z = z;
            e.roll = roll; e.pitch = pitch; e.yaw = yaw;
        }
        // 尝试读取 image_lidar 标定结果
        double cx, cy, cz, cr, cp, cyaw;
        if (readImageLidarCalibrated(proc_dir, cam.name, cx, cy, cz, cr, cp, cyaw)) {
            e.x = cx; e.y = cy; e.z = cz;
            e.roll = cr; e.pitch = cp; e.yaw = cyaw;
            e.ext_from_calib = true;
        } else if (readFishEyeCalibrated(proc_dir, cam.name, cx, cy, cz, cr, cp, cyaw)) {
            e.x = cx; e.y = cy; e.z = cz;
            e.roll = cr; e.pitch = cp; e.yaw = cyaw;
            e.ext_from_calib = true;
        } else {
            missing_warnings << QString("相机 %1：外参未标定（缺少 process/image_lidar_%1.yaml）")
                                .arg(QString::fromStdString(cam.name));
        }
        // 读取独立 intrinsics 文件
        e.intrinsics_path = config_mgr_->getCameraIntrinsicsPath(cam.name);
        if (!e.intrinsics_path.empty()) {
            e.has_intrinsics = loadIntrinsicsYaml(e.intrinsics_path, cam.name, e.intr);
        }
        // 若独立文件读不到，就用 vehicle.yaml 中内嵌的 intrinsics（若存在）
        if (!e.has_intrinsics) {
            const auto& inr = cam.intrinsics;
            if (!inr.K.empty() || !inr.new_K.empty() || !inr.D.empty()) {
                if (!inr.new_K.empty())    { e.intr.new_K = inr.new_K;       e.intr.has_new_K = true; }
                if (!inr.new_Size.empty()) { e.intr.new_Size = inr.new_Size; e.intr.has_new_Size = true; }
                if (!inr.scaled_K.empty()) { e.intr.scaled_K = inr.scaled_K; e.intr.has_scaled_K = true; }
                if (!inr.scaled_Size.empty()) { e.intr.scaled_Size = inr.scaled_Size; e.intr.has_scaled_Size = true; }
                if (!inr.K.empty())        { e.intr.K = inr.K;               e.intr.has_K = true; }
                if (!inr.Size.empty())     { e.intr.Size = inr.Size;         e.intr.has_Size = true; }
                if (!inr.D.empty())        { e.intr.D = inr.D;               e.intr.has_D = true; }
                e.intr.Epsilon = inr.Epsilon; e.intr.has_Epsilon = true;
                e.has_intrinsics = true;
            }
        }
        if (!e.has_intrinsics || e.intr.K.empty()) {
            missing_warnings << QString("相机 %1：内参缺失")
                                .arg(QString::fromStdString(cam.name));
        }
        cam_list.push_back(std::move(e));
    }

    for (const auto& l : lidars) {
        LidarExport e;
        e.name = l.name;
        ConfigManager::LidarMergedParams mp;
        if (config_mgr_->getMergedLidarParams(l.name, mp)) {
            e.has_install = true;
            e.x = mp.x; e.y = mp.y; e.z = mp.z;
            e.roll = mp.roll_deg; e.pitch = mp.pitch_deg; e.yaw = mp.yaw_deg;
            e.x_from_process = mp.x_from_process;
            e.y_from_process = mp.y_from_process;
            e.z_from_process = mp.z_from_process;
            e.roll_from_process = mp.roll_from_process;
            e.pitch_from_process = mp.pitch_from_process;
            e.yaw_from_process = mp.yaw_from_process;
        }
        bool any_calib = e.x_from_process || e.y_from_process || e.z_from_process ||
                         e.roll_from_process || e.pitch_from_process || e.yaw_from_process;
        if (!any_calib) {
            missing_warnings << QString("LiDAR %1：未标定（使用安装参数）")
                                .arg(QString::fromStdString(l.name));
        }
        lidar_list.push_back(std::move(e));
    }

    for (const auto& r : radars) {
        RadarExport e;
        e.name = r.name;
        double x, y, z, roll, pitch, yaw;
        if (config_mgr_->getRadarInstall(r.name, x, y, z, roll, pitch, yaw)) {
            e.has_install = true;
            e.x = x; e.y = y; e.z = z;
            e.roll = roll; e.pitch = pitch; e.yaw = yaw;
        }
        double cx, cy, cz, cr, cp, cyaw;
        if (readRadarLidarCalibrated(proc_dir, r.name, cx, cy, cz, cr, cp, cyaw)) {
            e.x = cx; e.y = cy; e.z = cz;
            e.roll = cr; e.pitch = cp; e.yaw = cyaw;
            e.calibrated = true;
        } else {
            missing_warnings << QString("毫米波雷达 %1：未标定（使用安装参数）")
                                .arg(QString::fromStdString(r.name));
        }
        radar_list.push_back(std::move(e));
    }

    // ---- 有未标定项则提示确认 ----
    if (!missing_warnings.isEmpty()) {
        const QString details = missing_warnings.join("\n");
        QMessageBox mb(this);
        mb.setWindowTitle("存在未标定参数");
        mb.setIcon(QMessageBox::Warning);
        mb.setText("以下传感器参数未标定，将使用车辆配置中的安装参数兜底。\n"
                   "是否仍然继续导出？");
        mb.setDetailedText(details);
        mb.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        mb.setDefaultButton(QMessageBox::No);
        mb.button(QMessageBox::Yes)->setText("继续导出");
        mb.button(QMessageBox::No)->setText("取消");
        if (mb.exec() != QMessageBox::Yes) {
            statusBar()->showMessage("导出已取消");
            return;
        }
    }

    // ---- 选择导出根目录 ----
    const QString default_root = QString::fromStdString(
        ros::package::getPath("calibration_gui") + "/export");
    QDir().mkpath(default_root);
    QString root = QFileDialog::getExistingDirectory(
        this, "选择参数导出根目录", default_root);
    if (root.isEmpty()) return;

    // ---- 以车号建立子目录 ----
    QString vid = QString::fromStdString(cfg.vehicle.id).trimmed();
    if (vid.isEmpty()) vid = "unknown_vehicle";
    const QString out_dir = root + "/" + vid;
    if (!QDir().mkpath(out_dir)) {
        QMessageBox::critical(this, "错误", "无法创建导出目录：\n" + out_dir);
        return;
    }

    // ---- 写 cameras_distortion.proj_cfg ----
    const std::string cam_file = (out_dir + "/cameras_distortion.proj_cfg").toStdString();
    {
        std::ofstream os(cam_file);
        if (!os.is_open()) {
            QMessageBox::critical(this, "错误",
                QString("无法写入文件：\n%1").arg(QString::fromStdString(cam_file)));
            return;
        }
        os << "# 由 calibration_gui 导出生成\n";
        os << "# 车辆ID: " << cfg.vehicle.id << "\n";
        os << "# 说明: 标有 [INSTALL] 的外参项使用车辆配置中的安装参数（未经标定）。\n\n";

        const std::vector<std::string> distortion_camera_order = {
            "front_120", "left_front_100", "right_front_100",
            "left_back_100", "right_back_100", "back_120"
        };
        std::vector<std::string> names;
        for (const auto& name : distortion_camera_order) {
            auto it = std::find_if(cam_list.begin(), cam_list.end(),
                [&](const CamExport& c) { return c.name == name; });
            if (it != cam_list.end())
                names.push_back(name);
        }
        os << "camera_names: " << joinNames(names) << "\n\n";

        for (const auto& c : cam_list) {
            os << c.name << ":\n";
            // intrinsics（按模板顺序）
            if (c.intr.has_new_Size)    os << "  new_Size: "    << joinVec(c.intr.new_Size)    << "\n";
            if (c.intr.has_scaled_Size) os << "  scaled_Size: " << joinVec(c.intr.scaled_Size) << "\n";
            if (c.intr.has_new_K)       os << "  new_K: "       << joinVecDouble(c.intr.new_K, 6) << "\n";
            if (c.intr.has_Size)        os << "  Size: "        << joinVec(c.intr.Size)        << "\n";
            if (c.intr.has_K)           os << "  K: "           << joinVecDouble(c.intr.K, 10) << "\n";
            if (c.intr.has_D)           os << "  D: "           << joinVecDouble(c.intr.D, 12) << "\n";
            if (c.intr.has_Epsilon)     os << "  Epsilon: "     << fmtDouble(c.intr.Epsilon, 12) << "\n";
            if (!c.has_intrinsics || c.intr.K.empty()) {
                os << "  # [WARN] 内参缺失，未写入 K/D/Size 等字段\n";
            }
            // extrinsics
            const std::string ext_tag = c.ext_from_calib ? "" : "  # [INSTALL] 未标定外参，使用安装参数";
            if (!ext_tag.empty()) os << ext_tag << "\n";
            os << "  t: [" << fmtDouble(c.x, 6) << ", " << fmtDouble(c.y, 6) << ", "
               << fmtDouble(c.z, 6) << "]\n";
            // 对其节点的 [pitch, yaw, -roll]
            os << "  R_euler: [" << fmtDouble(c.pitch, 6) << ", " << fmtDouble(c.yaw, 6) << ", "
               << fmtDouble(-c.roll, 6) << "]\n";
            os << "  vcs_to_ground_height: " << fmtDouble(cfg.vehicle.z_offset, 6) << "\n";
            os << "  base_euler: 0.0\n";
            if (c.intr.has_bev_roi)  os << "  bev_roi: "  << joinVec(c.intr.bev_roi)  << "\n";
            if (c.intr.has_ipmscale) os << "  ipmscale: " << c.intr.ipmscale << "\n";
            os << "\n";
        }
    }

    // ---- 写 sensor_calibration.proj_cfg ----
    const std::string sensor_file = (out_dir + "/sensor_calibration.proj_cfg").toStdString();
    {
        std::ofstream os(sensor_file);
        if (!os.is_open()) {
            QMessageBox::critical(this, "错误",
                QString("无法写入文件：\n%1").arg(QString::fromStdString(sensor_file)));
            return;
        }
        os << "# 由 calibration_gui 导出生成\n";
        os << "# 车辆ID: " << cfg.vehicle.id << "\n";
        os << "# 说明: 标有 [INSTALL] 的字段未经标定，直接使用车辆配置中的安装参数。\n";
        os << "# ====================================================================\n";

        std::vector<std::string> lnames;
        for (const auto& l : lidar_list) lnames.push_back(l.name);
        os << "lidar_names: " << joinNames(lnames) << "\n";

        for (const auto& l : lidar_list) {
            os << l.name << ":\n";
            auto tag = [](bool from_process) {
                return from_process ? "" : " # [INSTALL]";
            };
            bool any_t_installed = !l.x_from_process || !l.y_from_process || !l.z_from_process;
            os << "  lidar_t: [" << fmtDouble(l.x, 6) << ", " << fmtDouble(l.y, 6) << ", "
               << fmtDouble(l.z, 6) << "]";
            if (any_t_installed) {
                os << "  # ";
                os << (l.x_from_process ? "x=calib" : "x=[INSTALL]") << ", ";
                os << (l.y_from_process ? "y=calib" : "y=[INSTALL]") << ", ";
                os << (l.z_from_process ? "z=calib" : "z=[INSTALL]");
            }
            os << "\n";
            (void)tag;
            bool any_r_installed = !l.roll_from_process || !l.pitch_from_process || !l.yaw_from_process;
            os << "  lidar_R: [" << fmtDouble(l.roll, 6) << ", " << fmtDouble(l.pitch, 6) << ", "
               << fmtDouble(l.yaw, 6) << "]";
            if (any_r_installed) {
                os << "  # ";
                os << (l.roll_from_process  ? "roll=calib"  : "roll=[INSTALL]") << ", ";
                os << (l.pitch_from_process ? "pitch=calib" : "pitch=[INSTALL]") << ", ";
                os << (l.yaw_from_process   ? "yaw=calib"   : "yaw=[INSTALL]");
            }
            os << "\n";
        }
        os << "# ====================================================================\n\n";

        std::vector<std::string> rnames;
        for (const auto& r : radar_list) rnames.push_back(r.name);
        os << "radar_names: " << joinNames(rnames) << "\n";

        for (const auto& r : radar_list) {
            os << r.name << ":\n";
            const char* tag = r.calibrated ? "" : "  # [INSTALL] 未标定，使用安装参数";
            if (*tag) os << tag << "\n";
            os << "  radar_t: [" << fmtDouble(r.x, 6) << ", " << fmtDouble(r.y, 6) << ", "
               << fmtDouble(r.z, 6) << "]\n";
            os << "  radar_R: [" << fmtDouble(r.roll, 6) << ", " << fmtDouble(r.pitch, 6) << ", "
               << fmtDouble(r.yaw, 6) << "]\n";
        }
        os << "# ====================================================================\n";
    }

    // ---- 复制 process 目录下的 .bin 文件到目标目录 ----
    QStringList copied_bins;
    QStringList failed_bins;
    if (!proc_dir.empty()) {
        QDir pd(QString::fromStdString(proc_dir));
        const QStringList bins = pd.entryList(QStringList() << "*.bin", QDir::Files);
        for (const auto& name : bins) {
            const QString src = pd.filePath(name);
            const QString dst = out_dir + "/" + name;
            if (QFile::exists(dst)) QFile::remove(dst);
            if (QFile::copy(src, dst)) copied_bins << name;
            else                      failed_bins << name;
        }
    }

    // ---- 完成提示 ----
    QString msg = QString("导出完成：\n%1\n\n"
                          "- cameras_distortion.proj_cfg\n"
                          "- sensor_calibration.proj_cfg")
                      .arg(out_dir);
    if (!copied_bins.isEmpty()) {
        msg += "\n\nbin 文件：\n  " + copied_bins.join("\n  ");
    }
    if (!failed_bins.isEmpty()) {
        msg += "\n\n复制失败的 bin：\n  " + failed_bins.join("\n  ");
    }
    QMessageBox::information(this, "导出成功", msg);
    statusBar()->showMessage("已导出至：" + out_dir);
}

void MainWindow::loadTabsConfig()
{
    // 默认全部启用
    const std::vector<std::string> all_tabs = {
        "lidar_rp", "secondary_rp", "lidar_yaw", "lidar_gnss_yaw",
        "lidar_lidar", "image_lidar", "camera_cali", "bev",
        "front_stitch", "radar_lidar"
    };
    for (const auto& t : all_tabs) tabs_enabled_[t] = true;
    tabs_order_.clear();

    // 尝试从 ROS package 路径加载
    std::string pkg_path = ros::package::getPath("calibration_gui");
    if (pkg_path.empty()) {
        tabs_order_ = all_tabs;
        return;
    }
    const std::string cfg_file = pkg_path + "/config/tabs_config.yaml";

    try {
        YAML::Node root = YAML::LoadFile(cfg_file);
        if (!root["tabs"]) {
            tabs_order_ = all_tabs;
            return;
        }
        for (const auto& item : root["tabs"]) {
            const std::string name = item["name"].as<std::string>();
            const bool enabled = item["enabled"] ? item["enabled"].as<bool>() : true;
            tabs_enabled_[name] = enabled;
            tabs_order_.push_back(name);
        }
        // YAML 中未声明的 Tab 保持默认顺序追加到末尾
        for (const auto& t : all_tabs) {
            if (std::find(tabs_order_.begin(), tabs_order_.end(), t) == tabs_order_.end()) {
                tabs_order_.push_back(t);
            }
        }
    } catch (...) {
        // 文件不存在或解析失败时保持默认全部启用、默认顺序
        tabs_order_ = all_tabs;
    }
}

bool MainWindow::isTabEnabled(const std::string& name) const
{
    auto it = tabs_enabled_.find(name);
    return it == tabs_enabled_.end() || it->second;
}

} // namespace calibration_gui
