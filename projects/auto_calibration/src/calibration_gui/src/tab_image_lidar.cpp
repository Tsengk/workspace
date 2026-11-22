#include "calibration_gui/tab_image_lidar.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QFrame>
#include <QDir>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>

namespace calibration_gui {

// 与 lidar_camera_projection.launch 中 name 一致
static const std::string kNodeNs           = "/lidar_camera_projection_node";
static const std::string kProjectionPkg    = "lidar_camera_projection";
static const std::string kProjectionLaunch = "lidar_camera_projection.launch";
static const std::string kNodeKey          = "lidar_camera_projection";

namespace {

struct ImageLidarProcess {
    bool has{false};
    double x{0}, y{0}, z{0};
    double roll_deg{0}, pitch_deg{0}, yaw_deg{0};
};

// 读取 process/image_lidar_<camera>.yaml 的 calibrated 段
bool readImageLidarProcess(const std::string& proc_dir,
                           const std::string& camera,
                           ImageLidarProcess& out)
{
    out = ImageLidarProcess{};
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
        out.x         = c["x"].as<double>();
        out.y         = c["y"].as<double>();
        out.z         = c["z"].as<double>();
        out.roll_deg  = c["roll_deg"].as<double>();
        out.pitch_deg = c["pitch_deg"].as<double>();
        out.yaw_deg   = c["yaw_deg"].as<double>();
        out.has = true;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

} // namespace

TabImageLidar::TabImageLidar(std::shared_ptr<ConfigManager> cfg,
                             std::shared_ptr<RosWorker> ros,
                             QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
    connect(ros_.get(), &RosWorker::nodeStarted, this, &TabImageLidar::onNodeStarted);
    connect(ros_.get(), &RosWorker::nodeStopped, this, &TabImageLidar::onNodeStopped);
}

void TabImageLidar::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    auto* left = new QWidget(this);
    left->setFixedWidth(360);
    auto* vl = new QVBoxLayout(left);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(10);

    // ---- 节点管理 ----
    auto* grp_node = new QGroupBox("节点管理", this);
    auto* hl_node = new QHBoxLayout(grp_node);
    hl_node->setContentsMargins(8, 8, 8, 8);
    hl_node->setSpacing(8);
    btn_start_node_ = new QPushButton("▶ 启动节点", this);
    btn_stop_node_  = new QPushButton("⏹ 停止节点", this);
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(false);
    connect(btn_start_node_, &QPushButton::clicked, this, &TabImageLidar::onStartNode);
    connect(btn_stop_node_,  &QPushButton::clicked, this, &TabImageLidar::onStopNode);
    label_node_status_ = new QLabel("● 未运行", this);
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    hl_node->addWidget(btn_start_node_);
    hl_node->addWidget(btn_stop_node_);
    hl_node->addWidget(label_node_status_);
    vl->addWidget(grp_node);

    // ---- 可视化控制 ----
    auto* grp_viz = new QGroupBox("可视化控制", this);
    auto* vl_viz = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    vl_viz->setSpacing(6);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", this);
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabImageLidar::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    vl->addWidget(grp_viz);

    // ---- 传感器选择 ----
    auto* grp_sensor = new QGroupBox("传感器选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_lidar_  = new QComboBox(this);
    combo_camera_ = new QComboBox(this);
    combo_lidar_->addItem("请选择", QString());
    combo_camera_->addItem("请选择", QString());
    connect(combo_lidar_,  &QComboBox::currentTextChanged,
            this, &TabImageLidar::onLidarSelected);
    connect(combo_camera_, &QComboBox::currentTextChanged,
            this, &TabImageLidar::onCameraSelected);
    fl_s->addRow("参考激光雷达：", combo_lidar_);
    fl_s->addRow("目标相机：",     combo_camera_);

    label_lidar_status_ = new QLabel("—", this);
    label_lidar_status_->setWordWrap(true);
    fl_s->addRow("雷达标定：", label_lidar_status_);

    label_install_ = new QLabel("—", this);
    label_install_->setWordWrap(true);
    label_install_->setStyleSheet("QLabel{color:#8e8;}");
    fl_s->addRow("相机安装：", label_install_);
    vl->addWidget(grp_sensor);

    // ---- 6DOF 外参微调 ----
    auto* grp_tune = new QGroupBox("相机-LiDAR 外参（相对 LiDAR）微调", this);
    auto* fl_t = new QFormLayout(grp_tune);
    auto makeSpin = [this](double lo, double hi, double step, int dec,
                           const QString& suffix) {
        auto* s = new QDoubleSpinBox(this);
        s->setRange(lo, hi);
        s->setSingleStep(step);
        s->setDecimals(dec);
        s->setSuffix(suffix);
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TabImageLidar::onParamChanged);
        return s;
    };
    spin_x_     = makeSpin(-100.0,  100.0, 0.01, 4, " m");
    spin_y_     = makeSpin(-100.0,  100.0, 0.01, 4, " m");
    spin_z_     = makeSpin(-100.0,  100.0, 0.01, 4, " m");
    spin_roll_  = makeSpin(-180.0, 180.0, 0.05, 3, " °");
    spin_pitch_ = makeSpin(-180.0, 180.0, 0.05, 3, " °");
    spin_yaw_   = makeSpin(-180.0, 180.0, 0.05, 3, " °");
    fl_t->addRow("X：",     spin_x_);
    fl_t->addRow("Y：",     spin_y_);
    fl_t->addRow("Z：",     spin_z_);
    fl_t->addRow("Roll：",  spin_roll_);
    fl_t->addRow("Pitch：", spin_pitch_);
    fl_t->addRow("Yaw：",   spin_yaw_);
    vl->addWidget(grp_tune);

    // ---- 实时状态 ----
    auto* grp_status = new QGroupBox("实时状态", this);
    auto* fl_st = new QFormLayout(grp_status);
    label_data_status_ = new QLabel("未订阅", this);
    label_data_status_->setWordWrap(true);
    fl_st->addRow("数据：", label_data_status_);
    vl->addWidget(grp_status);

    // ---- 操作 ----
    auto* grp_btn = new QGroupBox("操作", this);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8);
    vl_b->setContentsMargins(8, 8, 8, 8);
    btn_save_  = new QPushButton("保存标定结果到 YAML", this);
    btn_reset_ = new QPushButton("重置为相机安装参数", this);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    connect(btn_save_,  &QPushButton::clicked, this, &TabImageLidar::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabImageLidar::onResetParams);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    auto* lbl_hint = new QLabel(
        "<small><b>使用步骤：</b><br>"
        "1. 加载车辆配置<br>"
        "2. 选已标定的参考激光雷达 + 目标相机<br>"
        "3. 点 [启动节点]（需相机内参文件与雷达已标定）<br>"
        "4. 开启可视化，拖 X/Y/Z/Roll/Pitch/Yaw 使点云贴合图像<br>"
        "5. 点 [保存标定结果到 YAML]</small>", this);
    lbl_hint->setWordWrap(true);
    lbl_hint->setStyleSheet("QLabel{color:#888;border:1px solid #333;padding:4px;}");
    vl->addWidget(lbl_hint);

    vl->addStretch();
    auto* scroll_left = new QScrollArea(this);
    scroll_left->setWidgetResizable(true);
    scroll_left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll_left->setFrameShape(QFrame::NoFrame);
    scroll_left->setWidget(left);
    scroll_left->setFixedWidth(left->width() + 16);
    h->addWidget(scroll_left);

    // ---- 右侧：投影叠加图 ----
    auto* right_box = new QGroupBox("投影叠加图（点云 → 相机图像）", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new ImageViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);
}

void TabImageLidar::applyConfig()
{
    combo_lidar_->blockSignals(true);
    combo_camera_->blockSignals(true);
    combo_lidar_->clear();
    combo_camera_->clear();
    combo_lidar_->addItem("请选择", QString());
    combo_camera_->addItem("请选择", QString());
    for (const auto& n : cfg_->lidarNames())
        combo_lidar_->addItem(QString::fromStdString(n));
    for (const auto& n : cfg_->cameraNames())
        combo_camera_->addItem(QString::fromStdString(n));
    combo_lidar_->blockSignals(false);
    combo_camera_->blockSignals(false);

    current_lidar_.clear();
    current_camera_.clear();
    prev_lidar_.clear();
    lidar_is_calibrated_ = false;
    label_lidar_status_->setText("—");
    label_install_->setText("—");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabImageLidar::deactivate()
{
    // 先停投影节点
    onStopNode();

    // 停 lidar driver
    if (!prev_lidar_.empty()) {
        ros_->killNode("driver_" + prev_lidar_);
        prev_lidar_.clear();
    }

    combo_lidar_->blockSignals(true);
    combo_camera_->blockSignals(true);
    combo_lidar_->setCurrentIndex(0);
    combo_camera_->setCurrentIndex(0);
    combo_lidar_->blockSignals(false);
    combo_camera_->blockSignals(false);

    current_lidar_.clear();
    current_camera_.clear();
    lidar_is_calibrated_ = false;
    label_lidar_status_->setText("—");
    label_install_->setText("—");
    label_data_status_->setText("未订阅");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabImageLidar::refreshLidarStatus()
{
    if (current_lidar_.empty()) {
        lidar_is_calibrated_ = false;
        label_lidar_status_->setText("—");
        return;
    }

    ConfigManager::LidarMergedParams params;
    bool has_install = cfg_->getMergedLidarParams(current_lidar_, params);
    bool any_from_process = params.x_from_process || params.y_from_process ||
                             params.z_from_process || params.roll_from_process ||
                             params.pitch_from_process || params.yaw_from_process;
    lidar_is_calibrated_ = has_install && any_from_process;

    if (lidar_is_calibrated_) {
        label_lidar_status_->setText("已标定（process/ 存在标定结果）");
        label_lidar_status_->setStyleSheet("QLabel{color:#4f4;}");
    } else {
        label_lidar_status_->setText("未标定，无法启动节点");
        label_lidar_status_->setStyleSheet("QLabel{color:#f66;}");
    }
}

void TabImageLidar::updateStartButtonState()
{
    const bool can_start = !current_lidar_.empty() &&
                           !current_camera_.empty() &&
                           lidar_is_calibrated_ &&
                           !ros_->isNodeRunning(kNodeKey);
    btn_start_node_->setEnabled(can_start);
}

void TabImageLidar::loadInitialParamsToSpins(const std::string& camera_name)
{
    double ix, iy, iz, iroll, ipitch, iyaw;
    if (!cfg_->getCameraInstall(camera_name, ix, iy, iz, iroll, ipitch, iyaw)) {
        label_install_->setText("—");
        return;
    }
    label_install_->setText(
        QString("安装 x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
        .arg(ix, 0, 'f', 3).arg(iy, 0, 'f', 3).arg(iz, 0, 'f', 3)
        .arg(iroll, 0, 'f', 2).arg(ipitch, 0, 'f', 2).arg(iyaw, 0, 'f', 2));

    // 优先读 process/image_lidar_<camera>.yaml 的 calibrated 段（直接透传）
    ImageLidarProcess p;
    const std::string proc_dir = cfg_->processDir();
    const bool has_process = readImageLidarProcess(proc_dir, camera_name, p);

    double sx, sy, sz, sroll, spitch, syaw;
    if (has_process) {
        sx = p.x;  sy = p.y;  sz = p.z;
        sroll = p.roll_deg;  spitch = p.pitch_deg;  syaw = p.yaw_deg;
    } else {
        // 安装参数 → (y, z, -x) 变换作为初值
        sx = iy;  sy = iz;  sz = -ix;
        sroll = iroll;  spitch = ipitch;  syaw = iyaw;
    }

    auto blockAll = [&](bool b) {
        spin_x_->blockSignals(b); spin_y_->blockSignals(b); spin_z_->blockSignals(b);
        spin_roll_->blockSignals(b); spin_pitch_->blockSignals(b); spin_yaw_->blockSignals(b);
    };
    blockAll(true);
    spin_x_->setValue(sx);       spin_y_->setValue(sy);       spin_z_->setValue(sz);
    spin_roll_->setValue(sroll); spin_pitch_->setValue(spitch); spin_yaw_->setValue(syaw);
    blockAll(false);
}

void TabImageLidar::onLidarSelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_lidar_.empty()) {
            ros_->killNode("driver_" + prev_lidar_);
            prev_lidar_.clear();
        }
        current_lidar_.clear();
        lidar_is_calibrated_ = false;
        label_lidar_status_->setText("—");
        updateStartButtonState();
        return;
    }

    const std::string new_lidar = name.toStdString();
    if (!prev_lidar_.empty() && prev_lidar_ != new_lidar) {
        ros_->killNode("driver_" + prev_lidar_);
    }
    prev_lidar_    = new_lidar;
    current_lidar_ = new_lidar;

    // 启动新 driver（如果配置了 launch_file）
    const std::string launch_path = cfg_->getLidarLaunchFilePath(current_lidar_);
    if (!launch_path.empty()) {
        const std::string packets_topic     = cfg_->getLidarPacketsTopic(current_lidar_);
        const std::string point_cloud_topic = cfg_->getLidarPointCloudTopic(current_lidar_);
        QStringList driver_args;
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        double x, y, z, roll, pitch, yaw;
        if (cfg_->getLidarInstall(current_lidar_, x, y, z, roll, pitch, yaw)) {
            ConfigManager::LidarMergedParams m;
            if (cfg_->getMergedLidarParams(current_lidar_, m)) {
                x = m.x; y = m.y; z = m.z;
                roll = m.roll_deg; pitch = m.pitch_deg; yaw = m.yaw_deg;
            }
            driver_args << QString("xoffset:=%1").arg(x)
                        << QString("yoffset:=%1").arg(y)
                        << QString("zoffset:=%1").arg(z)
                        << QString("roll:=%1").arg(roll)
                        << QString("pitch:=%1").arg(pitch)
                        << QString("yaw:=%1").arg(yaw)
                        << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        }
        ros_->launchNodeByPath(launch_path, "driver_" + current_lidar_, driver_args);
    }

    refreshLidarStatus();
    updateStartButtonState();
}

void TabImageLidar::onCameraSelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        current_camera_.clear();
        label_install_->setText("—");
        btn_reset_->setEnabled(false);
        updateStartButtonState();
        return;
    }
    current_camera_ = name.toStdString();
    loadInitialParamsToSpins(current_camera_);
    btn_reset_->setEnabled(true);
    updateStartButtonState();
}

void TabImageLidar::onStartNode()
{
    if (current_lidar_.empty() || current_camera_.empty()) {
        QMessageBox::warning(this, "无法启动", "请先选择参考激光雷达与目标相机。");
        return;
    }
    if (!lidar_is_calibrated_) {
        QMessageBox::warning(this, "无法启动",
            QString("LiDAR \"%1\" 尚未完成标定，process/ 下找不到标定结果。\n"
                    "请先在前置 Tab 完成该 LiDAR 的标定。")
                .arg(QString::fromStdString(current_lidar_)));
        return;
    }

    const std::string intrinsics = cfg_->getCameraIntrinsicsPath(current_camera_);
    if (intrinsics.empty()) {
        QMessageBox::warning(this, "无法启动",
            QString("相机 \"%1\" 未设置内参文件，请到 菜单→传感器→管理相机内参 指定。")
                .arg(QString::fromStdString(current_camera_)));
        return;
    }

    // image topic 从车辆配置直接拿
    std::string image_topic;
    for (const auto& c : cfg_->getConfig().cameras) {
        if (c.name == current_camera_) { image_topic = c.topic; break; }
    }
    const std::string lidar_topic = cfg_->getLidarPointCloudTopic(current_lidar_);
    output_topic_ = std::string("/lidar_camera_projection/") + current_camera_ + "/image";

    const std::string proc_dir = cfg_->processDir();
    if (!proc_dir.empty()) QDir().mkpath(QString::fromStdString(proc_dir));

    // spinbox 当前值即为 "要传给节点" 的外参：
    // - 若 process/image_lidar_<camera>.yaml 存在，loadInitialParamsToSpins 已填入 calibrated 值（直接透传）
    // - 若无，则填入的是安装参数做 (y, z, -x) 变换后的值
    const double cx     = spin_x_->value();
    const double cy     = spin_y_->value();
    const double cz     = spin_z_->value();
    const double croll  = spin_roll_->value();
    const double cpitch = spin_pitch_->value();
    const double cyaw   = spin_yaw_->value();

    QStringList args;
    args << QString("camera_file:=%1").arg(QString::fromStdString(intrinsics));
    if (!image_topic.empty())
        args << QString("image_topic:=%1").arg(QString::fromStdString(image_topic));
    if (!lidar_topic.empty())
        args << QString("lidar_topic:=%1").arg(QString::fromStdString(lidar_topic));
    args << QString("output_topic:=%1").arg(QString::fromStdString(output_topic_));
    if (!proc_dir.empty()) {
        args << QString("output_dir:=%1").arg(QString::fromStdString(proc_dir));
        args << QString("output_yaml_path:=%1/image_lidar_%2.yaml")
                 .arg(QString::fromStdString(proc_dir))
                 .arg(QString::fromStdString(current_camera_));
    }
    args << QString("x:=%1").arg(cx)
         << QString("y:=%1").arg(cy)
         << QString("z:=%1").arg(cz)
         << QString("roll:=%1").arg(croll)
         << QString("pitch:=%1").arg(cpitch)
         << QString("yaw:=%1").arg(cyaw);

    bool ok = ros_->launchNode(kProjectionPkg, kProjectionLaunch, kNodeKey, args);
    if (!ok) {
        label_node_status_->setText("● 启动失败");
        label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    }
}

void TabImageLidar::onStopNode()
{
    ros_->killNode(kNodeKey);
    ros_->unsubscribeAll();
    viewer_->clear();
    label_data_status_->setText("未订阅");
}

void TabImageLidar::onNodeStarted(const QString& key)
{
    if (key != QString::fromStdString(kNodeKey)) return;
    label_node_status_->setText("● 运行中");
    label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(true);
    btn_save_->setEnabled(true);

    if (viz_enabled_ && !output_topic_.empty()) {
        label_data_status_->setText("订阅中，等待图像...");
        ros_->subscribeImage(output_topic_,
            [this](cv::Mat img) { onImageReceived(img); });
    }
}

void TabImageLidar::onNodeStopped(const QString& key)
{
    if (key != QString::fromStdString(kNodeKey)) return;
    label_node_status_->setText("● 未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_stop_node_->setEnabled(false);
    btn_save_->setEnabled(false);
    updateStartButtonState();
}

void TabImageLidar::onImageReceived(cv::Mat img)
{
    if (!viz_enabled_) return;
    label_data_status_->setText(
        QString("收到图像 %1x%2").arg(img.cols).arg(img.rows));
    viewer_->showImage(img);
}

void TabImageLidar::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        if (ros_->isNodeRunning(kNodeKey) && !output_topic_.empty()) {
            label_data_status_->setText("订阅中，等待图像...");
            ros_->subscribeImage(output_topic_,
                [this](cv::Mat img) { onImageReceived(img); });
        }
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabImageLidar::onParamChanged(double /*v*/)
{
    if (!ros_->isNodeRunning(kNodeKey)) return;
    ros_->setParam(kNodeNs + "/x",     spin_x_->value());
    ros_->setParam(kNodeNs + "/y",     spin_y_->value());
    ros_->setParam(kNodeNs + "/z",     spin_z_->value());
    ros_->setParam(kNodeNs + "/roll",  spin_roll_->value());
    ros_->setParam(kNodeNs + "/pitch", spin_pitch_->value());
    ros_->setParam(kNodeNs + "/yaw",   spin_yaw_->value());
}

void TabImageLidar::onResetParams()
{
    if (current_camera_.empty()) return;
    loadInitialParamsToSpins(current_camera_);
    // 立刻把初始值也写给节点
    if (ros_->isNodeRunning(kNodeKey)) {
        ros_->setParam(kNodeNs + "/x",     spin_x_->value());
        ros_->setParam(kNodeNs + "/y",     spin_y_->value());
        ros_->setParam(kNodeNs + "/z",     spin_z_->value());
        ros_->setParam(kNodeNs + "/roll",  spin_roll_->value());
        ros_->setParam(kNodeNs + "/pitch", spin_pitch_->value());
        ros_->setParam(kNodeNs + "/yaw",   spin_yaw_->value());
    }
}

void TabImageLidar::onSaveCalib()
{
    if (current_camera_.empty() || current_lidar_.empty()) {
        QMessageBox::warning(this, "保存失败", "请先选择参考激光雷达与目标相机。");
        return;
    }
    const std::string proc = cfg_->processDir();
    if (proc.empty()) {
        QMessageBox::warning(this, "保存失败", "未加载车辆配置，无法确定 process 目录。");
        return;
    }
    QDir().mkpath(QString::fromStdString(proc));

    double ix, iy, iz, iroll, ipitch, iyaw;
    cfg_->getCameraInstall(current_camera_, ix, iy, iz, iroll, ipitch, iyaw);
    // installation 段也写到投影节点的 (y, z, -x) 坐标系，与 calibrated 对齐以便 delta 有意义
    const double inst_x = iy;
    const double inst_y = iz;
    const double inst_z = -ix;
    const double inst_roll  = iroll;
    const double inst_pitch = ipitch;
    const double inst_yaw   = iyaw;
    const double cx = spin_x_->value();
    const double cy = spin_y_->value();
    const double cz = spin_z_->value();
    const double croll  = spin_roll_->value();
    const double cpitch = spin_pitch_->value();
    const double cyaw   = spin_yaw_->value();

    std::time_t t = std::time(nullptr);
    char tbuf[64];
    std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    const std::string file = proc + "/image_lidar_" + current_camera_ + ".yaml";
    std::ofstream f(file);
    if (!f.is_open()) {
        QMessageBox::critical(this, "保存失败",
            QString("无法写入文件：\n%1").arg(QString::fromStdString(file)));
        return;
    }
    f << std::fixed << std::setprecision(6);
    f << "schema_version: 1\n"
      << "calib_type: image_lidar\n"
      << "sensor_name: " << current_camera_ << "\n"
      << "reference_lidar: " << current_lidar_ << "\n"
      << "generated_at: " << tbuf << "\n"
      << "method: manual_projection_alignment\n"
      << "coordinate_system: FLU\n"
      << "installation:\n"
      << "  x: "         << inst_x     << "\n"
      << "  y: "         << inst_y     << "\n"
      << "  z: "         << inst_z     << "\n"
      << "  roll_deg: "  << inst_roll  << "\n"
      << "  pitch_deg: " << inst_pitch << "\n"
      << "  yaw_deg: "   << inst_yaw   << "\n"
      << "calibrated:\n"
      << "  x: "         << cx     << "\n"
      << "  y: "         << cy     << "\n"
      << "  z: "         << cz     << "\n"
      << "  roll_deg: "  << croll  << "\n"
      << "  pitch_deg: " << cpitch << "\n"
      << "  yaw_deg: "   << cyaw   << "\n"
      << "delta:\n"
      << "  delta_x_m: "      << (cx - inst_x)         << "\n"
      << "  delta_y_m: "      << (cy - inst_y)         << "\n"
      << "  delta_z_m: "      << (cz - inst_z)         << "\n"
      << "  delta_roll_deg: " << (croll - inst_roll)   << "\n"
      << "  delta_pitch_deg: "<< (cpitch - inst_pitch) << "\n"
      << "  delta_yaw_deg: "  << (cyaw - inst_yaw)     << "\n";
    f.close();

    QMessageBox::information(this, "保存成功",
        QString("已写入：\n%1").arg(QString::fromStdString(file)));
}

} // namespace calibration_gui
