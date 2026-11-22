#include "calibration_gui/tab_lidar_lidar.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QCheckBox>
#include <QDir>
#include <QScrollArea>
#include <QFrame>

namespace calibration_gui {

namespace {
bool requirePrimaryFullyCalibrated(QWidget* parent,
                                   const std::shared_ptr<ConfigManager>& cfg,
                                   const std::string& lidar_name,
                                   ConfigManager::LidarMergedParams& params)
{
    if (!cfg->getMergedLidarParams(lidar_name, params)) {
        QMessageBox::warning(parent, "参数缺失", QString("未找到主雷达 %1 的安装参数。")
            .arg(QString::fromStdString(lidar_name)));
        return false;
    }
    if (!(params.z_from_process && params.roll_from_process && params.pitch_from_process && params.yaw_from_process)) {
        QMessageBox::warning(parent, "缺少主雷达前置标定",
            QString("主雷达 %1 缺少 roll/pitch/yaw/z 标定结果，请先完成 lidar_rp 和 lidar_yaw 标定。")
                .arg(QString::fromStdString(lidar_name)));
        return false;
    }
    return true;
}

bool getMergedLidarParamsOrWarn(QWidget* parent,
                                const std::shared_ptr<ConfigManager>& cfg,
                                const std::string& lidar_name,
                                ConfigManager::LidarMergedParams& params)
{
    if (cfg->getMergedLidarParams(lidar_name, params)) return true;
    QMessageBox::warning(parent, "参数缺失", QString("未找到雷达 %1 的安装参数。")
        .arg(QString::fromStdString(lidar_name)));
    return false;
}
}

static const std::string kNodeNs = "/lidar_lidar_cali_clicked";

TabLidarLidar::TabLidarLidar(std::shared_ptr<ConfigManager> cfg,
                              std::shared_ptr<RosWorker> ros,
                              QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
    connect(ros_.get(), &RosWorker::nodeStarted, this, &TabLidarLidar::onNodeStarted);
    connect(ros_.get(), &RosWorker::nodeStopped, this, &TabLidarLidar::onNodeStopped);
}

void TabLidarLidar::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    auto* left = new QWidget(this);
    left->setFixedWidth(350);
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
    btn_stop_node_->setEnabled(false);
    connect(btn_start_node_, &QPushButton::clicked, this, &TabLidarLidar::onStartNode);
    connect(btn_stop_node_,  &QPushButton::clicked, this, &TabLidarLidar::onStopNode);
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
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabLidarLidar::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    chk_show_primary_ = new QCheckBox("叠加显示主雷达原始点云", this);
    connect(chk_show_primary_, &QCheckBox::stateChanged, this, &TabLidarLidar::onRawCloudToggle);
    vl_viz->addWidget(chk_show_primary_);
    chk_show_secondary_ = new QCheckBox("叠加显示辅雷达原始点云", this);
    connect(chk_show_secondary_, &QCheckBox::stateChanged, this, &TabLidarLidar::onRawCloudToggle);
    vl_viz->addWidget(chk_show_secondary_);
    btn_pick_mode_ = new QPushButton("🖱 开启选点模式", this);
    btn_pick_mode_->setCheckable(true);
    btn_pick_mode_->setEnabled(false);
    connect(btn_pick_mode_, &QPushButton::toggled, this, &TabLidarLidar::onPickModeToggle);
    vl_viz->addWidget(btn_pick_mode_);
    vl->addWidget(grp_viz);

    // ---- 传感器选择 ----
    auto* grp_sensor = new QGroupBox("传感器选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_primary_ = new QComboBox(this);
    combo_secondary_ = new QComboBox(this);
    combo_primary_->addItem("请选择", QString());
    combo_secondary_->addItem("请选择", QString());
    connect(combo_primary_, &QComboBox::currentTextChanged,
            this, &TabLidarLidar::onPrimarySelected);
    connect(combo_secondary_, &QComboBox::currentTextChanged,
            this, &TabLidarLidar::onSecondarySelected);
    fl_s->addRow("主雷达：", combo_primary_);
    fl_s->addRow("辅雷达：", combo_secondary_);
    label_install_ = new QLabel("—", this);
    label_install_->setWordWrap(true);
    label_install_->setStyleSheet("QLabel{color:#8e8;}");
    fl_s->addRow("辅雷达安装：", label_install_);
    label_param_source_ = new QLabel("—", this);
    label_param_source_->setWordWrap(true);
    label_param_source_->setStyleSheet("QLabel{color:#fa8;}");
    fl_s->addRow("参数来源：", label_param_source_);
    vl->addWidget(grp_sensor);

    // ---- 安装参数微调 ----
    auto* grp_tune = new QGroupBox("辅雷达安装参数微调（写入节点 rosparam）", this);
    auto* fl_t = new QFormLayout(grp_tune);
    spin_x_ = new QDoubleSpinBox(this);
    spin_x_->setRange(-10.0, 10.0); spin_x_->setSingleStep(0.01);
    spin_x_->setDecimals(3); spin_x_->setSuffix(" m");
    connect(spin_x_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarLidar::onXChanged);
    fl_t->addRow("X 安装值：", spin_x_);
    spin_y_ = new QDoubleSpinBox(this);
    spin_y_->setRange(-10.0, 10.0); spin_y_->setSingleStep(0.01);
    spin_y_->setDecimals(3); spin_y_->setSuffix(" m");
    connect(spin_y_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarLidar::onYChanged);
    fl_t->addRow("Y 安装值：", spin_y_);
    spin_yaw_ = new QDoubleSpinBox(this);
    spin_yaw_->setRange(-180.0, 180.0); spin_yaw_->setSingleStep(0.1);
    spin_yaw_->setDecimals(3); spin_yaw_->setSuffix(" °");
    connect(spin_yaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarLidar::onYawChanged);
    fl_t->addRow("Yaw 安装值：", spin_yaw_);
    vl->addWidget(grp_tune);

    // ---- 阶段指示 ----
    auto* grp_phase = new QGroupBox("当前阶段", this);
    auto* fl_ph = new QFormLayout(grp_phase);
    label_phase_ = new QLabel("未开始", this);
    label_phase_->setWordWrap(true);
    label_phase_->setStyleSheet("QLabel{color:#4af;font-weight:bold;}");
    fl_ph->addRow("阶段：", label_phase_);
    label_data_status_ = new QLabel("未订阅", this);
    label_data_status_->setWordWrap(true);
    label_cali_status_ = new QLabel("—", this);
    label_cali_status_->setWordWrap(true);
    label_result_ = new QLabel("—", this);
    label_result_->setWordWrap(true);
    label_result_->setStyleSheet("QLabel{color:#ff0;}");
    fl_ph->addRow("数据：", label_data_status_);
    fl_ph->addRow("节点状态：", label_cali_status_);
    fl_ph->addRow("结果(Δ)：", label_result_);
    vl->addWidget(grp_phase);

    // ---- 操作按钮 ----
    auto* grp_btn = new QGroupBox("操作", this);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8); vl_b->setContentsMargins(8, 8, 8, 8);
    btn_confirm_primary_ = new QPushButton("① 确认主雷达（切换到辅雷达阶段）", this);
    btn_save_            = new QPushButton("② 保存标定结果到 YAML", this);
    btn_reset_           = new QPushButton("重置（清空全部数据）", this);
    btn_confirm_primary_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    connect(btn_confirm_primary_, &QPushButton::clicked, this, &TabLidarLidar::onConfirmPrimary);
    connect(btn_save_,            &QPushButton::clicked, this, &TabLidarLidar::onSaveCalib);
    connect(btn_reset_,           &QPushButton::clicked, this, &TabLidarLidar::onResetCalib);
    vl_b->addWidget(btn_confirm_primary_);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    // ---- 使用提示 ----
    auto* lbl_hint = new QLabel(
        "<small><b>前提：</b>主雷达完全标定，辅雷达 roll/pitch/z 已标定<br>"
        "<b>步骤：</b><br>"
        "1. 摆 3~5 个锥桶（两雷达共视范围内）<br>"
        "2. 启动节点，开启可视化<br>"
        "3. 开启选点模式，点击<b>主雷达</b>锥桶点（≥3个）<br>"
        "4. 点击[确认主雷达]切换到辅雷达阶段<br>"
        "5. 继续点击<b>辅雷达</b>锥桶点（≥3个）<br>"
        "6. 点击[保存标定结果]写入 YAML</small>", this);
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

    // ---- 右侧：点云视图 ----
    auto* right_box = new QGroupBox("选点可视化（红=主雷达点，绿=辅雷达点）", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new PointCloudViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);

    connect(viewer_, &PointCloudViewer::pointPicked,
            this, &TabLidarLidar::onPointPicked);
}

void TabLidarLidar::applyConfig()
{
    combo_primary_->blockSignals(true);
    combo_secondary_->blockSignals(true);
    combo_primary_->clear();
    combo_secondary_->clear();
    combo_primary_->addItem("请选择", QString());
    combo_secondary_->addItem("请选择", QString());
    const auto names = cfg_->lidarNames();
    for (const auto& n : names) {
        combo_primary_->addItem(QString::fromStdString(n));
        combo_secondary_->addItem(QString::fromStdString(n));
    }
    combo_primary_->blockSignals(false);
    combo_secondary_->blockSignals(false);

    node_ns_ = kNodeNs;
    prev_primary_.clear();
    prev_secondary_ll_.clear();
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_confirm_primary_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarLidar::deactivate()
{
    onStopNode();
    if (!prev_primary_.empty()) {
        ros_->killNode("driver_primary_" + prev_primary_);
        prev_primary_.clear();
    }
    if (!prev_secondary_ll_.empty()) {
        ros_->killNode("driver_secondary_" + prev_secondary_ll_);
        prev_secondary_ll_.clear();
    }
    combo_primary_->blockSignals(true);
    combo_secondary_->blockSignals(true);
    combo_primary_->setCurrentIndex(0);
    combo_secondary_->setCurrentIndex(0);
    combo_primary_->blockSignals(false);
    combo_secondary_->blockSignals(false);
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_confirm_primary_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarLidar::onPrimarySelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_primary_.empty()) {
            ros_->killNode("driver_primary_" + prev_primary_);
            prev_primary_.clear();
        }
        btn_confirm_primary_->setEnabled(false);
        return;
    }

    const std::string new_primary = name.toStdString();

    // 停止上一个主雷达 driver（如果切换到不同的雷达）
    if (!prev_primary_.empty() && prev_primary_ != new_primary) {
        ros_->killNode("driver_primary_" + prev_primary_);
    }

    prev_primary_ = new_primary;

    double x, y, z, roll, pitch, yaw;

    // 启动新主雷达 driver（如果配置了 launch_file）
    const std::string launch_path = cfg_->getLidarLaunchFilePath(new_primary);
    if (!launch_path.empty()) {
        const std::string packets_topic = cfg_->getLidarPacketsTopic(new_primary);
        const std::string point_cloud_topic = cfg_->getLidarPointCloudTopic(new_primary);
        QStringList driver_args;
        ConfigManager::LidarMergedParams params;
        if (!getMergedLidarParamsOrWarn(this, cfg_, new_primary, params)) return;
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        driver_args << QString("xoffset:=%1").arg(params.x)
                    << QString("yoffset:=%1").arg(params.y)
                    << QString("zoffset:=%1").arg(params.z)
                    << QString("roll:=%1").arg(params.roll_deg)
                    << QString("pitch:=%1").arg(params.pitch_deg)
                    << QString("yaw:=%1").arg(params.yaw_deg)
                    << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        QString cmd = QString("roslaunch %1").arg(QString::fromStdString(launch_path));
        for (const auto& a : driver_args) cmd += " " + a;
        ROS_INFO("[TabLidarLidar] primary driver launch cmd:\n%s", cmd.toStdString().c_str());
        ros_->launchNodeByPath(launch_path, "driver_primary_" + new_primary, driver_args);
    }

    btn_confirm_primary_->setEnabled(!combo_secondary_->currentText().isEmpty() && combo_secondary_->currentText() != "请选择");
    if (viz_enabled_) resubscribe();
}

void TabLidarLidar::onSecondarySelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_secondary_ll_.empty()) {
            ros_->killNode("driver_secondary_" + prev_secondary_ll_);
            prev_secondary_ll_.clear();
        }
        label_install_->setText("—");
        if (label_param_source_) label_param_source_->setText("—");
        btn_save_->setEnabled(false);
        btn_reset_->setEnabled(false);
        btn_confirm_primary_->setEnabled(!combo_primary_->currentText().isEmpty() && combo_primary_->currentText() != "请选择");
        return;
    }
    const std::string sec = name.toStdString();

    // 停止上一个辅雷达 driver（如果切换到不同的雷达）
    if (!prev_secondary_ll_.empty() && prev_secondary_ll_ != sec) {
        ros_->killNode("driver_secondary_" + prev_secondary_ll_);
    }

    prev_secondary_ll_ = sec;

    node_ns_ = kNodeNs;
    ConfigManager::LidarMergedParams params;
    if (getMergedLidarParamsOrWarn(this, cfg_, sec, params)) {
        label_install_->setText(
            QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
            .arg(params.x,0,'f',3).arg(params.y,0,'f',3).arg(params.z,0,'f',3)
            .arg(params.roll_deg,0,'f',2).arg(params.pitch_deg,0,'f',2).arg(params.yaw_deg,0,'f',2));
        spin_x_->blockSignals(true); spin_y_->blockSignals(true); spin_yaw_->blockSignals(true);
        spin_x_->setValue(params.x); spin_y_->setValue(params.y); spin_yaw_->setValue(params.yaw_deg);
        spin_x_->blockSignals(false); spin_y_->blockSignals(false); spin_yaw_->blockSignals(false);
    }

    // 启动新辅雷达 driver（如果配置了 launch_file）
    const std::string launch_path = cfg_->getLidarLaunchFilePath(sec);
    if (!launch_path.empty()) {
        const std::string packets_topic = cfg_->getLidarPacketsTopic(sec);
        const std::string point_cloud_topic = cfg_->getLidarPointCloudTopic(sec);
        QStringList driver_args;
        ConfigManager::LidarMergedParams params;
        if (!getMergedLidarParamsOrWarn(this, cfg_, sec, params)) return;
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        driver_args << QString("xoffset:=%1").arg(params.x)
                    << QString("yoffset:=%1").arg(params.y)
                    << QString("zoffset:=%1").arg(params.z)
                    << QString("roll:=%1").arg(params.roll_deg)
                    << QString("pitch:=%1").arg(params.pitch_deg)
                    << QString("yaw:=%1").arg(params.yaw_deg)
                    << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        QString cmd = QString("roslaunch %1").arg(QString::fromStdString(launch_path));
        for (const auto& a : driver_args) cmd += " " + a;
        ROS_INFO("[TabLidarLidar] secondary driver launch cmd:\n%s", cmd.toStdString().c_str());
        ros_->launchNodeByPath(launch_path, "driver_secondary_" + sec, driver_args);
    }

    btn_confirm_primary_->setEnabled(!combo_primary_->currentText().isEmpty() && combo_primary_->currentText() != "请选择");
    btn_save_->setEnabled(true);
    btn_reset_->setEnabled(true);

    if (viz_enabled_) resubscribe();
}

void TabLidarLidar::resubscribe()
{
    ros_->unsubscribeAll();
    cloud_primary_.reset();
    cloud_secondary_.reset();
    raw_primary_buf_.reset();
    raw_secondary_buf_.reset();
    viewer_->clear();
    label_data_status_->setText("订阅中，等待数据...");
    viewer_->showLoading(true);

    ros_->subscribeCloud(node_ns_ + "/primary_cones",
        [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) { onPrimaryCloudReceived(cloud); });
    ros_->subscribeCloud(node_ns_ + "/secondary_cones",
        [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) { onSecondaryCloudReceived(cloud); });
    ros_->subscribeStatus(node_ns_ + "/status",
        [this](const std::string& msg) { onStatusReceived(msg); });
    ros_->subscribeResult(node_ns_ + "/result",
        [this](double x, double y, double z) { onResultReceived(x, y, z); });

    const auto& cfg = cfg_->getConfig();
    std::string primary_topic, secondary_topic;
    for (const auto& l : cfg.lidars) {
        if (l.name == combo_primary_->currentText().toStdString())   primary_topic   = l.topic;
        if (l.name == combo_secondary_->currentText().toStdString()) secondary_topic = l.topic;
    }
    if (!primary_topic.empty()) {
        ros_->subscribeCloud(primary_topic,
            [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                if (!viz_enabled_ || !show_primary_cloud_) return;
                raw_primary_buf_ = cloud;
                refreshViewer();
            });
    }
    if (!secondary_topic.empty() && secondary_topic != primary_topic) {
        ros_->subscribeCloud(secondary_topic,
            [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                if (!viz_enabled_ || !show_secondary_cloud_) return;
                raw_secondary_buf_ = cloud;
                refreshViewer();
            });
    }
}

void TabLidarLidar::refreshViewer()
{
    viewer_->clear(false);
    if (show_primary_cloud_ && raw_primary_buf_)
        viewer_->addCloud(raw_primary_buf_, 0.45, 0.45, 0.45, 1.5);
    if (show_secondary_cloud_ && raw_secondary_buf_)
        viewer_->addCloud(raw_secondary_buf_, 0.55, 0.55, 0.35, 1.5);
    if (cloud_primary_ && !cloud_primary_->empty())
        viewer_->addCloud(cloud_primary_, 1.0, 0.3, 0.3, 8.0);
    if (cloud_secondary_ && !cloud_secondary_->empty())
        viewer_->addCloud(cloud_secondary_, 0.3, 1.0, 0.3, 8.0);
    viewer_->renderNow();
}

void TabLidarLidar::onPrimaryCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    viewer_->showLoading(false);
    cloud_primary_ = cloud;
    label_data_status_->setText(
        QString("主雷达点: %1  辅雷达点: %2")
        .arg(static_cast<int>(cloud_primary_ ? cloud_primary_->size() : 0))
        .arg(static_cast<int>(cloud_secondary_ ? cloud_secondary_->size() : 0)));
    refreshViewer();
}

void TabLidarLidar::onSecondaryCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    viewer_->showLoading(false);
    cloud_secondary_ = cloud;
    label_data_status_->setText(
        QString("主雷达点: %1  辅雷达点: %2")
        .arg(static_cast<int>(cloud_primary_ ? cloud_primary_->size() : 0))
        .arg(static_cast<int>(cloud_secondary_ ? cloud_secondary_->size() : 0)));
    refreshViewer();
}

void TabLidarLidar::onStatusReceived(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
    const QString s = QString::fromStdString(msg);
    if (s.contains("活跃=primary") || s.contains("活跃雷达=primary") || s.contains("阶段1") || s.contains("主雷达")) {
        label_phase_->setText("阶段 1/2 — 点击主雷达锥桶点");
        label_phase_->setStyleSheet("QLabel{color:#4af;font-weight:bold;}");
    } else if (s.contains("活跃=secondary") || s.contains("阶段2") || s.contains("辅雷达")) {
        label_phase_->setText("阶段 2/2 — 点击辅雷达锥桶点");
        label_phase_->setStyleSheet("QLabel{color:#fa4;font-weight:bold;}");
    } else if (s.contains("完成")) {
        label_phase_->setText("已完成");
        label_phase_->setStyleSheet("QLabel{color:#4f4;font-weight:bold;}");
    }
}

void TabLidarLidar::onResultReceived(double x, double y, double z)
{
    last_delta_x_   = x;
    last_delta_y_   = y;
    last_delta_yaw_ = z;
    label_result_->setText(
        QString("Δx=%1m  Δy=%2m  Δyaw=%3°")
        .arg(x,0,'f',4).arg(y,0,'f',4).arg(z,0,'f',4));
}

void TabLidarLidar::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        btn_pick_mode_->setEnabled(true);
        resubscribe();
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        if (btn_pick_mode_->isChecked()) btn_pick_mode_->setChecked(false);
        btn_pick_mode_->setEnabled(false);
        ros_->unsubscribeAll();
        cloud_primary_.reset(); cloud_secondary_.reset();
        raw_primary_buf_.reset(); raw_secondary_buf_.reset();
        viewer_->showLoading(false);
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabLidarLidar::onRawCloudToggle(int /*state*/)
{
    show_primary_cloud_   = chk_show_primary_->isChecked();
    show_secondary_cloud_ = chk_show_secondary_->isChecked();
    if (viz_enabled_) resubscribe();
}

void TabLidarLidar::onPickModeToggle(bool checked)
{
    viewer_->setPickMode(checked);
    if (checked) {
        btn_pick_mode_->setText("⏹ 停止选点");
        btn_pick_mode_->setStyleSheet(
            "QPushButton { background: #4a1a4a; color: #f4f; border: 1px solid #f4f; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
    } else {
        btn_pick_mode_->setText("🖱 开启选点模式");
        btn_pick_mode_->setStyleSheet("");
    }
}

void TabLidarLidar::onPointPicked(float x, float y, float z)
{
    const std::string frame_id = (current_phase_ == 1)
        ? combo_primary_->currentText().toStdString()
        : combo_secondary_->currentText().toStdString();
    ros_->publishPoint("/clicked_point", x, y, z, frame_id);
    label_data_status_->setText(
        QString("[%1] 已发点 x=%2 y=%3 z=%4")
        .arg(current_phase_ == 1 ? "主雷达" : "辅雷达")
        .arg(x,0,'f',3).arg(y,0,'f',3).arg(z,0,'f',3));
}

void TabLidarLidar::onConfirmPrimary()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/select_secondary", msg);
    label_cali_status_->setText(
        QString(ok ? "[确认成功] " : "[确认失败] ") + QString::fromStdString(msg));
    if (ok) {
        current_phase_ = 2;
        raw_primary_buf_.reset();
        label_phase_->setText("阶段 2/2 — 点击辅雷达锥桶点");
        label_phase_->setStyleSheet("QLabel{color:#fa4;font-weight:bold;}");
    }
}

void TabLidarLidar::onSaveCalib()
{
    std::string msg;
    bool ok = ros_->callSave(node_ns_ + "/save_calibration", msg);
    if (ok) {
        const std::string sec = combo_secondary_->currentText().toStdString();
        const QString proc_file = QString("%1/lidar_lidar_%2.yaml")
            .arg(QString::fromStdString(cfg_->processDir()))
            .arg(QString::fromStdString(sec));
        QString detail = QString("\n节点写入: %1").arg(proc_file);
        QMessageBox::information(this, "保存成功", QString::fromStdString(msg) + detail);
        label_phase_->setText("已完成");
        label_phase_->setStyleSheet("QLabel{color:#4f4;font-weight:bold;}");
    } else {
        QMessageBox::warning(this, "保存失败", QString::fromStdString(msg));
    }
}

void TabLidarLidar::onResetCalib()
{
    std::string msg;
    ros_->callReset(node_ns_ + "/reset_calibration", msg);
    current_phase_ = 1;
    label_phase_->setText("阶段 1/2 — 点击主雷达锥桶点");
    label_phase_->setStyleSheet("QLabel{color:#4af;font-weight:bold;}");
    label_cali_status_->setText("已重置");
    label_result_->setText("—");
    cloud_primary_.reset();
    cloud_secondary_.reset();
    viewer_->clear();
}

void TabLidarLidar::onXChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/secondary_lidar/x_install", v);
}

void TabLidarLidar::onYChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/secondary_lidar/y_install", v);
}

void TabLidarLidar::onYawChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/secondary_lidar/yaw_install_deg", v);
}

void TabLidarLidar::onStartNode()
{
    const std::string pri = combo_primary_->currentText().toStdString();
    const std::string sec = combo_secondary_->currentText().toStdString();
    ROS_INFO("[TabLidarLidar] launch clicked node: primary=%s secondary=%s sensor_name=%s",
             pri.c_str(), sec.c_str(), sec.c_str());
    ConfigManager::LidarMergedParams primary_params;
    if (!requirePrimaryFullyCalibrated(this, cfg_, pri, primary_params)) return;
    ConfigManager::LidarMergedParams secondary_params;
    if (!getMergedLidarParamsOrWarn(this, cfg_, sec, secondary_params)) return;
    QStringList args;
    const std::string primary_topic = cfg_->getLidarPointCloudTopic(pri);
    const std::string secondary_topic = cfg_->getLidarPointCloudTopic(sec);
    if (!primary_topic.empty()) args << QString("primary_lidar_topic:=%1").arg(QString::fromStdString(primary_topic));
    if (!secondary_topic.empty()) args << QString("secondary_lidar_topic:=%1").arg(QString::fromStdString(secondary_topic));

    args << QString("inst_x:=%1").arg(secondary_params.x)
         << QString("inst_y:=%1").arg(secondary_params.y)
         << QString("inst_z:=%1").arg(secondary_params.z)
         << QString("inst_yaw_deg:=%1").arg(secondary_params.yaw_deg)
         << QString("inst_z_offset:=%1").arg(cfg_->zOffset())
         << QString("sensor_name:=%1").arg(QString::fromStdString(sec));
    const std::string proc_dir = cfg_->processDir();
    if (!proc_dir.empty()) {
        QDir().mkpath(QString::fromStdString(proc_dir));
        args << QString("calibration_output_file:=%1/lidar_lidar_%2.yaml")
                .arg(QString::fromStdString(proc_dir))
                .arg(QString::fromStdString(sec));
    }
    bool ok = ros_->launchNode("lidar_lidar_cali",
                               "lidar_lidar_cali_clicked.launch",
                               "lidar_lidar_cali_clicked", args);
    if (!ok) {
        label_node_status_->setText("● 启动失败");
        label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    }
}

void TabLidarLidar::onStopNode()
{
    ros_->killNode("lidar_lidar_cali_clicked");
}

void TabLidarLidar::onNodeStarted(const QString& key)
{
    if (key != "lidar_lidar_cali_clicked") return;
    label_node_status_->setText("● 运行中");
    label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(true);
}

void TabLidarLidar::onNodeStopped(const QString& key)
{
    if (key != "lidar_lidar_cali_clicked") return;
    label_node_status_->setText("● 未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_start_node_->setEnabled(true);
    btn_stop_node_->setEnabled(false);
}

} // namespace calibration_gui
