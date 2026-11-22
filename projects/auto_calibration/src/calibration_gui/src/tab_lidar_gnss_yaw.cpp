#include "calibration_gui/tab_lidar_gnss_yaw.h"
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
#include <fstream>

namespace calibration_gui {

// lidar_gnss_yaw.launch: name="lidar_gnss_yaw"
static const std::string kNodeNs = "/lidar_gnss_yaw";

TabLidarGnssYaw::TabLidarGnssYaw(std::shared_ptr<ConfigManager> cfg,
                                  std::shared_ptr<RosWorker> ros,
                                  QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
}

void TabLidarGnssYaw::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    auto* left = new QWidget(this);
    left->setFixedWidth(330);
    auto* vl = new QVBoxLayout(left);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(10);

    // ---- 可视化控制（最顶部）----
    auto* grp_viz = new QGroupBox("可视化控制", this);
    auto* vl_viz = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    vl_viz->setSpacing(6);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", this);
    btn_viz_toggle_->setStyleSheet("");
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabLidarGnssYaw::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    chk_raw_cloud_ = new QCheckBox("叠加显示原始点云", this);
    chk_raw_cloud_->setChecked(false);
    connect(chk_raw_cloud_, &QCheckBox::stateChanged, this, &TabLidarGnssYaw::onRawCloudToggle);
    vl_viz->addWidget(chk_raw_cloud_);
    vl->addWidget(grp_viz);

    // 传感器选择
    auto* grp_sensor = new QGroupBox("传感器选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_lidar_ = new QComboBox(this);
    combo_lidar_->addItem("请选择", QString());
    connect(combo_lidar_, &QComboBox::currentTextChanged,
            this, &TabLidarGnssYaw::onLidarSelected);
    fl_s->addRow("激光雷达：", combo_lidar_);
    label_install_ = new QLabel("—", this);
    label_install_->setWordWrap(true);
    label_install_->setStyleSheet("QLabel{color:#8e8;}");
    fl_s->addRow("安装参数：", label_install_);
    label_param_source_ = new QLabel("—", this);
    label_param_source_->setWordWrap(true);
    label_param_source_->setStyleSheet("QLabel{color:#fa8;}");
    fl_s->addRow("参数来源：", label_param_source_);
    vl->addWidget(grp_sensor);

    // Yaw 初始值
    auto* grp_tune = new QGroupBox("Yaw 初始安装值", this);
    auto* fl_t = new QFormLayout(grp_tune);
    spin_yaw_ = new QDoubleSpinBox(this);
    spin_yaw_->setRange(-180.0, 180.0);
    spin_yaw_->setSingleStep(0.1);
    spin_yaw_->setDecimals(3);
    spin_yaw_->setSuffix(" °");
    connect(spin_yaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarGnssYaw::onYawChanged);
    fl_t->addRow("Yaw 安装值：", spin_yaw_);
    vl->addWidget(grp_tune);

    // 状态
    auto* grp_status = new QGroupBox("实时状态", this);
    auto* fl_st = new QFormLayout(grp_status);
    label_data_status_ = new QLabel("未订阅", this);
    label_data_status_->setWordWrap(true);
    label_cali_status_ = new QLabel("—", this);
    label_cali_status_->setWordWrap(true);
    label_result_ = new QLabel("—", this);
    label_result_->setWordWrap(true);
    label_result_->setStyleSheet("QLabel{color:#ff0;}");
    fl_st->addRow("数据：", label_data_status_);
    fl_st->addRow("采集状态：", label_cali_status_);
    fl_st->addRow("结果：", label_result_);
    vl->addWidget(grp_status);

    // 采集控制按钮
    auto* grp_btn = new QGroupBox("操作", this);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8);
    vl_b->setContentsMargins(8, 8, 8, 8);
    btn_start_ = new QPushButton("开始采集（驾车行驶）", this);
    btn_stop_  = new QPushButton("停止采集并计算 Yaw", this);
    btn_save_  = new QPushButton("保存标定结果到 YAML", this);
    btn_reset_ = new QPushButton("重置所有数据", this);
    btn_start_->setEnabled(false);
    btn_stop_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    btn_start_->setStyleSheet("QPushButton{background:#1a4a1a;}");
    btn_stop_->setStyleSheet("QPushButton{background:#4a1a1a;}");
    connect(btn_start_, &QPushButton::clicked, this, &TabLidarGnssYaw::onStartCollect);
    connect(btn_stop_,  &QPushButton::clicked, this, &TabLidarGnssYaw::onStopAndCalib);
    connect(btn_save_,  &QPushButton::clicked, this, &TabLidarGnssYaw::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabLidarGnssYaw::onResetCalib);
    vl_b->addWidget(btn_start_);
    vl_b->addWidget(btn_stop_);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    // 使用提示
    auto* lbl_hint = new QLabel(
        "<small><b>前提：</b>roll/pitch/z 已标定，GNSS 已初始化<br>"
        "<b>步骤：</b><br>"
        "1. 摆好锥桶（4~8个，间距1~2m，成一直线）<br>"
        "2. 启动 lidar_gnss_yaw 节点<br>"
        "3. 点[开始采集]→ 缓慢驾车观测锥桶<br>"
        "4. 至少绕行 2~3 次后点[停止并计算]<br>"
        "5. 查看结果满意后点[保存]</small>", this);
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

    // 右侧：锥桶点云
    auto* right_box = new QGroupBox("锥桶点云（采集帧叠加）", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new PointCloudViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);
}

void TabLidarGnssYaw::applyConfig()
{
    combo_lidar_->blockSignals(true);
    combo_lidar_->clear();
    combo_lidar_->addItem("请选择", QString());
    const auto names = cfg_->lidarNames();
    for (const auto& n : names)
        combo_lidar_->addItem(QString::fromStdString(n));
    combo_lidar_->blockSignals(false);

    current_lidar_.clear();
    node_ns_ = kNodeNs;
    collecting_ = false;
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_start_->setEnabled(false);
    btn_stop_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarGnssYaw::deactivate()
{
    if (!prev_lidar_.empty()) {
        ros_->killNode("driver_" + prev_lidar_);
        prev_lidar_.clear();
    }
    combo_lidar_->blockSignals(true);
    combo_lidar_->setCurrentIndex(0);
    combo_lidar_->blockSignals(false);
    current_lidar_.clear();
    collecting_ = false;
    raw_cloud_topic_.clear();
    ros_->unsubscribeAll();
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_start_->setEnabled(false);
    btn_stop_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarGnssYaw::onLidarSelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_lidar_.empty()) {
            ros_->killNode("driver_" + prev_lidar_);
            prev_lidar_.clear();
        }
        current_lidar_.clear();
        collecting_ = false;
        raw_cloud_topic_.clear();
        label_install_->setText("—");
        if (label_param_source_) label_param_source_->setText("—");
        label_data_status_->setText("未订阅");
        label_cali_status_->setText("空闲");
        label_result_->setText("—");
        btn_start_->setEnabled(false);
        btn_stop_->setEnabled(false);
        btn_save_->setEnabled(false);
        btn_reset_->setEnabled(false);
        return;
    }
    current_lidar_ = name.toStdString();
    node_ns_ = kNodeNs;
    collecting_ = false;

    ConfigManager::LidarMergedParams params;
    if (cfg_->getMergedLidarParams(current_lidar_, params)) {
        label_install_->setText(
            QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
            .arg(params.x, 0, 'f', 3).arg(params.y, 0, 'f', 3).arg(params.z, 0, 'f', 3)
            .arg(params.roll_deg, 0, 'f', 2).arg(params.pitch_deg, 0, 'f', 2).arg(params.yaw_deg, 0, 'f', 2));
        if (label_param_source_) label_param_source_->setText(cfg_->getMergedLidarParamSourceText(params));
        spin_yaw_->blockSignals(true);
        spin_yaw_->setValue(params.yaw_deg);
        spin_yaw_->blockSignals(false);
    }

    const auto& cfg = cfg_->getConfig();
    raw_cloud_topic_.clear();
    for (const auto& l : cfg.lidars) {
        if (l.name == current_lidar_) {
            raw_cloud_topic_ = l.topic;
            break;
        }
    }

    if (!prev_lidar_.empty() && prev_lidar_ != current_lidar_) {
        ros_->killNode("driver_" + prev_lidar_);
    }
    prev_lidar_ = current_lidar_;

    const std::string launch_path = cfg_->getLidarLaunchFilePath(current_lidar_);
    if (!launch_path.empty()) {
        const std::string packets_topic = cfg_->getLidarPacketsTopic(current_lidar_);
        const std::string point_cloud_topic = cfg_->getLidarPointCloudTopic(current_lidar_);
        QStringList driver_args;
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        if (cfg_->getMergedLidarParams(current_lidar_, params)) {
            driver_args << QString("xoffset:=%1").arg(params.x)
                        << QString("yoffset:=%1").arg(params.y)
                        << QString("zoffset:=%1").arg(params.z)
                        << QString("roll:=%1").arg(params.roll_deg)
                        << QString("pitch:=%1").arg(params.pitch_deg)
                        << QString("yaw:=%1").arg(params.yaw_deg)
                        << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        }
        ros_->launchNodeByPath(launch_path, "driver_" + current_lidar_, driver_args);
    }

    ros_->unsubscribeAll();
    raw_cloud_buf_.reset();
    viewer_->clear();
    label_data_status_->setText("未订阅");
    label_cali_status_->setText("空闲");
    label_result_->setText("—");

    btn_start_->setEnabled(true);
    btn_stop_->setEnabled(false);
    btn_save_->setEnabled(true);
    btn_reset_->setEnabled(true);

    if (viz_enabled_) {
        label_data_status_->setText("订阅中，等待数据...");
        viewer_->showLoading(true);
        if (!raw_cloud_topic_.empty()) {
            ros_->subscribeCloud(raw_cloud_topic_,
                [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                    onRawCloudReceived(cloud);
                });
        }
        ros_->subscribeCloud(node_ns_ + "/cone_cloud",
            [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                onCloudReceived(cloud);
            });
        ros_->subscribeStatus(node_ns_ + "/status",
            [this](const std::string& msg) { onStatusReceived(msg); });
        ros_->subscribeResult(node_ns_ + "/result",
            [this](double x, double y, double z) { onResultReceived(x, y, z); });
    }
}

void TabLidarGnssYaw::onRawCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    raw_cloud_buf_ = cloud;
    viewer_->showLoading(false);
    label_data_status_->setText(
        QString("原始点云 %1 点").arg(static_cast<int>(cloud->size())));
    if (show_raw_cloud_)
        viewer_->showCloud(cloud, "raw", 0.6, 0.6, 0.6);
    else
        viewer_->showCloud(cloud, "raw", 0.2, 0.8, 1.0);
}

void TabLidarGnssYaw::onCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    viewer_->showLoading(false);
    label_data_status_->setText(
        QString("锥桶点云 %1 点").arg(static_cast<int>(cloud->size())));
    viewer_->showCloud(cloud, "cones", 0.3, 0.8, 1.0);
}

void TabLidarGnssYaw::onStatusReceived(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
}

void TabLidarGnssYaw::onResultReceived(double x, double y, double z)
{
    // x=yaw_calib_deg, y=delta_yaw_deg, z=rmse_m
    last_yaw_calib_ = x;
    label_result_->setText(
        QString("Yaw标定值=%1°  delta=%2°  RMSE=%3m")
        .arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4));
}

void TabLidarGnssYaw::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        if (!current_lidar_.empty()) {
            onLidarSelected(QString::fromStdString(current_lidar_));
        }
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        raw_cloud_buf_.reset();
        viewer_->showLoading(false);
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabLidarGnssYaw::onRawCloudToggle(int state)
{
    show_raw_cloud_ = (state == Qt::Checked);
    if (viz_enabled_ && !current_lidar_.empty()) {
        onLidarSelected(QString::fromStdString(current_lidar_));
    }
}

void TabLidarGnssYaw::onStartCollect()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/start", msg);
    if (ok) {
        collecting_ = true;
        btn_start_->setEnabled(false);
        btn_stop_->setEnabled(true);
        label_cali_status_->setText("[采集中] " + QString::fromStdString(msg));
    } else {
        label_cali_status_->setText("[启动失败] " + QString::fromStdString(msg));
    }
}

void TabLidarGnssYaw::onStopAndCalib()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/stop", msg);
    collecting_ = false;
    btn_start_->setEnabled(true);
    btn_stop_->setEnabled(false);
    label_cali_status_->setText(
        QString(ok ? "[计算完成] " : "[停止失败] ") + QString::fromStdString(msg));
}

void TabLidarGnssYaw::onSaveCalib()
{
    std::string msg;
    bool ok = ros_->callSave(node_ns_ + "/save", msg);
    if (ok) {
        // 写到 process 目录
        const std::string proc = cfg_->processDir();
        QString proc_file;
        if (!proc.empty()) {
            QDir().mkpath(QString::fromStdString(proc));
            proc_file = QString("%1/lidar_gnss_yaw_%2.yaml")
                .arg(QString::fromStdString(proc))
                .arg(QString::fromStdString(current_lidar_));
            std::ofstream f(proc_file.toStdString());
            if (f.is_open()) {
                f << "lidar_name: "  << current_lidar_  << "\n"
                  << "yaw_calib_deg: " << last_yaw_calib_ << "\n";
            }
        }
        QString detail = proc_file.isEmpty()
            ? QString("\nYaw=%1°").arg(last_yaw_calib_,0,'f',4)
            : QString("\n已写入: %1\nYaw=%2°").arg(proc_file).arg(last_yaw_calib_,0,'f',4);
        QMessageBox::information(this, "保存成功", QString::fromStdString(msg) + detail);
    } else {
        QMessageBox::warning(this, "保存失败", QString::fromStdString(msg));
    }
}

void TabLidarGnssYaw::onResetCalib()
{
    if (collecting_) {
        std::string msg;
        ros_->callTrigger(node_ns_ + "/stop", msg);
        collecting_ = false;
    }
    std::string msg;
    ros_->callReset(node_ns_ + "/reset", msg);
    btn_start_->setEnabled(true);
    btn_stop_->setEnabled(false);
    label_cali_status_->setText("已重置");
    label_result_->setText("—");
    viewer_->clear();
}

void TabLidarGnssYaw::onYawChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/installation/yaw_deg", v);
}

} // namespace calibration_gui
