#include "calibration_gui/tab_lidar_yaw.h"
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

static const std::string kNodeNs = "/lidar_yaw_cali_clicked";

TabLidarYaw::TabLidarYaw(std::shared_ptr<ConfigManager> cfg,
                          std::shared_ptr<RosWorker> ros,
                          QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
    connect(ros_.get(), &RosWorker::nodeStarted, this, &TabLidarYaw::onNodeStarted);
    connect(ros_.get(), &RosWorker::nodeStopped, this, &TabLidarYaw::onNodeStopped);
}

void TabLidarYaw::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    auto* left = new QWidget(this);
    left->setFixedWidth(330);
    auto* vl = new QVBoxLayout(left);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(10);

    // ---- 节点管理（最顶部）----
    auto* grp_node = new QGroupBox("节点管理", this);
    auto* hl_node = new QHBoxLayout(grp_node);
    hl_node->setContentsMargins(8, 8, 8, 8);
    hl_node->setSpacing(8);
    btn_start_node_ = new QPushButton("▶ 启动节点", this);
    btn_stop_node_  = new QPushButton("⏹ 停止节点", this);
    btn_stop_node_->setEnabled(false);
    connect(btn_start_node_, &QPushButton::clicked, this, &TabLidarYaw::onStartNode);
    connect(btn_stop_node_,  &QPushButton::clicked, this, &TabLidarYaw::onStopNode);
    label_node_status_ = new QLabel("● 未运行", this);
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    hl_node->addWidget(btn_start_node_);
    hl_node->addWidget(btn_stop_node_);
    hl_node->addWidget(label_node_status_);
    vl->addWidget(grp_node);

    // ---- 可视化控制（最顶部）----
    auto* grp_viz = new QGroupBox("可视化控制", this);
    auto* vl_viz = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    vl_viz->setSpacing(6);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", this);
    btn_viz_toggle_->setStyleSheet("");
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabLidarYaw::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    chk_raw_cloud_ = new QCheckBox("叠加显示原始点云", this);
    chk_raw_cloud_->setChecked(false);
    connect(chk_raw_cloud_, &QCheckBox::stateChanged, this, &TabLidarYaw::onRawCloudToggle);
    vl_viz->addWidget(chk_raw_cloud_);
    btn_pick_mode_ = new QPushButton("🖱 开启选点模式", this);
    btn_pick_mode_->setCheckable(true);
    btn_pick_mode_->setEnabled(false);
    connect(btn_pick_mode_, &QPushButton::toggled, this, &TabLidarYaw::onPickModeToggle);
    vl_viz->addWidget(btn_pick_mode_);
    vl->addWidget(grp_viz);

    // 传感器选择
    auto* grp_sensor = new QGroupBox("传感器选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_lidar_ = new QComboBox(this);
    combo_lidar_->addItem("请选择", QString());
    connect(combo_lidar_, &QComboBox::currentTextChanged,
            this, &TabLidarYaw::onLidarSelected);
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

    // Yaw 微调
    auto* grp_tune = new QGroupBox("Yaw 安装参数微调", this);
    auto* fl_t = new QFormLayout(grp_tune);
    spin_yaw_ = new QDoubleSpinBox(this);
    spin_yaw_->setRange(-180.0, 180.0);
    spin_yaw_->setSingleStep(0.1);
    spin_yaw_->setDecimals(3);
    spin_yaw_->setSuffix(" °");
    connect(spin_yaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarYaw::onYawChanged);
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
    fl_st->addRow("标定：", label_cali_status_);
    fl_st->addRow("结果：", label_result_);
    vl->addWidget(grp_status);

    // 操作按钮
    auto* grp_btn = new QGroupBox("操作", this);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8);
    vl_b->setContentsMargins(8, 8, 8, 8);
    btn_start_ = new QPushButton("触发标定（计算当前均值）", this);
    btn_save_  = new QPushButton("保存标定结果到 YAML", this);
    btn_reset_ = new QPushButton("重置数据窗口", this);
    btn_start_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    connect(btn_start_, &QPushButton::clicked, this, &TabLidarYaw::onStartCalib);
    connect(btn_save_,  &QPushButton::clicked, this, &TabLidarYaw::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabLidarYaw::onResetCalib);
    vl_b->addWidget(btn_start_);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    // 使用提示
    auto* lbl_hint = new QLabel(
        "<small><b>前提：</b>roll/pitch/z 已标定完毕<br>"
        "<b>步骤：</b><br>"
        "1. 启动 lidar_yaw_cali 节点<br>"
        "2. 车辆正前方沿 X 轴摆 4~5 个锥桶<br>"
        "3. 等待状态 OK（黄色球出现）<br>"
        "4. 触发标定 → 查看 yaw_delta<br>"
        "5. 保存结果</small>", this);
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
    auto* right_box = new QGroupBox("锥桶点云（灰色=原始，黄球=锥桶中心）", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new PointCloudViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);

    connect(viewer_, &PointCloudViewer::pointPicked,
            this, &TabLidarYaw::onPointPicked);
}

void TabLidarYaw::applyConfig()
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
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_start_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarYaw::deactivate()
{
    onStopNode();
    if (!prev_lidar_.empty()) {
        ros_->killNode("driver_" + prev_lidar_);
        prev_lidar_.clear();
    }
    combo_lidar_->blockSignals(true);
    combo_lidar_->setCurrentIndex(0);
    combo_lidar_->blockSignals(false);
    current_lidar_.clear();
    raw_cloud_topic_.clear();
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_start_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarYaw::onLidarSelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_lidar_.empty()) {
            ros_->killNode("driver_" + prev_lidar_);
            prev_lidar_.clear();
        }
        current_lidar_.clear();
        raw_cloud_topic_.clear();
        label_install_->setText("—");
        if (label_param_source_) label_param_source_->setText("—");
        label_data_status_->setText("未订阅");
        label_cali_status_->setText("—");
        label_result_->setText("—");
        btn_start_->setEnabled(false);
        btn_save_->setEnabled(false);
        btn_reset_->setEnabled(false);
        return;
    }
    current_lidar_ = name.toStdString();
    node_ns_ = kNodeNs;

    ConfigManager::LidarMergedParams params;
    if (getMergedLidarParamsOrWarn(this, cfg_, current_lidar_, params)) {
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
        if (getMergedLidarParamsOrWarn(this, cfg_, current_lidar_, params)) {
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
    label_cali_status_->setText("—");
    label_result_->setText("—");

    btn_start_->setEnabled(true);
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

void TabLidarYaw::onRawCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    raw_cloud_buf_ = cloud;
    viewer_->showLoading(false);
    if (clicked_cloud_buf_ && !clicked_cloud_buf_->empty()) {
        viewer_->showRawAndTarget(cloud, clicked_cloud_buf_, 1.0, 0.9, 0.0);
        label_data_status_->setText(
            QString("原始 %1pt | 已选锥桶 %2 个")
            .arg(static_cast<int>(cloud->size()))
            .arg(static_cast<int>(clicked_cloud_buf_->size())));
    } else {
        label_data_status_->setText(
            QString("原始点云 %1 点").arg(static_cast<int>(cloud->size())));
        viewer_->showCloud(cloud, "raw", 0.2, 0.8, 1.0);
    }
}

void TabLidarYaw::onCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    clicked_cloud_buf_ = cloud;
    viewer_->showLoading(false);
    label_data_status_->setText(
        QString("已选锥桶 %1 个").arg(static_cast<int>(cloud->size())));
    // cone_cloud 是节点检测到的锥桶点，叠加在原始点云上，用大亮黄点显示
    if (raw_cloud_buf_)
        viewer_->showRawAndTarget(raw_cloud_buf_, cloud, 1.0, 0.9, 0.0);
    else
        viewer_->showCloud(cloud, "cones", 1.0, 0.9, 0.0);
}

void TabLidarYaw::onStatusReceived(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
}

void TabLidarYaw::onResultReceived(double x, double y, double z)
{
    // x=yaw_delta_deg, y=n_cones, z=confidence
    last_yaw_delta_ = x;
    label_result_->setText(
        QString("Yaw delta=%1°  锥桶=%2个  conf=%3")
        .arg(x, 0, 'f', 3).arg(static_cast<int>(y)).arg(z, 0, 'f', 2));
}

void TabLidarYaw::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        btn_pick_mode_->setEnabled(true);
        if (!current_lidar_.empty()) {
            onLidarSelected(QString::fromStdString(current_lidar_));
        }
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        if (btn_pick_mode_->isChecked())
            btn_pick_mode_->setChecked(false);
        btn_pick_mode_->setEnabled(false);
        ros_->unsubscribeAll();
        raw_cloud_buf_.reset();
        viewer_->showLoading(false);
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabLidarYaw::onRawCloudToggle(int state)
{
    show_raw_cloud_ = (state == Qt::Checked);
    if (viz_enabled_ && !current_lidar_.empty()) {
        onLidarSelected(QString::fromStdString(current_lidar_));
    }
}

void TabLidarYaw::onPickModeToggle(bool checked)
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

void TabLidarYaw::onPointPicked(float x, float y, float z)
{
    ros_->publishPoint("/clicked_point", x, y, z, current_lidar_);
    label_data_status_->setText(
        QString("已发点 x=%1 y=%2 z=%3")
        .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
}

void TabLidarYaw::onStartCalib()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/trigger_calibration", msg);
    label_cali_status_->setText(
        QString(ok ? "[触发成功] " : "[触发失败] ") + QString::fromStdString(msg));
}

void TabLidarYaw::onSaveCalib()
{
    std::string msg;
    bool ok = ros_->callSave(node_ns_ + "/save_calibration", msg);
    if (ok) {
        ConfigManager::LidarYawProcessParams yaw_params;
        if (cfg_->getLidarYawProcessParams(current_lidar_, yaw_params) && yaw_params.has_yaw) {
            ConfigManager::LidarMergedParams merged;
            if (cfg_->getMergedLidarParams(current_lidar_, merged)) {
                label_install_->setText(
                    QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
                    .arg(merged.x, 0, 'f', 3).arg(merged.y, 0, 'f', 3).arg(merged.z, 0, 'f', 3)
                    .arg(merged.roll_deg, 0, 'f', 2).arg(merged.pitch_deg, 0, 'f', 2).arg(yaw_params.yaw_deg, 0, 'f', 2));
                if (label_param_source_) {
                    merged.yaw_from_process = true;
                    label_param_source_->setText(cfg_->getMergedLidarParamSourceText(merged));
                }
            }
        }
        const QString proc_file = QString("%1/lidar_yaw_%2.yaml")
            .arg(QString::fromStdString(cfg_->processDir()))
            .arg(QString::fromStdString(current_lidar_));
        QString detail = QString("\n节点写入: %1").arg(proc_file);
        QMessageBox::information(this, "保存成功", QString::fromStdString(msg) + detail);
    } else {
        QMessageBox::warning(this, "保存失败", QString::fromStdString(msg));
    }
}

void TabLidarYaw::onResetCalib()
{
    std::string msg;
    ros_->callReset(node_ns_ + "/reset_calibration", msg);
    label_cali_status_->setText("已重置");
    label_result_->setText("—");
    clicked_cloud_buf_.reset();
    viewer_->clear();
}

void TabLidarYaw::onYawChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/installation/yaw_deg", v);
}

void TabLidarYaw::onStartNode()
{
    QStringList args;
    const auto& cfg = cfg_->getConfig();
    for (const auto& l : cfg.lidars) {
        if (l.name == current_lidar_) {
            args << QString("input_topic:=%1").arg(QString::fromStdString(l.topic));
            args << QString("lidar_frame:=%1").arg(QString::fromStdString(l.name));
            args << QString("sensor_name:=%1").arg(QString::fromStdString(l.name));
            break;
        }
    }
    ConfigManager::LidarMergedParams params;
    if (getMergedLidarParamsOrWarn(this, cfg_, current_lidar_, params)) {
        args << QString("inst_x:=%1").arg(params.x)
             << QString("inst_y:=%1").arg(params.y)
             << QString("inst_z:=%1").arg(params.z)
             << QString("inst_roll_deg:=%1").arg(params.roll_deg)
             << QString("inst_pitch_deg:=%1").arg(params.pitch_deg)
             << QString("inst_yaw_deg:=%1").arg(params.yaw_deg)
             << QString("inst_z_offset:=%1").arg(cfg_->zOffset());
    } else {
        return;
    }

    const std::string proc_dir = cfg_->processDir();
    if (!proc_dir.empty()) {
        QDir().mkpath(QString::fromStdString(proc_dir));
        args << QString("calibration_output_file:=%1/lidar_yaw_%2.yaml")
                .arg(QString::fromStdString(proc_dir))
                .arg(QString::fromStdString(current_lidar_));
    }

    bool ok = ros_->launchNode("lidar_yaw1", "lidar_yaw_cali_clicked.launch", "lidar_yaw_cali_clicked", args);
    if (!ok) {
        label_node_status_->setText("● 启动失败");
        label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    }
}

void TabLidarYaw::onStopNode()
{
    ros_->killNode("lidar_yaw_cali_clicked");
}

void TabLidarYaw::onNodeStarted(const QString& key)
{
    if (key != "lidar_yaw_cali_clicked") return;
    label_node_status_->setText("● 运行中");
    label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(true);
}

void TabLidarYaw::onNodeStopped(const QString& key)
{
    if (key != "lidar_yaw_cali_clicked") return;
    label_node_status_->setText("● 未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_start_node_->setEnabled(true);
    btn_stop_node_->setEnabled(false);
}

} // namespace calibration_gui
