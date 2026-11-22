#include "calibration_gui/tab_lidar_rp.h"
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

// 固定 ROS 节点命名空间（与 lidar_rp_cali_clicked.launch 中 name= 一致）
static const std::string kNodeNs = "/lidar_rp_cali_clicked";

TabLidarRp::TabLidarRp(std::shared_ptr<ConfigManager> cfg,
                        std::shared_ptr<RosWorker> ros,
                        QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
    connect(ros_.get(), &RosWorker::nodeStarted, this, &TabLidarRp::onNodeStarted);
    connect(ros_.get(), &RosWorker::nodeStopped, this, &TabLidarRp::onNodeStopped);
}

void TabLidarRp::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    // ---- 左侧控制面板 ----
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
    connect(btn_start_node_, &QPushButton::clicked, this, &TabLidarRp::onStartNode);
    connect(btn_stop_node_,  &QPushButton::clicked, this, &TabLidarRp::onStopNode);
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
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabLidarRp::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    chk_raw_cloud_ = new QCheckBox("叠加显示原始点云", this);
    chk_raw_cloud_->setChecked(false);
    connect(chk_raw_cloud_, &QCheckBox::stateChanged, this, &TabLidarRp::onRawCloudToggle);
    vl_viz->addWidget(chk_raw_cloud_);
    btn_pick_mode_ = new QPushButton("🖱 开启选点模式", this);
    btn_pick_mode_->setCheckable(true);
    btn_pick_mode_->setEnabled(false);
    connect(btn_pick_mode_, &QPushButton::toggled, this, &TabLidarRp::onPickModeToggle);
    vl_viz->addWidget(btn_pick_mode_);
    vl->addWidget(grp_viz);

    // 传感器选择
    auto* grp_sensor = new QGroupBox("传感器选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_lidar_ = new QComboBox(this);
    combo_lidar_->addItem("请选择", QString());
    connect(combo_lidar_, &QComboBox::currentTextChanged,
            this, &TabLidarRp::onLidarSelected);
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

    // 安装参数微调（写回 rosparam，供节点实时读取）
    auto* grp_tune = new QGroupBox("安装参数微调（写入节点 rosparam）", this);
    auto* fl_t = new QFormLayout(grp_tune);
    spin_roll_ = new QDoubleSpinBox(this);
    spin_roll_->setRange(-30.0, 30.0);
    spin_roll_->setSingleStep(0.1);
    spin_roll_->setDecimals(3);
    spin_roll_->setSuffix(" °");
    connect(spin_roll_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarRp::onRollChanged);
    fl_t->addRow("Roll 安装值：", spin_roll_);

    spin_pitch_ = new QDoubleSpinBox(this);
    spin_pitch_->setRange(-30.0, 30.0);
    spin_pitch_->setSingleStep(0.1);
    spin_pitch_->setDecimals(3);
    spin_pitch_->setSuffix(" °");
    connect(spin_pitch_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabLidarRp::onPitchChanged);
    fl_t->addRow("Pitch 安装值：", spin_pitch_);
    vl->addWidget(grp_tune);

    // 状态显示
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
    connect(btn_start_, &QPushButton::clicked, this, &TabLidarRp::onStartCalib);
    connect(btn_save_,  &QPushButton::clicked, this, &TabLidarRp::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabLidarRp::onResetCalib);
    vl_b->addWidget(btn_start_);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    // 使用提示
    auto* lbl_hint = new QLabel(
        "<small><b>使用步骤：</b><br>"
        "1. 启动 lidar_rp_cali 节点<br>"
        "2. 驶入平坦地面停车<br>"
        "3. 等待状态显示 OK（≥8帧）<br>"
        "4. 点击[触发标定]查看结果<br>"
        "5. 点击[保存]写入 YAML</small>", this);
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

    // ---- 右侧：地面点云视图 ----
    auto* right_box = new QGroupBox("地面点云（绿色）", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new PointCloudViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);

    connect(viewer_, &PointCloudViewer::pointPicked,
            this, &TabLidarRp::onPointPicked);
}

void TabLidarRp::applyConfig()
{
    combo_lidar_->blockSignals(true);
    combo_lidar_->clear();
    combo_lidar_->addItem("请选择", QString());
    const auto names = cfg_->lidarNames();
    for (const auto& n : names)
        combo_lidar_->addItem(QString::fromStdString(n));
    combo_lidar_->blockSignals(false);

    current_lidar_.clear();
    prev_lidar_.clear();
    node_ns_ = kNodeNs;
    label_install_->setText("—");
    if (label_param_source_) label_param_source_->setText("—");
    btn_start_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
}

void TabLidarRp::deactivate()
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

void TabLidarRp::onLidarSelected(const QString& name)
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

    const std::string new_lidar = name.toStdString();

    // 停止上一个 driver（如果切换到不同的雷达）
    if (!prev_lidar_.empty() && prev_lidar_ != new_lidar) {
        ros_->killNode("driver_" + prev_lidar_);
    }

    prev_lidar_    = new_lidar;
    current_lidar_ = new_lidar;
    node_ns_ = kNodeNs;

    loadLidarParamsToUi(current_lidar_);

    // 更新原始点云 topic（使用 point_cloud_topic）
    raw_cloud_topic_ = cfg_->getLidarPointCloudTopic(current_lidar_);

    // 启动新 driver（如果配置了 launch_file）
    const std::string launch_path = cfg_->getLidarLaunchFilePath(current_lidar_);
    if (!launch_path.empty()) {
        const std::string packets_topic = cfg_->getLidarPacketsTopic(current_lidar_);
        const std::string point_cloud_topic = cfg_->getLidarPointCloudTopic(current_lidar_);
        QStringList driver_args;
        double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;
        const bool has_install = cfg_->getLidarInstall(current_lidar_, x, y, z, roll, pitch, yaw);
        resolveLidarParams(current_lidar_, x, y, z, roll, pitch, yaw, nullptr);
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        if (has_install) {
            driver_args << QString("xoffset:=%1").arg(x)
                        << QString("yoffset:=%1").arg(y)
                        << QString("zoffset:=%1").arg(z)
                        << QString("roll:=%1").arg(roll)
                        << QString("pitch:=%1").arg(pitch)
                        << QString("yaw:=%1").arg(yaw)
                        << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        }
        QString cmd = QString("roslaunch %1").arg(QString::fromStdString(launch_path));
        for (const auto& a : driver_args) cmd += " " + a;
        ROS_INFO("[TabLidarRp] driver launch cmd:\n%s", cmd.toStdString().c_str());
        ros_->launchNodeByPath(launch_path, "driver_" + current_lidar_, driver_args);
    }

    // 重置订阅与视图
    ros_->unsubscribeAll();
    raw_cloud_buf_.reset();
    viewer_->clear();
    label_data_status_->setText("未订阅");
    label_cali_status_->setText("—");
    label_result_->setText("—");

    btn_start_->setEnabled(true);
    btn_save_->setEnabled(true);
    btn_reset_->setEnabled(true);

    // 如果可视化已开启，重新订阅，并显示"正在加载"
    if (viz_enabled_) {
        label_data_status_->setText("订阅中，等待数据...");
        viewer_->showLoading(true);
        // 始终订阅原始点云（直接渲染，不依赖标定节点）
        if (!raw_cloud_topic_.empty()) {
            ros_->subscribeCloud(raw_cloud_topic_,
                [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                    onRawCloudReceived(cloud);
                });
        }
        // 订阅标定节点输出（节点运行时才有数据）
        ros_->subscribeCloud(node_ns_ + "/clicked_cloud",
            [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                onCloudReceived(cloud);
            });
        ros_->subscribeStatus(node_ns_ + "/status",
            [this](const std::string& msg) {
                onStatusReceived(msg);
            });
        ros_->subscribeResult(node_ns_ + "/result",
            [this](double x, double y, double z) {
                onResultReceived(x, y, z);
            });
    }
}

void TabLidarRp::updateInstallLabel()
{
    QString source_text;
    double x, y, z, roll, pitch, yaw;
    resolveLidarParams(current_lidar_, x, y, z, roll, pitch, yaw, &source_text);
    label_install_->setText(
        QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
        .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3)
        .arg(roll, 0, 'f', 2).arg(pitch, 0, 'f', 2).arg(yaw, 0, 'f', 2));
    if (label_param_source_) {
        label_param_source_->setText(source_text);
    }
}

void TabLidarRp::loadLidarParamsToUi(const std::string& lidar_name)
{
    if (lidar_name.empty()) return;
    updateInstallLabel();

    double x, y, z, roll, pitch, yaw;
    resolveLidarParams(lidar_name, x, y, z, roll, pitch, yaw, nullptr);
    spin_roll_->blockSignals(true);
    spin_pitch_->blockSignals(true);
    spin_roll_->setValue(roll);
    spin_pitch_->setValue(pitch);
    spin_roll_->blockSignals(false);
    spin_pitch_->blockSignals(false);
}

void TabLidarRp::resolveLidarParams(const std::string& lidar_name,
                                    double& x, double& y, double& z,
                                    double& roll, double& pitch, double& yaw,
                                    QString* source_text) const
{
    ConfigManager::LidarMergedParams params;
    if (!cfg_->getMergedLidarParams(lidar_name, params)) {
        x = y = z = roll = pitch = yaw = 0.0;
        if (source_text) *source_text = "未找到车辆配置安装参数";
        return;
    }

    x = params.x;
    y = params.y;
    z = params.z;
    roll = params.roll_deg;
    pitch = params.pitch_deg;
    yaw = params.yaw_deg;

    if (source_text) {
        *source_text = cfg_->getMergedLidarParamSourceText(params);
    }
}

void TabLidarRp::onRawCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    raw_cloud_buf_ = cloud;
    viewer_->showLoading(false);
    // 若有已选点，叠加显示；否则只显示原始点云
    if (clicked_cloud_buf_ && !clicked_cloud_buf_->empty()) {
        viewer_->showRawAndTarget(cloud, clicked_cloud_buf_, 0.2, 1.0, 0.3);
        label_data_status_->setText(
            QString("原始 %1pt | 已选 %2 个")
            .arg(static_cast<int>(cloud->size()))
            .arg(static_cast<int>(clicked_cloud_buf_->size())));
    } else {
        label_data_status_->setText(
            QString("原始点云 %1 点").arg(static_cast<int>(cloud->size())));
        viewer_->showCloud(cloud, "raw", 0.2, 0.8, 1.0);
    }
}

void TabLidarRp::onCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    clicked_cloud_buf_ = cloud;
    viewer_->showLoading(false);
    label_data_status_->setText(
        QString("已选点 %1 个").arg(static_cast<int>(cloud->size())));
    // 叠加在原始点云上，用大亮绿点显示
    if (raw_cloud_buf_) {
        viewer_->showRawAndTarget(raw_cloud_buf_, cloud, 0.2, 1.0, 0.3);
    } else {
        viewer_->showCloud(cloud, "clicked", 0.2, 1.0, 0.3);
    }
}

void TabLidarRp::onStatusReceived(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
}

void TabLidarRp::onResultReceived(double x, double y, double z)
{
    // x=roll_deg, y=pitch_deg, z=confidence
    last_roll_ = x;
    last_pitch_ = y;
    label_result_->setText(
        QString("Roll=%1°  Pitch=%2°  conf=%3")
        .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 2));
}

void TabLidarRp::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        btn_pick_mode_->setEnabled(true);
        // 重新订阅
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

void TabLidarRp::onRawCloudToggle(int state)
{
    show_raw_cloud_ = (state == Qt::Checked);
    if (viz_enabled_ && !current_lidar_.empty()) {
        onLidarSelected(QString::fromStdString(current_lidar_));
    }
}

void TabLidarRp::onPickModeToggle(bool checked)
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

void TabLidarRp::onPointPicked(float x, float y, float z)
{
    ros_->publishPoint("/clicked_point", x, y, z, current_lidar_);
    label_data_status_->setText(
        QString("已发点 x=%1 y=%2 z=%3")
        .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
}

void TabLidarRp::onStartCalib()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/trigger_calibration", msg);
    label_cali_status_->setText(
        QString(ok ? "[触发成功] " : "[触发失败] ") + QString::fromStdString(msg));
}

void TabLidarRp::onSaveCalib()
{
    std::string msg;
    bool ok = ros_->callSave(node_ns_ + "/save_calibration", msg);
    if (ok) {
        ConfigManager::LidarRpProcessParams saved_params;
        const bool has_process = cfg_->getLidarRpProcessParams(current_lidar_, saved_params);

        if (has_process) {
            if (saved_params.has_z) last_z_ = saved_params.z;
            if (saved_params.has_roll) last_roll_ = saved_params.roll_deg;
            if (saved_params.has_pitch) last_pitch_ = saved_params.pitch_deg;
        }

        QString detail;
        if (has_process) {
            detail = QString("\n节点已写入: %1/lidar_rp_%2.yaml\nz=%3m  roll=%4°  pitch=%5°")
                .arg(QString::fromStdString(cfg_->processDir()))
                .arg(QString::fromStdString(current_lidar_))
                .arg(saved_params.has_z ? saved_params.z : last_z_, 0, 'f', 3)
                .arg(saved_params.has_roll ? saved_params.roll_deg : last_roll_, 0, 'f', 3)
                .arg(saved_params.has_pitch ? saved_params.pitch_deg : last_pitch_, 0, 'f', 3);
        } else {
            detail = QString("\n节点保存成功，但未读到 process 标定文件");
        }

        loadLidarParamsToUi(current_lidar_);
        QMessageBox::information(this, "保存成功", QString::fromStdString(msg) + detail);
    } else {
        QMessageBox::warning(this, "保存失败", QString::fromStdString(msg));
    }
}

void TabLidarRp::onResetCalib()
{
    std::string msg;
    ros_->callReset(node_ns_ + "/reset_calibration", msg);
    label_cali_status_->setText("已重置");
    label_result_->setText("—");
    clicked_cloud_buf_.reset();
    viewer_->clear();
}

void TabLidarRp::onRollChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/installation/roll_deg", v);
}

void TabLidarRp::onPitchChanged(double v)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/installation/pitch_deg", v);
}

void TabLidarRp::updateStatusBar(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
}

void TabLidarRp::onStartNode()
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
    double x, y, z, roll, pitch, yaw;
    if (cfg_->getLidarInstall(current_lidar_, x, y, z, roll, pitch, yaw)) {
        resolveLidarParams(current_lidar_, x, y, z, roll, pitch, yaw, nullptr);
        args << QString("inst_x:=%1").arg(x)
             << QString("inst_y:=%1").arg(y)
             << QString("inst_z:=%1").arg(z)
             << QString("inst_roll_deg:=%1").arg(roll)
             << QString("inst_pitch_deg:=%1").arg(pitch)
             << QString("inst_yaw_deg:=%1").arg(yaw)
             << QString("inst_z_offset:=%1").arg(cfg_->zOffset());
    }

    const std::string proc_dir = cfg_->processDir();
    if (!proc_dir.empty()) {
        QDir().mkpath(QString::fromStdString(proc_dir));
        args << QString("calibration_output_file:=%1/lidar_rp_%2.yaml")
                .arg(QString::fromStdString(proc_dir))
                .arg(QString::fromStdString(current_lidar_));
    }

    bool ok = ros_->launchNode("lidar_rp_cali", "lidar_rp_cali_clicked.launch", "lidar_rp_cali_clicked", args);
    if (!ok) {
        label_node_status_->setText("● 启动失败");
        label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    }
}

void TabLidarRp::onStopNode()
{
    ros_->killNode("lidar_rp_cali_clicked");
}

void TabLidarRp::onNodeStarted(const QString& key)
{
    if (key != "lidar_rp_cali_clicked") return;
    label_node_status_->setText("● 运行中");
    label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(true);
}

void TabLidarRp::onNodeStopped(const QString& key)
{
    if (key != "lidar_rp_cali_clicked") return;
    label_node_status_->setText("● 未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_start_node_->setEnabled(true);
    btn_stop_node_->setEnabled(false);
}

} // namespace calibration_gui
