#include "calibration_gui/tab_camera_cali.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QSplitter>
#include <QScrollArea>
#include <QFrame>
#include <QPushButton>

namespace calibration_gui {

// camera_cali.launch: name="camera_cali_node"
static const std::string kNodeNs = "/camera_cali_node";

TabCameraCali::TabCameraCali(std::shared_ptr<ConfigManager> cfg,
                             std::shared_ptr<RosWorker> ros,
                             QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
}

void TabCameraCali::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    // ---- 左侧控制 ----
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
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabCameraCali::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    vl->addWidget(grp_viz);

    auto* grp_sensor = new QGroupBox("相机选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_ref_    = new QComboBox(this);
    combo_target_ = new QComboBox(this);
    combo_ref_->addItem("-- 请先加载车辆配置 --");
    combo_target_->addItem("-- 请先加载车辆配置 --");
    connect(combo_ref_,    &QComboBox::currentTextChanged,
            this, &TabCameraCali::onRefCameraSelected);
    connect(combo_target_, &QComboBox::currentTextChanged,
            this, &TabCameraCali::onTargetCameraSelected);
    fl_s->addRow("参考相机：", combo_ref_);
    fl_s->addRow("待标定相机：", combo_target_);
    label_install_ = new QLabel("—", this);
    label_install_->setWordWrap(true);
    label_install_->setStyleSheet("QLabel{color:#8e8;}");
    fl_s->addRow("待标定安装：", label_install_);
    vl->addWidget(grp_sensor);

    // 外参微调
    auto* grp_tune = new QGroupBox("相机间外参微调（T_ref_target）", this);
    auto* fl_t = new QFormLayout(grp_tune);
    auto makeSpin = [this](double lo, double hi, double step, int dec,
                           const QString& suf) {
        auto* s = new QDoubleSpinBox(this);
        s->setRange(lo, hi); s->setSingleStep(step);
        s->setDecimals(dec); s->setSuffix(suf);
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TabCameraCali::onParamChanged);
        return s;
    };
    spin_x_     = makeSpin(-5.0, 5.0, 0.001, 4, " m");
    spin_y_     = makeSpin(-5.0, 5.0, 0.001, 4, " m");
    spin_z_     = makeSpin(-5.0, 5.0, 0.001, 4, " m");
    spin_roll_  = makeSpin(-90.0, 90.0, 0.01, 3, " °");
    spin_pitch_ = makeSpin(-90.0, 90.0, 0.01, 3, " °");
    spin_yaw_   = makeSpin(-180.0, 180.0, 0.01, 3, " °");
    fl_t->addRow("X：", spin_x_);
    fl_t->addRow("Y：", spin_y_);
    fl_t->addRow("Z：", spin_z_);
    fl_t->addRow("Roll：",  spin_roll_);
    fl_t->addRow("Pitch：", spin_pitch_);
    fl_t->addRow("Yaw：",   spin_yaw_);
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
    fl_st->addRow("平移结果：", label_result_);
    vl->addWidget(grp_status);

    // 操作
    auto* grp_btn = new QGroupBox("操作", this);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8);
    vl_b->setContentsMargins(8, 8, 8, 8);
    btn_start_ = new QPushButton("触发标定", this);
    btn_save_  = new QPushButton("保存结果", this);
    btn_reset_ = new QPushButton("重置", this);
    btn_start_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    connect(btn_start_, &QPushButton::clicked, this, &TabCameraCali::onStartCalib);
    connect(btn_save_,  &QPushButton::clicked, this, &TabCameraCali::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabCameraCali::onResetCalib);
    vl_b->addWidget(btn_start_);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    auto* lbl_hint = new QLabel(
        "<small><b>步骤：</b><br>"
        "1. 启动 camera_cali_node<br>"
        "2. 将 FASTLIVO2 靶标置于双相机共视区域<br>"
        "3. 多角度采集 ≥10 对图像<br>"
        "4. 触发标定 → 查看本质矩阵分解结果<br>"
        "5. 保存结果（6DOF 外参）</small>", this);
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

    // ---- 右侧：参考/目标图像并排 ----
    auto* right = new QWidget(this);
    auto* rl = new QVBoxLayout(right);
    rl->setContentsMargins(0, 0, 0, 0);
    rl->setSpacing(4);

    auto* grp_ref = new QGroupBox("参考相机（角点检测）", this);
    auto* ref_l = new QVBoxLayout(grp_ref);
    ref_l->setContentsMargins(4, 4, 4, 4);
    viewer_ref_ = new ImageViewer(this);
    ref_l->addWidget(viewer_ref_);

    auto* grp_tgt = new QGroupBox("待标定相机（角点检测）", this);
    auto* tgt_l = new QVBoxLayout(grp_tgt);
    tgt_l->setContentsMargins(4, 4, 4, 4);
    viewer_target_ = new ImageViewer(this);
    tgt_l->addWidget(viewer_target_);

    rl->addWidget(grp_ref, 1);
    rl->addWidget(grp_tgt, 1);
    h->addWidget(right, 1);
}

void TabCameraCali::applyConfig()
{
    combo_ref_->blockSignals(true);
    combo_target_->blockSignals(true);
    combo_ref_->clear();
    combo_target_->clear();
    for (const auto& n : cfg_->cameraNames()) {
        combo_ref_->addItem(QString::fromStdString(n));
        combo_target_->addItem(QString::fromStdString(n));
    }
    combo_ref_->blockSignals(false);
    combo_target_->blockSignals(false);

    const auto cams = cfg_->cameraNames();
    if (!cams.empty()) {
        combo_ref_->setCurrentIndex(0);
        if (cams.size() >= 2) combo_target_->setCurrentIndex(1);
        else                  combo_target_->setCurrentIndex(0);
        // Update labels without subscribing
        const QString tgt = combo_target_->currentText();
        if (!tgt.isEmpty() && !tgt.startsWith("--")) {
            node_ns_ = kNodeNs;
            double x, y, z, roll, pitch, yaw;
            if (cfg_->getCameraInstall(tgt.toStdString(), x, y, z, roll, pitch, yaw)) {
                label_install_->setText(
                    QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
                    .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3)
                    .arg(roll, 0, 'f', 2).arg(pitch, 0, 'f', 2).arg(yaw, 0, 'f', 2));
                auto block = [&](bool b) {
                    spin_x_->blockSignals(b); spin_y_->blockSignals(b);
                    spin_z_->blockSignals(b); spin_roll_->blockSignals(b);
                    spin_pitch_->blockSignals(b); spin_yaw_->blockSignals(b);
                };
                block(true);
                spin_x_->setValue(x); spin_y_->setValue(y); spin_z_->setValue(z);
                spin_roll_->setValue(roll); spin_pitch_->setValue(pitch); spin_yaw_->setValue(yaw);
                block(false);
            }
        }
        btn_start_->setEnabled(true);
        btn_save_->setEnabled(true);
        btn_reset_->setEnabled(true);
    }
}

void TabCameraCali::onRefCameraSelected(const QString& /*name*/) {}

void TabCameraCali::onTargetCameraSelected(const QString& name)
{
    if (name.isEmpty() || name.startsWith("--")) return;
    node_ns_ = kNodeNs;

    double x, y, z, roll, pitch, yaw;
    if (cfg_->getCameraInstall(name.toStdString(), x, y, z, roll, pitch, yaw)) {
        label_install_->setText(
            QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
            .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3)
            .arg(roll, 0, 'f', 2).arg(pitch, 0, 'f', 2).arg(yaw, 0, 'f', 2));
        auto block = [&](bool b) {
            spin_x_->blockSignals(b); spin_y_->blockSignals(b);
            spin_z_->blockSignals(b); spin_roll_->blockSignals(b);
            spin_pitch_->blockSignals(b); spin_yaw_->blockSignals(b);
        };
        block(true);
        spin_x_->setValue(x); spin_y_->setValue(y); spin_z_->setValue(z);
        spin_roll_->setValue(roll); spin_pitch_->setValue(pitch); spin_yaw_->setValue(yaw);
        block(false);
    }

    ros_->unsubscribeAll();
    viewer_ref_->clear();
    viewer_target_->clear();
    label_data_status_->setText("未订阅");
    label_cali_status_->setText("—");
    label_result_->setText("—");

    btn_start_->setEnabled(true);
    btn_save_->setEnabled(true);
    btn_reset_->setEnabled(true);

    if (viz_enabled_) {
        label_data_status_->setText("订阅中...");
        ros_->subscribeImage(node_ns_ + "/debug_ref",
            [this](cv::Mat img) { onRefImageReceived(img); });
        ros_->subscribeImage(node_ns_ + "/debug_target",
            [this](cv::Mat img) { onTargetImageReceived(img); });
        ros_->subscribeStatus(node_ns_ + "/status",
            [this](const std::string& msg) { onStatusReceived(msg); });
        ros_->subscribeResult(node_ns_ + "/result",
            [this](double x, double y, double z) { onResultReceived(x, y, z); });
    }
}

void TabCameraCali::onRefImageReceived(cv::Mat img)
{
    if (!viz_enabled_) return;
    label_data_status_->setText(
        QString("收到图像 %1x%2").arg(img.cols).arg(img.rows));
    viewer_ref_->showImage(img);
}

void TabCameraCali::onTargetImageReceived(cv::Mat img)
{
    if (!viz_enabled_) return;
    viewer_target_->showImage(img);
}

void TabCameraCali::onStatusReceived(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
}

void TabCameraCali::onResultReceived(double x, double y, double z)
{
    label_result_->setText(
        QString("tx=%1m  ty=%2m  tz=%3m")
        .arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4));
}

void TabCameraCali::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        const QString tgt = combo_target_->currentText();
        if (!tgt.isEmpty() && !tgt.startsWith("--")) {
            onTargetCameraSelected(tgt);
        }
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        viewer_ref_->clear();
        viewer_target_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabCameraCali::onStartCalib()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/trigger_calibration", msg);
    label_cali_status_->setText(
        QString(ok ? "[触发成功] " : "[触发失败] ") + QString::fromStdString(msg));
}

void TabCameraCali::onSaveCalib()
{
    std::string msg;
    bool ok = ros_->callSave(node_ns_ + "/save_calibration", msg);
    if (ok) {
        QMessageBox::information(this, "保存成功", QString::fromStdString(msg));
    } else {
        QMessageBox::warning(this, "保存失败", QString::fromStdString(msg));
    }
}

void TabCameraCali::onResetCalib()
{
    std::string msg;
    ros_->callReset(node_ns_ + "/reset_calibration", msg);
    viewer_ref_->clear();
    viewer_target_->clear();
    label_cali_status_->setText("已重置");
    label_result_->setText("—");
}

void TabCameraCali::onParamChanged(double /*v*/)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/extrinsic/x",         spin_x_->value());
    ros_->setParam(node_ns_ + "/extrinsic/y",         spin_y_->value());
    ros_->setParam(node_ns_ + "/extrinsic/z",         spin_z_->value());
    ros_->setParam(node_ns_ + "/extrinsic/roll_deg",  spin_roll_->value());
    ros_->setParam(node_ns_ + "/extrinsic/pitch_deg", spin_pitch_->value());
    ros_->setParam(node_ns_ + "/extrinsic/yaw_deg",   spin_yaw_->value());
}

} // namespace calibration_gui
