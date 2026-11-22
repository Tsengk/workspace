#include "calibration_gui/tab_front_stitch.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QFrame>
#include <ros/ros.h>

namespace calibration_gui {

TabFrontStitch::TabFrontStitch(std::shared_ptr<ConfigManager> cfg,
                               std::shared_ptr<RosWorker> ros,
                               QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
}

void TabFrontStitch::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    auto* left = new QWidget(this);
    left->setFixedWidth(380);
    auto* vl = new QVBoxLayout(left);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(10);

    // ---- 节点管理 ----
    auto* grp_node = new QGroupBox("节点管理", this);
    auto* hl_node = new QHBoxLayout(grp_node);
    hl_node->setContentsMargins(8, 8, 8, 8);
    btn_start_ = new QPushButton("▶ 启动节点", this);
    btn_stop_  = new QPushButton("⏹ 停止节点", this);
    btn_stop_->setEnabled(false);
    label_node_status_ = new QLabel("未运行", this);
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    connect(btn_start_, &QPushButton::clicked, this, &TabFrontStitch::onStartNode);
    connect(btn_stop_,  &QPushButton::clicked, this, &TabFrontStitch::onStopNode);
    hl_node->addWidget(btn_start_);
    hl_node->addWidget(btn_stop_);
    hl_node->addWidget(label_node_status_, 1);
    vl->addWidget(grp_node);

    // ---- 可视化控制 ----
    auto* grp_viz = new QGroupBox("可视化控制", this);
    auto* vl_viz = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", this);
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabFrontStitch::onVizToggle);
    label_data_status_ = new QLabel("未订阅", this);
    vl_viz->addWidget(btn_viz_toggle_);
    vl_viz->addWidget(label_data_status_);
    vl->addWidget(grp_viz);

    // ---- 相机选择 ----
    auto* grp_cams = new QGroupBox("前视三相机选择", this);
    auto* fl_c = new QFormLayout(grp_cams);
    fl_c->setSpacing(6);
    combo_left_  = new QComboBox(this);
    combo_front_ = new QComboBox(this);
    combo_right_ = new QComboBox(this);
    combo_left_->addItem("-- 请先加载车辆配置 --");
    combo_front_->addItem("-- 请先加载车辆配置 --");
    combo_right_->addItem("-- 请先加载车辆配置 --");
    connect(combo_left_,  &QComboBox::currentTextChanged, this, &TabFrontStitch::onLeftCameraSelected);
    connect(combo_front_, &QComboBox::currentTextChanged, this, &TabFrontStitch::onFrontCameraSelected);
    connect(combo_right_, &QComboBox::currentTextChanged, this, &TabFrontStitch::onRightCameraSelected);
    label_left_topic_  = new QLabel("—", this); label_left_topic_->setStyleSheet("QLabel{color:#8af;font-size:11px;}");
    label_front_topic_ = new QLabel("—", this); label_front_topic_->setStyleSheet("QLabel{color:#8af;font-size:11px;}");
    label_right_topic_ = new QLabel("—", this); label_right_topic_->setStyleSheet("QLabel{color:#8af;font-size:11px;}");
    fl_c->addRow("左相机：",  combo_left_);
    fl_c->addRow("",          label_left_topic_);
    fl_c->addRow("前相机：",  combo_front_);
    fl_c->addRow("",          label_front_topic_);
    fl_c->addRow("右相机：",  combo_right_);
    fl_c->addRow("",          label_right_topic_);
    vl->addWidget(grp_cams);

    // ---- 拼接参数 ----
    auto* grp_params = new QGroupBox("拼接参数（实时写入 rosparam）", this);
    auto* fl_p = new QFormLayout(grp_params);
    fl_p->setSpacing(6);

    spin_align_depth_z_ = new QDoubleSpinBox(this);
    spin_align_depth_z_->setRange(0.0, 50.0);
    spin_align_depth_z_->setSingleStep(0.5);
    spin_align_depth_z_->setDecimals(2);
    spin_align_depth_z_->setSuffix(" m");
    spin_align_depth_z_->setValue(5.0);
    connect(spin_align_depth_z_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabFrontStitch::onParamChanged);
    fl_p->addRow("前相机深度 align_depth_z：", spin_align_depth_z_);

    spin_anchor_left_x_ = new QSpinBox(this);
    spin_anchor_left_x_->setRange(0, 9999); spin_anchor_left_x_->setSuffix(" px");
    connect(spin_anchor_left_x_, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int){ onParamChanged(0); });
    fl_p->addRow("左锚点 anchor_left_x：", spin_anchor_left_x_);

    spin_anchor_left_y_ = new QSpinBox(this);
    spin_anchor_left_y_->setRange(0, 9999); spin_anchor_left_y_->setSuffix(" px");
    connect(spin_anchor_left_y_, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int){ onParamChanged(0); });
    fl_p->addRow("左锚点 anchor_left_y：", spin_anchor_left_y_);

    spin_anchor_right_x_ = new QSpinBox(this);
    spin_anchor_right_x_->setRange(0, 9999); spin_anchor_right_x_->setSuffix(" px");
    connect(spin_anchor_right_x_, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int){ onParamChanged(0); });
    fl_p->addRow("右锚点 anchor_right_x：", spin_anchor_right_x_);

    spin_anchor_right_y_ = new QSpinBox(this);
    spin_anchor_right_y_->setRange(0, 9999); spin_anchor_right_y_->setSuffix(" px");
    connect(spin_anchor_right_y_, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int){ onParamChanged(0); });
    fl_p->addRow("右锚点 anchor_right_y：", spin_anchor_right_y_);

    edit_stitched_topic_ = new QLineEdit(this);
    edit_stitched_topic_->setText("/multi_view_stitch_node/stitched_image");
    edit_stitched_topic_->setPlaceholderText("拼接结果图像 topic");
    fl_p->addRow("stitched_topic：", edit_stitched_topic_);

    vl->addWidget(grp_params);

    // ---- 提示 ----
    auto* lbl_hint = new QLabel(
        "<small><b>说明：</b><br>"
        "1. 选择左/前/右三个相机，topic 和内参自动确定<br>"
        "2. 设置前相机深度 align_depth_z（0~50m）<br>"
        "3. 设置左/右锚点像素坐标（不超过相机分辨率）<br>"
        "4. 启动节点，开启可视化查看拼接效果</small>", this);
    lbl_hint->setWordWrap(true);
    lbl_hint->setStyleSheet("QLabel{color:#888;border:1px solid #333;padding:6px;}");
    vl->addWidget(lbl_hint);

    vl->addStretch();
    auto* scroll_left = new QScrollArea(this);
    scroll_left->setWidgetResizable(true);
    scroll_left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll_left->setFrameShape(QFrame::NoFrame);
    scroll_left->setWidget(left);
    scroll_left->setFixedWidth(left->width() + 16);
    h->addWidget(scroll_left);

    // ---- 右侧：拼接图像预览 ----
    auto* right_box = new QGroupBox("拼接图像预览", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new ImageViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);
}

void TabFrontStitch::applyConfig()
{
    if (!cfg_->isLoaded()) return;

    combo_left_->blockSignals(true);
    combo_front_->blockSignals(true);
    combo_right_->blockSignals(true);
    combo_left_->clear();
    combo_front_->clear();
    combo_right_->clear();
    for (const auto& n : cfg_->cameraNames()) {
        combo_left_->addItem(QString::fromStdString(n));
        combo_front_->addItem(QString::fromStdString(n));
        combo_right_->addItem(QString::fromStdString(n));
    }
    combo_left_->blockSignals(false);
    combo_front_->blockSignals(false);
    combo_right_->blockSignals(false);

    const auto cams = cfg_->cameraNames();
    if (cams.size() >= 3) {
        combo_left_->setCurrentIndex(0);
        combo_front_->setCurrentIndex(1);
        combo_right_->setCurrentIndex(2);
    } else if (!cams.empty()) {
        combo_left_->setCurrentIndex(0);
        combo_front_->setCurrentIndex(0);
        combo_right_->setCurrentIndex(0);
    }
    // 触发 topic 标签更新
    onLeftCameraSelected(combo_left_->currentText());
    onFrontCameraSelected(combo_front_->currentText());
    onRightCameraSelected(combo_right_->currentText());
}

std::string TabFrontStitch::getCameraTopicByName(const std::string& name) const
{
    for (const auto& c : cfg_->getConfig().cameras)
        if (c.name == name) return c.topic;
    return "";
}

std::string TabFrontStitch::getCameraIntrinsicsByName(const std::string& name) const
{
    return cfg_->getCameraIntrinsicsPath(name);
}

void TabFrontStitch::onLeftCameraSelected(const QString& name)
{
    if (name.isEmpty() || name.startsWith("--")) return;
    current_left_ = name.toStdString();
    const std::string topic = getCameraTopicByName(current_left_);
    label_left_topic_->setText(QString("topic: %1").arg(QString::fromStdString(topic)));
    updateNodeParams();
}

void TabFrontStitch::onFrontCameraSelected(const QString& name)
{
    if (name.isEmpty() || name.startsWith("--")) return;
    current_front_ = name.toStdString();
    const std::string topic = getCameraTopicByName(current_front_);
    label_front_topic_->setText(QString("topic: %1").arg(QString::fromStdString(topic)));
    updateNodeParams();
}

void TabFrontStitch::onRightCameraSelected(const QString& name)
{
    if (name.isEmpty() || name.startsWith("--")) return;
    current_right_ = name.toStdString();
    const std::string topic = getCameraTopicByName(current_right_);
    label_right_topic_->setText(QString("topic: %1").arg(QString::fromStdString(topic)));
    updateNodeParams();
}

void TabFrontStitch::onParamChanged(double /*v*/)
{
    updateNodeParams();
}

void TabFrontStitch::updateNodeParams()
{
    const std::string ns = kNodeNs;
    // align_depth_z / anchor_* 动态写入，节点运行时实时生效
    ros_->setParam(ns + "/align_depth_z",  spin_align_depth_z_->value());
    ros_->setParam(ns + "/anchor/left/x",  static_cast<double>(spin_anchor_left_x_->value()));
    ros_->setParam(ns + "/anchor/left/y",  static_cast<double>(spin_anchor_left_y_->value()));
    ros_->setParam(ns + "/anchor/right/x", static_cast<double>(spin_anchor_right_x_->value()));
    ros_->setParam(ns + "/anchor/right/y", static_cast<double>(spin_anchor_right_y_->value()));

    ros::param::set(ns + "/left_image_topic",  getCameraTopicByName(current_left_));
    ros::param::set(ns + "/front_image_topic", getCameraTopicByName(current_front_));
    ros::param::set(ns + "/right_image_topic", getCameraTopicByName(current_right_));
    ros::param::set(ns + "/left_intrinsics",   getCameraIntrinsicsByName(current_left_));
    ros::param::set(ns + "/front_intrinsics",  getCameraIntrinsicsByName(current_front_));
    ros::param::set(ns + "/right_intrinsics",  getCameraIntrinsicsByName(current_right_));
    ros::param::set(ns + "/stitched_topic",    edit_stitched_topic_->text().toStdString());

    // ROS_INFO_STREAM("[TabFrontStitch] params -> "
    //     << "align_depth_z=" << spin_align_depth_z_->value()
    //     << " anchor_left=(" << spin_anchor_left_x_->value() << "," << spin_anchor_left_y_->value() << ")"
    //     << " anchor_right=(" << spin_anchor_right_x_->value() << "," << spin_anchor_right_y_->value() << ")"
    //     << " left_topic=" << getCameraTopicByName(current_left_)
    //     << " front_topic=" << getCameraTopicByName(current_front_)
    //     << " right_topic=" << getCameraTopicByName(current_right_));
}

void TabFrontStitch::onStartNode()
{
    // 先写参数，再启动节点
    updateNodeParams();

    // align_depth_z/anchor_* 由 launch 文件默认值决定，GUI 通过 rosparam 动态修改
    QStringList args;
    args << QString("left_image_topic:=%1").arg(
                QString::fromStdString(getCameraTopicByName(current_left_)))
         << QString("front_image_topic:=%1").arg(
                QString::fromStdString(getCameraTopicByName(current_front_)))
         << QString("right_image_topic:=%1").arg(
                QString::fromStdString(getCameraTopicByName(current_right_)))
         << QString("left_intrinsics:=%1").arg(
                QString::fromStdString(getCameraIntrinsicsByName(current_left_)))
         << QString("front_intrinsics:=%1").arg(
                QString::fromStdString(getCameraIntrinsicsByName(current_front_)))
         << QString("right_intrinsics:=%1").arg(
                QString::fromStdString(getCameraIntrinsicsByName(current_right_)))
         << QString("stitched_topic:=%1").arg(edit_stitched_topic_->text());

    // 打印完整启动命令到终端
    QString cmd = "roslaunch multi_view_stitch multi_view_stitch.launch";
    for (const auto& a : args) cmd += " " + a;
    ROS_INFO_STREAM("[TabFrontStitch] launch cmd: " << cmd.toStdString());

    bool ok = ros_->launchNode("multi_view_stitch", "multi_view_stitch.launch",
                               kNodeKey, args);
    if (ok) {
        label_node_status_->setText("运行中");
        label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
        btn_start_->setEnabled(false);
        btn_stop_->setEnabled(true);
    } else {
        QMessageBox::warning(this, "启动失败", "无法启动 multi_view_stitch 节点，请检查 launch 文件。");
    }
}

void TabFrontStitch::onStopNode()
{
    ros_->killNode(kNodeKey);
    label_node_status_->setText("未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_start_->setEnabled(true);
    btn_stop_->setEnabled(false);
    if (viz_enabled_) {
        viz_enabled_ = false;
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabFrontStitch::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        const std::string topic = edit_stitched_topic_->text().toStdString();
        ros_->unsubscribeAll();
        ros_->subscribeImage(topic, [this](cv::Mat img) { onStitchImageReceived(img); });
        label_data_status_->setText("订阅中...");
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabFrontStitch::onStitchImageReceived(cv::Mat img)
{
    if (!viz_enabled_) return;
    label_data_status_->setText(
        QString("拼接图 %1x%2").arg(img.cols).arg(img.rows));
    viewer_->showImage(img);
}

} // namespace calibration_gui
