#include "calibration_gui/tab_bev.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QGridLayout>
#include <QPushButton>
#include <QSplitter>
#include <QDir>
#include <fstream>
#include <opencv2/imgproc.hpp>

namespace calibration_gui {

static const std::string kNodeNs  = "/bev_surround_view_node";
static const std::string kNodeKey = "bev_surround_view_node";

static const char* kCamLabel[] = {"前", "后", "左", "右"};

TabBev::TabBev(std::shared_ptr<ConfigManager> cfg,
               std::shared_ptr<RosWorker> ros,
               QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
    connect(ros_.get(), &RosWorker::nodeStarted, this, &TabBev::onNodeStarted);
    connect(ros_.get(), &RosWorker::nodeStopped, this, &TabBev::onNodeStopped);
}

void TabBev::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    // ---- 左侧：滚动控制面板 ----
    auto* scroll = new QScrollArea(this);
    scroll->setFixedWidth(480);
    scroll->setWidgetResizable(true);
    auto* left = new QWidget();
    auto* vl = new QVBoxLayout(left);
    vl->setContentsMargins(4, 4, 4, 4);
    vl->setSpacing(10);

    // ---- 节点管理 ----
    auto* grp_node = new QGroupBox("节点管理", left);
    auto* hl_node  = new QHBoxLayout(grp_node);
    hl_node->setContentsMargins(8, 8, 8, 8);
    hl_node->setSpacing(8);
    btn_start_node_ = new QPushButton("▶ 启动节点", left);
    btn_stop_node_  = new QPushButton("⏹ 停止节点", left);
    btn_stop_node_->setEnabled(false);
    connect(btn_start_node_, &QPushButton::clicked, this, &TabBev::onStartNode);
    connect(btn_stop_node_,  &QPushButton::clicked, this, &TabBev::onStopNode);
    label_node_status_ = new QLabel("● 未运行", left);
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    hl_node->addWidget(btn_start_node_);
    hl_node->addWidget(btn_stop_node_);
    hl_node->addWidget(label_node_status_);
    vl->addWidget(grp_node);

    // ---- 可视化控制 ----
    auto* grp_viz = new QGroupBox("可视化控制", left);
    auto* vl_viz  = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", left);
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabBev::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    vl->addWidget(grp_viz);

    // ---- 4路相机绑定 ----
    auto* grp_cams = new QGroupBox("环视相机绑定", left);
    auto* fl_c = new QFormLayout(grp_cams);
    QComboBox** combos[] = {&combo_front_, &combo_rear_, &combo_left_, &combo_right_};
    for (int i = 0; i < 4; ++i) {
        *combos[i] = new QComboBox(left);
        (*combos[i])->addItem("-- 请先加载配置 --");
        connect(*combos[i], &QComboBox::currentTextChanged,
                [this, i](const QString& name) { onCameraSelected(i, name); });
        fl_c->addRow(QString("%1相机：").arg(kCamLabel[i]), *combos[i]);
    }
    vl->addWidget(grp_cams);

    // ---- BEV 参数 ----
    auto* grp_bev = new QGroupBox("BEV 参数", left);
    auto* fl_b = new QFormLayout(grp_bev);
    auto makeDSpin = [left](double lo, double hi, double step, int dec, const QString& suf) {
        auto* s = new QDoubleSpinBox(left);
        s->setRange(lo, hi); s->setSingleStep(step);
        s->setDecimals(dec); s->setSuffix(suf);
        return s;
    };
    spin_resolution_  = makeDSpin(0.005, 0.1, 0.005, 3, " m/px");
    spin_resolution_->setValue(0.02);
    label_px_per_m_ = new QLabel("50 px/m", left);
    label_px_per_m_->setStyleSheet("QLabel{color:#8e8;}");
    connect(spin_resolution_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double v) {
                label_px_per_m_->setText(
                    QString("%1 px/m").arg(v > 0 ? 1.0/v : 0.0, 0, 'f', 1));
                onBevParamChanged(v);
            });
    fl_b->addRow("分辨率：", spin_resolution_);
    fl_b->addRow("1m=", label_px_per_m_);

    spin_range_front_ = makeDSpin(50, 2000, 10, 0, " px");
    spin_range_rear_  = makeDSpin(50, 2000, 10, 0, " px");
    spin_range_left_  = makeDSpin(50, 1000, 10, 0, " px");
    spin_range_right_ = makeDSpin(50, 1000, 10, 0, " px");
    spin_range_front_->setValue(250);
    spin_range_rear_->setValue(150);
    spin_range_left_->setValue(125);
    spin_range_right_->setValue(125);
    for (auto* s : {spin_range_front_, spin_range_rear_, spin_range_left_, spin_range_right_})
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TabBev::onBevParamChanged);
    fl_b->addRow("前向ROI：", spin_range_front_);
    fl_b->addRow("后向ROI：", spin_range_rear_);
    fl_b->addRow("左向ROI：", spin_range_left_);
    fl_b->addRow("右向ROI：", spin_range_right_);

    spin_origin_offset_x_ = makeDSpin(-500, 500, 10, 0, " px");
    spin_origin_offset_y_ = makeDSpin(-500, 500, 10, 0, " px");
    spin_origin_offset_x_->setValue(0);
    spin_origin_offset_y_->setValue(0);
    spin_origin_offset_x_->setToolTip("正值=向前偏移（多看前方，少看后方）");
    spin_origin_offset_y_->setToolTip("正值=向左偏移（多看左方，少看右方）");
    for (auto* s : {spin_origin_offset_x_, spin_origin_offset_y_})
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &TabBev::onBevParamChanged);
    fl_b->addRow("中心前后偏移：", spin_origin_offset_x_);
    fl_b->addRow("中心左右偏移：", spin_origin_offset_y_);
    vl->addWidget(grp_bev);

    // ---- 4路相机安装参数（只读）----
    auto* grp_install = new QGroupBox("环视相机安装参数", left);
    auto* fl_inst = new QFormLayout(grp_install);
    for (int i = 0; i < 4; ++i) {
        cam_params_[i].install_label = new QLabel("—", left);
        cam_params_[i].install_label->setStyleSheet("QLabel{color:#8e8;}");
        cam_params_[i].install_label->setWordWrap(true);
        fl_inst->addRow(QString("%1：").arg(kCamLabel[i]), cam_params_[i].install_label);
    }
    vl->addWidget(grp_install);

    // ---- 状态 ----
    auto* grp_status = new QGroupBox("实时状态", left);
    auto* fl_st = new QFormLayout(grp_status);
    label_data_status_ = new QLabel("未订阅", left);
    label_cali_status_ = new QLabel("—", left);
    label_cali_status_->setWordWrap(true);
    label_result_ = new QLabel("—", left);
    label_result_->setStyleSheet("QLabel{color:#ff0;}");
    label_bev_info_ = new QLabel("—", left);
    label_bev_info_->setWordWrap(true);
    label_bev_info_->setStyleSheet("QLabel{color:#8ae;font-size:12px;}");
    fl_st->addRow("数据：",   label_data_status_);
    fl_st->addRow("标定：",   label_cali_status_);
    fl_st->addRow("结果：",   label_result_);
    fl_st->addRow("配置：",   label_bev_info_);
    vl->addWidget(grp_status);

    // ---- 操作 ----
    auto* grp_btn = new QGroupBox("操作", left);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8); vl_b->setContentsMargins(8, 8, 8, 8);
    btn_start_ = new QPushButton("触发 BEV 标定（单应矩阵计算）", left);
    btn_save_  = new QPushButton("保存 BEV 标定结果", left);
    btn_reset_ = new QPushButton("重置", left);
    btn_start_->setEnabled(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    connect(btn_start_, &QPushButton::clicked, this, &TabBev::onStartCalib);
    connect(btn_save_,  &QPushButton::clicked, this, &TabBev::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabBev::onResetCalib);
    vl_b->addWidget(btn_start_);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    auto* lbl_hint = new QLabel(
        "<small><b>步骤：</b><br>"
        "1. 加载车辆配置，绑定4路相机<br>"
        "2. 启动 bev_surround_view_node<br>"
        "3. 开启可视化，四角各放棋盘格靶标<br>"
        "4. 触发标定 → 查看 BEV 拼接图<br>"
        "5. 满意后保存单应矩阵</small>", left);
    lbl_hint->setWordWrap(true);
    lbl_hint->setStyleSheet("QLabel{color:#888;border:1px solid #333;padding:4px;}");
    vl->addWidget(lbl_hint);

    vl->addStretch();
    scroll->setWidget(left);
    h->addWidget(scroll);

    // ---- 右侧：BEV 拼接图（中间）+ 4路检测图（最右纵向）----
    // 用 QSplitter 分隔中间和右侧
    auto* mid_widget = new QWidget(this);
    auto* mid_vl = new QVBoxLayout(mid_widget);
    mid_vl->setContentsMargins(0, 0, 0, 0);
    auto* grp_bev_view = new QGroupBox("环视 BEV 拼接（青线=车辆原点）", mid_widget);
    auto* bev_vl = new QVBoxLayout(grp_bev_view);
    bev_vl->setContentsMargins(4, 4, 4, 4);
    viewer_bev_ = new ImageViewer(grp_bev_view);
    bev_vl->addWidget(viewer_bev_);
    mid_vl->addWidget(grp_bev_view);
    h->addWidget(mid_widget, 1);

    // 最右侧：4路检测图 2x2 四宫格，与中间 BEV 图 1:1 占位
    auto* right_widget = new QWidget(this);
    auto* right_vl = new QVBoxLayout(right_widget);
    right_vl->setContentsMargins(0, 0, 0, 0);
    auto* grp_det = new QGroupBox("检测可视化", right_widget);
    auto* det_grid = new QGridLayout(grp_det);
    det_grid->setContentsMargins(4, 4, 4, 4);
    det_grid->setSpacing(4);
    const char* det_labels[] = {"前", "后", "左", "右"};
    ImageViewer** det_viewers[] = {&viewer_front_, &viewer_rear_, &viewer_left_, &viewer_right_};
    // row0: 前(0,0) 后(0,1)   row1: 左(1,0) 右(1,1)
    const int grid_row[] = {0, 1, 0, 1};
    const int grid_col[] = {0, 0, 1, 1};
    for (int i = 0; i < 4; ++i) {
        auto* cell = new QWidget(grp_det);
        auto* cell_vl = new QVBoxLayout(cell);
        cell_vl->setContentsMargins(0, 0, 0, 0);
        cell_vl->setSpacing(2);
        auto* lbl = new QLabel(det_labels[i], cell);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setStyleSheet("QLabel{color:#aaa;font-size:11px;}");
        *det_viewers[i] = new ImageViewer(cell);
        (*det_viewers[i])->setMinimumHeight(100);
        cell_vl->addWidget(lbl);
        cell_vl->addWidget(*det_viewers[i], 1);
        det_grid->addWidget(cell, grid_row[i], grid_col[i]);
    }
    det_grid->setRowStretch(0, 1);
    det_grid->setRowStretch(1, 1);
    det_grid->setColumnStretch(0, 1);
    det_grid->setColumnStretch(1, 1);
    right_vl->addWidget(grp_det);
    h->addWidget(right_widget, 1);
}

void TabBev::applyConfig()
{
    QComboBox* combos[] = {combo_front_, combo_rear_, combo_left_, combo_right_};
    for (int i = 0; i < 4; ++i) combos[i]->blockSignals(true);
    for (int i = 0; i < 4; ++i) combos[i]->clear();
    for (const auto& n : cfg_->cameraNames())
        for (int i = 0; i < 4; ++i)
            combos[i]->addItem(QString::fromStdString(n));
    for (int i = 0; i < 4; ++i) combos[i]->blockSignals(false);

    const auto cams = cfg_->cameraNames();
    for (int i = 0; i < 4 && i < static_cast<int>(cams.size()); ++i) {
        combos[i]->setCurrentIndex(i);
        onCameraSelected(i, combos[i]->currentText());
    }

    node_ns_ = kNodeNs;

    // 从配置文件读取 topic
    const auto& bev_cfg = cfg_->getBevConfig();
    if (!bev_cfg.node_ns.empty())
        node_ns_ = bev_cfg.node_ns;
    topic_bev_image_  = bev_cfg.bev_image_topic.empty()           ? "/bev_surround_view/bev/fused"              : bev_cfg.bev_image_topic;
    topic_vis_front_  = bev_cfg.detection_vis_front_topic.empty() ? "/bev_surround_view/debug/corners/front"    : bev_cfg.detection_vis_front_topic;
    topic_vis_rear_   = bev_cfg.detection_vis_rear_topic.empty()  ? "/bev_surround_view/debug/corners/back"     : bev_cfg.detection_vis_rear_topic;
    topic_vis_left_   = bev_cfg.detection_vis_left_topic.empty()  ? "/bev_surround_view/debug/corners/left"     : bev_cfg.detection_vis_left_topic;
    topic_vis_right_  = bev_cfg.detection_vis_right_topic.empty() ? "/bev_surround_view/debug/corners/right"    : bev_cfg.detection_vis_right_topic;

    // 从配置初始化 BEV 参数 spinbox
    spin_resolution_->setValue(bev_cfg.resolution);
    spin_range_front_->setValue(bev_cfg.range_front);
    spin_range_rear_->setValue(bev_cfg.range_rear);
    spin_range_left_->setValue(bev_cfg.range_left);
    spin_range_right_->setValue(bev_cfg.range_right);
    spin_origin_offset_x_->setValue(bev_cfg.origin_offset_x);
    spin_origin_offset_y_->setValue(bev_cfg.origin_offset_y);

    label_bev_info_->setText(
        QString("输出: %1x%2 px\n"
                "BEV: %3\n"
                "前: %4\n"
                "后: %5\n"
                "左: %6\n"
                "右: %7")
        .arg(bev_cfg.output_width).arg(bev_cfg.output_height)
        .arg(QString::fromStdString(topic_bev_image_))
        .arg(QString::fromStdString(topic_vis_front_))
        .arg(QString::fromStdString(topic_vis_rear_))
        .arg(QString::fromStdString(topic_vis_left_))
        .arg(QString::fromStdString(topic_vis_right_)));

    btn_start_->setEnabled(true);
    btn_save_->setEnabled(true);
    btn_reset_->setEnabled(true);
}

void TabBev::onCameraSelected(int cam_idx, const QString& name)
{
    if (name.isEmpty() || name.startsWith("--")) return;
    double x, y, z, roll, pitch, yaw;
    if (cfg_->getCameraInstall(name.toStdString(), x, y, z, roll, pitch, yaw)) {
        cam_params_[cam_idx].install_label->setText(
            QString("x=%1  y=%2  z=%3\nr=%4°  p=%5°  y=%6°")
            .arg(x,0,'f',3).arg(y,0,'f',3).arg(z,0,'f',3)
            .arg(roll,0,'f',2).arg(pitch,0,'f',2).arg(yaw,0,'f',2));
    }
}

void TabBev::drawRoiLines(cv::Mat& img)
{
    if (img.empty()) return;
    int rf = static_cast<int>(spin_range_front_->value());
    int rr = static_cast<int>(spin_range_rear_->value());
    int rl = static_cast<int>(spin_range_left_->value());
    int rright = static_cast<int>(spin_range_right_->value());
    int w = img.cols;
    int h = img.rows;

    cv::Scalar color(0, 220, 220);
    int thick = 2;
    // 前向 ROI 线（水平）：距图像顶部 range_front 像素
    cv::line(img, {0, rf}, {w-1, rf}, color, thick);
    // 后向 ROI 线（水平）：距图像顶部 range_rear 像素（从上往下数）
    cv::line(img, {0, rr}, {w-1, rr}, color, thick);
    // 左向 ROI 线（垂直）：距图像左侧 range_left 像素
    cv::line(img, {rl, 0}, {rl, h-1}, color, thick);
    // 右向 ROI 线（垂直）：距图像左侧 range_right 像素（从左往右数）
    cv::line(img, {rright, 0}, {rright, h-1}, color, thick);
}

void TabBev::onBevImageReceived(cv::Mat img)
{
    if (!viz_enabled_) return;
    label_data_status_->setText(
        QString("BEV %1x%2").arg(img.cols).arg(img.rows));
    cv::Mat display = img.clone();
    drawRoiLines(display);
    viewer_bev_->showImage(display);
}

void TabBev::onDetVisReceived(int cam_idx, cv::Mat img)
{
    if (!viz_enabled_) return;
    ImageViewer* viewers[] = {viewer_front_, viewer_rear_, viewer_left_, viewer_right_};
    if (cam_idx >= 0 && cam_idx < 4)
        viewers[cam_idx]->showImage(img);
}

void TabBev::onStatusReceived(const std::string& msg)
{
    label_cali_status_->setText(QString::fromStdString(msg));
}

void TabBev::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        if (!node_ns_.empty()) {
            ros_->unsubscribeAll();
            ros_->subscribeImage(topic_bev_image_,
                [this](cv::Mat img) { onBevImageReceived(img); });
            ros_->subscribeStatus(node_ns_ + "/status",
                [this](const std::string& msg) { onStatusReceived(msg); });
            const std::string vis_topics[] = {
                topic_vis_front_, topic_vis_rear_,
                topic_vis_left_,  topic_vis_right_
            };
            for (int i = 0; i < 4; ++i) {
                int idx = i;
                ros_->subscribeImage(vis_topics[i],
                    [this, idx](cv::Mat img) { onDetVisReceived(idx, img); });
            }
            label_data_status_->setText("订阅中...");
        }
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        viewer_bev_->clear();
        viewer_front_->clear(); viewer_rear_->clear();
        viewer_left_->clear();  viewer_right_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabBev::onStartCalib()
{
    std::string msg;
    bool ok = ros_->callTrigger(node_ns_ + "/trigger_calibration", msg);
    label_cali_status_->setText(
        QString(ok ? "[触发成功] " : "[触发失败] ") + QString::fromStdString(msg));
}

void TabBev::onSaveCalib()
{
    std::string msg;
    bool ok = ros_->callSave(node_ns_ + "/save_calibration", msg);
    if (ok) {
        const std::string proc = cfg_->processDir();
        QString proc_file;
        if (!proc.empty()) {
            QDir().mkpath(QString::fromStdString(proc));
            proc_file = QString("%1/")
                .arg(QString::fromStdString(proc));
            // Write a summary marker so process dir has a record
            // std::ofstream f(proc_file.toStdString());
            // if (f.is_open())
            //     f << "bev_calibration_saved: true\n"
            //       << "node_save_msg: " << msg << "\n";
        }
        QString detail = proc_file.isEmpty()
            ? "" : QString("\n已写入标记: %1").arg(proc_file);
        QMessageBox::information(this, "保存成功",
            QString::fromStdString(msg) + detail);
    } else {
        QMessageBox::warning(this, "保存失败", QString::fromStdString(msg));
    }
}

void TabBev::onResetCalib()
{
    std::string msg;
    ros_->callReset(node_ns_ + "/reset_calibration", msg);
    viewer_bev_->clear();
    label_cali_status_->setText("已重置");
    label_result_->setText("—");
}

void TabBev::onBevParamChanged(double /*v*/)
{
    if (node_ns_.empty()) return;
    ros_->setParam(node_ns_ + "/bev/resolution",      spin_resolution_->value());
    ros_->setParam(node_ns_ + "/bev/range_front",    spin_range_front_->value());
    ros_->setParam(node_ns_ + "/bev/range_rear",     spin_range_rear_->value());
    ros_->setParam(node_ns_ + "/bev/range_left",     spin_range_left_->value());
    ros_->setParam(node_ns_ + "/bev/range_right",    spin_range_right_->value());
    ros_->setParam(node_ns_ + "/bev/origin_offset_x", spin_origin_offset_x_->value());
    ros_->setParam(node_ns_ + "/bev/origin_offset_y", spin_origin_offset_y_->value());
}

void TabBev::onStartNode()
{
    QStringList args;
    QComboBox* combos[] = {combo_front_, combo_rear_, combo_left_, combo_right_};
    const char* pos[]   = {"front", "rear", "left", "right"};

    for (int i = 0; i < 4; ++i) {
        const std::string cam_name = combos[i]->currentText().toStdString();
        if (cam_name.empty() || cam_name.find("--") != std::string::npos) continue;

        // topic
        const auto& cfg = cfg_->getConfig();
        for (const auto& c : cfg.cameras) {
            if (c.name == cam_name) {
                args << QString("%1_image_topic:=%2")
                        .arg(pos[i]).arg(QString::fromStdString(c.topic));
                break;
            }
        }
        // intrinsics file
        const std::string intr_path = cfg_->getCameraIntrinsicsPath(cam_name);
        if (!intr_path.empty())
            args << QString("%1_intrinsics_file:=%2")
                    .arg(pos[i]).arg(QString::fromStdString(intr_path));

        // installation
        double x, y, z, roll, pitch, yaw;
        if (cfg_->getCameraInstall(cam_name, x, y, z, roll, pitch, yaw)) {
            args << QString("inst_%1_x:=%2").arg(pos[i]).arg(x)
                 << QString("inst_%1_y:=%2").arg(pos[i]).arg(y)
                 << QString("inst_%1_z:=%2").arg(pos[i]).arg(z)
                 << QString("inst_%1_roll:=%2").arg(pos[i]).arg(roll)
                 << QString("inst_%1_pitch:=%2").arg(pos[i]).arg(pitch)
                 << QString("inst_%1_yaw:=%2").arg(pos[i]).arg(yaw);
        }
    }

    // BEV params
    args << QString("bev_resolution:=%1").arg(spin_resolution_->value())
         << QString("bev_range_front:=%1").arg(spin_range_front_->value())
         << QString("bev_range_rear:=%1").arg(spin_range_rear_->value())
         << QString("bev_range_left:=%1").arg(spin_range_left_->value())
         << QString("bev_range_right:=%1").arg(spin_range_right_->value())
         << QString("bev_origin_offset_x:=%1").arg(spin_origin_offset_x_->value())
         << QString("bev_origin_offset_y:=%1").arg(spin_origin_offset_y_->value())
         << QString("vcs_z_offset:=%1").arg(cfg_->zOffset());

    // output resolution and publish topics from vehicle config
    const auto& bev_cfg = cfg_->getBevConfig();
    args << QString("output_width:=%1").arg(bev_cfg.output_width)
         << QString("output_height:=%1").arg(bev_cfg.output_height);

    // keep node publish topics consistent with vehicle_sample.yaml
    if (!bev_cfg.bev_image_topic.empty())
        args << QString("bev_fused_topic:=%1").arg(QString::fromStdString(bev_cfg.bev_image_topic));
    if (!bev_cfg.detection_vis_front_topic.empty())
        args << QString("corners_front_topic:=%1").arg(QString::fromStdString(bev_cfg.detection_vis_front_topic));
    if (!bev_cfg.detection_vis_rear_topic.empty())
        args << QString("corners_back_topic:=%1").arg(QString::fromStdString(bev_cfg.detection_vis_rear_topic));
    if (!bev_cfg.detection_vis_left_topic.empty())
        args << QString("corners_left_topic:=%1").arg(QString::fromStdString(bev_cfg.detection_vis_left_topic));
    if (!bev_cfg.detection_vis_right_topic.empty())
        args << QString("corners_right_topic:=%1").arg(QString::fromStdString(bev_cfg.detection_vis_right_topic));

    // save path — map to the four output_*_yaml_path args declared in the launch file
    const std::string proc_dir = cfg_->processDir();
    if (!proc_dir.empty()) {
        QDir().mkpath(QString::fromStdString(proc_dir));
        const QString qdir = QString::fromStdString(proc_dir);
        args << QString("output_front_yaml_path:=%1/front_fish_eye.yaml").arg(qdir)
             << QString("output_rear_yaml_path:=%1/back_fish_eye.yaml").arg(qdir)
             << QString("output_left_yaml_path:=%1/left_fish_eye.yaml").arg(qdir)
             << QString("output_right_yaml_path:=%1/right_fish_eye.yaml").arg(qdir)
             << QString("output_front_bin_path:=%1/data_front_fish_eye.bin").arg(qdir)
             << QString("output_rear_bin_path:=%1/data_back_fish_eye.bin").arg(qdir)
             << QString("output_left_bin_path:=%1/data_left_fish_eye.bin").arg(qdir)
             << QString("output_right_bin_path:=%1/data_right_fish_eye.bin").arg(qdir);
    }

    // 打印完整 roslaunch 命令，方便手动调试
    QString cmd = "roslaunch bev_surround_view bev_surround_view_gui.launch";
    for (const auto& a : args) cmd += " " + a;
    ROS_INFO("[TabBev] launch cmd:\n%s", cmd.toStdString().c_str());

    bool ok = ros_->launchNode("bev_surround_view",
                               "bev_surround_view_gui.launch",
                               kNodeKey, args);
    if (!ok) {
        label_node_status_->setText("● 启动失败");
        label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    }
}

void TabBev::onStopNode()
{
    ros_->killNode(kNodeKey);
}

void TabBev::onNodeStarted(const QString& key)
{
    if (key != QString::fromStdString(kNodeKey)) return;
    label_node_status_->setText("● 运行中");
    label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(true);
}

void TabBev::onNodeStopped(const QString& key)
{
    if (key != QString::fromStdString(kNodeKey)) return;
    label_node_status_->setText("● 未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_start_node_->setEnabled(true);
    btn_stop_node_->setEnabled(false);
}

} // namespace calibration_gui
