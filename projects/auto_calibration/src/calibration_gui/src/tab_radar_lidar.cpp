#include "calibration_gui/tab_radar_lidar.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QDir>
#include <QScrollArea>
#include <QFrame>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include "lidar_shm_param.h"

namespace calibration_gui {

static const std::string kRadarParamNs = "/radar_lidar_cali";

namespace {

struct RadarLidarProcess {
    bool has{false};
    double x{0}, y{0}, z{0};
    double roll_deg{0}, pitch_deg{0}, yaw_deg{0};
};

// 读取 process/radar_lidar_<radar>.yaml 的 calibrated 段
bool readRadarLidarProcess(const std::string& proc_dir,
                           const std::string& radar,
                           RadarLidarProcess& out)
{
    out = RadarLidarProcess{};
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

bool lidarIsCalibrated(const std::shared_ptr<ConfigManager>& cfg,
                       const std::string& lidar)
{
    if (!cfg || lidar.empty()) return false;
    ConfigManager::LidarMergedParams p;
    if (!cfg->getMergedLidarParams(lidar, p)) return false;
    return p.x_from_process || p.y_from_process || p.z_from_process ||
           p.roll_from_process || p.pitch_from_process || p.yaw_from_process;
}

} // namespace

TabRadarLidar::TabRadarLidar(std::shared_ptr<ConfigManager> cfg,
                              std::shared_ptr<RosWorker> ros,
                              QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
}

void TabRadarLidar::setupUI()
{
    auto* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 12, 12, 12);
    h->setSpacing(12);

    // ---- 左侧控制面板 ----
    auto* left = new QWidget(this);
    left->setFixedWidth(360);
    auto* vl = new QVBoxLayout(left);
    vl->setContentsMargins(0, 0, 0, 0);
    vl->setSpacing(10);

    // ---- 可视化控制 ----
    auto* grp_viz = new QGroupBox("可视化控制", this);
    auto* vl_viz = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", this);
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabRadarLidar::onVizToggle);
    vl_viz->addWidget(btn_viz_toggle_);
    label_data_status_ = new QLabel("未订阅", this);
    label_data_status_->setWordWrap(true);
    vl_viz->addWidget(label_data_status_);
    vl->addWidget(grp_viz);

    // ---- 传感器选择 ----
    auto* grp_sensor = new QGroupBox("传感器选择", this);
    auto* fl_s = new QFormLayout(grp_sensor);
    combo_lidar_ = new QComboBox(this);
    combo_radar_ = new QComboBox(this);
    combo_lidar_->addItem("请选择", QString());
    combo_radar_->addItem("请选择", QString());
    connect(combo_lidar_, &QComboBox::currentTextChanged,
            this, &TabRadarLidar::onLidarSelected);
    connect(combo_radar_, &QComboBox::currentTextChanged,
            this, &TabRadarLidar::onRadarSelected);
    fl_s->addRow("参考激光雷达：", combo_lidar_);
    fl_s->addRow("目标毫米波雷达：", combo_radar_);

    label_lidar_install_ = new QLabel("—", this);
    label_lidar_install_->setWordWrap(true);
    label_lidar_install_->setStyleSheet("QLabel{color:#8af;}");
    fl_s->addRow("激光雷达安装：", label_lidar_install_);

    label_radar_install_ = new QLabel("—", this);
    label_radar_install_->setWordWrap(true);
    label_radar_install_->setStyleSheet("QLabel{color:#fa8;}");
    fl_s->addRow("毫米波安装：", label_radar_install_);
    vl->addWidget(grp_sensor);

    // ---- 6DOF 参数微调 ----
    auto* grp_tune = new QGroupBox("毫米波雷达安装参数微调（实时写入 rosparam）", this);
    auto* vl_tune_outer = new QVBoxLayout(grp_tune);
    vl_tune_outer->setContentsMargins(8, 8, 8, 8);
    vl_tune_outer->setSpacing(6);
    label_params_source_ = new QLabel("当前参数来源：—", this);
    label_params_source_->setWordWrap(true);
    label_params_source_->setStyleSheet("QLabel{color:#888;}");
    vl_tune_outer->addWidget(label_params_source_);
    auto* form_host = new QWidget(grp_tune);
    auto* fl_t = new QFormLayout(form_host);
    fl_t->setContentsMargins(0, 0, 0, 0);
    fl_t->setSpacing(6);
    vl_tune_outer->addWidget(form_host);

    spin_x_ = new QDoubleSpinBox(this);
    spin_x_->setRange(-20.0, 20.0); spin_x_->setSingleStep(0.01);
    spin_x_->setDecimals(3); spin_x_->setSuffix(" m");
    connect(spin_x_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabRadarLidar::onXChanged);
    fl_t->addRow("X：", spin_x_);

    spin_y_ = new QDoubleSpinBox(this);
    spin_y_->setRange(-20.0, 20.0); spin_y_->setSingleStep(0.01);
    spin_y_->setDecimals(3); spin_y_->setSuffix(" m");
    connect(spin_y_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabRadarLidar::onYChanged);
    fl_t->addRow("Y：", spin_y_);

    spin_z_ = new QDoubleSpinBox(this);
    spin_z_->setRange(-10.0, 10.0); spin_z_->setSingleStep(0.01);
    spin_z_->setDecimals(3); spin_z_->setSuffix(" m");
    connect(spin_z_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabRadarLidar::onZChanged);
    fl_t->addRow("Z：", spin_z_);

    spin_roll_ = new QDoubleSpinBox(this);
    spin_roll_->setRange(-180.0, 180.0); spin_roll_->setSingleStep(0.1);
    spin_roll_->setDecimals(3); spin_roll_->setSuffix(" °");
    connect(spin_roll_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabRadarLidar::onRollChanged);
    fl_t->addRow("Roll：", spin_roll_);

    spin_pitch_ = new QDoubleSpinBox(this);
    spin_pitch_->setRange(-180.0, 180.0); spin_pitch_->setSingleStep(0.1);
    spin_pitch_->setDecimals(3); spin_pitch_->setSuffix(" °");
    connect(spin_pitch_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabRadarLidar::onPitchChanged);
    fl_t->addRow("Pitch：", spin_pitch_);

    spin_yaw_ = new QDoubleSpinBox(this);
    spin_yaw_->setRange(-180.0, 180.0); spin_yaw_->setSingleStep(0.1);
    spin_yaw_->setDecimals(3); spin_yaw_->setSuffix(" °");
    connect(spin_yaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &TabRadarLidar::onYawChanged);
    fl_t->addRow("Yaw：", spin_yaw_);

    vl->addWidget(grp_tune);

    // ---- 操作按钮 ----
    auto* grp_btn = new QGroupBox("操作", this);
    auto* vl_b = new QVBoxLayout(grp_btn);
    vl_b->setSpacing(8); vl_b->setContentsMargins(8, 8, 8, 8);
    btn_save_  = new QPushButton("保存标定结果到 YAML", this);
    btn_reset_ = new QPushButton("重置为配置文件初始值", this);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    connect(btn_save_,  &QPushButton::clicked, this, &TabRadarLidar::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabRadarLidar::onResetParams);
    vl_b->addWidget(btn_save_);
    vl_b->addWidget(btn_reset_);
    vl->addWidget(grp_btn);

    // ---- 使用提示 ----
    auto* lbl_hint = new QLabel(
        "<small><b>标定流程：</b><br>"
        "1. 加载车辆配置，选择参考激光雷达与目标毫米波雷达<br>"
        "2. 开启可视化，观察两传感器点云叠加效果<br>"
        "   <span style='color:#8af;'>■</span> 蓝色小点 = 激光雷达 &nbsp;"
        "   <span style='color:#fa8;'>■</span> 橙色大点 = 毫米波雷达<br>"
        "3. 调整 X/Y/Z/Roll/Pitch/Yaw 使点云对齐<br>"
        "   参数实时写入 rosparam，可视化即时更新<br>"
        "4. 对齐满意后点击[保存]写入 YAML</small>", this);
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

    // ---- 右侧：点云视图 ----
    auto* right_box = new QGroupBox(
        "点云叠加视图（蓝色小点=激光雷达，橙色大点=毫米波雷达）", this);
    auto* rl = new QVBoxLayout(right_box);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    viewer_ = new PointCloudViewer(this);
    rl->addWidget(viewer_);
    h->addWidget(right_box, 1);
}

void TabRadarLidar::deactivate()
{
    if (!prev_lidar_rl_.empty()) {
        ros_->killNode("driver_lidar_" + prev_lidar_rl_);
        prev_lidar_rl_.clear();
    }
    if (!prev_radar_.empty()) {
        ros_->killNode("driver_radar_" + prev_radar_);
        prev_radar_.clear();
    }
    combo_lidar_->blockSignals(true);
    combo_radar_->blockSignals(true);
    combo_lidar_->setCurrentIndex(0);
    combo_radar_->setCurrentIndex(0);
    combo_lidar_->blockSignals(false);
    combo_radar_->blockSignals(false);
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    label_params_source_->setText("当前参数来源：—");
    label_params_source_->setStyleSheet("QLabel{color:#888;}");
}

void TabRadarLidar::applyConfig()
{
    if (!cfg_->isLoaded()) return;

    combo_lidar_->blockSignals(true);
    combo_radar_->blockSignals(true);
    combo_lidar_->clear();
    combo_radar_->clear();
    combo_lidar_->addItem("请选择", QString());
    combo_radar_->addItem("请选择", QString());

    for (const auto& n : cfg_->lidarNames())
        combo_lidar_->addItem(QString::fromStdString(n));
    for (const auto& n : cfg_->radarNames())
        combo_radar_->addItem(QString::fromStdString(n));

    combo_lidar_->blockSignals(false);
    combo_radar_->blockSignals(false);

    current_lidar_.clear();
    current_radar_.clear();
    prev_lidar_rl_.clear();
    prev_radar_.clear();
    label_lidar_install_->setText("—");
    label_radar_install_->setText("—");
    btn_save_->setEnabled(false);
    btn_reset_->setEnabled(false);
    label_params_source_->setText("当前参数来源：—");
    label_params_source_->setStyleSheet("QLabel{color:#888;}");
}

void TabRadarLidar::onLidarSelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_lidar_rl_.empty()) {
            ros_->killNode("driver_lidar_" + prev_lidar_rl_);
            prev_lidar_rl_.clear();
        }
        current_lidar_.clear();
        label_lidar_install_->setText("—");
        btn_save_->setEnabled(false);
        btn_reset_->setEnabled(false);
        return;
    }

    const std::string new_lidar = name.toStdString();

    // 参照激光雷达必须已经标定到 base_link，否则提示并重置选择
    if (!lidarIsCalibrated(cfg_, new_lidar)) {
        QMessageBox::warning(this, "参照激光雷达未标定",
            QString("激光雷达 \"%1\" 在 process/ 中没有标定结果。\n"
                    "请先完成该激光雷达的外参标定（lidar_rp / secondary_rp / lidar_yaw / lidar_lidar）后再使用本页。")
            .arg(name));
        if (!prev_lidar_rl_.empty()) {
            ros_->killNode("driver_lidar_" + prev_lidar_rl_);
            prev_lidar_rl_.clear();
        }
        current_lidar_.clear();
        label_lidar_install_->setText("—");
        btn_save_->setEnabled(false);
        btn_reset_->setEnabled(false);
        combo_lidar_->blockSignals(true);
        combo_lidar_->setCurrentIndex(0);
        combo_lidar_->blockSignals(false);
        return;
    }

    // 停止上一个激光雷达 driver（如果切换到不同的雷达）
    if (!prev_lidar_rl_.empty() && prev_lidar_rl_ != new_lidar) {
        ros_->killNode("driver_lidar_" + prev_lidar_rl_);
    }

    prev_lidar_rl_ = new_lidar;
    current_lidar_ = new_lidar;

    // 使用合并后的激光雷达参数（安装值 + process 标定结果）
    ConfigManager::LidarMergedParams merged;
    const bool has_merged = cfg_->getMergedLidarParams(current_lidar_, merged);
    if (has_merged) {
        label_lidar_install_->setText(
            QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°")
            .arg(merged.x,0,'f',3).arg(merged.y,0,'f',3).arg(merged.z,0,'f',3)
            .arg(merged.roll_deg,0,'f',2).arg(merged.pitch_deg,0,'f',2).arg(merged.yaw_deg,0,'f',2));
    }

    // 启动新激光雷达 driver（如果配置了 launch_file）
    const std::string launch_path = cfg_->getLidarLaunchFilePath(current_lidar_);
    if (!launch_path.empty()) {
        const std::string packets_topic = cfg_->getLidarPacketsTopic(current_lidar_);
        const std::string point_cloud_topic = cfg_->getLidarPointCloudTopic(current_lidar_);
        QStringList driver_args;
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        if (has_merged) {
            driver_args << QString("xoffset:=%1").arg(merged.x)
                        << QString("yoffset:=%1").arg(merged.y)
                        << QString("zoffset:=%1").arg(merged.z)
                        << QString("roll:=%1").arg(merged.roll_deg)
                        << QString("pitch:=%1").arg(merged.pitch_deg)
                        << QString("yaw:=%1").arg(merged.yaw_deg)
                        << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        }
        QString cmd = QString("roslaunch %1").arg(QString::fromStdString(launch_path));
        for (const auto& a : driver_args) cmd += " " + a;
        ROS_INFO("[TabRadarLidar] lidar driver launch cmd:\n%s", cmd.toStdString().c_str());
        ros_->launchNodeByPath(launch_path, "driver_lidar_" + current_lidar_, driver_args);
    }

    if (viz_enabled_) resubscribe();
}

void TabRadarLidar::onRadarSelected(const QString& name)
{
    if (name.isEmpty() || name == "请选择" || name.startsWith("--")) {
        if (!prev_radar_.empty()) {
            ros_->killNode("driver_radar_" + prev_radar_);
            prev_radar_.clear();
        }
        current_radar_.clear();
        label_radar_install_->setText("—");
        btn_save_->setEnabled(false);
        btn_reset_->setEnabled(false);
        label_params_source_->setText("当前参数来源：—");
        label_params_source_->setStyleSheet("QLabel{color:#888;}");
        return;
    }

    const std::string new_radar = name.toStdString();

    // 停止上一个毫米波雷达 driver（如果切换到不同的雷达）
    if (!prev_radar_.empty() && prev_radar_ != new_radar) {
        ros_->killNode("driver_radar_" + prev_radar_);
    }

    prev_radar_    = new_radar;
    current_radar_ = new_radar;

    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
    const bool has_install = cfg_->getRadarInstall(current_radar_, x, y, z, roll, pitch, yaw);

    // 优先加载 process/radar_lidar_<radar>.yaml 的 calibrated 段
    RadarLidarProcess proc_params;
    const bool has_process =
        readRadarLidarProcess(cfg_->processDir(), current_radar_, proc_params);

    double init_x = x, init_y = y, init_z = z;
    double init_roll = roll, init_pitch = pitch, init_yaw = yaw;
    if (has_process) {
        init_x     = proc_params.x;
        init_y     = proc_params.y;
        init_z     = proc_params.z;
        init_roll  = proc_params.roll_deg;
        init_pitch = proc_params.pitch_deg;
        init_yaw   = proc_params.yaw_deg;
    }

    if (has_install || has_process) {
        QString tag = has_process ? "当前参数来源：process 已标定结果（radar_lidar_<radar>.yaml）"
                                   : "当前参数来源：车辆配置安装值（vehicle.yaml）";
        QString color = has_process ? "#4f4" : "#fa8";
        label_params_source_->setText(tag);
        label_params_source_->setStyleSheet(QString("QLabel{color:%1;}").arg(color));

        QString label_tag = has_process ? "（已读取 process 标定结果）" : "（来自车辆配置安装值）";
        label_radar_install_->setText(
            QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°\n%7")
            .arg(init_x,0,'f',3).arg(init_y,0,'f',3).arg(init_z,0,'f',3)
            .arg(init_roll,0,'f',2).arg(init_pitch,0,'f',2).arg(init_yaw,0,'f',2)
            .arg(label_tag));
        // 加载初始值到 spinbox
        spin_x_->blockSignals(true);     spin_x_->setValue(init_x);         spin_x_->blockSignals(false);
        spin_y_->blockSignals(true);     spin_y_->setValue(init_y);         spin_y_->blockSignals(false);
        spin_z_->blockSignals(true);     spin_z_->setValue(init_z);         spin_z_->blockSignals(false);
        spin_roll_->blockSignals(true);  spin_roll_->setValue(init_roll);   spin_roll_->blockSignals(false);
        spin_pitch_->blockSignals(true); spin_pitch_->setValue(init_pitch); spin_pitch_->blockSignals(false);
        spin_yaw_->blockSignals(true);   spin_yaw_->setValue(init_yaw);     spin_yaw_->blockSignals(false);
        // 写入 rosparam 初始值
        ros_->setParam(kRadarParamNs + "/x_install",     init_x);
        ros_->setParam(kRadarParamNs + "/y_install",     init_y);
        ros_->setParam(kRadarParamNs + "/z_install",     init_z);
        ros_->setParam(kRadarParamNs + "/roll_install",  init_roll);
        ros_->setParam(kRadarParamNs + "/pitch_install", init_pitch);
        ros_->setParam(kRadarParamNs + "/yaw_install",   init_yaw);
        // 同步写入共享内存
        LidarShmParam::SetModuleParamSimple(current_radar_,
            init_x, init_y, init_z, init_roll, init_pitch, init_yaw);

        // 选中毫米波且参考激光雷达已通过校验，允许保存/重置
        btn_save_->setEnabled(true);
        btn_reset_->setEnabled(true);
    }

    // 启动新毫米波雷达 driver（如果配置了 launch_file）
    const std::string launch_path = cfg_->getRadarLaunchFilePath(current_radar_);
    if (!launch_path.empty()) {
        const std::string packets_topic     = cfg_->getRadarPacketsTopic(current_radar_);
        const std::string point_cloud_topic = cfg_->getRadarPointCloudTopic(current_radar_);
        QStringList driver_args;
        if (!packets_topic.empty())
            driver_args << QString("packets_topic:=%1").arg(QString::fromStdString(packets_topic));
        if (!point_cloud_topic.empty())
            driver_args << QString("point_cloud_topic:=%1").arg(QString::fromStdString(point_cloud_topic));
        if (has_install || has_process) {
            driver_args << QString("xoffset:=%1").arg(init_x)
                        << QString("yoffset:=%1").arg(init_y)
                        << QString("zoffset:=%1").arg(init_z)
                        << QString("roll:=%1").arg(init_roll)
                        << QString("pitch:=%1").arg(init_pitch)
                        << QString("yaw:=%1").arg(init_yaw)
                        << QString("top_z_ground_offset:=%1").arg(cfg_->zOffset());
        }
        QString cmd = QString("roslaunch %1").arg(QString::fromStdString(launch_path));
        for (const auto& a : driver_args) cmd += " " + a;
        ROS_INFO("[TabRadarLidar] radar driver launch cmd:\n%s", cmd.toStdString().c_str());
        ros_->launchNodeByPath(launch_path, "driver_radar_" + current_radar_, driver_args);
    }

    if (viz_enabled_) resubscribe();
}

void TabRadarLidar::onXChanged(double v)
{
    ros_->setParam(kRadarParamNs + "/x_install", v);
    writeRadarShm();
}

void TabRadarLidar::onYChanged(double v)
{
    ros_->setParam(kRadarParamNs + "/y_install", v);
    writeRadarShm();
}

void TabRadarLidar::onZChanged(double v)
{
    ros_->setParam(kRadarParamNs + "/z_install", v);
    writeRadarShm();
}

void TabRadarLidar::onRollChanged(double v)
{
    ros_->setParam(kRadarParamNs + "/roll_install", v);
    writeRadarShm();
}

void TabRadarLidar::onPitchChanged(double v)
{
    ros_->setParam(kRadarParamNs + "/pitch_install", v);
    writeRadarShm();
}

void TabRadarLidar::onYawChanged(double v)
{
    ros_->setParam(kRadarParamNs + "/yaw_install", v);
    writeRadarShm();
}

void TabRadarLidar::writeRadarShm()
{
    if (current_radar_.empty()) return;
    LidarShmParam::SetModuleParamSimple(current_radar_,
        spin_x_->value(), spin_y_->value(), spin_z_->value(),
        spin_roll_->value(), spin_pitch_->value(), spin_yaw_->value());
}

void TabRadarLidar::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        resubscribe();
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        cloud_lidar_.reset();
        cloud_radar_.reset();
        viewer_->showLoading(false);
        viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabRadarLidar::resubscribe()
{
    ros_->unsubscribeAll();
    cloud_lidar_.reset();
    cloud_radar_.reset();
    viewer_->clear();
    viewer_->showLoading(true);
    label_data_status_->setText("订阅中，等待数据...");

    // 订阅激光雷达原始 topic
    const auto& cfg = cfg_->getConfig();
    std::string lidar_topic, radar_topic;
    for (const auto& l : cfg.lidars) {
        if (l.name == current_lidar_) { lidar_topic = l.topic; break; }
    }
    for (const auto& r : cfg.radars) {
        if (r.name == current_radar_) { radar_topic = r.topic; break; }
    }

    if (!lidar_topic.empty()) {
        ros_->subscribeCloud(lidar_topic,
            [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                onLidarCloudReceived(cloud);
            });
    }
    if (!radar_topic.empty()) {
        ros_->subscribeCloud(radar_topic,
            [this](pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
                onRadarCloudReceived(cloud);
            });
    }
}

void TabRadarLidar::refreshViewer()
{
    viewer_->clear(false);
    // 激光雷达：蓝色，小点 (2px)
    if (cloud_lidar_ && !cloud_lidar_->empty())
        viewer_->addCloud(cloud_lidar_, 0.3, 0.6, 1.0, 2.0);
    // 毫米波雷达：橙色，大点 (8px)
    if (cloud_radar_ && !cloud_radar_->empty())
        viewer_->addCloud(cloud_radar_, 1.0, 0.6, 0.1, 8.0);
    viewer_->renderNow();
}

void TabRadarLidar::onLidarCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    viewer_->showLoading(false);
    cloud_lidar_ = cloud;
    label_data_status_->setText(
        QString("激光雷达点: %1  毫米波点: %2")
        .arg(static_cast<int>(cloud_lidar_ ? cloud_lidar_->size() : 0))
        .arg(static_cast<int>(cloud_radar_ ? cloud_radar_->size() : 0)));
    refreshViewer();
}

void TabRadarLidar::onRadarCloudReceived(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (!viz_enabled_) return;
    viewer_->showLoading(false);
    cloud_radar_ = cloud;
    label_data_status_->setText(
        QString("激光雷达点: %1  毫米波点: %2")
        .arg(static_cast<int>(cloud_lidar_ ? cloud_lidar_->size() : 0))
        .arg(static_cast<int>(cloud_radar_ ? cloud_radar_->size() : 0)));
    refreshViewer();
}

void TabRadarLidar::onSaveCalib()
{
    if (current_radar_.empty()) {
        QMessageBox::warning(this, "保存失败", "请先选择毫米波雷达。");
        return;
    }
    const double x     = spin_x_->value();
    const double y     = spin_y_->value();
    const double z     = spin_z_->value();
    const double roll  = spin_roll_->value();
    const double pitch = spin_pitch_->value();
    const double yaw   = spin_yaw_->value();

    // 读取配置中的安装初值用于 delta 计算
    double inst_x = 0, inst_y = 0, inst_z = 0;
    double inst_roll = 0, inst_pitch = 0, inst_yaw = 0;
    const bool has_install =
        cfg_->getRadarInstall(current_radar_, inst_x, inst_y, inst_z,
                              inst_roll, inst_pitch, inst_yaw);

    const std::string proc = cfg_->processDir();
    if (proc.empty()) {
        QMessageBox::warning(this, "保存失败", "未加载车辆配置，无法确定 process 目录。");
        return;
    }
    QDir().mkpath(QString::fromStdString(proc));
    const QString proc_file = QString("%1/radar_lidar_%2.yaml")
        .arg(QString::fromStdString(proc))
        .arg(QString::fromStdString(current_radar_));

    std::ofstream f(proc_file.toStdString());
    if (!f.is_open()) {
        QMessageBox::warning(this, "保存失败",
            QString("无法写入：%1").arg(proc_file));
        return;
    }

    std::time_t t = std::time(nullptr);
    char ts_buf[32];
    std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    f << std::fixed << std::setprecision(6);
    f << "schema_version: 1\n";
    f << "calib_type: radar_lidar\n";
    f << "sensor_name: "      << current_radar_ << "\n";
    f << "reference_lidar: "  << current_lidar_ << "\n";
    f << "generated_at: "     << ts_buf << "\n";
    f << "method: manual_alignment\n";
    f << "coordinate_system: FLU\n";
    f << "installation:\n";
    f << "  x: "         << (has_install ? inst_x     : 0.0) << "\n";
    f << "  y: "         << (has_install ? inst_y     : 0.0) << "\n";
    f << "  z: "         << (has_install ? inst_z     : 0.0) << "\n";
    f << "  roll_deg: "  << (has_install ? inst_roll  : 0.0) << "\n";
    f << "  pitch_deg: " << (has_install ? inst_pitch : 0.0) << "\n";
    f << "  yaw_deg: "   << (has_install ? inst_yaw   : 0.0) << "\n";
    f << "  z_offset: "  << cfg_->zOffset() << "\n";
    f << "calibrated:\n";
    f << "  x: "         << x     << "\n";
    f << "  y: "         << y     << "\n";
    f << "  z: "         << z     << "\n";
    f << "  roll_deg: "  << roll  << "\n";
    f << "  pitch_deg: " << pitch << "\n";
    f << "  yaw_deg: "   << yaw   << "\n";
    if (has_install) {
        f << "delta:\n";
        f << "  delta_x: "         << (x     - inst_x)     << "\n";
        f << "  delta_y: "         << (y     - inst_y)     << "\n";
        f << "  delta_z: "         << (z     - inst_z)     << "\n";
        f << "  delta_roll_deg: "  << (roll  - inst_roll)  << "\n";
        f << "  delta_pitch_deg: " << (pitch - inst_pitch) << "\n";
        f << "  delta_yaw_deg: "   << (yaw   - inst_yaw)   << "\n";
    }
    f.close();

    // 同步共享内存，确保 driver 端 get_shm_cfg 拿到最新值
    LidarShmParam::SetModuleParamSimple(current_radar_, x, y, z, roll, pitch, yaw);

    label_radar_install_->setText(
        QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°\n（已读取 process 标定结果）")
        .arg(x,0,'f',3).arg(y,0,'f',3).arg(z,0,'f',3)
        .arg(roll,0,'f',2).arg(pitch,0,'f',2).arg(yaw,0,'f',2));
    label_params_source_->setText("当前参数来源：process 已标定结果（radar_lidar_<radar>.yaml）");
    label_params_source_->setStyleSheet("QLabel{color:#4f4;}");
    QMessageBox::information(this, "保存成功",
        QString("已写入：%1").arg(proc_file));
}

void TabRadarLidar::onResetParams()
{
    if (current_radar_.empty()) return;
    double x, y, z, roll, pitch, yaw;
    if (cfg_->getRadarInstall(current_radar_, x, y, z, roll, pitch, yaw)) {
        spin_x_->blockSignals(true);     spin_x_->setValue(x);       spin_x_->blockSignals(false);
        spin_y_->blockSignals(true);     spin_y_->setValue(y);       spin_y_->blockSignals(false);
        spin_z_->blockSignals(true);     spin_z_->setValue(z);       spin_z_->blockSignals(false);
        spin_roll_->blockSignals(true);  spin_roll_->setValue(roll);  spin_roll_->blockSignals(false);
        spin_pitch_->blockSignals(true); spin_pitch_->setValue(pitch);spin_pitch_->blockSignals(false);
        spin_yaw_->blockSignals(true);   spin_yaw_->setValue(yaw);    spin_yaw_->blockSignals(false);
        ros_->setParam(kRadarParamNs + "/x_install",     x);
        ros_->setParam(kRadarParamNs + "/y_install",     y);
        ros_->setParam(kRadarParamNs + "/z_install",     z);
        ros_->setParam(kRadarParamNs + "/roll_install",  roll);
        ros_->setParam(kRadarParamNs + "/pitch_install", pitch);
        ros_->setParam(kRadarParamNs + "/yaw_install",   yaw);
        LidarShmParam::SetModuleParamSimple(current_radar_, x, y, z, roll, pitch, yaw);
        label_radar_install_->setText(
            QString("x=%1m  y=%2m  z=%3m\nroll=%4°  pitch=%5°  yaw=%6°\n（来自车辆配置安装值）")
            .arg(x,0,'f',3).arg(y,0,'f',3).arg(z,0,'f',3)
            .arg(roll,0,'f',2).arg(pitch,0,'f',2).arg(yaw,0,'f',2));
        label_params_source_->setText("当前参数来源：车辆配置安装值（vehicle.yaml）");
        label_params_source_->setStyleSheet("QLabel{color:#fa8;}");
    }
}

} // namespace calibration_gui
