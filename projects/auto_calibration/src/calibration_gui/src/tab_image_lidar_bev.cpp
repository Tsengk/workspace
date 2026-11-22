#include "calibration_gui/tab_image_lidar_bev.h"
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

// 前视标定：lidar_camera_projection 包（与其 launch 中 name 一致）
static const std::string kFrontNodeNs     = "/lidar_camera_projection_node";
static const std::string kFrontPkg        = "lidar_camera_projection";
static const std::string kFrontLaunch     = "lidar_camera_projection.launch";
static const std::string kFrontNodeKey    = "lidar_camera_projection";

// BEV 标定：lidar_camera_projection_bev 包（与其 launch 中 name 一致）
static const std::string kBevNodeNs       = "/lidar_camera_projection_bev_node";
static const std::string kBevPkg          = "lidar_camera_projection_bev";
static const std::string kBevLaunch       = "lidar_camera_projection_bev.launch";
static const std::string kBevNodeKey      = "lidar_camera_projection_bev";

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

TabImageLidarBev::TabImageLidarBev(std::shared_ptr<ConfigManager> cfg,
                             std::shared_ptr<RosWorker> ros,
                             QWidget* parent)
    : QWidget(parent), cfg_(cfg), ros_(ros)
{
    setupUI();
    // BEV 相机服务初始化失败不影响前视标定使用，仅禁用 BEV 视角渲染
    updateModeUI();
    connect(ros_.get(), &RosWorker::nodeStarted, this, &TabImageLidarBev::onNodeStarted);
    connect(ros_.get(), &RosWorker::nodeStopped, this, &TabImageLidarBev::onNodeStopped);
}

std::string TabImageLidarBev::nodeKey() const
{
    return mode_ == CalibMode::Bev ? kBevNodeKey : kFrontNodeKey;
}

std::string TabImageLidarBev::nodeNs() const
{
    return mode_ == CalibMode::Bev ? kBevNodeNs : kFrontNodeNs;
}

std::string TabImageLidarBev::projectionPkg() const
{
    return mode_ == CalibMode::Bev ? kBevPkg : kFrontPkg;
}

std::string TabImageLidarBev::projectionLaunch() const
{
    return mode_ == CalibMode::Bev ? kBevLaunch : kFrontLaunch;
}

// bool TabImageLidarBev::initBEVMap(){
//    bevsize = cv::Size(1280,480);
//    bevcanvas = cv::Mat::zeros(bevsize,CV_8UC3);
//    cct = cv::Point2f(640,bevsize.height/2);
//   scale = 7;
//     const std::string temp_dir = cfg_->templateDir();
//     std::string camera_config_yaml = temp_dir+"/cameras_distortion.proj_cfg";
//  // AINFO<<"camera_config_yaml:"<<camera_config_yaml<<std::endl;
//     cam_service_ptr.reset(new ::perception::PerCameraService());
//     cam_service_ptr->InitCamera(camera_config_yaml);
//     bev_map_ready_ = cam_service_ptr->IsInit();
//      std::cout<<"hqw___________________________________001:"<<bev_map_ready_<<std::endl;
//     return bev_map_ready_;
// }
void TabImageLidarBev::setupUI()
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
    connect(btn_start_node_, &QPushButton::clicked, this, &TabImageLidarBev::onStartNode);
    connect(btn_stop_node_,  &QPushButton::clicked, this, &TabImageLidarBev::onStopNode);
    label_node_status_ = new QLabel("● 未运行", this);
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    hl_node->addWidget(btn_start_node_);
    hl_node->addWidget(btn_stop_node_);
    hl_node->addWidget(label_node_status_);
    vl->addWidget(grp_node);

    // ---- 标定方式选择 ----
    auto* grp_mode = new QGroupBox("标定方式选择", this);
    auto* hl_mode = new QHBoxLayout(grp_mode);
    hl_mode->setContentsMargins(8, 8, 8, 8);
    hl_mode->setSpacing(8);
    btn_mode_front_ = new QRadioButton("前视标定", this);
    btn_mode_bev_   = new QRadioButton("BEV标定", this);
    mode_group_ = new QButtonGroup(this);
    mode_group_->setExclusive(true);
    mode_group_->addButton(btn_mode_front_);
    mode_group_->addButton(btn_mode_bev_);
    btn_mode_front_->setChecked(true);
    connect(btn_mode_front_, &QRadioButton::clicked, this, &TabImageLidarBev::onSelectModeFront);
    connect(btn_mode_bev_,   &QRadioButton::clicked, this, &TabImageLidarBev::onSelectModeBev);
    hl_mode->addWidget(btn_mode_front_);
    hl_mode->addWidget(btn_mode_bev_);
    vl->addWidget(grp_mode);

    // ---- 可视化控制 ----
    auto* grp_viz = new QGroupBox("可视化控制", this);
    auto* vl_viz = new QVBoxLayout(grp_viz);
    vl_viz->setContentsMargins(8, 8, 8, 8);
    vl_viz->setSpacing(6);
    btn_viz_toggle_ = new QPushButton("▶ 开启可视化", this);
    connect(btn_viz_toggle_, &QPushButton::clicked, this, &TabImageLidarBev::onVizToggle);
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
            this, &TabImageLidarBev::onLidarSelected);
    connect(combo_camera_, &QComboBox::currentTextChanged,
            this, &TabImageLidarBev::onCameraSelected);
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
                this, &TabImageLidarBev::onParamChanged);
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
    connect(btn_save_,  &QPushButton::clicked, this, &TabImageLidarBev::onSaveCalib);
    connect(btn_reset_, &QPushButton::clicked, this, &TabImageLidarBev::onResetParams);
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

    // ---- 右侧：投影叠加视图（前视=图像，BEV=点云） ----
    right_box_ = new QGroupBox("投影叠加图（点云 → 相机图像）", this);
    auto* rl = new QVBoxLayout(right_box_);
    rl->setContentsMargins(8, 8, 8, 8);
    rl->setSpacing(0);
    view_stack_ = new QStackedWidget(this);
    viewer_       = new ImageViewer(this);       // page 0：前视图像
    cloud_viewer_ = new PointCloudViewer(this);  // page 1：BEV 点云
    view_stack_->addWidget(viewer_);
    view_stack_->addWidget(cloud_viewer_);
    rl->addWidget(view_stack_);
    h->addWidget(right_box_, 1);
}

void TabImageLidarBev::onSelectModeFront()
{

    // 点击“前视标定”＝切换到前视方式并启动对应节点（与“启动节点”按钮效果一致）

    setCalibMode(CalibMode::Front);
    if (!ros_->isNodeRunning(nodeKey()))
        onStartNode();
}

void TabImageLidarBev::onSelectModeBev()
{
    // 点击“BEV标定”＝切换到 BEV 方式并启动对应节点（与“启动节点”按钮效果一致）
    setCalibMode(CalibMode::Bev);
    if (!ros_->isNodeRunning(nodeKey()))
        onStartNode();
}

void TabImageLidarBev::setCalibMode(CalibMode mode)
{
    if (mode_ == mode) {
        updateModeUI();
        return;
    }
    // 切换前先停掉当前方式对应的投影节点，避免两个包节点同时运行
    onStopNode();
    mode_ = mode;
    updateModeUI();
    // 保存文件命名随方式不同，重新载入当前相机的初值以对应新方式
    // if (!current_camera_.empty()) {
    //     loadInitialParamsToSpins(current_camera_);
    // }
    updateStartButtonState();
}

void TabImageLidarBev::updateModeUI()
{
    const bool is_bev = (mode_ == CalibMode::Bev);
    if (btn_mode_front_) {
        btn_mode_front_->blockSignals(true);
        btn_mode_front_->setChecked(!is_bev);
        btn_mode_front_->blockSignals(false);
    }
    if (btn_mode_bev_) {
        btn_mode_bev_->blockSignals(true);
        btn_mode_bev_->setChecked(is_bev);
        btn_mode_bev_->blockSignals(false);
    }
    // 切换中央显示：前视=图像视图，BEV=点云视图
    if (view_stack_) {
        view_stack_->setCurrentIndex(is_bev ? 1 : 0);
    }
    if (right_box_) {
        right_box_->setTitle(is_bev
            ? "投影叠加图（相机BEV真彩色地面点 + 主雷达点云）"
            : "投影叠加图（点云 → 相机图像）");
    }
}

void TabImageLidarBev::applyConfig()
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

void TabImageLidarBev::deactivate()
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

void TabImageLidarBev::refreshLidarStatus()
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

void TabImageLidarBev::updateStartButtonState()
{
    const bool can_start = !current_lidar_.empty() &&
                           !current_camera_.empty() &&
                           lidar_is_calibrated_ &&
                           !ros_->isNodeRunning(nodeKey());
    btn_start_node_->setEnabled(can_start);
}

void TabImageLidarBev::loadInitialParamsToSpins(const std::string& camera_name)
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

void TabImageLidarBev::onLidarSelected(const QString& name)
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

void TabImageLidarBev::onCameraSelected(const QString& name)
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

void TabImageLidarBev::onStartNode()
{
    if (current_lidar_.empty() || current_camera_.empty()) {
        QMessageBox::warning(this, "无法启动", "请先选择参考激光雷达与目标相机。");
        return;
    }

      cameras_dist_file_= cfg_->templateDir()+"/cameras_distortion.proj_cfg";
    if (cameras_dist_file_.empty()) {
        QMessageBox::warning(this, "无法启动",
            QString("cameras_distortion.proj_cfg 文件不存在")
                .arg(QString::fromStdString(cameras_dist_file_)));
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
    output_cloud_topic_ = std::string("/lidar_camera_projection/") + current_camera_ + "/cloud";

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
    args << QString("current_camera:=%1").arg(QString::fromStdString(current_camera_));
    args << QString("camera_file:=%1").arg(QString::fromStdString(intrinsics));
    args << QString("cameras_dist_file:=%1").arg(QString::fromStdString(cameras_dist_file_));
    if (!image_topic.empty())
        args << QString("image_topic:=%1").arg(QString::fromStdString(image_topic));
    if (!lidar_topic.empty())
        args << QString("lidar_topic:=%1").arg(QString::fromStdString(lidar_topic));
    args << QString("output_topic:=%1").arg(QString::fromStdString(output_topic_));
    if (mode_ == CalibMode::Bev)
        args << QString("output_cloud_topic:=%1").arg(QString::fromStdString(output_cloud_topic_));
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
         << QString("yaw:=%1").arg(cyaw)
         << QString("vcs_to_ground_height:=%1").arg(cfg_->zOffset());

    bool ok = ros_->launchNode(projectionPkg(), projectionLaunch(), nodeKey(), args);
    if (!ok) {
        label_node_status_->setText("● 启动失败");
        label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    }
}

void TabImageLidarBev::onStopNode()
{
    ros_->killNode(nodeKey());
    ros_->unsubscribeAll();
    viewer_->clear();
    if (cloud_viewer_) cloud_viewer_->clear();
    label_data_status_->setText("未订阅");
}

void TabImageLidarBev::onNodeStarted(const QString& key)
{
    if (key != QString::fromStdString(nodeKey())) return;
    label_node_status_->setText("● 运行中");
    label_node_status_->setStyleSheet("QLabel{color:#4f4;}");
    btn_start_node_->setEnabled(false);
    btn_stop_node_->setEnabled(true);
    btn_save_->setEnabled(true);

    if (viz_enabled_) {
        startVizSubscription();
    }
}

// BEV 模式订阅点云话题并逐点真彩色渲染；前视模式订阅图像话题
void TabImageLidarBev::startVizSubscription()
{
    if (!ros_->isNodeRunning(nodeKey())) return;
    if (mode_ == CalibMode::Bev) {
        if (output_cloud_topic_.empty()) return;
        label_data_status_->setText("订阅中，等待点云...");
        ros_->subscribeCloudRGB(output_cloud_topic_,
            [this](pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
                onCloudReceived(cloud);
            });
    } else {
        if (output_topic_.empty()) return;
        label_data_status_->setText("订阅中，等待图像...");
        ros_->subscribeImage(output_topic_,
            [this](cv::Mat img) { onImageReceived(img); });
    }
}

void TabImageLidarBev::onCloudReceived(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (!viz_enabled_ || !cloud_viewer_) return;
    label_data_status_->setText(
        QString("收到点云 %1 点").arg(cloud ? cloud->size() : 0));
    cloud_viewer_->showCloudRGB(cloud, 2.0);
}

// void  TabImageLidarBev::InitCameraBevMaps(){
//     cv::Point2f  ipmcct(bevsize.width/2, bevsize.height/2);
//   float ipmscale = 15.0;

//   mapx_f120 = Mat(bevsize,CV_32F,-1);
//   mapy_f120 = Mat(bevsize,CV_32F,-1);
//   // mapx_f30 = Mat(bevsize,CV_32F,-1);
//   // mapy_f30 = Mat(bevsize,CV_32F,-1);
//   mapx_lf = Mat(bevsize,CV_32F,-1);
//   mapy_lf = Mat(bevsize,CV_32F,-1);
//   mapx_rf = Mat(bevsize,CV_32F,-1);
//   mapy_rf = Mat(bevsize,CV_32F,-1);
//   mapx_lr = Mat(bevsize,CV_32F,-1);
//   mapy_lr = Mat(bevsize,CV_32F,-1);
//   mapx_rr = Mat(bevsize,CV_32F,-1);
//   mapy_rr = Mat(bevsize,CV_32F,-1);
//   mapx_rear = Mat(bevsize,CV_32F,-1);
//   mapy_rear = Mat(bevsize,CV_32F,-1);
//   float vcs_to_ground_height = cam_service_ptr->cam_utils[CAM_FRONT].vcs_to_ground_height;
//   for(int y=0;y<bevsize.height;y++) {
//     for(int x=0;x<bevsize.width;x++) {
//       Point3f pvcs((x-ipmcct.x)/ipmscale,(y-ipmcct.y)/ipmscale,-vcs_to_ground_height), pimg;
//       if(cam_service_ptr->ConvertPointVCS2IMG(CAM_FRONT, pvcs, pimg)) {
//         mapx_f120.at<float>(y,x) = pimg.x;
//         mapy_f120.at<float>(y,x) = pimg.y;
//         // std::cout<<"cam_service_ptr\n";
//       }
//       // if(cam_service_ptr->PointVcsToImg(CAM_FRONT_H120_CROP, pvcs, pimg)) {
//       //   mapx_f30.at<float>(y,x) = pimg.x;
//       //   mapy_f30.at<float>(y,x) = pimg.y;
//       // }
//       if(cam_service_ptr->PointVcsToImg(CAM_LEFT_FRONT, pvcs, pimg)) {
//         mapx_lf.at<float>(y,x) = pimg.x;
//         mapy_lf.at<float>(y,x) = pimg.y;
//       }
//       if(cam_service_ptr->PointVcsToImg(CAM_RIGHT_FRONT, pvcs, pimg)) {
//         mapx_rf.at<float>(y,x) = pimg.x;
//         mapy_rf.at<float>(y,x) = pimg.y;
//       }
//       if(cam_service_ptr->PointVcsToImg(CAM_LEFT_REAR, pvcs, pimg)) {
//         mapx_lr.at<float>(y,x) = pimg.x;
//         mapy_lr.at<float>(y,x) = pimg.y;
//       }
//       if(cam_service_ptr->PointVcsToImg(CAM_RIGHT_REAR, pvcs, pimg)) {
//         mapx_rr.at<float>(y,x) = pimg.x;
//         mapy_rr.at<float>(y,x) = pimg.y;
//       }
//       if(cam_service_ptr->PointVcsToImg(CAM_REAR, pvcs, pimg)) {
//         mapx_rear.at<float>(y,x) = pimg.x;
//         mapy_rear.at<float>(y,x) = pimg.y;
//       }
//     }
//   }
// }
// void  TabImageLidarBev::bevDrawIPMImage(cv::Mat &canvas, cv::Mat& img,bool draw_seg) {
//     if(bevsize != Size(1280,480)) {
//         bevsize = Size(1280,480);
//         bevcanvas = Mat::zeros(bevsize,CV_8UC3);
//   }
//   //dr_mode = false;
//   cct = cv::Point2f(bevsize.width/2, bevsize.height/2);
//   scale = 15.0;

//   Mat bevf120 = Mat::zeros(mapx_f120.size(),CV_8UC3);
//   Mat bevlf = bevf120.clone();
//   Mat bevrf = bevf120.clone();
//   Mat bevlr = bevf120.clone();
//   Mat bevrr = bevf120.clone();
//   Mat bevrear = bevf120.clone();
//   Mat bevf30 = bevf120.clone();
//   cv::Mat front_img,left_front_img,left_back_img,right_front_img,right_back_img,back_img;

// cv::Mat imgi = img.clone();

//     // int width = imgi.width;
//     // int height = imgi.height;
//     if(imgi.empty()) return;

// if(current_camera_.compare("left_front_100")  ){

//   remap(imgi, bevlf, mapx_lf, mapy_lf, INTER_AREA, BORDER_CONSTANT);

// }else if(current_camera_.compare("right_front_100")){
  
//   remap(imgi, bevrf, mapx_rf, mapy_rf, INTER_AREA, BORDER_CONSTANT);

// }else if(current_camera_.compare("front_120")){
  
//   remap(imgi, bevf120, mapx_f120, mapy_f120, INTER_AREA, BORDER_CONSTANT);

// }else if(current_camera_.compare("left_back_100")){
  
//    remap(imgi, bevlr, mapx_lr, mapy_lr, INTER_AREA, BORDER_CONSTANT);

// }else if(current_camera_.compare("right_back_100")){
  
//   remap(imgi, bevrr, mapx_rr, mapy_rr, INTER_AREA, BORDER_CONSTANT);

// }else if(current_camera_.compare("back_120")){
  
//   remap(imgi, bevrear, mapx_rear, mapy_rear, INTER_AREA, BORDER_CONSTANT);

// }

// Mat  ipm = bevf120*0.5+ bevlf*0.5 + bevrf*0.5 + bevlr*0.5 + bevrr*0.5 + bevrear*0.5;

// // 检查 canvas 是否为空或类型不匹配，避免 scaleAdd 类型错误
// if (canvas.empty() || canvas.type() != ipm.type() || canvas.size() != ipm.size()) {
//     canvas = ipm.clone();
// } else {
//     // 确保类型一致后再进行混合
//     cv::scaleAdd(canvas, 0.5, ipm, canvas);
// }
// /////////////////////////////////////////////////////////////////
// //   std::vector<cv::Mat> imgs;
// //   imgs.resize(CAM_MAX_NUM);
// //   for (int i = 0; i < CAM_MAX_NUM; i++)
// //   {
// //     auto &vseg = framedata.vision_perception[i].vp_result.vision_segment.seg;
// //     // auto &img = imgs[i];
// //     imgs[i] = framedata.images[i].image.clone();
// //     // if (!draw_seg)
// //     //   continue;
// //     int width = vseg.width;
// //     int height = vseg.height;
// //     if(width>0 && height>0)
// //     {
// //        cv::Mat showseg = cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
// //        for(int y=0;y<height;y++) {
// //         // uchar* rowdata = vseg.seg.ptr<uchar>(y);
// //         cv::Vec3b* dst_rowdata = showseg.ptr<cv::Vec3b>(y);
// //         for(int x=0;x<width;x++) 
// //         {
// //           if(x > 0 && x < showseg.cols && y > 0 && y < showseg.rows) {
// //             if((y*width+x)<vseg.data.size())
// //               dst_rowdata[x] = SEGMENT_COLORS[vseg.data[y*width+x]];
// //           }
// //         }
// //       }
// //       if (imgs[i].empty())
// //       {
// //         // cv::resize(showseg,showseg,cv::Size(vseg.camera_width,vseg.camera_height),0,0,INTER_NEAREST);
// //         imgs[i] = showseg;
// //         // AINFO<<"imgDrawSeg cannot Get img:"<<CAM_ID_TO_NAME.at(i)<<std::endl;
// //         continue;
// //       }
// //       else
// //       {
// //         if (!showseg.empty() &&showseg.cols != imgs[i].cols || showseg.rows != imgs[i].rows)
// //           cv::resize(showseg, showseg, imgs[i].size(), 0, 0, INTER_NEAREST);
// //         imgs[i] = imgs[i] * 0.5 + showseg * 0.5;
// //       }
// //     }
// //   }
// //   for (int i=0;i<CAM_MAX_NUM;i++)
// //   {
// //     if(!imgs[i].empty())
// //     {
// //       int cam_img_width = cam_service_ptr->cam_utils[i].camera_width;
// //       int cam_img_height = cam_service_ptr->cam_utils[i].camera_height;

// //       cv::resize(imgs[i],imgs[i],cv::Size(cam_img_width,cam_img_height),0,0,INTER_NEAREST);
// //     }
// //   }
  


// //   // cv::resize(imgs[CAM_LEFT_FRONT].image,left_front_img,cv::Size(1920,1080),0,0,INTER_NEAREST);
// //   // cv::resize(framedata.images[CAM_RIGHT_FRONT].image,right_front_img,cv::Size(1920,1080),0,0,INTER_NEAREST);
// //   // cv::resize(framedata.images[CAM_FRONT_H100].image,front_img,cv::Size(1920,1080),0,0,INTER_NEAREST);
// //   // cv::resize(framedata.images[CAM_LEFT_REAR].image,left_back_img,cv::Size(1920,1080),0,0,INTER_NEAREST);
// //   // cv::resize(framedata.images[CAM_RIGHT_REAR].image,right_back_img,cv::Size(1920,1080),0,0,INTER_NEAREST);
// //   // cv::resize(framedata.images[CAM_REAR].image,back_img,cv::Size(1920,1080),0,0,INTER_NEAREST);
// //   if(!imgs[CAM_LEFT_FRONT].empty())remap(imgs[CAM_LEFT_FRONT], bevlf, mapx_lf, mapy_lf, INTER_AREA, BORDER_CONSTANT);
// //   if(!imgs[CAM_RIGHT_FRONT].empty())remap(imgs[CAM_RIGHT_FRONT], bevrf, mapx_rf, mapy_rf, INTER_AREA, BORDER_CONSTANT);
// //   if(!imgs[CAM_FRONT].empty())remap(imgs[CAM_FRONT], bevf120, mapx_f120, mapy_f120, INTER_AREA, BORDER_CONSTANT);
// //   if(!imgs[CAM_LEFT_REAR].empty())remap(imgs[CAM_LEFT_REAR], bevlr, mapx_lr, mapy_lr, INTER_AREA, BORDER_CONSTANT);
// //   if(!imgs[CAM_RIGHT_REAR].empty())remap(imgs[CAM_RIGHT_REAR], bevrr, mapx_rr, mapy_rr, INTER_AREA, BORDER_CONSTANT);
// //   if(!imgs[CAM_REAR].empty())remap(imgs[CAM_REAR], bevrear, mapx_rear, mapy_rear, INTER_AREA, BORDER_CONSTANT);
// //   // remap(framedata.images[6].image, bevf30, mapx_f30, mapy_f30, INTER_AREA, BORDER_CONSTANT);
// //   Mat ipm;
// //   if(camviewmode>=3&&camviewmode<=9)  
// //   {
// //     switch(camviewmode)
// //     {
// //       case 3:
// //         ipm = bevf120*0.5;
// //         break;
// //       case 4:
// //         ipm = bevlf*0.5;
// //         break;
// //       case 5:
// //         ipm = bevrf*0.5;
// //         break;
// //       case 6:
// //         ipm = bevlr*0.5;
// //         break;
// //       case 7:
// //         ipm = bevrr*0.5;
// //         break;
// //       case 8:
// //         ipm = bevrear*0.5;
// //         break;
// //       default:
// //         break;
// //     }
// //   }
// //   else 
// //     ipm = bevf120*0.5+ bevlf*0.5 + bevrf*0.5 + bevlr*0.5 + bevrr*0.5 + bevrear*0.5;
// //    canvas = canvas*0.5 + ipm;
// }
void TabImageLidarBev::onNodeStopped(const QString& key)
{
    if (key != QString::fromStdString(nodeKey())) return;
    label_node_status_->setText("● 未运行");
    label_node_status_->setStyleSheet("QLabel{color:#e88;}");
    btn_stop_node_->setEnabled(false);
    btn_save_->setEnabled(false);
    updateStartButtonState();
}

void TabImageLidarBev::onImageReceived(cv::Mat img)
{
    if (!viz_enabled_) return;
    label_data_status_->setText(
   QString("收到图像 %1x%2").arg(img.cols).arg(img.rows));
    viewer_->showImage(img);

}

void TabImageLidarBev::onVizToggle()
{
    viz_enabled_ = !viz_enabled_;
    if (viz_enabled_) {
        btn_viz_toggle_->setText("⏹ 关闭可视化");
        btn_viz_toggle_->setStyleSheet(
            "QPushButton { background: #1a4a1a; color: #4f4; border: 1px solid #4f4; "
            "padding: 7px 16px; border-radius: 4px; min-height: 28px; }");
        startVizSubscription();
    } else {
        btn_viz_toggle_->setText("▶ 开启可视化");
        btn_viz_toggle_->setStyleSheet("");
        ros_->unsubscribeAll();
        viewer_->clear();
        if (cloud_viewer_) cloud_viewer_->clear();
        label_data_status_->setText("未订阅");
    }
}

void TabImageLidarBev::onParamChanged(double /*v*/)
{

    std::cout<<"hqwSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS"<<std::endl;
    if (!ros_->isNodeRunning(nodeKey())) return;
    const std::string ns = nodeNs();
    ros_->setParam(ns + "/x",     spin_x_->value());
    ros_->setParam(ns + "/y",     spin_y_->value());
    ros_->setParam(ns + "/z",     spin_z_->value());
    ros_->setParam(ns + "/roll",  spin_roll_->value());
    ros_->setParam(ns + "/pitch", spin_pitch_->value());
    ros_->setParam(ns + "/yaw",   spin_yaw_->value());
}

void TabImageLidarBev::onResetParams()
{
    if (current_camera_.empty()) return;
    loadInitialParamsToSpins(current_camera_);
    // 立刻把初始值也写给节点
    if (ros_->isNodeRunning(nodeKey())) {
        const std::string ns = nodeNs();
        ros_->setParam(ns + "/x",     spin_x_->value());
        ros_->setParam(ns + "/y",     spin_y_->value());
        ros_->setParam(ns + "/z",     spin_z_->value());
        ros_->setParam(ns + "/roll",  spin_roll_->value());
        ros_->setParam(ns + "/pitch", spin_pitch_->value());
        ros_->setParam(ns + "/yaw",   spin_yaw_->value());
    }
}

void TabImageLidarBev::onSaveCalib()
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

    const std::string fname = (mode_ == CalibMode::Bev)
        ? ("/image_lidar_bev_" + current_camera_ + ".yaml")
        : ("/image_lidar_" + current_camera_ + ".yaml");
    const std::string file = proc + fname;
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