#include "xsjsimui.h"
#include "ui_xsjsimui.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>
#include <QDateTime>
#include <QDir>
#include <QTextCodec>
#include <QVBoxLayout>

XsJSimUi::XsJSimUi(QWidget* parent)
    : QMainWindow(parent)
    , titleBar(nullptr)
    , ui(new Ui::XsJSimUi)
    , update_timer_(new QTimer(this))
{
    ui->setupUi(this);
    
    // 设置无边框
    setWindowFlags(Qt::FramelessWindowHint);
    
    // 创建自定义标题栏
    titleBar = new CustomTitleBar(this);
    titleBar->setTitle("仿真系统");
    
    // 将标题栏放置在窗口顶部
    titleBar->setGeometry(0, 0, this->width(), 30);
    titleBar->show();
    
    // 调整主窗口内容的位置，为标题栏留出空间
    if (ui->centralwidget) {
        ui->centralwidget->setContentsMargins(0, 35, 0, 0);
    }
    
    // 确保标题栏在最上层
    titleBar->raise();
    
    // 设置定时器更新智能体列表
    connect(update_timer_, &QTimer::timeout, this, &XsJSimUi::updateObjectsList);
    update_timer_->start(1000); // 每秒更新一次
    
    // 连接ComboBox的信号
    connect(ui->comboBox_objects, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &XsJSimUi::onObjectSelected);

    // 连接ComboBox的信号
    connect(ui->comboBox_target, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &XsJSimUi::onTargetSelected);
    ui->comboBox_target->addItem("车辆目标", QStringLiteral("2"));
    ui->comboBox_target->addItem("飞机目标", QStringLiteral("4"));
            
    // 初始选择自身
    selected_uid_ = QString::fromStdString(sim_controller_.getSelfObject().uid);
}

XsJSimUi::~XsJSimUi()
{
    if(update_timer_) {
        update_timer_->stop();
        delete update_timer_;
    }
    delete ui;
}

void XsJSimUi::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    
    // 调整标题栏大小以适应窗口宽度
    if (titleBar) {
        titleBar->setGeometry(0, 0, event->size().width(), 30);
        titleBar->raise(); // 确保标题栏始终在最上层
    }
}

void XsJSimUi::updateObjectsList()
{
    // 保存当前选中的文本
    QString currentText = ui->comboBox_objects->currentText();
    
    // 清空并添加默认选项
    ui->comboBox_objects->clear();
    ui->comboBox_objects->addItem("选择智能体", "");
    
    // 获取所有智能体
    const auto& objects = sim_controller_.getTotalObjects();
    for(const auto& pair : objects) {
        const auto& obj = pair.second;
        QString displayText = QString::fromStdString(obj.name);
        if(obj.isOnline) {
            ui->comboBox_objects->addItem(displayText, QString::fromStdString(obj.uid));
        }
    }
    
    // 恢复之前选中的项
    int index = ui->comboBox_objects->findText(currentText);
    if(index != -1) {
        ui->comboBox_objects->setCurrentIndex(index);
    } else if (currentText != "选择智能体") {
        // 如果之前选中的项不存在且不是默认选项，则选择第一个有效项（如果存在）
        ui->comboBox_objects->setCurrentIndex(ui->comboBox_objects->count() > 1 ? 1 : 0);
    }
}

void XsJSimUi::onObjectSelected(int index)
{
    if(index > 0) { // 跳过默认选项
        selected_uid_ = ui->comboBox_objects->currentData().toString();
    } else {
        selected_uid_ = QString::fromStdString(sim_controller_.getSelfObject().uid);
    }
}

void XsJSimUi::onTargetSelected(int index)
{

   // if(index > 0) { // 跳过默认选项
        QString targettypename  = ui->comboBox_target->currentData().toString();
        if(targettypename == "2"){
            _targetInfo.tarhet_type = 2;
            _targetInfo.name="车辆目标";
            _targetInfo.target_id =QString::number(targetidIndex++);
            ui->comboBox_target->setCurrentIndex(0);
        }else if(targettypename == "4"){
            _targetInfo.tarhet_type = 4;
            _targetInfo.name = "飞机目标";
            _targetInfo.target_id =QString::number(targetidIndex++);
            ui->comboBox_target->setCurrentIndex(1);
       }


    // } else {
    //     selected_uid_ = QString::fromStdString(sim_controller_.getSelfObject().uid);
    // }
}

void XsJSimUi::startTimer(MqClient* mqClient,rest_rcp_client* _rcp_client,const CommonCfg& _cfg,const GKDCommonCfg& gkd_cfg) {
    mqttClient_ = mqClient;
    rpcClient_ = _rcp_client;
    sim_controller_.InitMqttAndRcp(mqClient,_rcp_client,_cfg,gkd_cfg);
    sim_controller_.startTimer();
   // sim_controller_.cameraGoto(32.50972,118.09972, 1000.0, 0.0, 0.0, 2000);

}

void XsJSimUi::on_btn_clear_targets_routes_clicked()
{
    if (!rpcClient_ || !rpcClient_->m_rcpClientAdapter ||
        !rpcClient_->m_rcpClientAdapter->m_rcpClient ||
        !rpcClient_->m_rcpClientAdapter->m_rcpClient->has_connected()) {
        QMessageBox::warning(this, tr("清除请求未提交"),
                             tr("国遥RPC未连接，未提交清除请求"));
        return;
    }
    clearPlannedRoutes();
    qInfo() << "规划路线清除请求已提交";
    QMessageBox::information(this, tr("请求已提交"),
                             tr("规划路线清除请求已提交"));
}

void XsJSimUi::clearPlannedRoutes()
{
    sim_controller_.revTaskData(R"({"data":"dj_finish"})");
}

void XsJSimUi::on_btn_publish303_clicked()
{
    auto* connection = mqttClient_ ? mqttClient_->GetMqttConn() : nullptr;
    if (!connection || !connection->connected()) {
        qWarning() << "QB/303报文未发送：MQTT未连接";
        QMessageBox::warning(this, tr("发送失败"), tr("MQTT未连接，未发送报文"));
        return;
    }

    QJsonObject content;
    content["battle_unit_code"] = QStringLiteral("ys");
    content["battle_unit_name"] = QStringLiteral("ys");
    content["platform_type"] = 3;

    QJsonObject message;
    message["SID"] = QStringLiteral("2");
    message["TM"] = 1;
    message["CTN"] = content;

    if (!connection->PrimitivePublish(QStringLiteral("QB/303"), message)) {
        qWarning() << "QB/303报文提交失败";
        QMessageBox::critical(this, tr("发送失败"), tr("QB/303报文提交失败，请检查MQTT连接"));
        return;
    }

    qInfo() << "QB/303报文已提交："
            << QJsonDocument(message).toJson(QJsonDocument::Compact);
    QMessageBox::information(this, tr("报文已提交"), tr("已向QB/303提交演示平台视频流推送请求报文"));
}

// 显示操作结果的辅助函数
void XsJSimUi::showOperationResult(const ApiResponse& response, const QString& operation) {
    QString title = response.success ? "操作成功" : "操作失败";
    QString message = QString::fromStdString(response.message);
    if(message.isEmpty()) {
        message = response.success ? operation + "成功" : operation + "失败";
    }
    
    if(response.success) {
        QMessageBox::information(this, title, message);
    } else {
        QMessageBox::warning(this, title, message);
    }
}

void XsJSimUi::on_btn_stop_uav_clicked()
{
    auto response = sim_controller_.stopUav();
    showOperationResult(response, "回收侦察无人机");
}

void XsJSimUi::on_btn_start_uav_clicked()
{
    auto response = sim_controller_.startUav();
    showOperationResult(response, "释放侦察无人机");
}

void XsJSimUi::on_action_exit_triggered()
{
    close();
}



void XsJSimUi::on_btn_start_fpv_clicked()
{
    auto response = sim_controller_.releaseFpv(2);
    showOperationResult(response, "释放穿越无人机");
}

void XsJSimUi::on_btn_stop_fpv_clicked()
{
    auto response = sim_controller_.stopFpv();
    showOperationResult(response, "回收穿越无人机");
}

void XsJSimUi::on_cmd_reset_clicked()
{
    auto response = sim_controller_.resetCommand();
    showOperationResult(response, "重置命令");
}

void XsJSimUi::on_btn_open_cabin_clicked()
{
    auto response = sim_controller_.openCabin();
    showOperationResult(response, "打开舱体");
}

void XsJSimUi::on_btn_close_cabin_clicked()
{
    auto response = sim_controller_.closeCabin();
    showOperationResult(response, "关闭舱体");
}

/**
 *type 1:left 2:right 3:top 4:bottom
 *测试自车自主任务
 **/
void XsJSimUi::on_btn_top_clicked()
{
    auto response = sim_controller_.testSelfMove(MoveDirection::TOP, selected_uid_.toStdString());
    showOperationResult(response, "前进");
}

void XsJSimUi::on_btn_right_clicked()
{
    auto response = sim_controller_.testSelfMove(MoveDirection::RIGHT, selected_uid_.toStdString());
    showOperationResult(response, "右转");
}

void XsJSimUi::on_btn_left_clicked()
{
    auto response = sim_controller_.testSelfMove(MoveDirection::LEFT, selected_uid_.toStdString());
    showOperationResult(response, "左转");
}

void XsJSimUi::on_btn_bottom_clicked()
{
    auto response = sim_controller_.testSelfMove(MoveDirection::BOTTOM, selected_uid_.toStdString());
    showOperationResult(response, "后退");
}

//执行侦察
void XsJSimUi::on_btn_scout_clicked()
{
    auto response = sim_controller_.revMqttData("",test_run_nav);
    showOperationResult(response, "执行侦察");
}

//执行打击
void XsJSimUi::on_btn_strike_clicked()
{
    auto response = sim_controller_.revMqttData("",test_run_fpv);
    showOperationResult(response, "执行打击");
}


void XsJSimUi::on_btn_uav_strike_clicked()
{
    auto response = sim_controller_.runFpv(false,"300");
    showOperationResult(response, "单独打击");

}


void XsJSimUi::on_btn_uav_land_clicked()
{
    auto response = sim_controller_.runFpv(true,"");
    showOperationResult(response, "降落");
}

// Test load task button handler
void XsJSimUi::on_btn_test_load_task_clicked()
{
    // Open file dialog to select txt file
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Select Task JSON File",
        QDir::currentPath(),
        "Text Files (*.txt);;JSON Files (*.json);;All Files (*.*)"
    );

    if (fileName.isEmpty()) {
        return; // User cancelled file selection
    }

    // Read file content
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Cannot open file: " + fileName);
        return;
    }

    QTextStream in(&file);
    in.setCodec("UTF-8"); // Ensure UTF-8 encoding for reading
    QString jsonContent = in.readAll();
    file.close();

    if (jsonContent.isEmpty()) {
        QMessageBox::warning(this, "Error", "File content is empty");
        return;
    }

    // Clean JSON content (remove possible BOM and extra whitespace)
    jsonContent = jsonContent.trimmed();
    if (jsonContent.startsWith(QChar(0xFEFF))) {
        jsonContent = jsonContent.mid(1); // Remove BOM
    }

    qDebug() << "Read file content length:" << jsonContent.length();
    qDebug() << "First 100 characters of file content:" << jsonContent.left(100);
    qDebug() << "Last 100 characters of file content:" << jsonContent.right(100);

    // Validate JSON format
    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonContent.toUtf8(), &parseError);

    if (parseError.error != QJsonParseError::NoError) {
        QString errorMsg = QString("JSON parsing failed:\nError: %1\nPosition: %2\nNearby content: %3")
            .arg(parseError.errorString())
            .arg(parseError.offset)
            .arg(jsonContent.mid(qMax(0, parseError.offset - 20), 40));

        qDebug() << "JSON parsing error details:" << errorMsg;
        QMessageBox::warning(this, "JSON Format Error", errorMsg);
        return;
    }

    // Construct LOAD_TASK command JSON data
    QJsonObject loadTaskJson;
    loadTaskJson["cmd"] = "LOAD_TASK";
    loadTaskJson["sn"] = "test_load_task_" + QString::number(QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000);
    loadTaskJson["data"] = jsonDoc.object(); // Use read JSON as data field
    loadTaskJson["timestamp"] = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000;

    // Convert to string
    QJsonDocument loadTaskDoc(loadTaskJson);
    QString loadTaskStr = loadTaskDoc.toJson(QJsonDocument::Compact);

    qDebug() << "Preparing to call loadTask, data:" << loadTaskStr;

    // Call loadTask interface
    auto response = sim_controller_.revMqttData("", loadTaskStr.toStdString());

    // Output detailed information to console
    qDebug() << "loadTask call completed, result:" << (response.success ? "Success" : "Failed");
    // if (!response.message.empty()) {
    //     qDebug() << "Response message:" << QString::fromStdString(response.message);
    // }

    // // Display result
    // showOperationResult(response, "Load Task");
}

// Test location button handler
void XsJSimUi::on_btn_test_location_clicked()
{
    qDebug() << "Test location button clicked";

    // 调用sim_controller_的cameraGoto方法测试定位
    // auto response = sim_controller_.cameraGoto(28.2076, 112.895, 1000.0, 45.0, 0.0, 2000);
    auto response = sim_controller_.cameraGoto(28.239722,112.803889, 1000.0, 0.0, 0.0, 2000);
    // 显示操作结果
    showOperationResult(response, "测试定位");
}

void XsJSimUi::on_btn_test_location2_clicked()
{
    qDebug() << "Test location button clicked";

    // 调用sim_controller_的cameraGoto方法测试定位
    // auto response = sim_controller_.cameraGoto(28.2076, 112.895, 1000.0, 45.0, 0.0, 2000);
    auto response = sim_controller_.cameraGoto(29.728611,106.516667, 1000.0, 0.0, 0.0, 2000);
    // 显示操作结果
    showOperationResult(response, "测试定位");
}

void XsJSimUi::on_btn_test_location3_clicked()
{
    qDebug() << "Test location button clicked";

    // 调用sim_controller_的cameraGoto方法测试定位
    // auto response = sim_controller_.cameraGoto(28.2076, 112.895, 1000.0, 45.0, 0.0, 2000);
    auto response = sim_controller_.cameraGoto(32.510,118.102, 1000.0, 0.0, 0.0, 2000);
    // 显示操作结果
    showOperationResult(response, "测试定位");
}

void XsJSimUi::on_btn_test_location4_clicked()
{

    auto response = sim_controller_.cameraGoto(30.58588,114.18727, 1000.0, 0.0, 0.0, 2000);
}

void XsJSimUi::on_pushButton_gante_clicked()
{
    // APIBridgeSDK::SchemeInfo info;
    // // info.schmeId = "abcdefg";
    // // info.taskId = "task00001";
    // // info.schemeName = "测试方案";
    // // info.jsoninfo = "{\"BehaviorList\":[{\"data\":{\"Camerafocus\":0,\"CamerafocusCamerarotateX\":0,\"deltaX\":8,\"deltaY\":0,\"deltaZ\":0,\"duration\":1500,\"rotateY\":0,\"start_time\":0},\"duration\":1500,\"modelId\":\"osmgydrkzo24nfl6redf\",\"name\":\"       \",\"text\":\"       A\",\"time\":0,\"type\":2,\"uuid\":\"{14fb4083-7119-4da4-bba2-3cb8059a1c80}\"},{\"data\":{\"Camerafocus\":0,\"CamerafocusCamerarotateX\":0,\"deltaX\":10,\"deltaY\":0,\"deltaZ\":0,\"duration\":1000,\"pos_lat\":29.728957,\"pos_lon\":106.516698,\"rotateY\":6},\"duration\":1000,\"modelId\":\"osmgydrkzo24nfl6redf\",\"name\":\"       \",\"text\":\"       \",\"time\":46545,\"type\":2,\"uuid\":\"{e0ea8876-5aff-49f2-bba8-7fac9a3df884}\"}],\"EntityModel\":[{\"alt\":[0,0,0],\"entityType\":1001,\"layerName\":\" ջ \",\"modelId\":\"osmgydrkzo24nfl6redf\",\"name\":\"       ˻ 0345\",\"pos\":[106.51229110188409,29.729679526129807,500],\"scale\":1.3422554864e-313}],\"MovePath\":[{\"data\":{\"height\":0,\"speed\":100},\"duration\":16761,\"endTime\":16761,\"modelId\":\"osmgydrkzo24nfl6redf\",\"pathId\":\"{5953ea93-83bf-4c3e-8a52-8aae19d405f1}\",\"pts\":[[106.512291,29.72968,500,0],[106.513968,29.729656,500,5836],[106.513712,29.730295,500,8547],[106.513908,29.730714,500,10359],[106.514671,29.730197,500,13726],[106.515407,29.729792,500,16761]],\"speed\":27.77938033682105,\"startTime\":0,\"text\":\"        A\"},{\"data\":{\"height\":0,\"speed\":80},\"duration\":32515,\"endTime\":49276,\"modelId\":\"osmgydrkzo24nfl6redf\",\"pathId\":\"{44c7d128-f7e9-4c33-b261-34eda8870543}\",\"pts\":[[106.515407,29.729792,500,16761],[106.51816468213153,29.72946090483011,258.7360411779955,33025],[106.51664326774312,29.728541863867914,265.52826986275613,41092],[106.51772064400376,29.72808311326343,261.62148016598076,46314],[106.51803917422917,29.727561015155715,260.7900840854272,49276]],\"speed\":22.222899882189235,\"startTime\":16761,\"text\":\"        \"}]}";
    sim_controller_.slotLoadScheme();
   // m_rcpClientAdapter->loadScheme(info);
}

void XsJSimUi::on_pushButton_pushtarget_clicked()
{
    _targetInfo.lon = ui->textEdit_lon->toPlainText().toDouble();
    _targetInfo.lat = ui->textEdit_lat->toPlainText().toDouble();
    sim_controller_.PublishMD(_targetInfo.name,_targetInfo.tarhet_type,_targetInfo.lon,_targetInfo.lat);
}

void XsJSimUi::on_btn_test_ers_clicked()
{

}

// Fire points display button handler
void XsJSimUi::on_btn_show_fire_points_clicked()
{
    qDebug() << "Fire points display button clicked";

    // 调用sim_controller_的showFirePoints方法显示火力点
    auto response = sim_controller_.showFirePoints();
    // 显示操作结果
    showOperationResult(response, "火力点显示");
}

// Safe zone display button handler
void XsJSimUi::on_btn_show_safe_zone_clicked()
{
    qDebug() << "Safe zone display button clicked";

    // 调用sim_controller_的showSafeZone方法显示安全区
    auto response = sim_controller_.showSafeZone();
    // 显示操作结果
    showOperationResult(response, "安全区显示");
}

// Avoidance area display button handler
void XsJSimUi::on_btn_show_avoidance_area_clicked()
{
    qDebug() << "Avoidance area display button clicked";

    // 调用sim_controller_的showAvoidanceArea方法显示规避面
    auto response = sim_controller_.showAvoidanceArea();
    // 显示操作结果
    showOperationResult(response, "规避面显示");
}

// Yield points display button handler
void XsJSimUi::on_btn_show_yield_points_clicked()
{
    qDebug() << "Yield points display button clicked";

    // 调用sim_controller_的showYieldPoints方法显示避让点
    auto response = sim_controller_.showYieldPoints();
    // 显示操作结果
    showOperationResult(response, "避让点显示");
}
//加载模拟态势数据按钮
void XsJSimUi::on_btn_vechile_clicked()
{
    qDebug() << "vechile button clicked";
    auto response = sim_controller_.loadVechileMode();
    // 显示操作结果
    showOperationResult(response, "加载态势数据");
    
}
