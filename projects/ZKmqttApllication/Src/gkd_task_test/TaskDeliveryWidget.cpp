#include "TaskDeliveryWidget.h"
#include "ui_TaskDeliveryWidget.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QTextCodec>
#include <QDebug>
#include <QPushButton>
#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QListWidget>
#include <QDateTime>
#include "../gkd/c2manager.h"

TaskDeliveryWidget::TaskDeliveryWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TaskDeliveryWidget)
{
    ui->setupUi(this);
    
    connect(ui->loadTaskTemplateButton, &QPushButton::clicked, this, &TaskDeliveryWidget::onLoadTaskTemplateClicked);
    connect(ui->taskTemplateListWidget, &QListWidget::itemClicked, this, &TaskDeliveryWidget::onTaskTemplateSelected);
    connect(ui->addWaypointButton, &QPushButton::clicked, this, &TaskDeliveryWidget::onAddWaypointClicked);
    connect(ui->deliverTaskButton, &QPushButton::clicked, this, &TaskDeliveryWidget::onDeliverTaskClicked);
    
    // 设置默认时间
    ui->startTimeEdit->setDateTime(QDateTime::currentDateTime());
    ui->endTimeEdit->setDateTime(QDateTime::currentDateTime().addSecs(3600));
    
    clearTaskDisplay();
    loadTaskTemplateList();
}

TaskDeliveryWidget::~TaskDeliveryWidget()
{
    clearWaypointWidgets();
    delete ui;
}

void TaskDeliveryWidget::setC2Manager(std::shared_ptr<C2Manager> c2Manager)
{
    m_c2Manager = c2Manager;
}

void TaskDeliveryWidget::onLoadTaskTemplateClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("选择任务模板文件"), 
        QString(), 
        tr("文本文件 (*.txt);;所有文件 (*.*)"));
    
    if (!fileName.isEmpty()) {
        parseTemplateFile(fileName);
        emit taskDelivered(currentTaskData);
        QMessageBox::information(this, tr("成功"), tr("任务模板加载完成"));
    }
}

void TaskDeliveryWidget::loadTaskTemplateList()
{
    ui->taskTemplateListWidget->clear();
    
    QString dirPath = "D:/gy/gydata/任务文件SJ";
    QDir directory(dirPath);
    
    if (!directory.exists()) {
        qDebug() << "任务模板目录不存在:" << dirPath;
        return;
    }
    
    QStringList filters;
    filters << "*.txt";
    directory.setNameFilters(filters);
    
    QStringList templateFiles = directory.entryList(QDir::Files);
    
    for (const QString &fileName : templateFiles) {
        QListWidgetItem *item = new QListWidgetItem(fileName);
        item->setData(Qt::UserRole, directory.absoluteFilePath(fileName));
        ui->taskTemplateListWidget->addItem(item);
    }
    
    qDebug() << "加载了" << templateFiles.size() << "个任务模板文件";
}

void TaskDeliveryWidget::onTaskTemplateSelected(QListWidgetItem *item)
{
    if (!item) return;
    
    QString filePath = item->data(Qt::UserRole).toString();
    parseTemplateFile(filePath);
}

void TaskDeliveryWidget::parseTemplateFile(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开文件: " + filePath);
        return;
    }
    
    QTextStream in(&file);
    in.setCodec("UTF-8"); // 设置UTF-8编码
    QString content = in.readAll();
    file.close();
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(content.toUtf8(), &error);
    
    if (error.error != QJsonParseError::NoError) {
        QMessageBox::warning(this, "错误", "TXT文件中的JSON解析失败: " + error.errorString());
        return;
    }
    
    QJsonObject rootObj = doc.object();
    if (!rootObj.contains("mission")) {
        QMessageBox::warning(this, "错误", "JSON格式不正确，缺少mission字段");
        return;
    }
    
    // 保存当前文件路径和JSON数据
    currentTemplateFilePath = filePath;
    currentTaskData = rootObj;
    
    displayTaskInfo(currentTaskData);
    emit taskDelivered(rootObj);
}

void TaskDeliveryWidget::displayTaskInfo(const QJsonObject &taskObj)
{
    if (taskObj.contains("mission")) {
        QJsonObject missionObj = taskObj["mission"].toObject();
        
        ui->taskIdEdit->setText(missionObj["task_id"].toString());
        ui->taskNameEdit->setText(missionObj["task_name"].toString());
        ui->taskTypeEdit->setText(missionObj.contains("task_type") ? QString::number(missionObj["task_type"].toInt()) : "");
        ui->taskPriorityEdit->setText(missionObj.contains("priority") ? QString::number(missionObj["priority"].toInt()) : "1");
        ui->taskDescEdit->setPlainText(missionObj.contains("description") ? missionObj["description"].toString() : "");
        
        // 显示时间信息
        if (missionObj.contains("start_time")) {
            QDateTime startTime = QDateTime::fromString(missionObj["start_time"].toString(), "yyyy-MM-dd hh:mm:ss");
            if (startTime.isValid()) {
                ui->startTimeEdit->setDateTime(startTime);
            }
        }
        if (missionObj.contains("end_time")) {
            QDateTime endTime = QDateTime::fromString(missionObj["end_time"].toString(), "yyyy-MM-dd hh:mm:ss");
            if (endTime.isValid()) {
                ui->endTimeEdit->setDateTime(endTime);
            }
        }
        
        if (missionObj.contains("task_data")) {
            QJsonObject taskDataObj = missionObj["task_data"].toObject();
            
            if (taskDataObj.contains("graphics")) {
                QJsonArray graphicsArray = taskDataObj["graphics"].toArray();
                if (!graphicsArray.isEmpty()) {
                    QJsonObject graphicObj = graphicsArray[0].toObject();
                    
                    if (graphicObj.contains("point")) {
                        QJsonObject pointObj = graphicObj["point"].toObject();
                        ui->longitudeEdit->setText(QString::number(pointObj["longitude"].toDouble(), 'f', 8));
                        ui->latitudeEdit->setText(QString::number(pointObj["latitude"].toDouble(), 'f', 8));
                        ui->altitudeEdit->setText(QString::number(pointObj["altitude"].toDouble()));
                    }
                    
                    // 处理航点信息
                    if (graphicObj.contains("waypoints")) {
                        QJsonArray waypointsArray = graphicObj["waypoints"].toArray();
                        updateWaypointsFromJson(waypointsArray);
                    }
                }
            }
        }
    }
    
    // 设置默认智能体配置
    ui->agentTypeCombo->setCurrentIndex(0);
    ui->agentModeCombo->setCurrentIndex(0);
    ui->agentRouteCombo->setCurrentIndex(0);
    ui->agentSpeedEdit->setText("10.0");
    ui->payloadEdit->setText("标准载荷");
    ui->commFreqEdit->setText("2.4GHz");
}

void TaskDeliveryWidget::clearTaskDisplay()
{
    ui->taskIdEdit->clear();
    ui->taskNameEdit->clear();
    ui->taskTypeEdit->clear();
    ui->taskPriorityEdit->clear();
    ui->taskDescEdit->clear();
    ui->longitudeEdit->clear();
    ui->latitudeEdit->clear();
    ui->altitudeEdit->clear();
    ui->agentIdEdit->clear();
    ui->agentSpeedEdit->clear();
    ui->payloadEdit->clear();
    ui->commFreqEdit->clear();
    
    // 重置时间
    ui->startTimeEdit->setDateTime(QDateTime::currentDateTime());
    ui->endTimeEdit->setDateTime(QDateTime::currentDateTime().addSecs(3600));
    
    clearWaypointWidgets();
}

void TaskDeliveryWidget::onAddWaypointClicked()
{
    addWaypointWidget(0.0, 0.0, 0.0);
}

void TaskDeliveryWidget::onDeleteWaypointClicked()
{
    // 由各个删除按钮的lambda函数处理
}

void TaskDeliveryWidget::addWaypointWidget(double longitude, double latitude, double altitude)
{
    WaypointWidget* widget = new WaypointWidget();
    widget->waypointIndex = waypointWidgets.size();
    
    // 创建组框
    widget->groupBox = new QGroupBox(QString("航点 %1").arg(widget->waypointIndex + 1));
    widget->groupBox->setStyleSheet(
        "QGroupBox { font-weight: bold; padding-top: 10px; margin-top: 5px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }"
    );
    
    // 创建水平布局
    QHBoxLayout* mainLayout = new QHBoxLayout();
    
    // 经度输入框
    QLabel* lonLabel = new QLabel("经度:");
    widget->longitudeEdit = new QLineEdit();
    widget->longitudeEdit->setText(QString::number(longitude, 'f', 8));
    widget->longitudeEdit->setPlaceholderText("请输入经度");
    
    // 纬度输入框
    QLabel* latLabel = new QLabel("纬度:");
    widget->latitudeEdit = new QLineEdit();
    widget->latitudeEdit->setText(QString::number(latitude, 'f', 8));
    widget->latitudeEdit->setPlaceholderText("请输入纬度");
    
    // 高度输入框
    QLabel* altLabel = new QLabel("高度:");
    widget->altitudeEdit = new QLineEdit();
    widget->altitudeEdit->setText(QString::number(altitude));
    widget->altitudeEdit->setPlaceholderText("请输入高度");
    
    // 删除按钮
    widget->deleteButton = new QPushButton("删除");
    widget->deleteButton->setStyleSheet(
        "QPushButton { background-color: #f44336; color: white; border: none; padding: 4px 8px; border-radius: 3px; }"
        "QPushButton:hover { background-color: #d32f2f; }"
    );
    widget->deleteButton->setMaximumWidth(60);
    
    // 连接删除按钮信号
    connect(widget->deleteButton, &QPushButton::clicked, [this, widget]() {
        removeWaypointWidget(widget);
    });
    
    // 添加到布局
    mainLayout->addWidget(lonLabel);
    mainLayout->addWidget(widget->longitudeEdit);
    mainLayout->addWidget(latLabel);
    mainLayout->addWidget(widget->latitudeEdit);
    mainLayout->addWidget(altLabel);
    mainLayout->addWidget(widget->altitudeEdit);
    mainLayout->addWidget(widget->deleteButton);
    
    widget->groupBox->setLayout(mainLayout);
    
    // 添加到列表和界面
    waypointWidgets.append(widget);
    ui->waypointPointsLayout->addWidget(widget->groupBox);
}

void TaskDeliveryWidget::removeWaypointWidget(WaypointWidget* widget)
{
    if (!widget || !waypointWidgets.contains(widget)) {
        return;
    }
    
    // 从布局中移除
    ui->waypointPointsLayout->removeWidget(widget->groupBox);
    
    // 从列表中移除
    waypointWidgets.removeAll(widget);
    
    // 删除控件
    delete widget->groupBox;
    delete widget;
    
    // 更新剩余航点的标题
    for (int i = 0; i < waypointWidgets.size(); ++i) {
        waypointWidgets[i]->waypointIndex = i;
        waypointWidgets[i]->groupBox->setTitle(QString("航点 %1").arg(i + 1));
    }
}

void TaskDeliveryWidget::clearWaypointWidgets()
{
    while (!waypointWidgets.isEmpty()) {
        removeWaypointWidget(waypointWidgets.first());
    }
}

void TaskDeliveryWidget::updateWaypointsFromJson(const QJsonArray &waypointsArray)
{
    clearWaypointWidgets();
    
    for (int i = 0; i < waypointsArray.size(); ++i) {
        QJsonObject waypointObj = waypointsArray[i].toObject();
        double longitude = waypointObj["longitude"].toDouble();
        double latitude = waypointObj["latitude"].toDouble();
        double altitude = waypointObj["altitude"].toDouble();
        
        addWaypointWidget(longitude, latitude, altitude);
    }
}

QJsonArray TaskDeliveryWidget::getWaypointArrayFromWidgets()
{
    QJsonArray waypointArray;
    
    for (WaypointWidget* widget : waypointWidgets) {
        if (!widget) continue;
        
        QJsonObject waypointObj;
        
        bool longitudeOk, latitudeOk, altitudeOk;
        double longitude = widget->longitudeEdit->text().toDouble(&longitudeOk);
        double latitude = widget->latitudeEdit->text().toDouble(&latitudeOk);
        double altitude = widget->altitudeEdit->text().toDouble(&altitudeOk);
        
        if (longitudeOk && latitudeOk && altitudeOk) {
            waypointObj["longitude"] = longitude;
            waypointObj["latitude"] = latitude;
            waypointObj["altitude"] = altitude;
            waypointArray.append(waypointObj);
        }
    }
    
    return waypointArray;
}

QJsonObject TaskDeliveryWidget::buildTaskJson()
{
    QJsonObject rootObj;
    QJsonObject missionObj;
    
    missionObj["task_id"] = ui->taskIdEdit->text();
    missionObj["task_name"] = ui->taskNameEdit->text();
    missionObj["task_type"] = ui->taskTypeEdit->text().isEmpty() ? 1 : ui->taskTypeEdit->text().toInt();
    missionObj["priority"] = ui->taskPriorityEdit->text().isEmpty() ? 1 : ui->taskPriorityEdit->text().toInt();
    missionObj["status"] = "待执行";
    missionObj["start_time"] = ui->startTimeEdit->dateTime().toString("yyyy-MM-dd hh:mm:ss");
    missionObj["end_time"] = ui->endTimeEdit->dateTime().toString("yyyy-MM-dd hh:mm:ss");
    missionObj["description"] = ui->taskDescEdit->toPlainText();
    
    QJsonObject taskDataObj;
    QJsonArray graphicsArray;
    QJsonObject graphicObj;
    
    // 目标点信息
    QJsonObject pointObj;
    pointObj["longitude"] = ui->longitudeEdit->text().toDouble();
    pointObj["latitude"] = ui->latitudeEdit->text().toDouble();
    pointObj["altitude"] = ui->altitudeEdit->text().toDouble();
    graphicObj["point"] = pointObj;
    
    // 航点信息
    QJsonArray waypointArray = getWaypointArrayFromWidgets();
    if (!waypointArray.isEmpty()) {
        graphicObj["waypoints"] = waypointArray;
    }
    
    graphicsArray.append(graphicObj);
    taskDataObj["graphics"] = graphicsArray;
    
    // 智能体配置信息
    QJsonObject agentConfigObj;
    agentConfigObj["agent_type"] = ui->agentTypeCombo->currentText();
    agentConfigObj["agent_id"] = ui->agentIdEdit->text();
    agentConfigObj["control_mode"] = ui->agentModeCombo->currentText();
    agentConfigObj["speed"] = ui->agentSpeedEdit->text().isEmpty() ? 10.0 : ui->agentSpeedEdit->text().toDouble();
    agentConfigObj["route_plan"] = ui->agentRouteCombo->currentText();
    agentConfigObj["payload"] = ui->payloadEdit->text();
    agentConfigObj["comm_freq"] = ui->commFreqEdit->text();
    
    taskDataObj["agent_config"] = agentConfigObj;
    missionObj["task_data"] = taskDataObj;
    rootObj["mission"] = missionObj;
    
    return rootObj;
}

void TaskDeliveryWidget::onDeliverTaskClicked()
{
    // 检查必填项
    if (ui->taskIdEdit->text().isEmpty() || ui->taskNameEdit->text().isEmpty()) {
        QMessageBox::warning(this, "警告", "请填写完整的任务基本信息");
        return;
    }
    
    if (ui->longitudeEdit->text().isEmpty() || ui->latitudeEdit->text().isEmpty()) {
        QMessageBox::warning(this, "警告", "请填写目标位置信息");
        return;
    }
    
    if (ui->agentIdEdit->text().isEmpty()) {
        QMessageBox::warning(this, "警告", "请填写智能体ID");
        return;
    }
    
    if (!m_c2Manager) {
        QMessageBox::critical(this, "错误", "C2Manager未初始化，无法下发任务");
        return;
    }
    
    try {
        // 构建任务JSON数据
        QJsonObject taskJson = buildTaskJson();
        
        // 将JSON数据转换为字符串
        QJsonDocument jsonDoc(taskJson);
        QString jsonString = jsonDoc.toJson(QJsonDocument::Compact);
        
        // 调用C2Manager的processTaskPlan方法
        m_c2Manager->processTaskPlan(jsonString.toStdString());
        
        // 显示成功消息
        QMessageBox::information(this, "成功", 
            QString("任务下发成功！\n任务ID: %1\n任务名称: %2\n目标智能体: %3")
                .arg(ui->taskIdEdit->text())
                .arg(ui->taskNameEdit->text())
                .arg(ui->agentIdEdit->text()));
        
        emit taskDelivered(taskJson);
        
        qDebug() << "任务下发成功，任务数据：" << jsonString;
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "错误", 
            QString("任务下发失败: %1").arg(e.what()));
    } catch (...) {
        QMessageBox::critical(this, "错误", "任务下发时出现未知错误");
    }
}