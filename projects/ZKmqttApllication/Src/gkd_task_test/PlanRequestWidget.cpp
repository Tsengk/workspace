#include "PlanRequestWidget.h"
#include "ui_PlanRequestWidget.h"
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
#include "../gkd/c2manager.h"

PlanRequestWidget::PlanRequestWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlanRequestWidget)
{
    ui->setupUi(this);
    
    connect(ui->loadPlanFileButton, &QPushButton::clicked, this, &PlanRequestWidget::onLoadPlanFileClicked);
    connect(ui->planFileListWidget, &QListWidget::itemClicked, this, &PlanRequestWidget::onPlanFileSelected);
    connect(ui->saveChangesButton, &QPushButton::clicked, this, &PlanRequestWidget::onSaveChangesClicked);
    connect(ui->addRegionPointButton, &QPushButton::clicked, this, &PlanRequestWidget::onAddRegionPointClicked);
    connect(ui->planRequestButton, &QPushButton::clicked, this, &PlanRequestWidget::onPlanRequestClicked);
    
    clearInfoDisplay();

    // loadPlanFileList();
}

PlanRequestWidget::~PlanRequestWidget()
{
    clearRegionPointWidgets();
    delete ui;
}

void PlanRequestWidget::onLoadPlanFileClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("选择筹划文件"), 
        QString(), 
        tr("文本文件 (*.txt);;所有文件 (*.*)"));
    
    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream in(&file);
            in.setCodec("UTF-8"); // 设置UTF-8编码
            QString content = in.readAll();
            file.close();
            
            emit planFileLoaded(fileName);
            QMessageBox::information(this, tr("成功"), tr("筹划文件加载完成"));
        } else {
            QMessageBox::warning(this, tr("错误"), tr("无法打开文件: %1").arg(fileName));
        }
    }
}

void PlanRequestWidget::loadPlanFileList()
{
    ui->planFileListWidget->clear();
    
    QString dirPath = "D:/gy/gydata/任务文件SJ";
    QDir directory(dirPath);
    
    if (!directory.exists()) {
        qDebug() << "目录不存在:" << dirPath;
        return;
    }
    
    QStringList filters;
    filters << "*.txt";
    directory.setNameFilters(filters);
    
    QStringList txtFiles = directory.entryList(QDir::Files);
    
    for (const QString &fileName : txtFiles) {
        QListWidgetItem *item = new QListWidgetItem(fileName);
        item->setData(Qt::UserRole, directory.absoluteFilePath(fileName));
        ui->planFileListWidget->addItem(item);
    }
    
    qDebug() << "加载了" << txtFiles.size() << "个TXT文件";
}

void PlanRequestWidget::onPlanFileSelected(QListWidgetItem *item)
{
    if (!item) return;
    
    QString filePath = item->data(Qt::UserRole).toString();
    parseTxtFile(filePath);
}

void PlanRequestWidget::parseTxtFile(const QString &filePath)
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
    currentFilePath = filePath;
    currentJsonData = rootObj;
    
    QJsonObject missionObj = rootObj["mission"].toObject();
    
    if (missionObj.contains("task_data")) {
        QJsonObject taskDataObj = missionObj["task_data"].toObject();
        
        // 检查是否同时包含graphics和sar_info
        bool hasGraphics = taskDataObj.contains("graphics");
        bool hasSarInfo = taskDataObj.contains("sar_info");
        bool hasPointInGraphics = false;
        bool hasRegionInGraphics = false;
        
        if (hasGraphics) {
            QJsonArray graphicsArray = taskDataObj["graphics"].toArray();
            if (!graphicsArray.isEmpty()) {
                QJsonObject graphicObj = graphicsArray[0].toObject();
                hasPointInGraphics = graphicObj.contains("point");
                hasRegionInGraphics = graphicObj.contains("region");
            }
        }
        
        // 优先级：SAR任务(有sar_info) > 区域任务(有region) > 点任务(有point) > 基础任务
        if (hasSarInfo) {
            displayNewSarTaskInfo(rootObj, missionObj, taskDataObj);
            showInfoGroups(true, false); // 显示SAR信息，不显示区域信息
        } else if (hasRegionInGraphics) {
            displayNewRegionTaskInfo(rootObj, missionObj, taskDataObj);
            showInfoGroups(false, true); // 不显示SAR信息，显示区域信息
        } else if (hasPointInGraphics) {
            displayPointTaskInfo(rootObj, missionObj, taskDataObj);
            showInfoGroups(false, false); // 不显示SAR和区域信息
        } else {
            displayBasicTaskInfo(rootObj, missionObj);
            showInfoGroups(false, false);
        }
    } else {
        QMessageBox::warning(this, "警告", "未知的任务类型");
        clearInfoDisplay();
    }
    
    emit taskInfoChanged(rootObj);
}

void PlanRequestWidget::displayNewSarTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj, const QJsonObject &taskDataObj)
{
    ui->taskIdValue->setText(missionObj["task_id"].toString());
    ui->taskNameValue->setText(missionObj["task_name"].toString());
    ui->taskTypeEdit->setText(QString::number(missionObj["task_type"].toInt()));
    
    // 显示任务优先级、状态、时间和描述信息（如果存在）
    ui->taskPriorityValue->setText(missionObj.contains("priority") ? QString::number(missionObj["priority"].toInt()) : "-");
    ui->taskStatusValue->setText(missionObj.contains("status") ? missionObj["status"].toString() : "-");
    ui->startTimeValue->setText(missionObj.contains("start_time") ? missionObj["start_time"].toString() : "-");
    ui->endTimeValue->setText(missionObj.contains("end_time") ? missionObj["end_time"].toString() : "-");
    ui->taskDescValue->setText(missionObj.contains("description") ? missionObj["description"].toString() : "-");
    
    if (taskDataObj.contains("graphics")) {
        QJsonArray graphicsArray = taskDataObj["graphics"].toArray();
        if (!graphicsArray.isEmpty()) {
            QJsonObject graphicObj = graphicsArray[0].toObject();
            if (graphicObj.contains("point")) {
                QJsonObject pointObj = graphicObj["point"].toObject();
                ui->longitudeEdit->setText(QString::number(pointObj["longitude"].toDouble(), 'f', 6));
                ui->latitudeEdit->setText(QString::number(pointObj["latitude"].toDouble(), 'f', 6));
                ui->altitudeEdit->setText(QString::number(pointObj["altitude"].toDouble()));
            }
        }
    }
    
    if (taskDataObj.contains("sar_info")) {
        QJsonObject sarObj = taskDataObj["sar_info"].toObject();
        ui->sarHeightEdit->setText(QString::number(sarObj["sar_height"].toDouble()));
        ui->sarAngleEdit->setText(QString::number(sarObj["sar_angle"].toDouble()));
        ui->sarHeadingEdit->setText(QString::number(sarObj["sar_heading"].toDouble()));
        
        // 显示额外的SAR信息（如果存在）
        ui->sarRangeEdit->setText(sarObj.contains("sar_range") ? QString::number(sarObj["sar_range"].toDouble()) : "-");
        ui->sarResolutionEdit->setText(sarObj.contains("sar_resolution") ? QString::number(sarObj["sar_resolution"].toDouble()) : "-");
        ui->sarBandwidthEdit->setText(sarObj.contains("sar_bandwidth") ? QString::number(sarObj["sar_bandwidth"].toDouble()) : "-");
        ui->sarModeEdit->setText(sarObj.contains("sar_mode") ? sarObj["sar_mode"].toString() : "-");
    }
    
    clearRegionPointWidgets();
}

void PlanRequestWidget::displayNewRegionTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj, const QJsonObject &taskDataObj)
{
    ui->taskIdValue->setText(missionObj["task_id"].toString());
    ui->taskNameValue->setText(missionObj["task_name"].toString());
    ui->taskTypeEdit->setText(QString::number(missionObj["task_type"].toInt()));
    
    // 显示任务优先级、状态、时间和描述信息（如果存在）
    ui->taskPriorityValue->setText(missionObj.contains("priority") ? QString::number(missionObj["priority"].toInt()) : "-");
    ui->taskStatusValue->setText(missionObj.contains("status") ? missionObj["status"].toString() : "-");
    ui->startTimeValue->setText(missionObj.contains("start_time") ? missionObj["start_time"].toString() : "-");
    ui->endTimeValue->setText(missionObj.contains("end_time") ? missionObj["end_time"].toString() : "-");
    ui->taskDescValue->setText(missionObj.contains("description") ? missionObj["description"].toString() : "-");
    
    ui->longitudeEdit->setText("-");
    ui->latitudeEdit->setText("-");
    ui->altitudeEdit->setText("-");
    ui->sarHeightEdit->setText("-");
    ui->sarAngleEdit->setText("-");
    ui->sarHeadingEdit->setText("-");
    ui->sarRangeEdit->setText("-");
    ui->sarResolutionEdit->setText("-");
    ui->sarBandwidthEdit->setText("-");
    ui->sarModeEdit->setText("-");
    
    // 清空现有的区域坐标点控件
    clearRegionPointWidgets();
    
    if (taskDataObj.contains("graphics")) {
        QJsonArray graphicsArray = taskDataObj["graphics"].toArray();
        if (!graphicsArray.isEmpty()) {
            QJsonObject graphicObj = graphicsArray[0].toObject();
            if (graphicObj.contains("region")) {
                QJsonArray regionArray = graphicObj["region"].toArray();
                updateRegionPointsFromJson(regionArray);
            }
        }
    }
}

void PlanRequestWidget::displayPointTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj, const QJsonObject &taskDataObj)
{
    ui->taskIdValue->setText(missionObj["task_id"].toString());
    ui->taskNameValue->setText(missionObj["task_name"].toString());
    ui->taskTypeEdit->setText(QString::number(missionObj["task_type"].toInt()));
    
    // 显示任务优先级、状态、时间和描述信息（如果存在）
    ui->taskPriorityValue->setText(missionObj.contains("priority") ? QString::number(missionObj["priority"].toInt()) : "-");
    ui->taskStatusValue->setText(missionObj.contains("status") ? missionObj["status"].toString() : "-");
    ui->startTimeValue->setText(missionObj.contains("start_time") ? missionObj["start_time"].toString() : "-");
    ui->endTimeValue->setText(missionObj.contains("end_time") ? missionObj["end_time"].toString() : "-");
    ui->taskDescValue->setText(missionObj.contains("description") ? missionObj["description"].toString() : "-");
    
    // 显示点位信息
    if (taskDataObj.contains("graphics")) {
        QJsonArray graphicsArray = taskDataObj["graphics"].toArray();
        if (!graphicsArray.isEmpty()) {
            QJsonObject graphicObj = graphicsArray[0].toObject();
            if (graphicObj.contains("point")) {
                QJsonObject pointObj = graphicObj["point"].toObject();
                ui->longitudeEdit->setText(QString::number(pointObj["longitude"].toDouble(), 'f', 6));
                ui->latitudeEdit->setText(QString::number(pointObj["latitude"].toDouble(), 'f', 6));
                ui->altitudeEdit->setText(QString::number(pointObj["altitude"].toDouble()));
            }
        }
    }
    
    // 清空SAR信息
    ui->sarHeightEdit->setText("-");
    ui->sarAngleEdit->setText("-");
    ui->sarHeadingEdit->setText("-");
    ui->sarRangeEdit->setText("-");
    ui->sarResolutionEdit->setText("-");
    ui->sarBandwidthEdit->setText("-");
    ui->sarModeEdit->setText("-");
    clearRegionPointWidgets();
}

void PlanRequestWidget::displayBasicTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj)
{
    ui->taskIdValue->setText(missionObj["task_id"].toString());
    ui->taskNameValue->setText(missionObj["task_name"].toString());
    ui->taskTypeEdit->setText(QString::number(missionObj["task_type"].toInt()));
    
    // 显示任务优先级、状态、时间和描述信息（如果存在）
    ui->taskPriorityValue->setText(missionObj.contains("priority") ? QString::number(missionObj["priority"].toInt()) : "-");
    ui->taskStatusValue->setText(missionObj.contains("status") ? missionObj["status"].toString() : "-");
    ui->startTimeValue->setText(missionObj.contains("start_time") ? missionObj["start_time"].toString() : "-");
    ui->endTimeValue->setText(missionObj.contains("end_time") ? missionObj["end_time"].toString() : "-");
    ui->taskDescValue->setText(missionObj.contains("description") ? missionObj["description"].toString() : "-");
    
    ui->longitudeEdit->setText("-");
    ui->latitudeEdit->setText("-");
    ui->altitudeEdit->setText("-");
    ui->sarHeightEdit->setText("-");
    ui->sarAngleEdit->setText("-");
    ui->sarHeadingEdit->setText("-");
    ui->sarRangeEdit->setText("-");
    ui->sarResolutionEdit->setText("-");
    ui->sarBandwidthEdit->setText("-");
    ui->sarModeEdit->setText("-");
    clearRegionPointWidgets();
}

void PlanRequestWidget::clearInfoDisplay()
{
    ui->taskIdValue->setText("-");
    ui->taskNameValue->setText("-");
    ui->taskTypeEdit->setText("-");
    ui->taskPriorityValue->setText("-");
    ui->taskStatusValue->setText("-");
    ui->startTimeValue->setText("-");
    ui->endTimeValue->setText("-");
    ui->taskDescValue->setText("-");
    ui->longitudeEdit->setText("-");
    ui->latitudeEdit->setText("-");
    ui->altitudeEdit->setText("-");
    ui->sarHeightEdit->setText("-");
    ui->sarAngleEdit->setText("-");
    ui->sarHeadingEdit->setText("-");
    ui->sarRangeEdit->setText("-");
    ui->sarResolutionEdit->setText("-");
    ui->sarBandwidthEdit->setText("-");
    ui->sarModeEdit->setText("-");
    clearRegionPointWidgets();
}

void PlanRequestWidget::showInfoGroups(bool showSar, bool showRegion)
{
    ui->sarInfoGroupBox->setVisible(showSar);
    ui->regionInfoGroupBox->setVisible(showRegion);
    ui->positionInfoGroupBox->setVisible(showSar);
}

void PlanRequestWidget::onSaveChangesClicked()
{
    if (currentFilePath.isEmpty()) {
        QMessageBox::warning(this, "警告", "没有加载的文件可以保存");
        return;
    }
    
    if (currentJsonData.isEmpty()) {
        QMessageBox::warning(this, "警告", "没有有效的数据可以保存");
        return;
    }
    
    try {
        // 创建修改后的JSON对象
        QJsonObject modifiedJsonData = currentJsonData;
        QJsonObject missionObj = modifiedJsonData["mission"].toObject();
        
        // 更新任务类型
        if (!ui->taskTypeEdit->text().isEmpty() && ui->taskTypeEdit->text() != "-") {
            bool ok;
            int taskType = ui->taskTypeEdit->text().toInt(&ok);
            if (ok) {
                missionObj["task_type"] = taskType;
            }
        }
        
        QJsonObject taskDataObj = missionObj["task_data"].toObject();
        
        // 更新位置信息
        if (taskDataObj.contains("graphics")) {
            QJsonArray graphicsArray = taskDataObj["graphics"].toArray();
            if (!graphicsArray.isEmpty()) {
                QJsonObject graphicObj = graphicsArray[0].toObject();
                
                if (graphicObj.contains("point")) {
                    QJsonObject pointObj = graphicObj["point"].toObject();
                    
                    // 更新经度
                    if (!ui->longitudeEdit->text().isEmpty() && ui->longitudeEdit->text() != "-") {
                        bool ok;
                        double longitude = ui->longitudeEdit->text().toDouble(&ok);
                        if (ok) {
                            pointObj["longitude"] = longitude;
                        }
                    }
                    
                    // 更新纬度
                    if (!ui->latitudeEdit->text().isEmpty() && ui->latitudeEdit->text() != "-") {
                        bool ok;
                        double latitude = ui->latitudeEdit->text().toDouble(&ok);
                        if (ok) {
                            pointObj["latitude"] = latitude;
                        }
                    }
                    
                    // 更新高度
                    if (!ui->altitudeEdit->text().isEmpty() && ui->altitudeEdit->text() != "-") {
                        bool ok;
                        double altitude = ui->altitudeEdit->text().toDouble(&ok);
                        if (ok) {
                            pointObj["altitude"] = altitude;
                        }
                    }
                    
                    graphicObj["point"] = pointObj;
                    graphicsArray[0] = graphicObj;
                    taskDataObj["graphics"] = graphicsArray;
                }
                
                // 更新区域信息
                if (graphicObj.contains("region")) {
                    // 从区域输入框获取新的区域数据
                    QJsonArray newRegionArray = getRegionArrayFromWidgets();
                    if (!newRegionArray.isEmpty()) {
                        graphicObj["region"] = newRegionArray;
                        graphicsArray[0] = graphicObj;
                        taskDataObj["graphics"] = graphicsArray;
                    }
                }
            }
        }
        
        // 更新SAR信息
        if (taskDataObj.contains("sar_info")) {
            QJsonObject sarObj = taskDataObj["sar_info"].toObject();
            
            // 更新SAR高度
            if (!ui->sarHeightEdit->text().isEmpty() && ui->sarHeightEdit->text() != "-") {
                bool ok;
                double sarHeight = ui->sarHeightEdit->text().toDouble(&ok);
                if (ok) {
                    sarObj["sar_height"] = sarHeight;
                }
            }
            
            // 更新SAR角度
            if (!ui->sarAngleEdit->text().isEmpty() && ui->sarAngleEdit->text() != "-") {
                bool ok;
                double sarAngle = ui->sarAngleEdit->text().toDouble(&ok);
                if (ok) {
                    sarObj["sar_angle"] = sarAngle;
                }
            }
            
            // 更新SAR航向
            if (!ui->sarHeadingEdit->text().isEmpty() && ui->sarHeadingEdit->text() != "-") {
                bool ok;
                double sarHeading = ui->sarHeadingEdit->text().toDouble(&ok);
                if (ok) {
                    sarObj["sar_heading"] = sarHeading;
                }
            }
            
            // 更新SAR距离
            if (!ui->sarRangeEdit->text().isEmpty() && ui->sarRangeEdit->text() != "-") {
                bool ok;
                double sarRange = ui->sarRangeEdit->text().toDouble(&ok);
                if (ok) {
                    sarObj["sar_range"] = sarRange;
                }
            }
            
            // 更新SAR分辨率
            if (!ui->sarResolutionEdit->text().isEmpty() && ui->sarResolutionEdit->text() != "-") {
                bool ok;
                double sarResolution = ui->sarResolutionEdit->text().toDouble(&ok);
                if (ok) {
                    sarObj["sar_resolution"] = sarResolution;
                }
            }
            
            // 更新SAR带宽
            if (!ui->sarBandwidthEdit->text().isEmpty() && ui->sarBandwidthEdit->text() != "-") {
                bool ok;
                double sarBandwidth = ui->sarBandwidthEdit->text().toDouble(&ok);
                if (ok) {
                    sarObj["sar_bandwidth"] = sarBandwidth;
                }
            }
            
            // 更新SAR模式
            if (!ui->sarModeEdit->text().isEmpty() && ui->sarModeEdit->text() != "-") {
                sarObj["sar_mode"] = ui->sarModeEdit->text();
            }
            
            taskDataObj["sar_info"] = sarObj;
        }
        
        missionObj["task_data"] = taskDataObj;
        modifiedJsonData["mission"] = missionObj;
        
        // 保存到文件
        QJsonDocument doc(modifiedJsonData);
        QByteArray jsonData = doc.toJson(QJsonDocument::Indented);
        
        QFile file(currentFilePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "错误", "无法写入文件: " + currentFilePath);
            return;
        }
        
        QTextStream out(&file);
        out.setCodec("UTF-8"); // 设置UTF-8编码
        out << jsonData;
        file.close();
        
        // 更新内存中的数据
        currentJsonData = modifiedJsonData;
        
        QMessageBox::information(this, "成功", "文件保存成功!");
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "错误", QString("保存文件时出错: %1").arg(e.what()));
    } catch (...) {
        QMessageBox::critical(this, "错误", "保存文件时出现未知错误");
    }
}

// 区域坐标点管理方法实现
void PlanRequestWidget::addRegionPointWidget(double longitude, double latitude, double altitude)
{
    RegionPointWidget* widget = new RegionPointWidget();
    widget->pointIndex = regionPointWidgets.size();
    
    // 创建组框
    widget->groupBox = new QGroupBox(QString("坐标点 %1").arg(widget->pointIndex + 1));
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
        removeRegionPointWidget(widget);
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
    regionPointWidgets.append(widget);
    ui->regionPointsLayout->addWidget(widget->groupBox);
}

void PlanRequestWidget::removeRegionPointWidget(RegionPointWidget* widget)
{
    if (!widget || !regionPointWidgets.contains(widget)) {
        return;
    }
    
    // 从布局中移除
    ui->regionPointsLayout->removeWidget(widget->groupBox);
    
    // 从列表中移除
    regionPointWidgets.removeAll(widget);
    
    // 删除控件
    delete widget->groupBox;
    delete widget;
    
    // 更新剩余坐标点的标题
    for (int i = 0; i < regionPointWidgets.size(); ++i) {
        regionPointWidgets[i]->pointIndex = i;
        regionPointWidgets[i]->groupBox->setTitle(QString("坐标点 %1").arg(i + 1));
    }
}

void PlanRequestWidget::clearRegionPointWidgets()
{
    while (!regionPointWidgets.isEmpty()) {
        removeRegionPointWidget(regionPointWidgets.first());
    }
}

void PlanRequestWidget::updateRegionPointsFromJson(const QJsonArray &regionArray)
{
    clearRegionPointWidgets();
    
    for (int i = 0; i < regionArray.size(); ++i) {
        QJsonObject pointObj = regionArray[i].toObject();
        double longitude = pointObj["longitude"].toDouble();
        double latitude = pointObj["latitude"].toDouble();
        double altitude = pointObj["altitude"].toDouble();
        
        addRegionPointWidget(longitude, latitude, altitude);
    }
}

QJsonArray PlanRequestWidget::getRegionArrayFromWidgets()
{
    QJsonArray regionArray;
    
    for (RegionPointWidget* widget : regionPointWidgets) {
        if (!widget) continue;
        
        QJsonObject pointObj;
        
        bool longitudeOk, latitudeOk, altitudeOk;
        double longitude = widget->longitudeEdit->text().toDouble(&longitudeOk);
        double latitude = widget->latitudeEdit->text().toDouble(&latitudeOk);
        double altitude = widget->altitudeEdit->text().toDouble(&altitudeOk);
        
        if (longitudeOk && latitudeOk && altitudeOk) {
            pointObj["longitude"] = longitude;
            pointObj["latitude"] = latitude;
            pointObj["altitude"] = altitude;
            regionArray.append(pointObj);
        }
    }
    
    return regionArray;
}

void PlanRequestWidget::onAddRegionPointClicked()
{
    addRegionPointWidget(0.0, 0.0, 0.0);
}

void PlanRequestWidget::onDeleteRegionPointClicked()
{
    // 这个方法由各个删除按钮的lambda函数处理
}

void PlanRequestWidget::setC2Manager(std::shared_ptr<C2Manager> c2Manager)
{
    m_c2Manager = c2Manager;
}

void PlanRequestWidget::onPlanRequestClicked()
{
    // 检查是否有选中的文件和有效的JSON数据
    if (currentFilePath.isEmpty()) {
        QMessageBox::warning(this, "警告", "请先选择一个筹划文件");
        return;
    }
    
    if (currentJsonData.isEmpty()) {
        QMessageBox::warning(this, "警告", "当前没有有效的任务数据");
        return;
    }
    
    if (!m_c2Manager) {
        QMessageBox::critical(this, "错误", "C2Manager未初始化，无法发送规划请求");
        return;
    }
    
    try {
        // 将当前的JSON数据转换为字符串
        QJsonDocument jsonDoc(currentJsonData);
        QString jsonString = jsonDoc.toJson(QJsonDocument::Compact);
        
        // 调用C2Manager的processTaskPlan方法
        m_c2Manager->processTaskPlan(jsonString.toStdString());
        
        // 显示成功消息
        QMessageBox::information(this, "成功", 
            QString("规划请求已发送\n任务ID: %1\n任务名称: %2")
                .arg(currentJsonData["mission"].toObject()["task_id"].toString())
                .arg(currentJsonData["mission"].toObject()["task_name"].toString()));
        
        qDebug() << "规划请求已发送，任务数据：" << jsonString;
        
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "错误", 
            QString("发送规划请求时出错: %1").arg(e.what()));
    } catch (...) {
        QMessageBox::critical(this, "错误", "发送规划请求时出现未知错误");
    }
}
