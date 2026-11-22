#include "VelocityConfigWidget.h"
#include "ui_VelocityConfigWidget.h"
#include "../common/gkd_config.h"
#include <QTableWidgetItem>
#include <QStringList>
#include <QFile>
#include <QDir>

VelocityConfigWidget::VelocityConfigWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VelocityConfigWidget),
    gkdConfig(nullptr)
{
    ui->setupUi(this);

    // 获取配置文件路径
    configFilePath = QString("%1/device_velocity_config.ini").arg(QCoreApplication::applicationDirPath());

    // 连接信号槽
    connect(ui->refreshButton, &QPushButton::clicked, this, &VelocityConfigWidget::onRefreshClicked);
    connect(ui->saveButton, &QPushButton::clicked, this, &VelocityConfigWidget::onSaveClicked);
    connect(ui->deviceTaskTable, &QTableWidget::cellChanged, this, &VelocityConfigWidget::onDeviceTaskTableCellChanged);
    connect(ui->defaultTable, &QTableWidget::cellChanged, this, &VelocityConfigWidget::onDefaultTableCellChanged);

    // 设置表格属性
    setupUI();

    // 初始加载配置
    loadConfiguration();
}

VelocityConfigWidget::~VelocityConfigWidget()
{
    delete ui;
}

void VelocityConfigWidget::setGKDConfig(std::shared_ptr<GKDCommonCfg> gkdConfig)
{
    this->gkdConfig = gkdConfig;
    // 重新加载配置
    loadConfiguration();
}

void VelocityConfigWidget::setupUI()
{
    // 设置设备+任务类型表格
    ui->deviceTaskTable->setColumnCount(3);
    QStringList deviceTaskHeaders;
    deviceTaskHeaders << "设备编号" << "任务类型" << "速度值";
    ui->deviceTaskTable->setHorizontalHeaderLabels(deviceTaskHeaders);
    ui->deviceTaskTable->horizontalHeader()->setStretchLastSection(true);
    ui->deviceTaskTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    // 设置默认速度表格
    ui->defaultTable->setColumnCount(2);
    QStringList defaultHeaders;
    defaultHeaders << "设备编号" << "默认速度";
    ui->defaultTable->setHorizontalHeaderLabels(defaultHeaders);
    ui->defaultTable->horizontalHeader()->setStretchLastSection(true);
    ui->defaultTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    // 设置表格编辑模式
    ui->deviceTaskTable->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    ui->defaultTable->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);

    showStatusMessage("界面初始化完成");
}

void VelocityConfigWidget::loadConfiguration()
{
    // 清空临时数据
    tempDeviceTaskVelocityMap.clear();
    tempDeviceVelocityMap.clear();

    // 读取配置文件
    QSettings velocitySettings(configFilePath, QSettings::IniFormat);

    // 检查配置文件是否存在
    if (!QFile::exists(configFilePath)) {
        showStatusMessage("配置文件不存在，将创建默认配置", true);
        // 如果GKDCommonCfg可用，让它创建默认配置
        if (gkdConfig) {
            gkdConfig->loadDeviceVelocityConfig();
        }
        return;
    }

    // 读取设备+任务类型速度配置
    velocitySettings.beginGroup("DeviceTaskVelocity");
    QStringList taskKeys = velocitySettings.allKeys();
    foreach (const QString &key, taskKeys) {
        bool ok;
        int velocity = velocitySettings.value(key).toInt(&ok);
        if (ok) {
            tempDeviceTaskVelocityMap[key] = velocity;
        }
    }
    velocitySettings.endGroup();

    // 读取默认设备速度配置
    velocitySettings.beginGroup("DefaultVelocity");
    QStringList defaultKeys = velocitySettings.allKeys();
    foreach (const QString &key, defaultKeys) {
        bool ok;
        int velocity = velocitySettings.value(key).toInt(&ok);
        if (ok) {
            tempDeviceVelocityMap[key] = velocity;
        }
    }
    velocitySettings.endGroup();

    // 更新表格显示
    populateDeviceTaskTable();
    populateDefaultTable();

    showStatusMessage(QString("配置加载完成 - 设备+任务类型: %1个, 默认设备: %2个")
                     .arg(tempDeviceTaskVelocityMap.size())
                     .arg(tempDeviceVelocityMap.size()));
}

void VelocityConfigWidget::populateDeviceTaskTable()
{
    ui->deviceTaskTable->setRowCount(tempDeviceTaskVelocityMap.size());

    int row = 0;
    QMap<QString, int>::const_iterator i;
    for (i = tempDeviceTaskVelocityMap.constBegin(); i != tempDeviceTaskVelocityMap.constEnd(); ++i) {
        QString key = i.key();
        int velocity = i.value();

        // 解析设备编号和任务类型 (格式: deviceId_taskType)
        QStringList parts = key.split("_");
        QString deviceId = parts.size() > 0 ? parts[0] : "";
        QString taskType = parts.size() > 1 ? parts[1] : "";

        // 设备编号列 (只读)
        QTableWidgetItem *deviceItem = new QTableWidgetItem(deviceId);
        deviceItem->setFlags(deviceItem->flags() & ~Qt::ItemIsEditable);
        ui->deviceTaskTable->setItem(row, DEVICE_TASK_DEVICE_COL, deviceItem);

        // 任务类型列 (只读)
        QTableWidgetItem *taskItem = new QTableWidgetItem(taskType);
        taskItem->setFlags(taskItem->flags() & ~Qt::ItemIsEditable);
        ui->deviceTaskTable->setItem(row, DEVICE_TASK_TYPE_COL, taskItem);

        // 速度值列 (可编辑)
        QTableWidgetItem *velocityItem = new QTableWidgetItem(QString::number(velocity));
        ui->deviceTaskTable->setItem(row, DEVICE_TASK_VELOCITY_COL, velocityItem);

        row++;
    }
}

void VelocityConfigWidget::populateDefaultTable()
{
    ui->defaultTable->setRowCount(tempDeviceVelocityMap.size());

    int row = 0;
    QMap<QString, int>::const_iterator i;
    for (i = tempDeviceVelocityMap.constBegin(); i != tempDeviceVelocityMap.constEnd(); ++i) {
        QString deviceId = i.key();
        int velocity = i.value();

        // 设备编号列 (只读)
        QTableWidgetItem *deviceItem = new QTableWidgetItem(deviceId);
        deviceItem->setFlags(deviceItem->flags() & ~Qt::ItemIsEditable);
        ui->defaultTable->setItem(row, DEFAULT_DEVICE_COL, deviceItem);

        // 默认速度列 (可编辑)
        QTableWidgetItem *velocityItem = new QTableWidgetItem(QString::number(velocity));
        ui->defaultTable->setItem(row, DEFAULT_VELOCITY_COL, velocityItem);

        row++;
    }
}

void VelocityConfigWidget::saveConfiguration()
{
    // 创建配置文件目录（如果不存在）
    QDir dir = QFileInfo(configFilePath).dir();
    if (!dir.exists()) {
        dir.mkpath(".");
    }

    QSettings velocitySettings(configFilePath, QSettings::IniFormat);
    velocitySettings.clear(); // 清空现有配置

    // 保存设备+任务类型速度配置
    velocitySettings.beginGroup("DeviceTaskVelocity");
    velocitySettings.setValue("# 设备+任务类型速度映射配置", "");
    velocitySettings.setValue("# 格式: 设备编号_任务类型=速度值", "");

    QMap<QString, int>::const_iterator i;
    for (i = tempDeviceTaskVelocityMap.constBegin(); i != tempDeviceTaskVelocityMap.constEnd(); ++i) {
        velocitySettings.setValue(i.key(), i.value());
    }
    velocitySettings.endGroup();

    // 保存默认设备速度配置
    velocitySettings.beginGroup("DefaultVelocity");
    velocitySettings.setValue("# 默认设备速度（当找不到具体任务类型组合时使用）", "");

    QMap<QString, int>::const_iterator j;
    for (j = tempDeviceVelocityMap.constBegin(); j != tempDeviceVelocityMap.constEnd(); ++j) {
        velocitySettings.setValue(j.key(), j.value());
    }
    velocitySettings.endGroup();

    velocitySettings.sync();

    // 如果有GKDCommonCfg实例，重新加载配置
    if (gkdConfig) {
        qDebug() << "保存完成，开始重新加载GKDCommonCfg配置...";
        gkdConfig->loadDeviceVelocityConfig();
        qDebug() << "GKDCommonCfg配置重新加载完成";
        showStatusMessage("配置保存成功，参数已重新加载！");
    } else {
        qDebug() << "警告：GKDCommonCfg实例为空，无法重新加载配置";
        showStatusMessage("配置保存成功，但无法重新加载参数（GKDCommonCfg实例为空）", true);
    }

    emit configSaved();
}

void VelocityConfigWidget::onRefreshClicked()
{
    loadConfiguration();
    emit configReloaded();
}

void VelocityConfigWidget::onSaveClicked()
{
    saveConfiguration();
}

void VelocityConfigWidget::onDeviceTaskTableCellChanged(int row, int column)
{
    if (column == DEVICE_TASK_VELOCITY_COL) {
        QTableWidgetItem *deviceItem = ui->deviceTaskTable->item(row, DEVICE_TASK_DEVICE_COL);
        QTableWidgetItem *taskItem = ui->deviceTaskTable->item(row, DEVICE_TASK_TYPE_COL);
        QTableWidgetItem *velocityItem = ui->deviceTaskTable->item(row, DEVICE_TASK_VELOCITY_COL);

        if (deviceItem && taskItem && velocityItem) {
            QString key = QString("%1_%2").arg(deviceItem->text()).arg(taskItem->text());
            bool ok;
            int velocity = velocityItem->text().toInt(&ok);

            if (ok && velocity >= 0) {
                tempDeviceTaskVelocityMap[key] = velocity;
                showStatusMessage(QString("已修改 %1 的速度为 %2").arg(key).arg(velocity));
            } else {
                showStatusMessage("速度值必须是非负整数", true);
                // 恢复原值
                if (tempDeviceTaskVelocityMap.contains(key)) {
                    velocityItem->setText(QString::number(tempDeviceTaskVelocityMap[key]));
                }
            }
        }
    }
}

void VelocityConfigWidget::onDefaultTableCellChanged(int row, int column)
{
    if (column == DEFAULT_VELOCITY_COL) {
        QTableWidgetItem *deviceItem = ui->defaultTable->item(row, DEFAULT_DEVICE_COL);
        QTableWidgetItem *velocityItem = ui->defaultTable->item(row, DEFAULT_VELOCITY_COL);

        if (deviceItem && velocityItem) {
            QString deviceId = deviceItem->text();
            bool ok;
            int velocity = velocityItem->text().toInt(&ok);

            if (ok && velocity >= 0) {
                tempDeviceVelocityMap[deviceId] = velocity;
                showStatusMessage(QString("已修改设备 %1 的默认速度为 %2").arg(deviceId).arg(velocity));
            } else {
                showStatusMessage("速度值必须是非负整数", true);
                // 恢复原值
                if (tempDeviceVelocityMap.contains(deviceId)) {
                    velocityItem->setText(QString::number(tempDeviceVelocityMap[deviceId]));
                }
            }
        }
    }
}

void VelocityConfigWidget::showStatusMessage(const QString &message, bool isError)
{
    ui->statusLabel->setText(message);

    if (isError) {
        ui->statusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #ffebee;"
            "    border: 1px solid #f44336;"
            "    color: #d32f2f;"
            "    padding: 5px;"
            "    border-radius: 3px;"
            "}"
        );
    } else {
        ui->statusLabel->setStyleSheet(
            "QLabel {"
            "    background-color: #e8f5e8;"
            "    border: 1px solid #4caf50;"
            "    color: #2e7d32;"
            "    padding: 5px;"
            "    border-radius: 3px;"
            "}"
        );
    }

    qDebug() << "VelocityConfigWidget:" << message;
}