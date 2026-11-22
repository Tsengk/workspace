#include "ElectromagneticTestWidget.h"
#include "ui_ElectromagneticTestWidget.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QApplication>
#include <QInputDialog>
#include <QLineEdit>
#include "../common/events.h"
#include "../utils/eventbus.hpp"

ElectromagneticTestWidget::ElectromagneticTestWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ElectromagneticTestWidget)
{
    ui->setupUi(this);
    
    setupTable();
    
    connect(ui->addTaskButton, &QPushButton::clicked, this, &ElectromagneticTestWidget::onAddTaskClicked);
    connect(ui->deleteTaskButton, &QPushButton::clicked, this, &ElectromagneticTestWidget::onDeleteTaskClicked);
    connect(ui->deleteOneTaskButton, &QPushButton::clicked, this, &ElectromagneticTestWidget::onDeleteOneTaskClicked);
    connect(ui->emTaskTable, &QTableWidget::itemChanged, this, &ElectromagneticTestWidget::onTableItemChanged);
    
    // 订阅电磁侦干任务添加事件
    EB.Subscribe<std::function<void(const Events::ElectromagneticTaskEvent&)>, const Events::ElectromagneticTaskEvent&>(
        Events::ADD_ELECTROMAGNETIC_TASK, 
        [this](const Events::ElectromagneticTaskEvent& event) {
            QString taskId = QString::fromStdString(event.taskId);
            QString unitId = QString::fromStdString(event.unitId);
            QString targetId = QString::fromStdString(event.targetId);
            QString frequency = QString::fromStdString(event.frequency);
            
            bool success = addElectromagneticTask(taskId, unitId, targetId, frequency);
            if (success) {
                qDebug() << "通过事件成功添加电磁侦干任务：" << taskId;
            } else {
                qDebug() << "通过事件添加电磁侦干任务失败：" << taskId;
            }
        });
}

ElectromagneticTestWidget::~ElectromagneticTestWidget()
{
    delete ui;
}

void ElectromagneticTestWidget::setupTable()
{
    ui->emTaskTable->setColumnCount(5);

    QStringList headers;
    headers << "任务编号" << "作战单元编号" << "目标id" << "频率" << "操作";
    ui->emTaskTable->setHorizontalHeaderLabels(headers);

    // 基础行为设置
    ui->emTaskTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->emTaskTable->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->emTaskTable->setAlternatingRowColors(true);
    ui->emTaskTable->setSortingEnabled(false);
    ui->emTaskTable->setShowGrid(true);

    // 设置表格样式
    ui->emTaskTable->setStyleSheet(
        "QTableWidget {"
        "    background-color: #2b2b2b;"
        "    color: #ffffff;"
        "    gridline-color: #3a3a3a;"
        "    font-family: 'Microsoft YaHei';"
        "    font-size: 12px;"
        "    selection-background-color: #4a4a4a;"
        "    alternate-background-color: #353535;"
        "}"
        "QTableWidget::item {"
        "    padding: 8px;"
        "    border: none;"
        "    background-color: transparent;"
        "}"
        "QTableWidget::item:selected {"
        "    background-color: #4a90e2;"
        "    color: #ffffff;"
        "}"
        "QTableWidget::item:hover {"
        "    background-color: #404040;"
        "}"
        "QHeaderView::section {"
        "    background-color: #404040;"
        "    color: #ffffff;"
        "    padding: 10px;"
        "    border: 1px solid #3a3a3a;"
        "    font-weight: bold;"
        "    font-size: 12px;"
        "}"
        "QHeaderView::section:horizontal {"
        "    border-top: none;"
        "}"
        "QHeaderView::section:vertical {"
        "    border-left: none;"
        "}"
        "QScrollBar:vertical {"
        "    background-color: #2b2b2b;"
        "    width: 12px;"
        "    border: none;"
        "}"
        "QScrollBar::handle:vertical {"
        "    background-color: #555555;"
        "    border-radius: 6px;"
        "    min-height: 20px;"
        "}"
        "QScrollBar::handle:vertical:hover {"
        "    background-color: #666666;"
        "}"
    );

    // 设置行高并隐藏行号
    ui->emTaskTable->verticalHeader()->setDefaultSectionSize(40);
    ui->emTaskTable->verticalHeader()->setVisible(false);

    // 列宽和调整模式 - 自适应拉伸
    ui->emTaskTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);     // 任务编号 - 自动拉伸
    ui->emTaskTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);     // 作战单元编号 - 自动拉伸
    ui->emTaskTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);     // 目标id - 自动拉伸
    ui->emTaskTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);     // 频率 - 自动拉伸
    ui->emTaskTable->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);       // 操作 - 固定宽度

    // 设置操作列固定宽度，其他列自动分配
    ui->emTaskTable->setColumnWidth(4, 80);   // 操作固定宽度

    // 表头设置
    ui->emTaskTable->horizontalHeader()->setStretchLastSection(false);
    ui->emTaskTable->horizontalHeader()->setHighlightSections(false);
    ui->emTaskTable->horizontalHeader()->setSectionsClickable(true);

    // 启用表格的自适应调整
    ui->emTaskTable->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    ui->emTaskTable->resizeColumnsToContents();
}

void ElectromagneticTestWidget::addTableRow(const QString &taskId, const QString &unitId,
                                           const QString &targetId, const QString &frequency)
{
    // 检查targetId是否已存在
    if (!targetId.isEmpty()) {
        for (int i = 0; i < ui->emTaskTable->rowCount(); ++i) {
            QTableWidgetItem *targetItem = ui->emTaskTable->item(i, 2);
            qDebug() << "targetItem:" << targetItem;
            if (targetItem) {
                qDebug() << "targetItem->text():" << targetItem->text();
            }
            qDebug() << "targetId:" << targetId;
            if (targetItem && targetItem->text() == targetId) {
                // 找到已存在的targetId，处理frequency
                QComboBox *existingCombo = qobject_cast<QComboBox*>(ui->emTaskTable->cellWidget(i, 3));
                if (existingCombo && !frequency.isEmpty()) {
                    QString currentFreq = existingCombo->currentText();
                    if (currentFreq != frequency) {
                        // 检查是否已存在该频率选项
                        bool freqExists = false;
                        for (int j = 0; j < existingCombo->count(); ++j) {
                            qDebug() << existingCombo->itemText(j);
                            qDebug() << frequency;
                            if (existingCombo->itemText(j) == frequency) {
                                freqExists = true;
                                break;
                            }
                        }
                        qDebug() << frequency;
                        // 如果频率不存在则添加
                        if (!freqExists) {
                            existingCombo->addItem(frequency);
                        }
                    }
                }
                return; // 处理完毕，直接返回
            }
        }
    }

    // targetId不存在，添加新行
    int row = ui->emTaskTable->rowCount();
    ui->emTaskTable->insertRow(row);

    ui->emTaskTable->setItem(row, 0, new QTableWidgetItem(taskId.isEmpty() ? generateTaskId() : taskId));
    ui->emTaskTable->setItem(row, 1, new QTableWidgetItem(unitId));
    ui->emTaskTable->setItem(row, 2, new QTableWidgetItem(targetId));
    QComboBox *frequencyCombo = new QComboBox(this);
    frequencyCombo->setEditable(true);
    if (!frequency.isEmpty()) {
        frequencyCombo->addItem(frequency);
        // frequencyCombo->setCurrentText(frequency);
    }
    ui->emTaskTable->setCellWidget(row, 3, frequencyCombo);

    QPushButton *executeBtn = new QPushButton("执行", this);
    executeBtn->setProperty("row", row);
    connect(executeBtn, &QPushButton::clicked, [this, row]() {
        executeTask(row);
    });
    ui->emTaskTable->setCellWidget(row, 4, executeBtn);
}

void ElectromagneticTestWidget::onAddTaskClicked()
{
    addTableRow();
    qDebug() << "添加新的电磁侦干任务行";
}

void ElectromagneticTestWidget::onDeleteTaskClicked()
{
    removeSelectedRows();
}

void ElectromagneticTestWidget::onDeleteOneTaskClicked()
{
    bool ok;
    QString taskId = QInputDialog::getText(this, "删除任务",
                                          "请输入要删除的任务编号:",
                                          QLineEdit::Normal,
                                          "", &ok);
    if (ok && !taskId.isEmpty()) {
        // bool success = removeTask(taskId);
        EB.Publish<const std::string &>(Events::TEST_REMOVE_ZK_ITEM, taskId.toStdString());

        // if (success) {
        //     QMessageBox::information(this, "成功", "任务删除成功！");
        // } else {
        //     QMessageBox::warning(this, "失败", "未找到指定的任务编号，请检查后重试。");
        // }
    }
}

void ElectromagneticTestWidget::removeSelectedRows()
{
    QList<int> selectedRows;
    for (int i = 0; i < ui->emTaskTable->rowCount(); ++i) {
        if (ui->emTaskTable->item(i, 0) && ui->emTaskTable->item(i, 0)->isSelected()) {
            selectedRows.append(i);
        }
    }
    
    if (selectedRows.isEmpty()) {
        int currentRow = ui->emTaskTable->currentRow();
        if (currentRow >= 0) {
            selectedRows.append(currentRow);
        }
    }
    
    if (selectedRows.isEmpty()) {
        QMessageBox::information(this, "提示", "请选择要删除的任务行");
        return;
    }
    
    std::sort(selectedRows.rbegin(), selectedRows.rend());
    
    for (int row : selectedRows) {
        QJsonObject taskData = getTaskDataFromRow(row);
        QString taskId = taskData["taskId"].toString();
        ui->emTaskTable->removeRow(row);
        emit taskDeleted(taskId, true);
    }
    
    qDebug() << "删除了" << selectedRows.size() << "个电磁侦干任务";
}

void ElectromagneticTestWidget::onTableItemChanged(QTableWidgetItem *item)
{
    if (!item) return;
    
    int row = item->row();
    QJsonObject taskData = getTaskDataFromRow(row);
    emit taskModified(taskData);
    
    qDebug() << "电磁侦干任务数据已修改：" << taskData;
}

void ElectromagneticTestWidget::executeTask(int row)
{
    if (row < 0 || row >= ui->emTaskTable->rowCount()) {
        return;
    }
    
    QJsonObject taskData = getTaskDataFromRow(row);
    
    QMessageBox::information(this, "任务执行", 
        QString("执行电磁侦干任务：\n任务编号：%1\n作战单元：%2\n目标ID：%3\n频率：%4")
        .arg(taskData["taskId"].toString())
        .arg(taskData["unitId"].toString())
        .arg(taskData["targetId"].toString())
        .arg(taskData["frequency"].toString()));
    
    // 通过事件总线发布任务执行事件
    Events::ElectromagneticTaskEvent event(
        taskData["taskId"].toString().toStdString(),
        taskData["unitId"].toString().toStdString(),
        taskData["targetId"].toString().toStdString(),
        taskData["frequency"].toString().toStdString()
    );
    
    EB.Publish<const Events::ElectromagneticTaskEvent&>(Events::EXECUTE_ELECTROMAGNETIC_TASK, event);
    qDebug() << "发布电磁侦干任务执行事件：" << taskData["taskId"].toString();
}

QString ElectromagneticTestWidget::generateTaskId()
{
    return QString("EM_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
}

QJsonObject ElectromagneticTestWidget::getTaskDataFromRow(int row)
{
    QJsonObject taskData;
    
    if (row < 0 || row >= ui->emTaskTable->rowCount()) {
        return taskData;
    }
    
    auto getItemText = [this](int row, int col) -> QString {
        QTableWidgetItem *item = ui->emTaskTable->item(row, col);
        return item ? item->text() : QString();
    };
    
    taskData["taskId"] = getItemText(row, 0);
    taskData["unitId"] = getItemText(row, 1);
    taskData["targetId"] = getItemText(row, 2);

    // 特殊处理第3列的frequency（ComboBox）
    QComboBox *frequencyCombo = qobject_cast<QComboBox*>(ui->emTaskTable->cellWidget(row, 3));
    if (frequencyCombo) {
        taskData["frequency"] = frequencyCombo->currentText();
    } else {
        taskData["frequency"] = QString();
    }
    taskData["taskType"] = "electromagnetic_test";
    taskData["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    
    return taskData;
}

bool ElectromagneticTestWidget::addTask(const QString &taskId, const QString &unitId, 
                                       const QString &targetId, const QString &frequency)
{
    if (taskId.isEmpty()) {
        qDebug() << "添加任务失败：任务编号不能为空";
        return false;
    }
    
    // if (hasTask(taskId)) {
    //     qDebug() << "添加任务失败：任务编号已存在：" << taskId;
    //     return false;
    // }
    
    addTableRow(taskId, unitId, targetId, frequency);
    
    QJsonObject taskData;
    taskData["taskId"] = taskId;
    taskData["unitId"] = unitId;
    taskData["targetId"] = targetId;
    taskData["frequency"] = frequency;
    taskData["taskType"] = "electromagnetic_test";
    taskData["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    
    emit taskAdded(taskData);
    qDebug() << "成功添加电磁侦干任务：" << taskId;
    
    return true;
}

bool ElectromagneticTestWidget::removeTask(const QString &taskId)
{
    if (taskId.isEmpty()) {
        emit taskRemovalResult(taskId, false, "任务编号不能为空");
        return false;
    }
    
    int row = findTaskRow(taskId);
    if (row < 0) {
        emit taskRemovalResult(taskId, false, "未找到指定的任务编号");
        return false;
    }
    
    QJsonObject taskData = getTaskDataFromRow(row);
    ui->emTaskTable->removeRow(row);
    
    emit taskDeleted(taskId, true);
    emit taskRemovalResult(taskId, true, "任务删除成功");
    qDebug() << "成功删除电磁侦干任务：" << taskId;
    
    return true;
}

void ElectromagneticTestWidget::clearAllTasks()
{
    int rowCount = ui->emTaskTable->rowCount();
    for (int i = rowCount - 1; i >= 0; --i) {
        QJsonObject taskData = getTaskDataFromRow(i);
        QString taskId = taskData["taskId"].toString();
        ui->emTaskTable->removeRow(i);
        emit taskDeleted(taskId, true);
    }
    
    qDebug() << "清空所有电磁侦干任务，共删除" << rowCount << "个任务";
}

int ElectromagneticTestWidget::getTaskCount() const
{
    return ui->emTaskTable->rowCount();
}

bool ElectromagneticTestWidget::hasTask(const QString &taskId) const
{
    return findTaskRow(taskId) >= 0;
}

bool ElectromagneticTestWidget::addElectromagneticTask(const QString &taskId, const QString &unitId, 
                                                      const QString &targetId, const QString &frequency)
{
    return addTask(taskId, unitId, targetId, frequency);
}

int ElectromagneticTestWidget::findTaskRow(const QString &taskId) const
{
    for (int row = 0; row < ui->emTaskTable->rowCount(); ++row) {
        QTableWidgetItem *item = ui->emTaskTable->item(row, 0);
        if (item && item->text() == taskId) {
            return row;
        }
    }
    return -1;
}