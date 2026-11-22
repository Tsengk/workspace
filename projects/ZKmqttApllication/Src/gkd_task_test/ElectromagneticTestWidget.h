#ifndef ELECTROMAGNETICTESTWIDGET_H
#define ELECTROMAGNETICTESTWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QPushButton>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace Ui {
class ElectromagneticTestWidget;
}

class ElectromagneticTestWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ElectromagneticTestWidget(QWidget *parent = nullptr);
    ~ElectromagneticTestWidget();
    
    // 对外提供的添加任务接口
    bool addTask(const QString &taskId, const QString &unitId, 
                 const QString &targetId, const QString &frequency);
    
    // 对外提供的删除任务接口（根据任务编号删除）
    bool removeTask(const QString &taskId);
    
    // 删除所有任务
    void clearAllTasks();
    
    // 获取任务数量
    int getTaskCount() const;
    
    // 根据任务编号查找任务是否存在
    bool hasTask(const QString &taskId) const;

public slots:
    // 外部模块调用的槽函数，添加电磁侦干任务
    bool addElectromagneticTask(const QString &taskId, const QString &unitId, 
                               const QString &targetId, const QString &frequency);

signals:
    void taskAdded(const QJsonObject &taskInfo);
    void taskModified(const QJsonObject &taskInfo);
    void taskDeleted(const QString &taskId, bool success);
    void taskRemovalResult(const QString &taskId, bool success, const QString &message);

private slots:
    void onAddTaskClicked();
    void onDeleteTaskClicked();
    void onDeleteOneTaskClicked();
    void onTableItemChanged(QTableWidgetItem *item);

private:
    Ui::ElectromagneticTestWidget *ui;
    
    void setupTable();
    void addTableRow(const QString &taskId = "", const QString &unitId = "", 
                     const QString &targetId = "", const QString &frequency = "");
    void removeSelectedRows();
    QString generateTaskId();
    QJsonObject getTaskDataFromRow(int row);
    void executeTask(int row);
    int findTaskRow(const QString &taskId) const;
};

#endif // ELECTROMAGNETICTESTWIDGET_H