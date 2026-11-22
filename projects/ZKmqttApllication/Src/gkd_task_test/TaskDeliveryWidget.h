#ifndef TASKDELIVERYWIDGET_H
#define TASKDELIVERYWIDGET_H

#include <QWidget>
#include <QListWidgetItem>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <memory>

class C2Manager;

namespace Ui {
class TaskDeliveryWidget;
}

struct WaypointWidget {
    QGroupBox* groupBox;
    QLineEdit* longitudeEdit;
    QLineEdit* latitudeEdit;
    QLineEdit* altitudeEdit;
    QPushButton* deleteButton;
    int waypointIndex;
};

class TaskDeliveryWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TaskDeliveryWidget(QWidget *parent = nullptr);
    ~TaskDeliveryWidget();

    void setC2Manager(std::shared_ptr<C2Manager> c2Manager);

signals:
    void taskDelivered(const QJsonObject &taskInfo);

private slots:
    void onLoadTaskTemplateClicked();
    void onTaskTemplateSelected(QListWidgetItem *item);
    void onAddWaypointClicked();
    void onDeleteWaypointClicked();
    void onDeliverTaskClicked();

private:
    Ui::TaskDeliveryWidget *ui;
    QString currentTemplateFilePath;
    QJsonObject currentTaskData;
    QList<WaypointWidget*> waypointWidgets;
    std::shared_ptr<C2Manager> m_c2Manager;
    
    void loadTaskTemplateList();
    void parseTemplateFile(const QString &filePath);
    void displayTaskInfo(const QJsonObject &taskObj);
    void clearTaskDisplay();
    
    void addWaypointWidget(double longitude = 0.0, double latitude = 0.0, double altitude = 0.0);
    void removeWaypointWidget(WaypointWidget* widget);
    void clearWaypointWidgets();
    void updateWaypointsFromJson(const QJsonArray &waypointsArray);
    QJsonArray getWaypointArrayFromWidgets();
    
    QJsonObject buildTaskJson();
};

#endif // TASKDELIVERYWIDGET_H