#ifndef PLANREQUESTWIDGET_H
#define PLANREQUESTWIDGET_H

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

// 前向声明
class C2Manager;

namespace Ui {
class PlanRequestWidget;
}

// 区域坐标点编辑组件
struct RegionPointWidget {
    QGroupBox* groupBox;
    QLineEdit* longitudeEdit;
    QLineEdit* latitudeEdit;
    QLineEdit* altitudeEdit;
    QPushButton* deleteButton;
    int pointIndex;
};

class PlanRequestWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlanRequestWidget(QWidget *parent = nullptr);
    ~PlanRequestWidget();

    // 设置C2Manager实例的接口
    void setC2Manager(std::shared_ptr<C2Manager> c2Manager);

signals:
    void planFileLoaded(const QString &filePath);
    void taskInfoChanged(const QJsonObject &taskInfo);

private slots:
    void onLoadPlanFileClicked();
    void onPlanFileSelected(QListWidgetItem *item);
    void onSaveChangesClicked();
    void onAddRegionPointClicked();
    void onDeleteRegionPointClicked();
    void onPlanRequestClicked(); // 新增的规划请求槽函数

private:
    Ui::PlanRequestWidget *ui;
    QString currentFilePath; // 当前加载的文件路径
    QJsonObject currentJsonData; // 当前的JSON数据
    QList<RegionPointWidget*> regionPointWidgets; // 区域坐标点控件列表
    std::shared_ptr<C2Manager> m_c2Manager; // C2Manager实例
    
    void loadPlanFileList();
    void parseTxtFile(const QString &filePath);
    void displayNewSarTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj, const QJsonObject &taskDataObj);
    void displayNewRegionTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj, const QJsonObject &taskDataObj);
    void displayPointTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj, const QJsonObject &taskDataObj);
    void displayBasicTaskInfo(const QJsonObject &rootObj, const QJsonObject &missionObj);
    void clearInfoDisplay();
    void showInfoGroups(bool showSar, bool showRegion);
    
    // 区域坐标点管理方法
    void addRegionPointWidget(double longitude = 0.0, double latitude = 0.0, double altitude = 0.0);
    void removeRegionPointWidget(RegionPointWidget* widget);
    void clearRegionPointWidgets();
    void updateRegionPointsFromJson(const QJsonArray &regionArray);
    QJsonArray getRegionArrayFromWidgets();
};

#endif // PLANREQUESTWIDGET_H