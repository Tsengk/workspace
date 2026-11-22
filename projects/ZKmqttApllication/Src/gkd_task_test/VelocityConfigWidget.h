#ifndef VELOCITYCONFIGWIDGET_H
#define VELOCITYCONFIGWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QHeaderView>
#include <QMessageBox>
#include <QSettings>
#include <QCoreApplication>
#include <QDebug>
#include <memory>

// 前向声明
struct GKDCommonCfg;

namespace Ui {
class VelocityConfigWidget;
}

class VelocityConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit VelocityConfigWidget(QWidget *parent = nullptr);
    ~VelocityConfigWidget();

    // 设置GKDCommonCfg实例
    void setGKDConfig(std::shared_ptr<GKDCommonCfg> gkdConfig);

signals:
    void configSaved();
    void configReloaded();

private slots:
    void onRefreshClicked();
    void onSaveClicked();
    void onDeviceTaskTableCellChanged(int row, int column);
    void onDefaultTableCellChanged(int row, int column);

private:
    void setupUI();
    void loadConfiguration();
    void saveConfiguration();
    void populateDeviceTaskTable();
    void populateDefaultTable();
    void showStatusMessage(const QString &message, bool isError = false);

    Ui::VelocityConfigWidget *ui;

    // UI组件
    QVBoxLayout *mainLayout;
    QGroupBox *deviceTaskGroup;
    QGroupBox *defaultGroup;
    QTableWidget *deviceTaskTable;
    QTableWidget *defaultTable;
    QPushButton *refreshButton;
    QPushButton *saveButton;
    QLabel *statusLabel;

    // 数据
    std::shared_ptr<GKDCommonCfg> gkdConfig;
    QMap<QString, int> tempDeviceTaskVelocityMap;
    QMap<QString, int> tempDeviceVelocityMap;

    // 配置文件路径
    QString configFilePath;

    // 表格列定义
    enum DeviceTaskColumns {
        DEVICE_TASK_DEVICE_COL = 0,
        DEVICE_TASK_TYPE_COL = 1,
        DEVICE_TASK_VELOCITY_COL = 2
    };

    enum DefaultColumns {
        DEFAULT_DEVICE_COL = 0,
        DEFAULT_VELOCITY_COL = 1
    };
};

#endif // VELOCITYCONFIGWIDGET_H