#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define WIN32_LEAN_AND_MEAN
#include <QMainWindow>
#include <QSystemTrayIcon>
#include <QMenu>
#include <QAction>
#include <QResizeEvent>
#include <QtWebEngineWidgets/QtWebEngineWidgets>
#include "mqtt/mq_client.h"
#include "utils/uav_data_parser.h"
#include "utils/utils_common.hpp"
#include "xsjsimui.h"
#include "gkd/c2manager.h"
#include "customtitlebar.h"
#include "Src/gkd_task_test/gkd_task_test.h"
#include "Src/gkd_task_test/GkdDataFusion.h"
#include "Src/gkd_game_test/GkdDataFusion_by.h"
#include "common/common_config.h"
#include "common/gkd_config.h"
#include "utils/log_sys.h"


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE
class MqClient;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_1_clicked();


    void on_action_exit_triggered();


    void on_pushButton_2_clicked();
    void on_pushButton_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();

    void on_pushButton_chat_clicked();
    void showMainWindow();
    void onTrayIconActivated(QSystemTrayIcon::ActivationReason reason);

    void setupSystemTray();
    void createTrayMenu();
    void setBYType(bool l,bool r1,bool r2, bool r3, bool r4); //军事博弈状态类型设定 基于零和模型

protected:
    void closeEvent(QCloseEvent *event) override;
    void changeEvent(QEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    CustomTitleBar *titleBar;
    QSystemTrayIcon *trayIcon;
    QMenu *trayIconMenu;
    QAction *showAction;
    QAction *hideAction;
    QAction *quitAction;
    
    MqClient _mqttClient;
    Ui::MainWindow *ui;
    std::shared_ptr<MqClient> _mqClient;
    std::shared_ptr<MqClient> _mq_gkd_Client;
    DroneDataParser* data_parser_;
    std::shared_ptr<rest_rcp_client> _rcp_client;
    std::shared_ptr<RCPServerBridge> _rcp;
    XsJSimUi* _xsj_sim_ui;

    std::shared_ptr<C2Manager> _c2Manager;
    gkd_task_test* _gkd_task_test;

    std::shared_ptr<GkdDataFusion> _gkdDataFusion;
   std::shared_ptr<GkdDataFusion_by> _gkdDataFusion_by;//军事博弈相关

    // 配置对象作为成员变量，防止过早销毁
    CommonCfg _cfg;
    GKDCommonCfg _gkd_cfg;

    // 日志系统
    Loggers _logger;


};
#endif // MAINWINDOW_H
