#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "mqtt/mqconfig.h"
#include "common/gkd_config.h"
#include <QCloseEvent>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <QDesktopServices>
#include <QUrl>
#include <QStandardPaths>
#include <QDir>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , titleBar(nullptr)
    , trayIcon(nullptr)
    , trayIconMenu(nullptr)
{
    // 尽早初始化日志系统
    _logger.initLogger();
    _logger.INFO("========== 应用程序启动 ==========");
    _logger.INFO("MainWindow 构造函数开始");

    ui->setupUi(this);
    
    // 设置无边框
    setWindowFlags(Qt::FramelessWindowHint);
    
    // 创建自定义标题栏
    titleBar = new CustomTitleBar(this);
    titleBar->setTitle("信胜指控终端");
    
    // 将标题栏放置在窗口顶部
    titleBar->setGeometry(0, 0, this->width(), 30);
    titleBar->show();
    
    // 调整主窗口内容的位置，为标题栏留出空间
    if (ui->centralwidget) {
        ui->centralwidget->setContentsMargins(0, 35, 0, 0);
    }
    
    // 确保标题栏在最上层
    titleBar->raise();
    
    // 设置系统托盘
    setupSystemTray();
    
    // setWindowTitle("信胜指控终端");
   // QString filePath = QString("%1/log_"+QString::fromStdString(currentTime())+".txt").arg(QCoreApplication::applicationDirPath());
    //Logger::outfile.open(filePath.toStdString());
   //Logger::path ="";
    //Logger::target = log_target::file_and_terminal;
    // Logger::level = log_level::debug;

    // 加载配置文件 - 使用成员变量
   if (!_cfg.loadFromIni()) {
       qDebug() << "加载配置文件失败，将使用默认配置";
       _logger.WARNING("加载配置文件失败，将使用默认配置");
   } else {
       _logger.INFO("配置文件加载成功");
   }

    if (!_gkd_cfg.loadFromIni()) {
        qDebug() << "加载GKD配置文件失败，将使用默认配置";
        _logger.WARNING("加载GKD配置文件失败，将使用默认配置");
    } else {
        _logger.INFO("GKD配置文件加载成功");
    }

    string clientid = "2-2_mqtt_client";
    _mqClient= std::make_shared<MqClient>();
    _mqClient->Start(clientid,_cfg.ip,_cfg.port,_cfg.keepalive);
    _logger.INFO("MQTT客户端启动成功");

    _rcp_client = std::make_shared<rest_rcp_client>();
    _rcp_client->startConnectRcpServer(_cfg);
    _logger.INFO("RCP客户端启动成功");

    _c2Manager = std::make_shared<C2Manager>(_mqClient,_rcp_client,&_gkd_cfg);
    _logger.INFO("C2Manager初始化成功");

    _c2Manager->MQTTSubscribe();
    _c2Manager->MQTTSubscribe_DT();
    _c2Manager->RPCSubscribe(); //(2)(4) !!!!!!!!!!!!!!!!!!!!! 订阅任务请求计划
//    mqClient.GetMqttConn()->Subscribe("gh/901",[this](const QJsonObject& root) {
//        auto data = root["data"].toString();
//        QString message = data + " ";
//        QJsonObject send_json = root;
//        qDebug() << "receive" << data;
//        auto data2 = data_parser_->parseJson(root);
//        if(data2 == nullptr){
//            return;
//        }

//    });

    _xsj_sim_ui = new XsJSimUi();
    _xsj_sim_ui->startTimer(&(*_mqClient),&(*_rcp_client),_cfg,_gkd_cfg);
    _logger.INFO("仿真UI初始化成功");

    _gkd_task_test = new gkd_task_test();
    // 将C2Manager实例传递给gkd_task_test，以便PlanRequestWidget可以使用
    _gkd_task_test->setC2Manager(_c2Manager);
    // 将GKDCommonCfg实例传递给gkd_task_test，确保速度配置修改能够影响C2Manager
    // 注意：这里传递的是MainWindow的_gkd_cfg的shared_ptr包装，确保是同一个实例
    _gkd_task_test->setGKDConfig(std::shared_ptr<GKDCommonCfg>(&_gkd_cfg, [](GKDCommonCfg*){}));


    _gkdDataFusion = std::make_shared<GkdDataFusion>(nullptr,_cfg);
    _gkdDataFusion_by = std::make_shared<GkdDataFusion_by>(nullptr,_cfg);
    _logger.INFO("GKD数据融合模块初始化成功");
    _logger.INFO("========== MainWindow 初始化完成 ==========");

    connect(_gkdDataFusion_by.get(), &GkdDataFusion_by::setBYType, this, &MainWindow::setBYType);

}

MainWindow::~MainWindow()
{
    delete _xsj_sim_ui;
    delete _gkd_task_test;
    delete ui;
    if (trayIcon) {
        trayIcon->hide();
        delete trayIcon;
    }
    if (trayIconMenu) {
        delete trayIconMenu;
    }
}

void MainWindow::on_pushButton_1_clicked()
{
    QString topic = "ZK/101";
    // json sub_task = {
    //                  "task_id","GH_001",
    //                  "task_name","建图规划请求",
    //                  "mission_type",1

    // };

    // json response;
    // response["data"] = "ss";
    // mqClient.GetMqttConn()->PrimitivePublish(topic,response.dump());



    // QJsonObject root{
    //                  {"result", true}, {"topic_header", 1}, {"jurisdiction", 7}};
    // mqClient.GetMqttConn()->PrimitivePublish(topic, root);
}

void MainWindow::on_action_exit_triggered()
{
    // 真正退出应用程序
    if (trayIcon) {
        trayIcon->hide();
    }
    QApplication::quit();
}


void MainWindow::on_pushButton_2_clicked()
{
    ui->label_tips->setText("装载测试数据.......");
    if(_gkdDataFusion->isHidden()){
        _gkdDataFusion->show();
    }
    _gkdDataFusion->raise();
    _gkdDataFusion->activateWindow();
    ui->label_tips->setText("");
}

void MainWindow::on_pushButton_clicked()
{
    if(_gkd_task_test->isHidden()){
        _gkd_task_test->show();
    }
    _gkd_task_test->raise();
    _gkd_task_test->activateWindow();
}

void MainWindow::on_pushButton_3_clicked()
{
    if(_xsj_sim_ui->isHidden()){
        _xsj_sim_ui->show();
    }
}

void MainWindow::on_pushButton_4_clicked()
{
    ui->label_tips->setText("装载测试数据.......");
    if(_gkdDataFusion_by->isHidden()){
        _gkdDataFusion_by->show();
    }
    _gkdDataFusion_by->raise();
    _gkdDataFusion_by->activateWindow();
    ui->label_tips->setText("");
}

void MainWindow::setupSystemTray()
{
    createTrayMenu();
    
    trayIcon = new QSystemTrayIcon(this);
    trayIcon->setContextMenu(trayIconMenu);
    
    // 设置托盘图标，使用项目中的ZK图标
    QString iconPath = QString("%1/icons/zk_tray_icon.svg").arg(QCoreApplication::applicationDirPath());
    QIcon icon = QIcon(iconPath);
    if (icon.isNull()) {
        // 如果SVG图标加载失败，尝试其他版本
        iconPath = QString("%1/icons/zk_icon_v2.svg").arg(QCoreApplication::applicationDirPath());
        icon = QIcon(iconPath);
        if (icon.isNull()) {
            // 如果仍然失败，创建一个简单的默认图标
            icon = style()->standardIcon(QStyle::SP_ComputerIcon);
        }
    }
    trayIcon->setIcon(icon);
    
    trayIcon->setToolTip(tr("ZKAPP"));
    
    // 连接托盘图标的激活信号
    connect(trayIcon, &QSystemTrayIcon::activated,
            this, &MainWindow::onTrayIconActivated);
    
    // 显示托盘图标
    trayIcon->show();
   // onTrayIconActivated();

    // 将托盘图标注册到TrayNotificationManager
    TrayNotificationManager::getInstance().setTrayIcon(trayIcon);
    
    // 显示托盘消息提示用户程序在后台运行
    trayIcon->showMessage(tr("ZKAPP"),
                         tr("程序已最小化到系统托盘，双击托盘图标或右键菜单可以恢复窗口。"),
                         QSystemTrayIcon::Information, 3000);
      showMainWindow();
}

void MainWindow::createTrayMenu()
{
    showAction = new QAction(tr("显示主窗口"), this);
    connect(showAction, &QAction::triggered, this, &MainWindow::showMainWindow);
    
    hideAction = new QAction(tr("隐藏主窗口"), this);
    connect(hideAction, &QAction::triggered, this, &QWidget::hide);
    
    quitAction = new QAction(tr("退出程序"), this);
    connect(quitAction, &QAction::triggered, this, &MainWindow::on_action_exit_triggered);
    
    trayIconMenu = new QMenu(this);
    trayIconMenu->addAction(showAction);
    trayIconMenu->addAction(hideAction);
    trayIconMenu->addSeparator();
    trayIconMenu->addAction(quitAction);
}

void MainWindow::setBYType(bool l,bool r1, bool r2, bool r3, bool r4)
{

    _c2Manager->setBYType(l,r1,r2,r3,r4);
}

void MainWindow::showMainWindow()
{
    show();
    raise();
    activateWindow();
}

void MainWindow::onTrayIconActivated(QSystemTrayIcon::ActivationReason reason)
{
    switch (reason) {
    case QSystemTrayIcon::DoubleClick:
        showMainWindow();
        break;
    case QSystemTrayIcon::Trigger:
        // 单击事件，可以根据需要处理
        break;
    default:
        break;
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (trayIcon && trayIcon->isVisible()) {
        // 隐藏到托盘而不是真正关闭
        hide();
        event->ignore();
        
        // 首次最小化到托盘时提示用户
        static bool firstTime = true;
        if (firstTime) {
            trayIcon->showMessage(tr("ZKAPP"),
                                 tr("程序仍在后台运行。要完全退出程序，请在托盘图标上右键选择退出。"),
                                 QSystemTrayIcon::Information, 3000);
            firstTime = false;
        }
    } else {
        // 如果托盘不可用，则正常关闭
        event->accept();
    }
}

void MainWindow::changeEvent(QEvent *event)
{
    QMainWindow::changeEvent(event);
    if (event->type() == QEvent::WindowStateChange) {
        if (isMinimized() && trayIcon && trayIcon->isVisible()) {
            // 最小化时隐藏到托盘
            hide();
        }
    }
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);

    // 调整标题栏大小以适应窗口宽度
    if (titleBar) {
        titleBar->setGeometry(0, 0, event->size().width(), 30);
        titleBar->raise(); // 确保标题栏始终在最上层
    }
}

void MainWindow::on_pushButton_chat_clicked()
{
    // 从配置对象中获取聊天参数
    QString serverUrl = QString("http://%1:%2/app-api/im/open/user-online")
                       .arg(QString::fromStdString(_gkd_cfg.chat_server_ip))
                       .arg(QString::fromStdString(_gkd_cfg.chat_server_port));

    qDebug() << "Chat Server URL:" << serverUrl;

    QString userName = QString::fromStdString(_gkd_cfg.chat_user_name);
    QString userMobile = QString::fromStdString(_gkd_cfg.chat_user_mobile);
    QString userAvatar = QString::fromStdString(_gkd_cfg.chat_user_avatar);
    QString serverIp = QString::fromStdString(_gkd_cfg.chat_server_ip);
    QString serverPort = QString::fromStdString(_gkd_cfg.chat_server_port);

    // 创建网络请求管理器
    QNetworkAccessManager *manager = new QNetworkAccessManager(this);

    // 构建JSON请求体
    QJsonObject requestBody;
    requestBody["avatar"] = userAvatar;
    requestBody["name"] = userName;
    requestBody["gender"] = 1;
    requestBody["mobile"] = userMobile.toLongLong();

    QJsonDocument jsonDoc(requestBody);
    QByteArray jsonData = jsonDoc.toJson();

    // 创建HTTP请求
    QNetworkRequest request;
    request.setUrl(QUrl(serverUrl));
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    request.setRawHeader("tenant-id", "1");

    // 发送POST请求
    QNetworkReply *reply = manager->post(request, jsonData);

    // 连接请求完成信号
    connect(reply, &QNetworkReply::finished, this, [=]() {
        if (reply->error() == QNetworkReply::NoError) {
            // 解析响应
            QByteArray responseData = reply->readAll();
            QJsonDocument responseDoc = QJsonDocument::fromJson(responseData);
            QJsonObject responseObj = responseDoc.object();

            if (responseObj.contains("data")) {
                QJsonObject dataObj = responseObj["data"].toObject();

                // 生成token
                QJsonDocument tokenDoc(dataObj);
                QString tokenJson = tokenDoc.toJson(QJsonDocument::Compact);
                QString token = QString::fromUtf8(tokenJson.toUtf8().toBase64());

                // 获取accessToken用于WebSocket
                QString accessToken = dataObj["accessToken"].toString();

                // 生成HTML内容
                QString htmlContent = QString(
                    "<!DOCTYPE html>"
                    "<html>"
                    "<head>"
                    "<meta charset='UTF-8'>"
                    "<title>聊天窗口</title>"
                    "<style>"
                    "body{margin:0;padding:20px;background:#f5f5f5}"
                    "#container{max-width:1200px;margin:0 auto;background:#fff;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,0.1)}"
                    "#header{padding:16px;background:#2c3e50;color:#fff;font-size:18px;font-weight:bold}"
                    "iframe{width:100%;height:700px;border:none}"
                    "#ws-info{padding:12px 16px;background:#e3f2fd;color:#1976d2;font-size:14px;border-top:1px solid #e0e0e0}"
                    "</style>"
                    "</head>"
                    "<body>"
                    "<div id='container'>"
                    "<div id='header'>在线聊天</div>"
                    "<iframe src='http://%1:%2/client/login?token=%3&tenantId=1'></iframe>"
                    "<div id='ws-info'>WebSocket地址: ws://%1:%2/infra/ws?token=%4</div>"
                    "</div>"
                    "</body>"
                    "</html>"
                ).arg(serverIp).arg(serverPort).arg(token).arg(accessToken);

                // 创建临时HTML文件
                QString tempDir = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
                QString htmlFilePath = QDir(tempDir).absoluteFilePath("chat.html");

                QFile htmlFile(htmlFilePath);
                if (htmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&htmlFile);
                    out.setCodec("UTF-8");
                    out << htmlContent;
                    htmlFile.close();

                    // 打开HTML文件
                    QDesktopServices::openUrl(QUrl::fromLocalFile(htmlFilePath));

                    QMessageBox::information(this, "成功", "聊天窗口已打开");
                } else {
                    QMessageBox::critical(this, "错误", "无法创建HTML文件");
                }
            } else {
                QMessageBox::critical(this, "错误", "服务器响应格式错误");
            }
        } else {
            QMessageBox::critical(this, "网络错误",
                QString("请求失败: %1").arg(reply->errorString()));
        }

        reply->deleteLater();
        manager->deleteLater();
    });
}

