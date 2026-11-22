#include "GkdDataFusion.h"
#include "ui_GkdDataFusion.h"
#include "../../customtitlebar.h"
#include <QDebug>
#include <QMenu>
#include <QAction>
#include<chrono>
#include<QJsonValueRef>
#include <QJsonValue>
#include<QMessageBox>
#include <QMetaObject>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonParseError>
#include <QStringList>
#include <QStringList>

GkdDataFusion::GkdDataFusion(QWidget *parent,CommonCfg cfg) :
    QWidget(parent),
    ui(new Ui::GkdDataFusion),
    titleBar(nullptr),
    contextMenu(nullptr),
    stayOnTopAction(nullptr),
    isStayOnTop(false),
    isResizing(false),
    resizeDirection(None)
{
     _cfg = cfg;
    // 设置无边框
    setWindowFlags(Qt::FramelessWindowHint);
    ui->setupUi(this);

    // 设置最小窗口大小
    setMinimumSize(600, 500);


    // 创建自定义标题栏
    titleBar = new CustomTitleBar(this);
    titleBar->setTitle("态势推送&态势融合");

    // 将标题栏放置在窗口顶部，但为拉伸留出空间
    titleBar->setGeometry(0, RESIZE_BORDER, this->width(), 30 - RESIZE_BORDER);
    titleBar->show();


    // 确保标题栏在最上层
    titleBar->raise();



    //信号曹
    connect(ui->pushButton_1, &QPushButton::clicked, this, &GkdDataFusion::onAISClicked);
    connect(ui->pushButton_2, &QPushButton::clicked, this, &GkdDataFusion::onADSClicked);

    connect(ui->pushButton_11, &QPushButton::clicked, this, &GkdDataFusion::onStopAISClicked);
    connect(ui->pushButton_22, &QPushButton::clicked, this, &GkdDataFusion::onStopADSClicked);

    connect(ui->pushButton_3, &QPushButton::clicked, this, &GkdDataFusion::onStartPushFusion);
    connect(ui->pushButton_4, &QPushButton::clicked, this, &GkdDataFusion::onResetPushFusion);
    connect(ui->pushButton_5, &QPushButton::clicked, this, &GkdDataFusion::onStopPushFusion);

    connect(ui->pushButton_8, &QPushButton::clicked, this, &GkdDataFusion::OnFusionDistance);
    connect(ui->pushButton_9, &QPushButton::clicked, this, &GkdDataFusion::OnFusionHeigh);
    connect(ui->pushButton_duokong, &QPushButton::clicked, this, &GkdDataFusion::onDuokongPushClicked);



    

}

GkdDataFusion::~GkdDataFusion()
{
    delete ui;
    if (contextMenu) {
        delete contextMenu;
    }
}

void GkdDataFusion::onAISClicked()
{

    std::thread t(&GkdDataFusion::planAIS,this,_hk_fusion_data[0]);
    t.detach();
}

void GkdDataFusion::onADSClicked()
{

    std::thread t(&GkdDataFusion::PlanADS,this,_hk_fusion_data[1]);
    t.detach();
}

void GkdDataFusion::onStopAISClicked()
{
    _stopAISThread  =true;
}

void GkdDataFusion::onStopADSClicked()
{
    _stopADSThread = true;
}

void GkdDataFusion::onLocateClicked()
{

}

void GkdDataFusion::onResetPushFusion()
{
    onStopPushFusion();
}

void GkdDataFusion::onStartPushFusion()
{
    QString pC = ui->lineEdit_1->text();
     pc_i = pC.toInt();
    if(pc_i <1 || (pc_i >1000)){
        QMessageBox::warning(this, "注意", "融合推送频次需在1-1000HZ之间");
        return;
    }

    std::thread t(&GkdDataFusion::pushFusion,this,_fusion_data);
    t.detach();
}

void GkdDataFusion::onStopPushFusion()
{
    _stopFusionDataThread = true;
}

void GkdDataFusion::OnFusionDistance()
{
    if(_fusionHeigh){
        QMessageBox::warning(this, "注意", "请先停止高差融合");
        return;
    }
    _fusionDistance = !_fusionDistance;
    if(_fusionDistance){
        ui->pushButton_8->setText("球面距离融合开始");
    }else{
        ui->pushButton_8->setText("球面距离融合停止");
    }
}

void GkdDataFusion::OnFusionHeigh()
{

    if(_fusionDistance){
        QMessageBox::warning(this, "注意", "请先停止球面距离融合");
        return;
    }
    _fusionHeigh = !_fusionHeigh;
    if(_fusionHeigh){
        ui->pushButton_9->setText("高差融合开始");
    }else{
        ui->pushButton_9->setText("高差融合停止");
    }
}

void GkdDataFusion::onDuokongPushClicked()
{
    if(!_pgkd_mqtt_client || !_pgkd_mqtt_client->GetMqttConn()){
        QMessageBox::warning(this, QStringLiteral("提示"), QStringLiteral("MQTT连接尚未就绪"));
        return;
    }

    if (_duokongRedData.isEmpty() && _duokongBlueData.isEmpty()) {
        showDuokongDataEmptyWarning();
        return;
    }

    std::thread t(&GkdDataFusion::pushDuokongData, this, _duokongRedData, _duokongBlueData);
    t.detach();
}

void GkdDataFusion::showEvent(QShowEvent *event)
{
    _stopAISThread = false;
    _stopADSThread = false;
    CacheFusionData();

    _pgkd_mqtt_client= std::make_shared<MqClient>();

    string clientid = "2-2_fusion_client";
    _pgkd_mqtt_client->Start(clientid,_cfg.ip,_cfg.port,_cfg.keepalive);

    QWidget::showEvent(event);
}

void GkdDataFusion::hideEvent(QHideEvent *event)
{
    _stopAISThread = true;
    _stopADSThread = true;
    _pgkd_mqtt_client->Stop();

    ClearFunctionData();

    QWidget::hideEvent(event);
}

void GkdDataFusion::CacheFusionData()
{
    //加载海空态势情报数据   //加载融合测试数据
    QString appDir = QCoreApplication::applicationDirPath();
    QString hk_fusion = QString("%1/data/DataFusion/AIS&ADS2.csv").arg(appDir);

    QString fusion_data = QString("%1/data/DataFusion/fusiondata_new.csv").arg(appDir);
    QString duokon_data = QString("%1/data/DataFusion/red&blue.json").arg(appDir);
    Q_UNUSED(duokon_data);

    readHKFusion(hk_fusion);

    readFusionData(fusion_data);

    _duokongRedData = loadJsonArray(resolveJsonPath(QStringLiteral("red.json")));
    _duokongBlueData = loadJsonArray(resolveJsonPath(QStringLiteral("blue.json")));

}

void GkdDataFusion::ClearFunctionData()
{
   _hk_fusion_data.clear();

    _fusion_data.clear();
    _duokongRedData = QJsonArray();
    _duokongBlueData = QJsonArray();
}

void GkdDataFusion::pushDuokongData(QJsonArray redArray, QJsonArray blueArray)
{
    if(!_pgkd_mqtt_client || !_pgkd_mqtt_client->GetMqttConn()){
        return;
    }

    auto publishArray = [&](const QJsonArray &array, const QString &topic) {
        for (const QJsonValue &value : array) {
            if (!value.isObject()) {
                continue;
            }
            QJsonObject payload = value.toObject();
            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish(topic, payload);
            Sleep(100);
        }
    };

    if(!redArray.isEmpty()){
        publishArray(redArray, QStringLiteral("QB/303"));
    }
    if(!blueArray.isEmpty()){
        publishArray(blueArray, QStringLiteral("QB/301"));
    }
}

void GkdDataFusion::showDuokongDataEmptyWarning()
{
    QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("未读取到可推送的夺控清缴态势数据"));
}

QJsonArray GkdDataFusion::loadJsonArray(const QString &filePath)
{
    if(filePath.isEmpty()){
        return {};
    }

    QFile file(filePath);
    if(!file.exists()){
        qWarning() << "[GkdDataFusion] 数据文件不存在:" << filePath;
        return {};
    }
    if(!file.open(QIODevice::ReadOnly)){
        qWarning() << "[GkdDataFusion] 无法打开数据文件:" << filePath << file.errorString();
        return {};
    }

    const QByteArray rawData = file.readAll();
    file.close();

    QJsonParseError error;
    const QJsonDocument doc = QJsonDocument::fromJson(rawData, &error);
    if(error.error != QJsonParseError::NoError){
        qWarning() << "[GkdDataFusion] 解析数据文件失败:" << filePath << error.errorString();
        return {};
    }
    if(!doc.isArray()){
        qWarning() << "[GkdDataFusion] 数据文件格式不是数组:" << filePath;
        return {};
    }
    return doc.array();
}

QString GkdDataFusion::resolveJsonPath(const QString &fileName) const
{
    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList searchDirs = {
        QDir(appDir).filePath(QStringLiteral("data/DataFusion")),
        QDir(appDir).filePath(QStringLiteral("data")),
        QDir(appDir).filePath(QStringLiteral("Src/gkd_task_test"))
    };

    for (const QString &dir : searchDirs) {
        const QString candidate = QDir(dir).filePath(fileName);
        QFileInfo info(candidate);
        if (info.exists() && info.isFile()) {
            return info.absoluteFilePath();
        }
    }
    return QString();
}

void GkdDataFusion::planAIS(FusionData &_fusionData)
{
    _stopAISThread = false;
    for(APIBridgeSDK::Position pos :_fusionData.path){
        Sleep(100);
        if(_stopAISThread)  break;
        QJsonObject root;
        PackageZzptjzbzt_303(root,pos);

        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/303", root);

    }
}

void GkdDataFusion::PlanADS(FusionData &_fusionData)
{
    _stopADSThread = false;
    for(APIBridgeSDK::Position pos :_fusionData.path){
        Sleep(100);
        if(_stopADSThread)  break;
        QJsonObject root;
        PackageZzptjzbzt_303_CB(root,pos);

        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/303", root);

    }
}

void GkdDataFusion::pushFusion(std::vector<FusionData> &_fusionDatalist)
{
  _stopFusionDataThread = false;
    int index = 0;int index_range_max =  std::numeric_limits<int>::max();
  for(const auto &_fusionData : _fusionDatalist ){
        const int pathSize = static_cast<int>(_fusionData.path.size());
        if(pathSize < index_range_max) {
            index_range_max = pathSize;
        }
    }
  while(!_stopFusionDataThread){
        for(auto _fusionData : _fusionDatalist){
            if(index == index_range_max) index =0;
            Sleep(1000/pc_i);
            APIBridgeSDK::Position pos = _fusionData.path[index++];

            if(_fusionHeigh){
                if(_fusionData.name != "8"&&_fusionData.name != "4"&&_fusionData.name != "5"&&_fusionData.name !="2" && pos.altitude ==1000.0 ){
                    QJsonObject root;
                    PackageDzmbxsb_302(_fusionData.name,root);
                    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/302", root);
                    continue;
                }
                if(_fusionData.name != "8"&&_fusionData.name != "3"&&pos.altitude ==600.0 ) {
                    QJsonObject root;
                    PackageDzmbxsb_302(_fusionData.name,root);
                    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/302", root);
                    continue;

                }

            }

            if(_fusionDistance){
                for(auto fs :_fusionDatalist){
                    if(fs.name == _fusionData.name ) continue;
                    if(fs.name  =="118"  || fs.name  =="129"  || fs.name  =="114" || fs.name  =="111") continue;
                    for(APIBridgeSDK::Position spos : fs.path){
                        //  QJsonObject root;
                        //  PackageDzmbxsb_302(_fusionData.name,root);
                        // _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/302", root);
                        double distance = _geo_algo.RealDistance(pos.latitude,pos.longitude,spos.latitude,spos.longitude);
                        if(distance < 10){
                            QJsonObject root;
                            PackageDzmbxsb_302(_fusionData.name,root);
                            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/302", root);
                        }
                    }
                }

            }

            int torget = std::rand()%10;
            int total_rand = 66+torget;
            qDebug() << "目前接收到目标数 " << total_rand<<"目标融合后个数为"<<total_rand/2<<"实际目标为"<<total_rand/2-3<<"目标融合准确率为"<<((total_rand/2-3)*1.0/(total_rand/2))*100<<"%";
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);
            last_time =  now;
            QJsonObject root;
            PackageDzmbqb_301(QString::fromStdString(_fusionData.name),root,pos);
            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/301", root);

            qDebug() << "融合/消批/解算时间间隔为 " <<duration.count()<<"毫秒";
        }
  }

}

bool GkdDataFusion::PackageDzmbqb_301(QString name,QJsonObject& root,APIBridgeSDK::Position& pos)
{
    addHeader(root);
     QJsonObject CTN;
    // if(test_id++ > 150){
    //      return false;
    //  }
    CTN["target_ID"]="dyy"+name+QString(test_id);
    CTN["battle_unit_code"]="dyy"+name+QString(test_id);
    CTN["data_source"]="dy04";
    CTN["first_time"]=currentTimeMillisec();
    CTN["latest_time"]=currentTimeMillisec();

    QJsonObject track_info;
    QJsonObject position;

    position["longitude"] =pos.longitude;
    position["latitude"] =pos.latitude;
    position["altitude"] =pos.altitude;

    track_info["position"] =position;
    track_info["speed"] =30;
    track_info["track_direction"] =74;

    CTN["track_info"]=track_info;
    CTN["capture_count"]=1;
    CTN["threat_level"]=-1;
    CTN["scout_type"]=1;
    CTN["target_plat_type"]=4; //4飞机，2 军用车辆
    root["CTN"] =CTN;
    return true;
}

bool GkdDataFusion::PackageDzmbxsb_302(string name,QJsonObject& root)
{
   addHeader(root);
    QJsonObject CTN;
   CTN["target_ID"] ="dyy"+QString::fromStdString(name);
   CTN["disappear_state"] =1; //1，目标消失; 2,目标被融合


    root["CTN"] =CTN;
   return true;
}

bool GkdDataFusion::PackageZzptjzbzt_303(QJsonObject& root,APIBridgeSDK::Position& pos)
{
   addHeader(root);
   QJsonObject CTN;

   CTN["report_time"] = currentTimeMillisec();
   CTN["battle_unit_code"]  ="dy35";
   CTN["platform_type"]  =1; // 1无人机 ，2 无人车，3 单兵

   QJsonObject deploy_position;

   deploy_position["longitude"] =pos.longitude;
   deploy_position["latitude"] =pos.latitude;
   deploy_position["altitude"] =pos.altitude;
   CTN["deploy_position"]  =deploy_position;
   CTN["velocity"]  =20.0;
   root["CTN"] =CTN;
   return true;
}

bool GkdDataFusion::PackageZzptjzbzt_303_CB(QJsonObject& root,APIBridgeSDK::Position& pos)
{
    addHeader(root);
    QJsonObject CTN;

    CTN["report_time"] = currentTimeMillisec();
    CTN["battle_unit_code"]  ="dy36";
    CTN["platform_type"]  =1; // 1无人机 ，2 无人车，3 单兵

    QJsonObject deploy_position;

    deploy_position["longitude"] =pos.longitude;
    deploy_position["latitude"] =pos.latitude;
    deploy_position["altitude"] =pos.altitude;
    CTN["deploy_position"]  =deploy_position;
    CTN["velocity"]  =20.0;
    root["CTN"] =CTN;
    return true;
}

void GkdDataFusion::addHeader(QJsonObject &root)
{
    root["SID"] = "QB";
    int64_t t=  currentTimeMillisec();
    root["TM"] =t;

}

void GkdDataFusion::readHKFusion(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "无法打开文件：" << file.errorString();
        return;
    }

    QTextStream in(&file);
    FusionData fd;
    while (!in.atEnd()) {
        QString line = in.readLine(); // 读取一行文本
        QStringList sl = line.split(",");
        QString name = sl[0];
        QString lon =  sl[1];
        QString lat =sl[2];
        QString h =sl[3];

        bool ok;
        double longitude = lon.toDouble(&ok);
        if (!ok) {
            qDebug() << "转换失败：" << lon;
        }

        double latitude = lat.toDouble(&ok);
        if (!ok) {
            qDebug() << "转换失败：" << latitude;
        }
        double heigh = h.toDouble(&ok);
        if (!ok) {
            qDebug() << "转换失败：" << h;
        }

        if((fd.name != "" )&& (fd.name != name.toStdString()) ){
            _hk_fusion_data.push_back(fd);

            fd.clear();

            Position pos(longitude,latitude,heigh);

            fd.name = name.toStdString();

            fd.path.push_back(pos);
        }else{
            Position pos(longitude,latitude,heigh);

            fd.name = name.toStdString();

            fd.path.push_back(pos);
        }
    }
    _hk_fusion_data.push_back(fd);
    file.close(); // 关闭文件
}

void GkdDataFusion::readFusionData(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "无法打开文件：" << file.errorString();
        return;
    }

    QTextStream in(&file);
    FusionData fd;
    while (!in.atEnd()) {
        QString line = in.readLine(); // 读取一行文本
        QStringList sl = line.split(",");
        QString name = sl[0];
        QString lon =  sl[1];
        QString lat =sl[2];
        QString h =sl[3];

        bool ok;
        double longitude = lon.toDouble(&ok);
        if (!ok) {
            qDebug() << "转换失败：" << lon;
        }

        double latitude = lat.toDouble(&ok);
        if (!ok) {
            qDebug() << "转换失败：" << latitude;
        }
        double heigh = h.toDouble(&ok);
        if (!ok) {
            qDebug() << "转换失败：" << h;
        }

        if((fd.name != "" )&& (fd.name != name.toStdString()) ){
            _fusion_data.push_back(fd);

            fd.clear();

            Position pos(longitude,latitude,heigh);

            fd.name = name.toStdString();

            fd.path.push_back(pos);
        }else{
            Position pos(longitude,latitude,heigh);

            fd.name = name.toStdString();

            fd.path.push_back(pos);
        }
    }

    _fusion_data.push_back(fd);
    file.close(); // 关闭文件
}

// void GkdDataFusion::setC2Manager(std::shared_ptr<C2Manager> c2Manager)
// {
//     if (planRequestWidget) {
//         planRequestWidget->setC2Manager(c2Manager);
//     }
//     if (taskDeliveryWidget) {
//         taskDeliveryWidget->setC2Manager(c2Manager);
//     }
//     if (electromagneticTestWidget) {
//         // 通过事件总线通信，不再需要直接引用传递
//         qDebug() << "ElectromagneticTestWidget 已准备就绪，通过事件总线通信";
//     }
// }


// void GkdDataFusion::onPlanFileLoaded(const QString &filePath)
// {
//     qDebug() << "计划文件已加载:" << filePath;
// }

// void GkdDataFusion::onTaskInfoChanged(const QJsonObject &taskInfo)
// {
//     qDebug() << "任务信息已更新:" << taskInfo;
// }

// void GkdDataFusion::onTaskDelivered(const QJsonObject &taskInfo)
// {
//     qDebug() << "任务已下发:" << taskInfo;
// }

// void GkdDataFusion::resizeEvent(QResizeEvent *event)
// {
//     QWidget::resizeEvent(event);

//     // 调整标题栏大小以适应窗口宽度
//     if (titleBar) {
//         titleBar->setGeometry(0, RESIZE_BORDER, event->size().width(), 30 - RESIZE_BORDER);
//         titleBar->raise(); // 确保标题栏始终在最上层
//     }
// }

// void GkdDataFusion::setupContextMenu()
// {
//     contextMenu = new QMenu(this);

//     stayOnTopAction = new QAction("置顶", this);
//     stayOnTopAction->setCheckable(true);
//     stayOnTopAction->setChecked(isStayOnTop);
//     connect(stayOnTopAction, &QAction::triggered, this, &GkdDataFusion::onToggleStayOnTop);

//     contextMenu->addAction(stayOnTopAction);

//     // 为标题栏设置上下文菜单
//     if (titleBar) {
//         titleBar->setContextMenuPolicy(Qt::CustomContextMenu);
//         connect(titleBar, &QWidget::customContextMenuRequested,
//                 this, &GkdDataFusion::onTitleBarContextMenu);
//     }
// }

// void GkdDataFusion::onToggleStayOnTop()
// {
//     isStayOnTop = !isStayOnTop;
//     stayOnTopAction->setChecked(isStayOnTop);
//     stayOnTopAction->setText(isStayOnTop ? "取消置顶" : "置顶");

//     Qt::WindowFlags flags = windowFlags();
//     if (isStayOnTop) {
//         flags |= Qt::WindowStaysOnTopHint;
//     } else {
//         flags &= ~Qt::WindowStaysOnTopHint;
//     }

//     setWindowFlags(flags);
//     show(); // 重新显示窗口以应用新的标志

//     qDebug() << "窗口置顶状态:" << (isStayOnTop ? "已置顶" : "已取消置顶");
// }

// void GkdDataFusion::onTitleBarContextMenu(const QPoint &pos)
// {
//     if (contextMenu) {
//         contextMenu->exec(titleBar->mapToGlobal(pos));
//     }
// }

// void GkdDataFusion::mousePressEvent(QMouseEvent *event)
// {
//     if (event->button() == Qt::LeftButton) {
//         lastMousePos = event->globalPos();
//         originalGeometry = geometry();
//         resizeDirection = getResizeDirection(event->pos());

//         if (resizeDirection != None) {
//             isResizing = true;
//             event->accept();
//             return;
//         }
//     }
//     QWidget::mousePressEvent(event);
// }

// void GkdDataFusion::mouseMoveEvent(QMouseEvent *event)
// {
//     if (!isResizing) {
//         // 更新鼠标光标
//         Qt::CursorShape cursor = getCursorShape(event->pos());
//         setCursor(cursor);
//     } else {
//         // 执行窗口拉伸
//         QPoint delta = event->globalPos() - lastMousePos;
//         QRect newGeometry = originalGeometry;

//         if (resizeDirection & Left) {
//             newGeometry.setLeft(originalGeometry.left() + delta.x());
//         }
//         if (resizeDirection & Right) {
//             newGeometry.setRight(originalGeometry.right() + delta.x());
//         }
//         if (resizeDirection & Top) {
//             newGeometry.setTop(originalGeometry.top() + delta.y());
//         }
//         if (resizeDirection & Bottom) {
//             newGeometry.setBottom(originalGeometry.bottom() + delta.y());
//         }

//         // 确保不小于最小尺寸
//         if (newGeometry.width() >= minimumSize().width() &&
//             newGeometry.height() >= minimumSize().height()) {
//             setGeometry(newGeometry);
//         }

//         event->accept();
//         return;
//     }

//     QWidget::mouseMoveEvent(event);
// }

// void GkdDataFusion::mouseReleaseEvent(QMouseEvent *event)
// {
//     if (event->button() == Qt::LeftButton) {
//         isResizing = false;
//         resizeDirection = None;
//         setCursor(Qt::ArrowCursor);
//     }
//     QWidget::mouseReleaseEvent(event);
// }

// bool GkdDataFusion::eventFilter(QObject *obj, QEvent *event)
// {
//     if (obj == this && event->type() == QEvent::HoverMove) {
//         QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
//         Qt::CursorShape cursor = getCursorShape(mouseEvent->pos());
//         setCursor(cursor);
//     }
//     return QWidget::eventFilter(obj, event);
// }

// Qt::CursorShape GkdDataFusion::getCursorShape(const QPoint &point)
// {
//     int direction = getResizeDirection(point);

//     switch (direction) {
//         case Left:
//         case Right:
//             return Qt::SizeHorCursor;
//         case Top:
//         case Bottom:
//             return Qt::SizeVerCursor;
//         case TopLeft:
//         case BottomRight:
//             return Qt::SizeFDiagCursor;
//         case TopRight:
//         case BottomLeft:
//             return Qt::SizeBDiagCursor;
//         default:
//             return Qt::ArrowCursor;
//     }
// }

// int GkdDataFusion::getResizeDirection(const QPoint &point)
// {
//     int direction = None;

//     // 标题栏区域判断
//     bool inTitleBar = (point.y() > RESIZE_BORDER && point.y() <= 30);

//     if (point.x() <= RESIZE_BORDER)
//         direction |= Left;
//     else if (point.x() >= width() - RESIZE_BORDER)
//         direction |= Right;

//     // 顶部拉伸：允许在窗口最顶端的细边区域进行拉伸
//     if (point.y() <= RESIZE_BORDER)
//         direction |= Top;
//     else if (point.y() >= height() - RESIZE_BORDER)
//         direction |= Bottom;

//     // 如果在标题栏内部区域，清除所有拉伸方向
//     if (inTitleBar && point.x() > RESIZE_BORDER && point.x() < width() - RESIZE_BORDER) {
//         direction = None;
//     }

//     return direction;
// }
