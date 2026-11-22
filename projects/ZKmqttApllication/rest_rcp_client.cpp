#include "rest_rcp_client.h"
#include <QDebug>
#include <QDateTime>
#include <QBuffer>

rest_rcp_client::rest_rcp_client(QWidget *parent)
    : QWidget(parent)
    , m_rcpClientAdapter(nullptr)
{
    ui.setupUi(this);

}

void rest_rcp_client::startConnectRcpServer(const CommonCfg& _cfg){
    m_rcpClientAdapter = new RcpClientAdapter(_cfg.rpc_ip, _cfg.rpc_port);
    buildConnect();
}

void rest_rcp_client::subscribeTask(const string& key,TPCallback& cb)
{
    m_rcpClientAdapter->subscribeTask(key,cb);
}


void rest_rcp_client::buildConnect()
{
//    connect(ui.pushButton_goto, &QPushButton::clicked, [=]() {slotSDKGoto(); });
//    connect(ui.pushButton_north, &QPushButton::clicked, [=]() {slotSDKUptoNorth(); });
//    connect(ui.pushButton_zoom, &QPushButton::clicked, [=]() {slotSDKZoom(); });
//    connect(ui.pushButton_updatePointMark, &QPushButton::clicked, [=]() {slotSDKUpdatePointMark(); });
//    connect(ui.pushButton_updateRegionMark, &QPushButton::clicked, [=]() {slotSDKUpdateRegionMark(); });
//    connect(ui.pushButton_setTargetVisible, &QPushButton::clicked, [=]() {slotSDKSetTargetVisible(); });
//    connect(ui.pushButton_updateRoute, &QPushButton::clicked, [=]() {slotSDKUpdateRoute(); });
//    connect(ui.pushButton_showChatReply, &QPushButton::clicked, [=]() {slotSDKShowChatReply(); });
//    connect(ui.pushButton_showAlertData, &QPushButton::clicked, [=]() {slotSDKShowAlertData(); });
}

void rest_rcp_client::slotSDKGoto()
{
    try {
        if(m_rcpClientAdapter){
              m_rcpClientAdapter->cameraGoto(25, 120, 5000000);
        }
    } catch (const std::exception& e) {
        qDebug() << "slotSDKGoto:" << e.what();
    }
}

void rest_rcp_client::slotSDKZoom()
{
    if (ui.checkBox_in->checkState() == Qt::Checked)
    {
        m_rcpClientAdapter->zoom(true);
    }
    else
    {
        m_rcpClientAdapter->zoom(false);
    }
}

void rest_rcp_client::slotSDKUptoNorth()
{
    m_rcpClientAdapter->globeCameraUpToNorth();
}

void rest_rcp_client::slotSDKUpdatePointMark()
{
    m_rcpClientAdapter->updatePointMark("test0001", APIBridgeSDK::Position(120, 39, 1000), 10, "标记点1", "#FF0000");
}

void rest_rcp_client::slotSDKUpdateRegionMark()
{
    qDebug() << "Client slotSDKUpdateRegionMark: init";
    qDebug() << "Qt Version:" << QT_VERSION_STR;
    
    try {
        // 创建测试数据
        std::vector<APIBridgeSDK::Position> vecPos;
        
        // 添加调试信息
        qDebug() << "Creating test positions...";
        vecPos.push_back(APIBridgeSDK::Position(120, 39, 1000));
        vecPos.push_back(APIBridgeSDK::Position(120, 39.5, 1000));
        
        // 打印详细的数据结构信息
        qDebug() << "Vector size:" << vecPos.size();
        qDebug() << "Position struct size:" << sizeof(APIBridgeSDK::Position);
        
        for(const auto& pos : vecPos) {
            qDebug() << "Position data:"
                     << "lon:" << pos.longitude
                     << "lat:" << pos.latitude
                     << "alt:" << pos.altitude;
        }
        
        // 尝试序列化数据（用于调试）
        msgpack::sbuffer sbuf;
        msgpack::packer<msgpack::sbuffer> packer(sbuf);
        packer.pack(vecPos);
        
        qDebug() << "Serialized data size:" << sbuf.size();
        
        // 调用RPC
        qDebug() << "Calling updateRegionMark...";
        m_rcpClientAdapter->updateRegionMark("test0002", vecPos, "hello", "#00FF00");
        
        qDebug() << "Call successful";
    }
//    catch (const rest_rpc::rpc_error& e) {
//        qDebug() << "RPC Error:" << e.what();
//        qDebug() << "Error code:" << e.code();
//    }
    catch (const std::exception& e) {
        qDebug() << "Standard exception:" << e.what();
#ifdef _CPPRTTI  // 只有在启用RTTI时才使用typeid
        qDebug() << "Exception type:" << typeid(e).name();
#endif
    }
    catch (...) {
        qDebug() << "Unknown exception occurred";
    }
    
    qDebug() << "Client slotSDKUpdateRegionMark: end";
}

void rest_rcp_client::slotSDKSetTargetVisible()
{
    static bool s_bVisi = false;
    m_rcpClientAdapter->setTargetVisible(ui.lineEdit->text().toStdString(), s_bVisi);
    s_bVisi = !s_bVisi;
}

void rest_rcp_client::slotSDKUpdateRoute()
{
    std::vector<APIBridgeSDK::Position> vecPos;
    vecPos.push_back(APIBridgeSDK::Position(121, 39, 1000));
    vecPos.push_back(APIBridgeSDK::Position(121, 39.5, 1000));
    vecPos.push_back(APIBridgeSDK::Position(121.5, 39.5, 1000));
    vecPos.push_back(APIBridgeSDK::Position(121.5, 39, 1000));
    m_rcpClientAdapter->updateRoute("test0003", vecPos);
}

void rest_rcp_client::slotSDKShowChatReply()
{
    m_rcpClientAdapter->showChatReply("你好，我是一个大模型回复对话");
}

void rest_rcp_client::slotSDKShowAlertData()
{
    QImage img(QString("%1/wujing.png").arg(QApplication::applicationDirPath()));
    QByteArray byteArray;
    QBuffer buffer(&byteArray);
    buffer.open(QIODevice::WriteOnly);
    img.save(&buffer, "png");
    buffer.close();

    APIBridgeSDK::MilitaryAlertData data;
    data.alertType = "异常军事活动检测";
    data.image.imageData.assign(byteArray.begin(), byteArray.end());
    data.image.width = img.width();
    data.image.height = img.height();
    data.image.channels = img.hasAlphaChannel() ? 4 : 3;
    qDebug()<<"data.image.imageData:" <<data.image.imageData.size();
    data.image.format = "png";
    data.summary = "警报概要描述";
    data.details.timestamp = static_cast<std::time_t>(QDateTime::currentMSecsSinceEpoch());
    data.details.imageType = "Drone";
    data.details.sensorType = "Electro-Optic";
    data.details.imageQuality = "HD";
    data.details.algorithm = "Vehicle Detection V 2.3.1";
    data.details.location.longitude = 120;
    data.details.location.latitude = 39;
    data.details.location.altitude = 100;
    m_rcpClientAdapter->showAlertData(data);
    qDebug()<<"slotSDKShowAlertData end:";

}
void rest_rcp_client::slotSchemeStart()
{
    m_rcpClientAdapter->schemeStartPlay();
}

void rest_rcp_client::slotSchemePause()
{
    m_rcpClientAdapter->schemePausePlay();
}

void rest_rcp_client::slotSchemeContinue()
{
    m_rcpClientAdapter->schemeContinuePlay();
}

void rest_rcp_client::slotSchemeEnd()
{
    m_rcpClientAdapter->schemeEndPlay();
}
void rest_rcp_client::slotLoadScheme()
{
    APIBridgeSDK::SchemeInfo info;
    info.schmeId = "abcdefg2";
    info.taskId = "task000012";
    info.schemeName = "测试方案2";
    info.jsoninfo = "{\"BehaviorList\":[{\"data\":{\"Camerafocus\":0,\"CamerafocusCamerarotateX\":0,\"deltaX\":8,\"deltaY\":0,\"deltaZ\":0,\"duration\":1500,\"rotateY\":0,\"start_time\":0},\"duration\":1500,\"modelId\":\"osmgydrkzo24nfl6redf\",\"name\":\"       \",\"text\":\"       A\",\"time\":0,\"type\":2,\"uuid\":\"{14fb4083-7119-4da4-bba2-3cb8059a1c80}\"},{\"data\":{\"Camerafocus\":0,\"CamerafocusCamerarotateX\":0,\"deltaX\":10,\"deltaY\":0,\"deltaZ\":0,\"duration\":1000,\"pos_lat\":29.728957,\"pos_lon\":106.516698,\"rotateY\":6},\"duration\":1000,\"modelId\":\"osmgydrkzo24nfl6redf\",\"name\":\"       \",\"text\":\"       \",\"time\":46545,\"type\":2,\"uuid\":\"{e0ea8876-5aff-49f2-bba8-7fac9a3df884}\"}],\"EntityModel\":[{\"alt\":[0,0,0],\"entityType\":1001,\"layerName\":\" ջ \",\"modelId\":\"osmgydrkzo24nfl6redf\",\"name\":\"       ˻ 0345\",\"pos\":[106.51229110188409,29.729679526129807,500],\"scale\":1.3422554864e-313}],\"MovePath\":[{\"data\":{\"height\":0,\"speed\":100},\"duration\":16761,\"endTime\":16761,\"modelId\":\"osmgydrkzo24nfl6redf\",\"pathId\":\"{5953ea93-83bf-4c3e-8a52-8aae19d405f1}\",\"pts\":[[106.512291,29.72968,500,0],[106.513968,29.729656,500,5836],[106.513712,29.730295,500,8547],[106.513908,29.730714,500,10359],[106.514671,29.730197,500,13726],[106.515407,29.729792,500,16761]],\"speed\":27.77938033682105,\"startTime\":0,\"text\":\"        A\"},{\"data\":{\"height\":0,\"speed\":80},\"duration\":32515,\"endTime\":49276,\"modelId\":\"osmgydrkzo24nfl6redf\",\"pathId\":\"{44c7d128-f7e9-4c33-b261-34eda8870543}\",\"pts\":[[106.515407,29.729792,500,16761],[106.51816468213153,29.72946090483011,258.7360411779955,33025],[106.51664326774312,29.728541863867914,265.52826986275613,41092],[106.51772064400376,29.72808311326343,261.62148016598076,46314],[106.51803917422917,29.727561015155715,260.7900840854272,49276]],\"speed\":22.222899882189235,\"startTime\":16761,\"text\":\"        \"}]}";
    m_rcpClientAdapter->loadScheme(info);
}
void rest_rcp_client::slotAddEntityMission()
{
    APIBridgeSDK::EntityMission info;
    info.entityId = "osmgydrkzo24nfl6redf";				//      Է     entityId
    info.missionDes = "侦察";
    info.uuId = "ABCDEFG";
    info.missionType = RPC_MissionType::Mission_Common;
    info.startTime = 10000;
    info.duration = 5000;
    m_rcpClientAdapter->addEntityMission(info);
}

void rest_rcp_client::slotAddAdHocMisson()
{
    APIBridgeSDK::EntityMission info;
    info.entityId = "osmgydrkzo24nfl6redf";
    info.missionDes = "追击";
    info.uuId = "1234567";
    info.missionType = RPC_MissionType::Mission_AdHoc;
    info.startTime = 20000;
    m_rcpClientAdapter->addEntityMission(info);
}

void rest_rcp_client::slotAbondonBeh()
{
    std::string missionId = "ABCDEFG";
    m_rcpClientAdapter->abandonMission(missionId);
}
void rest_rcp_client::slotReomveEntityMission()
{
    std::string missionId = "ABCDEFG";
    m_rcpClientAdapter->removeEntityMission(missionId);
}


void rest_rcp_client::slotRemoveScheme()
{
    std::string schemeID = "ABCDEFG";
    m_rcpClientAdapter->removeScheme(schemeID);
}
#pragma region
//**********************************************测试例子代码begin**********************************************
//void rest_rcp_client::slotGetPersonSync()
//{
//	auto _personResult = m_rcpClientAdapter->getPersonSync(32, "hjh");
//	qDebug() << _personResult.age << QString::fromStdString(_personResult.name) << QVector<int>::fromStdVector(_personResult.vec);
//}
//
//void rest_rcp_client::slotGetPersonAsync()
//{
//	std::function<void(const Person& person)> fn = [=](const Person& person) {
//		qDebug() << person.age << QString::fromStdString(person.name) << QVector<int>::fromStdVector(person.vec);
//	};
//	m_rcpClientAdapter->getPersonAsync(32, "hjh", fn);
//}
//
//void rest_rcp_client::slotGetDoubleStrSync()
//{
//	auto _strResult = m_rcpClientAdapter->getDoubleStrSync("hjh is talented");
//	qDebug() << QString::fromStdString(_strResult);
//}
//
//void rest_rcp_client::slotGetDoubleStrAsync()
//{
//	std::function<void(const std::string& str)> fn = [=](const std::string& returnValue) {
//		qDebug() << QString::fromStdString(returnValue);
//	};
//	m_rcpClientAdapter->getDoubleStrAsync("hjh is talented", fn);
//}
//
//void rest_rcp_client::slotCallFunctionSync()
//{
//	Person _person;
//	_person.age = 32;
//	_person.name = "hjh";
//	_person.vec.emplace_back(1);
//	_person.vec.emplace_back(2);
//	_person.vec.emplace_back(3);
//	_person.vec.emplace_back(4);
//	_person.vec.emplace_back(5);
//	m_rcpClientAdapter->callFunction(_person);
//}
//**********************************************测试例子代码end**********************************************
#pragma endregion




void rest_rcp_client::on_pushButton_updateRoute_clicked()
{
    slotSDKUpdateRoute();
}


void rest_rcp_client::on_pushButton_goto_clicked()
{
    slotSDKGoto();
}


void rest_rcp_client::on_pushButton_north_clicked()
{
    slotSDKUptoNorth();
}


void rest_rcp_client::on_pushButton_zoom_clicked()
{
    slotSDKZoom();
}


void rest_rcp_client::on_pushButton_updatePointMark_clicked()
{
    slotSDKUpdatePointMark();
}


void rest_rcp_client::on_pushButton_updateRegionMark_clicked()
{
    slotSDKUpdateRegionMark();
}


void rest_rcp_client::on_pushButton_setTargetVisible_clicked()
{
    slotSDKSetTargetVisible();
}


void rest_rcp_client::on_pushButton_showChatReply_clicked()
{
    slotSDKShowChatReply();
}


void rest_rcp_client::on_pushButton_showAlertData_clicked()
{
    slotSDKShowAlertData();
}


void rest_rcp_client::on_btn_sch_start_clicked()
{
    slotSchemeStart();
}


void rest_rcp_client::on_btn_sch_pause_clicked()
{
    slotSchemePause();
}


void rest_rcp_client::on_btn_sch_continue_clicked()
{
    slotSchemeContinue();
}


void rest_rcp_client::on_btn_sch_end_clicked()
{
    slotSchemeEnd();
}


void rest_rcp_client::on_btn_showScheme_clicked()
{
    slotLoadScheme();
}


void rest_rcp_client::on_btn_addbeh_clicked()
{
    slotAddEntityMission();
}


void rest_rcp_client::on_btn_addhocBeh_clicked()
{
    slotAddAdHocMisson();
}


void rest_rcp_client::on_btn_abandonbeh_clicked()
{
    slotAbondonBeh();
}

