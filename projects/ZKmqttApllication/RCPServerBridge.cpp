#include "RCPServerBridge.h"

//#include "PluginCore/MsgHelper.h"
//#include "CommonInc/PluginGisInclude.h"
//#include "EVGisRender/EVGisRender.h"
//#include "EVGisRender/IRenderAdoptorInterface.h"
//#include "CommonInc/MainSystemInc/CommonBusinessStructDefines.h"

//using namespace XTWJ::GisRender_Space;

RCPServerBridge::RCPServerBridge():
	m_rcpServer(nullptr)
{
	m_rcpServer = new rpc_server(9000, std::thread::hardware_concurrency(), 3600);
	//连接错误的回调
	m_rcpServer->set_network_err_callback(
		[](std::shared_ptr<connection> conn, std::string reason) {
		qDebug() << "remote client address: " << QString::fromStdString(conn->remote_address())
			<< " networking error, reason: " << QString::fromLocal8Bit(reason.data()) << "\n";
	});
	registHandler();
	m_rcpServer->async_run();
}

RCPServerBridge::~RCPServerBridge()
{
	delete m_rcpServer;
}

void RCPServerBridge::registHandler()
{
	//类的成员函数用法
	m_rcpServer->register_handler("cameraGoto", &RCPServerBridge::cameraGoto, this);
	m_rcpServer->register_handler("zoom", &RCPServerBridge::zoom, this);
	m_rcpServer->register_handler("globeCameraUpToNorth", &RCPServerBridge::globeCameraUpToNorth, this);
	m_rcpServer->register_handler("updatePointMark", &RCPServerBridge::updatePointMark, this);
	m_rcpServer->register_handler("updateRegionMark", &RCPServerBridge::updateRegionMark, this);
	m_rcpServer->register_handler("setTargetVisible", &RCPServerBridge::setTargetVisible, this);
	m_rcpServer->register_handler("updateRoute", &RCPServerBridge::updateRoute, this);
	m_rcpServer->register_handler("showChatReply", &RCPServerBridge::showChatReply, this);
	m_rcpServer->register_handler("showAlertData", &RCPServerBridge::showAlertData, this);
	m_rcpServer->register_handler("loadTesk", &RCPServerBridge::loadTask, this);

	//普通函数用法
	//m_rcpServer->register_handler("getDoubleStr", getDoubleStr);
}

void RCPServerBridge::cameraGoto(rpc_conn conn, double latitude, double longitude, double heading, double tilt, double altitude, int time_ms)
{
//	GotoParam _param;
//	_param.lat = latitude;
//	_param.lon = longitude;
//	_param.heading = heading;
//	_param.altitude = altitude;
//	_param.tilt = tilt;
//	_param.timeMs = time_ms;
	
//	QVariant _var;
//	_var.setValue(_param);
	
//	MsgHelper::sendPluginCMDMsg(PluginEventType::MSG_CMD_Goto, _var);
    qDebug() << "cameraGoto: ";
}

void RCPServerBridge::zoom(rpc_conn conn, bool isZoomIn)
{
//	MsgHelper::sendPluginCMDMsg(PluginEventType::MSG_CMD_Zoom, isZoomIn);
    qDebug() << "zoom: ";
}

void RCPServerBridge::globeCameraUpToNorth(rpc_conn conn)
{
//	MsgHelper::sendPluginCMDMsg(PluginEventType::MSG_CMD_CameraUpToNorth, QVariant());
    qDebug() << "globeCameraUpToNorth: ";

}

void RCPServerBridge::updatePointMark(rpc_conn conn, const std::string& uuid, const APIBridgeSDK::Position& pos, const std::string& note)
{
//	IRenderAdoptorInterface* pRAI = EVGisRender::getInstance()->getRenderImpl();
//	if (!pRAI)
//	{
//		return;
//	}

//	XTWJ::GraphKey key;
//	key.layerid = "Mark";
//	key.graphid = QString::fromLocal8Bit(uuid.data());

//	XTWJ::GisRender_Space::PointGraphGroupData data;
//	data.imgPath = "pointA.png";
//	data.size = 30;
//	data.picOffset = qMakePair(0, -15);
//	data.position = XTWJ::Position(pos.lon, pos.lat, pos.alt);
//	data.picLieDown = false;
//	XTWJ::GisRender_Space::TextGraphData textData;
//	textData.textpos = data.position;
//	textData.text = QString::fromLocal8Bit(note.data());
//	data.textStyle = textData;

//	pRAI->createUpdatePointGraph(key, data);
    qDebug() << "updatePointMark: ";

}

void RCPServerBridge::updateRegionMark(rpc_conn conn, const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos, const std::string& note)
{
//	IRenderAdoptorInterface* pRAI = EVGisRender::getInstance()->getRenderImpl();
//	if (!pRAI)
//	{
//		return;
//	}

//	XTWJ::GraphKey key;
//	key.layerid = "Mark";
//	key.graphid = QString::fromLocal8Bit(uuid.data());

//	XTWJ::GisRender_Space::RectGraphGroupData data;
//	QVector<XTWJ::Position> pts;
//	for (int i = 0; i < vecPos.size(); ++i)
//	{
//		pts.append(XTWJ::Position(vecPos[i].lon, vecPos[i].lat, vecPos[i].alt));
//	}
//	data.pts = pts;
//	data.lineClr = Qt::red;
//	data.fillClr = QColor(255, 0, 0, 100);
//	data.lineWidth = 1;
//	data.selectable = false;
//	data.editable = false;

//	XTWJ::GisRender_Space::TextGraphData textData;
//	textData.text = QString::fromLocal8Bit(note.data());;
//	textData.textClr = Qt::red;
//	textData.textFont = QFont();
//	textData.textFont.value.setPixelSize(16);
//	data.textStyle = textData;

//	pRAI->createUpdateRectGraph(key, data);
    qDebug() << "updateRegionMark:  RCPServerBridgeRCPServerBridgeRCPServerBridge";

}

void RCPServerBridge::setTargetVisible(rpc_conn conn, const std::string& tgtid, bool bVisible)
{
//	IRenderAdoptorInterface* pRAI = EVGisRender::getInstance()->getRenderImpl();
//	if (!pRAI)
//	{
//		return;
//	}

//	XTWJ::GisRender_Space::TargetKey targetKey;
//	targetKey.layerid = "platform";
//	targetKey.targetid = QString::fromLocal8Bit(tgtid.data());
//	pRAI->setTargetVisible(targetKey, bVisible);
    qDebug() << "setTargetVisible: ";

}

void RCPServerBridge::updateRoute(rpc_conn conn, const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos)
{
//	IRenderAdoptorInterface* pRAI = EVGisRender::getInstance()->getRenderImpl();
//	if (!pRAI)
//	{
//		return;
//	}

//	XTWJ::GraphKey key;
//	key.layerid = "Route";
//	key.graphid = QString::fromLocal8Bit(uuid.data());

//	XTWJ::GisRender_Space::LineGraphGroupData data;
//	QVector<XTWJ::Position> pts;
//	for (int i = 0; i < vecPos.size(); ++i)
//	{
//		pts.append(XTWJ::Position(vecPos[i].lon, vecPos[i].lat, vecPos[i].alt));
//	}
//	data.pts = pts;
//	data.lineClr = Qt::red;
//	data.lineWidth = 2;
//	data.selectable = false;
//	data.editable = false;

//	pRAI->createUpdateLineGraph(key, data);
    qDebug() << "updateRoute: ";

}

void RCPServerBridge::showChatReply(rpc_conn conn, const std::string& content)
{
//	MsgHelper::sendPluginCMDMsg(PluginEventType::MSG_CMD_CharShowText, QVariant(QString::fromLocal8Bit(content.data())));
    qDebug() << "showChatReply: ";

}

void RCPServerBridge::showAlertData(rpc_conn conn, const APIBridgeSDK::MilitaryAlertData& data)
{
//	ChatShowAlertTgt chatShow;
//	chatShow.alertType = QString::fromLocal8Bit(data.alertType.data());
//	bool bLoadImageSuc =  chatShow.image.loadFromData(data.image.imageData.data(), data.image.imageData.size(), data.image.format.c_str());
//	if (!bLoadImageSuc)
//	{
//		qWarning() << QString::fromLocal8Bit("加载图像失败，格式可能不支持：") << QString::fromStdString(data.image.format.c_str());
//	}
//	chatShow.summary = QString::fromLocal8Bit(data.summary.data());
//	chatShow.timestamp = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(data.details.timestamp));
//	chatShow.imageType = QString::fromLocal8Bit(data.details.imageType.data());
//	chatShow.sensorType = QString::fromLocal8Bit(data.details.sensorType.data());
//	chatShow.imageQuality = QString::fromLocal8Bit(data.details.imageQuality.data());
//	chatShow.algorithm = QString::fromLocal8Bit(data.details.algorithm.data());
//	chatShow.latitude = data.details.location.latitude;
//	chatShow.longitude = data.details.location.longitude;
//	chatShow.altitude = data.details.location.altitude;

//	MsgHelper::sendPluginCMDMsg(PluginEventType::MSG_CMD_ChatShowAlertTgt, QVariant::fromValue(chatShow));
    qDebug() << "showAlertData: " << QString::fromStdString(data.details.algorithm);

}

void RCPServerBridge::loadTask(rpc_conn conn, const std::string& taskJsonStr)
{
    qDebug() << "loadTask called with data:" << QString::fromStdString(taskJsonStr);

    // TODO: Implement actual task loading logic here
    // For now, just log the received task data
    // You can add your specific task processing logic based on your requirements

    qDebug() << "loadTask completed successfully";
}



#pragma region regionName
//********************************提供的服务示例代码begin***********************************
//Person rest_rcp_server::getPerson(rpc_conn conn, int age, const std::string& name)
//{
//	Person _person;
//	_person.age = age;
//	_person.name = name;
//	_person.vec.emplace_back(1);
//	_person.vec.emplace_back(2);
//	_person.vec.emplace_back(3);
//	_person.vec.emplace_back(4);
//	_person.vec.emplace_back(5);
//
//	//模拟耗时操作
//	std::this_thread::sleep_for(std::chrono::seconds(2));
//	qDebug() << _person.age << QString::fromStdString(_person.name) << QVector<int>::fromStdVector(_person.vec);
//	return _person;
//}
//
//void rest_rcp_server::callFunction(rpc_conn conn, const Person& person)
//{
//	qDebug() << person.age << QString::fromStdString(person.name) << QVector<int>::fromStdVector(person.vec);
//}
//********************************提供的服务示例代码begin***********************************
#pragma endregion regionName
