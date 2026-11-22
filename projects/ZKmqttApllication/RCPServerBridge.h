#pragma once

#include <QObject>
#include <QDebug>

#include "SDKStructDefine.h"
using namespace rest_rpc;
using namespace rpc_service;

class RCPServerBridge 
{
public:
	RCPServerBridge();
	~RCPServerBridge();

	void registHandler();

private:
	//*****************************************服务方法 begin*****************************************************
	//相机视角goto
	//latitude	经度
	//longitude 纬度
	//heading	相机朝向
	//tilt		相机倾角
	//altitude	相机高度
	//time_ms	相机移动的时间
	void cameraGoto(rpc_conn conn, double latitude, double longitude, double heading, double tilt, double altitude, int time_ms);

	//场景缩放
	//true  放大	false 缩小
	void zoom(rpc_conn conn, bool isZoomIn);

	//三维地球朝北
	void globeCameraUpToNorth(rpc_conn conn);

	//点标绘接口
	//uuid	唯一标识
	//pos	地理位置
	//note	显示注记
	void updatePointMark(rpc_conn conn, const std::string& uuid, const APIBridgeSDK::Position& pos, const std::string& note);

	//区域标绘接口
	//uuid	唯一标识
	//vecPos点组
	//note	显示注记
	void updateRegionMark(rpc_conn conn, const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos, const std::string& note);

	//目标显隐接口
	//tgtid		目标id
	//bVisible	显隐
	void setTargetVisible(rpc_conn conn, const std::string& tgtid, bool bVisible);

	//路线显示接口
	//uuid	唯一标识
	//vecPos点组
	void updateRoute(rpc_conn conn, const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos);

	//大模型对话回复显示接口
	//content 回复内容
	void showChatReply(rpc_conn conn, const std::string& content);

	//可疑目标显示卡片接口
	//data 告警数据
	void showAlertData(rpc_conn conn, const APIBridgeSDK::MilitaryAlertData& data);

	//加载任务接口
	//taskJsonStr 任务JSON字符串
	void loadTask(rpc_conn conn, const std::string& taskJsonStr);

	//甘特图



	//*****************************************服务方法 end*****************************************************
private:
	rpc_server* m_rcpServer;
};
