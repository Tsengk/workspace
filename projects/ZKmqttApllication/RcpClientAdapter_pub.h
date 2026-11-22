#pragma once


#include "SDKStructDefine.h"
#include "gkd/task_response_define.h"
//#include "CommonInc/MainSystemInc/SDKStructDefine.h"
using namespace rest_rpc;
using namespace rpc_service;


#define ASYNCCALLBACK(className) std::function<void(asio::error_code ec, string_view data)> _function = \
	[=](asio::error_code ec, string_view data) \
		{ \
			if (ec) \
			{ \
				std::cout << ec.message() << " " << data << "\n"; return; \
			} \
			auto _result = as<className>(data); fn(_result); \
		}; \

class RcpClientAdapter
{
public:
	RcpClientAdapter(std::string id, int port)
	{
		m_rcpClient = new rpc_client(id, port);
		connectToRcpServer();
	}
    void subscribeTask(const string& key,TPCallback& cb){

        m_rcpClient->subscribe(key,"",cb);
    }
	//*****************************************���񷽷� begin*****************************************************
	//����ӽ�goto
	//latitude	����
	//longitude γ��
	//heading	�������
	//tilt		������
	//altitude	����߶�
	//time_ms	����ƶ���ʱ��
	void cameraGoto(double latitude, double longitude, double altitude, double tilt = 0, double heading = 0, int time_ms = 1000)
	{
		m_rcpClient->call("cameraGoto", latitude, longitude, heading, tilt, altitude, time_ms);
	}

	//��������
	//true  �Ŵ�	false ��С
	void zoom(bool isZoomIn)
	{
		m_rcpClient->call("zoom", isZoomIn);
	}

	//��ά���򳯱�
	void globeCameraUpToNorth()
	{
		m_rcpClient->call("globeCameraUpToNorth");
	}

	//����ӿ�
	//uuid	Ψһ��ʶ
	//pos	����λ��
	//note	��ʾע��
	void updatePointMark(const std::string& uuid, const APIBridgeSDK::Position& pos, const std::string& note)
	{
		m_rcpClient->call("updatePointMark", uuid, pos, note);
	}

	//������ӿ�
	//uuid	Ψһ��ʶ
	//vecPos����
	//note	��ʾע��
	void updateRegionMark(const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos, const std::string& note)
	{
		m_rcpClient->call("updateRegionMark", uuid, vecPos, note);
	}

	//Ŀ�������ӿ�
	//tgtid		Ŀ��id
	//bVisible	����
	void setTargetVisible(const std::string& tgtid, bool bVisible)
	{
		m_rcpClient->call("setTargetVisible", tgtid, bVisible);
	}

	//·����ʾ�ӿ�
	//uuid	Ψһ��ʶ
	//vecPos����
	void updateRoute(const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos)
	{
		m_rcpClient->call("updateRoute", uuid, vecPos);
	}

	//��ģ�ͶԻ��ظ���ʾ�ӿ�
	//content �ظ�����
	void showChatReply(const std::string& content)
	{
		m_rcpClient->call("showChatReply", content);
	}

	//����Ŀ����ʾ��Ƭ�ӿ�
	//data �澯����
	void showAlertData(const APIBridgeSDK::MilitaryAlertData& data)
	{
		m_rcpClient->call("showAlertData", data);
	}

	//������ʼ����
	void schemeStartPlay()
	{
		m_rcpClient->call("schemeStart");
	}

	//������ͣ����
	void schemePausePlay()
	{
		m_rcpClient->call("schemePause");
	}

	//������������
	void schemeContinuePlay()
	{
		m_rcpClient->call("schemeContinue");
	}

	//������������
	void schemeEndPlay()
	{
		m_rcpClient->call("schemeEnd");
	}

	//��������
	void loadTask(const std::string taskJsonstr)
	{
		m_rcpClient->call("loadTesk", taskJsonstr);
	}

	//���ط���
	void loadScheme(const APIBridgeSDK::SchemeInfo& shcemeData)
	{
		m_rcpClient->call("loadScheme", shcemeData);
	}

    //���ط���
    void loadAction(const APIBridgeSDK::ActionInfo& actionData)
    {
        m_rcpClient->call("onAction", actionData);
    }

    //������Ƶ��
    void loadVideoStream(const APIBridgeSDK::VideoStreamList& actionData)
    {
        m_rcpClient->call("onStreamVideoPath", actionData);
    }

	//����ʵ����Ϊ(����)
	void addEntityMission(const APIBridgeSDK::EntityMission& missionData)
	{
		m_rcpClient->call("addEntityMission", missionData);
	}

	//ɾ��ʵ����Ϊ(����)
	void removeEntityMission(const std::string& missionData)
	{
		m_rcpClient->call("removeEntityMission", missionData);
	}

	//ɾ������
	void removeScheme(const std::string& schemeId)
	{
		m_rcpClient->call("removeScheme", schemeId);
	}

	//����ʵ����Ϊ
	void abandonMission(const std::string& missionId)
	{
		m_rcpClient->call("abandonMission", missionId);
	}

	//加载任务
	void loadTask(const std::string& taskJsonStr)
	{
		m_rcpClient->call("loadTaskInfomation", taskJsonStr);
	}

	//�л�ʵ��״̬
	void switchEntityRunningState(const std::string& entityId, RPC_EntityRunningState entityState)
	{
		m_rcpClient->call("switchEntityRunningState", entityId, entityState);
	}

	//����������ҷ�̬������
	void createOrUpdateOwnPlatformData(const APIBridgeSDK::OwnPlatform& ownData)
	{
		m_rcpClient->call("updateOwnPlatData", ownData);
	}

	void createOrUpdateOwnPlatformData(const std::vector<APIBridgeSDK::OwnPlatform>& ownDatas)
	{
		m_rcpClient->call("batchUpdateOwnPlatData", ownDatas);
	}

	//�������µз�̽������
	void createOrUpdateTargetPlatformData(const APIBridgeSDK::Targetlatform& targetData)
	{
		m_rcpClient->call("updateTargetPlatData", targetData);
	}

	void createOrUpdateTargetPlatformData(const std::vector<APIBridgeSDK::Targetlatform>& targetData)
	{
		m_rcpClient->call("batchUpdateTargetPlatData", targetData);
	}

	//ɾ���з����ҷ��豸����
	void removePlatformData(const std::string& model_id)
	{
		m_rcpClient->call("removePlatformData", model_id);
	}


	//����һ�����Ŀ��
	void addIsmiTargetInfo(const APIBridgeSDK::ImsiTargetInfo& info)
	{
		m_rcpClient->call("addIsmiTargetInfo", info);
	}

	//�Ƴ�һ�����Ŀ��
	void removeImsiTargetInfo(const std::string& imsi)
	{
		m_rcpClient->call("removeImsiTargetInfo", imsi);
	}
    //�Ƴ����е��Ŀ��
    void removeAllImsiTargetInfo(const std::string& imsi)
    {
        m_rcpClient->call("removeAllImsiTargetInfo");
    }
	//*****************************************���񷽷� end*****************************************************
#pragma region regionName
	//**********************************************�������Ӵ���begin**********************************************
	//Person getPersonSync(int age ,const std::string& name)
	//{
	//	auto _personResult = m_rcpClient->call<Person>("getPerson", age, name);
	//	return _personResult;
	//}

	//void getPersonAsync(int age, const std::string& name,std::function<void(const Person& returnValue)> fn)
	//{
	//	ASYNCCALLBACK(Person);
	//	m_rcpClient->async_call("getPerson", _function, age, name);
	//}

	//std::string getDoubleStrSync(const std::string& str)
	//{
	//	return m_rcpClient->call<std::string>("getDoubleStr", str);
	//}

	//void getDoubleStrAsync(const std::string& str, std::function<void(const std::string& returnValue)> fn)
	//{
	//	ASYNCCALLBACK(std::string);
	//	m_rcpClient->async_call("getDoubleStr", _function, str);
	//}

	////���ݼ׷���������ṩ�ӿ�
	//void callFunction(const Person& person)
	//{
	//	m_rcpClient->call("callFunction", person);
	//}
	//**********************************************�������Ӵ���end**********************************************
#pragma endregion regionName

private:
	void connectToRcpServer()
	{
		m_rcpClient->enable_auto_reconnect(); // automatic reconnect
		m_rcpClient->enable_auto_heartbeat(); // automatic heartbeat

		bool r = m_rcpClient->connect();
		if (!r)
		{
			std::cout << "connect timeout";
			return;
		}

		m_rcpClient->set_error_callback([](asio::error_code ec) {
			std::cout << ec.message();
		});
		//�Զ���������־��ӡ��������ɾ��
		int count = 0;
		while (true)
		{
			if (m_rcpClient->has_connected())
			{
				std::cout << "connected ok\n";
				break;
			}
			else
			{
				std::cout << "connected failed: " << count++ << "\n";
			}
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
public:
	rpc_client* m_rcpClient;
};



