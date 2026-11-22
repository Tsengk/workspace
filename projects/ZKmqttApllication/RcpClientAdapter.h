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
    // void syncSituCapture(const string& key,std::function<void(bool)> callback = nullptr){
    //     auto async_callback = [callback](asio::error_code ec, string_view data) {
    //         if (callback) {
    //             callback(ec ? false : true);  // 调用成功回调
    //         }
    //     };
    //     m_rcpClient->async_call("syncSituCapture",async_callback);
    // }
	//*****************************************���񷽷� begin*****************************************************
	//����ӽ�goto
	//latitude	����
	//longitude γ��
	//heading	�������
	//tilt		������
	//altitude	����߶�
	//time_ms	����ƶ���ʱ��
    void cameraGoto(double latitude, double longitude, double altitude, double tilt = 0, double heading = 0, int time_ms = 1000,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("cameraGoto",async_callback, latitude, longitude, heading, tilt, altitude, time_ms);
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
	void updatePointMark(const std::string& uuid, const APIBridgeSDK::Position& pos, int pixelRadius, const std::string& note, const std::string& colorStr)
	{
		m_rcpClient->call("updatePointMark", uuid, pos, pixelRadius, note, colorStr);
	}

	//Բ���ӿ�
	//uuid	Ψһ��ʶ
	//pos	����λ��
	//note	��ʾע��
	void updateCircleMark(const std::string& uuid, const APIBridgeSDK::Position& pos, int distanceRadius, const std::string& note, const std::string& colorStr)
	{
		m_rcpClient->call("updateCircleMark", uuid, pos, distanceRadius, note, colorStr);
	}

	//·����ʾ�ӿ�
	//uuid	Ψһ��ʶ
	//vecPos����
    void updateLineMark(const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos, const std::string& colorStr, std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };

        m_rcpClient->async_call("updateLineMark",async_callback, uuid, vecPos, colorStr);
	}

	//������ӿ�
	//uuid	Ψһ��ʶ
	//vecPos����
	//note	��ʾע��
    void updateRegionMark(const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos, const std::string& note, const std::string& colorStr,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("updateRegionMark",async_callback, uuid, vecPos, note,colorStr);
	}

	//ɾ�����
    void deleteMark(const std::string& uuid,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("deleteMark", async_callback,uuid);
	}

	//Ŀ�������ӿ�
	//tgtid		Ŀ��id
	//bVisible	����
	void setTargetVisible(const std::string& tgtid, bool bVisible)
	{
		m_rcpClient->call("setTargetVisible", tgtid, bVisible);
	}

    //目标动画接口
    //tgtid		目标id
    //bEnable	true:开启动画/false:关闭动画
    // void setTargetAnimationEnable(const std::string& tgtid, bool bEnable,,std::function<void(bool)> callback = nullptr)
    // {
    //     auto async_callback = [callback](asio::error_code ec, string_view data) {
    //         if (callback) {
    //             callback(ec ? false : true);  // 调用成功回调
    //         }
    //     };
    //     m_rcpClient->async_call("setTargetAnimationEnable",async_callback, tgtid, bEnable);
    // }

	//·����ʾ�ӿ�
	//uuid	Ψһ��ʶ
	//vecPos����
	void updateRoute(const std::string& uuid, const std::vector<APIBridgeSDK::Position>& vecPos)
	{
		m_rcpClient->call("updateRoute", uuid, vecPos);
	}

	//��ģ�ͶԻ��ظ���ʾ�ӿ�
	//content �ظ�����
    void showChatReply(const std::string& content,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("showChatReply",async_callback, content);
	}

	//����Ŀ����ʾ��Ƭ�ӿ�
	//data �澯����
	void showAlertData(const APIBridgeSDK::MilitaryAlertData& data)
	{
		m_rcpClient->call("showAlertData", data);
	}

	//������ʼ����
    void schemeStartPlay(std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("schemeStart", async_callback);
    // m_rcpClient->call("schemeStart");
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
    void schemeEndPlay(std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("schemeEnd", async_callback);

        //m_rcpClient->call("schemeEnd");
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

	void updateScheme(const APIBridgeSDK::SchemeInfo& shcemeData)
	{
		m_rcpClient->call("updateScheme", shcemeData);
	}

    //???????
    void loadAction(const APIBridgeSDK::ActionInfo& actionData)
    {
        m_rcpClient->call("onAction", actionData);
	}

    //������Ƶ��
    void loadVideoStream(const APIBridgeSDK::VideoStreamList& actionData,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("onStreamVideoPath",async_callback, actionData);
    }
	//����ʵ����Ϊ(����)
    void addEntityMission(const APIBridgeSDK::EntityMission& missionData,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addEntityMission",async_callback, missionData);
	}

	//ɾ��ʵ����Ϊ(����)
    void removeEntityMission(const std::string& missionData,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeEntityMission", async_callback,missionData);
	}

	//ɾ������
	void removeScheme(const std::string& schemeId)
	{
		m_rcpClient->call("removeScheme", schemeId);
	}

	//����ʵ����Ϊ
    void abandonMission(const std::string& missionId,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("abandonMission",async_callback, missionId);
	}

	//�л�ʵ��״̬
	void switchEntityRunningState(const std::string& entityId, RPC_EntityRunningState entityState)
	{
		m_rcpClient->call("switchEntityRunningState", entityId, entityState);
	}

	//����������ҷ�̬������
    void createOrUpdateOwnPlatformData(const APIBridgeSDK::OwnPlatform& ownData,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("updateOwnPlatData",async_callback, ownData);
	}

    void createOrUpdateOwnPlatformData(const std::vector<APIBridgeSDK::OwnPlatform>& ownDatas,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调

            }
        };
        m_rcpClient->async_call("batchUpdateOwnPlatData",async_callback, ownDatas);
	}

	//�������µз�̽������
    void createOrUpdateTargetPlatformData(const APIBridgeSDK::Targetlatform& targetData,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("updateTargetPlatData", async_callback,targetData);
	}

    void createOrUpdateTargetPlatformData(const std::vector<APIBridgeSDK::Targetlatform>& targetData,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("batchUpdateTargetPlatData",async_callback, targetData);
	}

	//ɾ���з����ҷ��豸����
    void removePlatformData(const std::string& model_id,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removePlatformData",async_callback, model_id);
	}


	//����һ�����Ŀ��
    void addIsmiTargetInfo(const APIBridgeSDK::ImsiTargetInfo& info,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addIsmiTargetInfo",async_callback, info);
	}

	//�Ƴ�һ�����Ŀ��
    void removeImsiTargetInfo(const std::string& imsi,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeImsiTargetInfo",async_callback, imsi);
	}

	//�Ƴ����е��Ŀ��
	void removeAllImsiTargetInfoAsync(std::function<void(bool)> callback = nullptr)
	{
		auto async_callback = [callback](asio::error_code ec, string_view data) {
			if (callback) {
				callback(ec ? false : true);  // 调用成功回调
			}
		};
		m_rcpClient->async_call("removeAllImsiTargetInfo", async_callback);
	}

    void removeAllImsiTargetInfo(std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeAllImsiTargetInfo",async_callback);
	}

	//������������Ϣ
    void addIsmiZGInfo(const APIBridgeSDK::ImsiZGInfo& info,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addZgInfo", async_callback,info);
	}

	//�Ƴ�һ����������Ϣ
    void removeIsmiZGInfo(const std::string& target_id,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeZgInfo",async_callback, target_id);
	}

	void removeIsmiAllZGInfoAsync(std::function<void(bool)> callback = nullptr)
	{
		auto async_callback = [callback](asio::error_code ec, string_view data) {
			if (callback) {
				callback(ec ? false : true);  // 调用成功回调
			}
		};
		m_rcpClient->async_call("removeAllZgInfo", async_callback);
	}

	//�Ƴ����е�����Ŀ��
    void removeIsmiAllZGInfo(std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeAllZgInfo",async_callback);
	}

    void updateElectInfoText(const std::string& info,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("updateElectStateInfo",async_callback, info);
    }
	//����һ���ؼ��¼�
    void addKeyEvent(const APIBridgeSDK::KeyEventDataSDK& data,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addKeyEvent",async_callback, data);
	}
	//����һ��ؼ��¼�
    void addKeyEventGroup(const std::list<APIBridgeSDK::KeyEventDataSDK>& data,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addKeyEventGroup",async_callback, data);
	}
	//ɾ���ؼ��¼�
    void removeEvent(const std::string& id,std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeEvent",async_callback, id);
	}
	//��չؼ��¼�
    void clearAllEvent(std::function<void(bool)> callback = nullptr)
	{
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("clearAllEvent",async_callback);
	}
    //电磁信息界面显隐
    void showElectWindow(std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);
            }
        };
        m_rcpClient->async_call("showElectWindow",async_callback);
    }
    void hideElectWindow(std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);
            }
        };
        m_rcpClient->async_call("hideElectWindow",async_callback);
    }

    //开启关闭目标雷达包络
    void openEnvelop(const APIBridgeSDK::EntityEnvelop& data,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("openEnvelop",async_callback, data);
    }
    void closeEnvelop(const std::string& model_id,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("closeEnvelop",async_callback, model_id);
    }

    //�����ر�PAɨ���״�
    void openPARadar(const APIBridgeSDK::PARadarData& data,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("openPARadar",async_callback, data);
    }
    void closePARadar(const std::string& model_id,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("closePARadar", async_callback,model_id);
    }

    //显示威胁范围
    void showThreatHemisphere(const APIBridgeSDK::DefenseSphere& data,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };

        m_rcpClient->async_call("showThreatHemisphere",async_callback, data);
    }
    void removeThreatHemisphere(const std::string& model_id,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeThreatHemisphere",async_callback, model_id);
    }

    //弹出消息提示框
    //title 标题
    //content 内容
    //mode 消息类型 0success 1warning 2info 3error
    //diplaySec 显示时长 秒
    void showNotification(const std::string& title, const std::string& content, int mode, int displaySec,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("showNotification",async_callback, title, content, mode, displaySec);
    }

    void showTextWindow(const APIBridgeSDK::TextWndInfo& data,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("showTextWindow",async_callback, data);
    }

    //显示特效
    void showParticleEffect(const APIBridgeSDK::ParticleEffect& data,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("showParticleEffect",async_callback, data);
    }
    //移除特效
    void removeParticleEffect(const std::string& particleId,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("removeParticleEffect",async_callback, particleId);
    }
    //删除所有粒子效果
    void clearAllParticleEffect(std::function<void(bool)> callback = nullptr)
    {

        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("clearAllParticleEffect",async_callback);
    }
    //大模型消息显示窗口
    void showLargeModelMsg(const APIBridgeSDK::LargeModelMsg& data,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("showLargeModelMsg",async_callback, data);
    }

    //展示作战目标
    void addMissionTargetInfo(const APIBridgeSDK::MissionTargetInfo& info,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addMissionTargetInfo",async_callback, info);
    }
    //清除作战目标
    void clearMissionTargetInfo(std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("clearMissionTargetInfo",async_callback);
    }
    //展示兵力推荐
    void addForceRecommondInfo(const std::list<APIBridgeSDK::ForceRecommondInfo>& info,std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("addForceRecommondInfo",async_callback, info);
    }
    //清除兵力推荐
    void clearForceRecommondInfo(std::function<void(bool)> callback = nullptr)
    {
        auto async_callback = [callback](asio::error_code ec, string_view data) {
            if (callback) {
                callback(ec ? false : true);  // 调用成功回调
            }
        };
        m_rcpClient->async_call("clearForceRecommondInfo",async_callback);
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



