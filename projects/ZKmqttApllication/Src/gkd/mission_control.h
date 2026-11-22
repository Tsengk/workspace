#ifndef MissionControl_H
#define MissionControl_H
#include<memory>
#include<QJsonArray>
#include<QUuid>
#include<QJsonDocument>
#include<utility>
#include<mutex>
#include"common/common.hpp"
#include"gkd/mission.h"
#include"gkd/intelligence.h"
#include"SDKStructDefine.h"
/**
*任务调度管理类
*by hqw
**/
using namespace APIBridgeSDK;
struct mosquitto;  // 与mosquitto库保持一致
namespace MQTTMission{
struct EntityRunningState {
    string model_id;
    string uuid;
    int status; //0空闲,1任务执行中，2任务暂停
};

struct TargetPlatformDisappear{
    string target_ID;
    int disappear_state;
    int count =0;
};
class MissionControl{
public:
    MissionControl();
    ~MissionControl();

    void reSet(); //重置任务
    bool hasReceiveAllForce(); // 对应任务的任务规划兵力是否全部接受完成
    void packageGantt( APIBridgeSDK::SchemeInfo& info);

private:
    QJsonObject createBehaviorData(const MissionPlan& mp, int duration, int speed);
    QJsonObject createBehaviorItem(const MissionPlan& mp, const QJsonObject& data, const QString& uuid, int duration);
    QJsonObject createEntityModel(const MissionPlan& mp, int entityType, double scale);

public:
    bool hasOwnPlatform(string model_id);
    void addOwnPlatform(OwnPlatform& ownP);
    bool getOwnPlatform(const string model_id, OwnPlatform& ownP);
    void removeOwnPlatform(string model_id);

    bool hasTargetlatform(string model_id);
    void addTargetlatform(Targetlatform& tarp);
    bool getTargetlatform(const string model_id, Targetlatform& tarp);
    void removeTargetlatform(string model_id);

  //  void updateTargetDisappear(TargetPlatformDisappear* td);
    int _tforce_num = 0;  //最新任务的兵力个数登记
    Mission  _mission; //请求任务信息
    std::vector<APIBridgeSDK::OwnPlatform> _ownPlatform_list; // 我方装备缓存序列
    std::vector<string>   _new_ownPlatform_list;  //新增平台id记录；

    std::vector<APIBridgeSDK::Targetlatform> _targetPlatform_list; //目标缓存序列
    std::vector<string>   _new_targetPlatform_list;  //新增目标id记录；
    std::vector<TargetPlatformDisappear> _targetPlatformDisappear_list;// 目标消失索引队列

    std::vector<MissionPlan> _missionPlanList; //任务规划结果,如电磁ZG当前任务的后期规划结果不在这里
    std::vector<MissionPlan> _missionImmediateIntPlaneList; //任务临机干预规划结果
    std::vector<EntityRunningState> _model_uuid_pair; //任务对应的模型行为状态

    MissionExeStatus _missionStatus; //任务状态信息

private:

    std::mutex mtx;
};
}

#endif // MissionControl_H
