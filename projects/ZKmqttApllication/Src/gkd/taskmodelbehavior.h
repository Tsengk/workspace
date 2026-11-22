#ifndef TaskModelBehavior_H
#define TaskModelBehavior_H
#include<map>
#include<vector>
#include"mission.h"
#include "SDKStructDefine.h"
using namespace std;
/**
*任务行为效果类
*by hqw
**/
class TaskModelbehavior{
public:
    TaskModelbehavior();
     void packgeEffect();

    ~TaskModelbehavior();
public:
    bool isElecSignFly(int tasktype); //光电单机基区域 和悬停目标识别
    bool isPhoneElectLocation(int tasktype); //手机电磁识别定位
    bool isStrikeTargetTask(int tasktype); //山猫车打击任务
    bool isImmediateIntTask(int tasktype); //临机干预任务，目前电磁ZG用到
    bool isImmediateIntTaskKD(int tasktype); //临机干预任务，空地协同任务
    void getStrikeTarget(MQTTMission::Position& target);
    void setStrikeTarget(MQTTMission::Position& target);
    bool isShowelEctEnvelop(int taskid, string model_id);
    bool isShowPARadar(int taskid, string model_id);
    bool isShoweElectWindow(int taskid);

public:
    MQTTMission::Position _strikeTarget;
    std::multimap<int,string> electEnvelopList; // 任务对应的球形包络
    std::multimap<int,string> pRaadarList; // 任务对应的扇形状包络

    APIBridgeSDK::EntityEnvelop _phoneElectEnvelop; //包络圆形状
    APIBridgeSDK::PARadarData _phoneElectPAR; //包络扇形

    std::vector<string> _particleEffect_id_list; // 粒子效果id序列
    APIBridgeSDK::ParticleEffect _particleEffect1; // 烟尘 0.1
    std::vector<APIBridgeSDK::Position> _pE1_list;
     APIBridgeSDK::ParticleEffect _particleEffect2; // 起火烟雾  0.1
    std::vector<APIBridgeSDK::Position> _pE2_list;
     APIBridgeSDK::ParticleEffect _particleEffect3; //曳光弹效果 100 战场周边距离较远
     std::vector<APIBridgeSDK::Position> _pE3_list;
 public:
    std::vector<APIBridgeSDK::EntityEnvelop> _phoneElectEnvelop_list;//包络圆形
    std::vector<APIBridgeSDK::PARadarData> _pARadarData_list; //包络扇形
    std::vector<APIBridgeSDK::ParticleEffect> _particleEffect_list; //粒子效果
};
#endif // TaskModelBehavior_H
