#ifndef DTControl_H
#define DTControl_H
#include<memory>
#include<QJsonArray>
#include<QUuid>
#include<QJsonDocument>
#include<utility>
#include"gkd/dt_zzmb.h"
#include"gkd/mission_control.h"
#include"common/common.hpp"
#include"gkd/mission.h"
#include"gkd/intelligence.h"
#include"SDKStructDefine.h"
/**
* 接入导调；
* 基于导调内容构建兵力推荐内容
*by hqw
**/
using namespace APIBridgeSDK;
struct mosquitto;
namespace MQTTMission {

class DTControl{
public:
    DTControl();
    ~DTControl();
    void Clear();
    void ReSetZzmb();
    void CreateorGHBltjList(const QJsonObject &root); // 规划兵力推荐列表
    void CreateorDTZzmb(const QJsonObject &root); //导调作战目标

    void getBltjList(std::map<std::string,std::vector<MissionPlan> >& missionPlaneSet);
    void getDTZzmb(std::vector<DTZzmb>& zzmb_list);
    std::string getZzmblx(int type);

public:
    QJsonObject  _dt_root; //最新导调结果保存
    std::vector<string> dt_uuid_list; //导调系统绘制要素uuid
private:
    std::map<std::string,std::vector<MissionPlan> > _missionPlaneSet; //兵力推荐方案集
    std::vector<MQTTMission::DTZzmb> _zzmb_list;

};
}

#endif // DTControl_H
