#ifndef DTZzmb_H
#define DTZzmb_H
#include<memory>
#include<QJsonArray>
#include<QUuid>
#include<QJsonDocument>
#include<utility>
#include"common/common.hpp"
#include"gkd/mission.h"
#include"gkd/intelligence.h"
#include"SDKStructDefine.h"
/**
* 接入导调
*by hqw
**/
using namespace APIBridgeSDK;
struct mosquitto;
namespace MQTTMission {
struct ZzmbType{

};

struct Region{
    string area_name;
    std::vector<Position> point_list;
};
struct DTZzmb{
public:




public:
    int zzmb_id;	 //    作战目标ID
    int zzmb_type;	//作战目标类型
    int zzmb_num;	 //作战目标个数
    string zzmb_mc; // 作战目标名称
    string zzmb_desc; //作战目标描述
    int zzmb_sf; // 属方
    int zzmb_cxsj;	//持续时间
    bool zzmb_zdcf;	// 是否主动触发
    int zzmb_cfcs;	//触发参数
    bool zzmb_cftj;	//是否胜利触发 optional
    Region   zzmb_zzmbqy;	//目标区域 optional
    Position zzmb_cld;	//撤离点 optional
    int zzmb_rychl;	 //人员存活率 optional
    int zzmb_zbwhl;	 //装备完好率 optional
    int  zzmb_cgtj_1;	//成功条件1 optional
    int  zzmb_cgtj_2;	//成功条件2 optional
    int  zzmb_cgtj_3;	//成功条件3 optional
    int  zzmb_cgtj_4;	//成功条件4 optional
    int  zzmb_sbtj_1; //失败条件1 optional
    int  zzmb_sbtj_2;//失败条件2 optional
    int zzmb_sbtj_3;//失败条件3 optional
    int  zzmb_sbtj_4;//失败条件4 optional
};
}

#endif // DTZzmb_H
