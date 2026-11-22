#include"gkd/dt_control.h"
#include <QDebug>
namespace MQTTMission{
DTControl::DTControl()
{

}

DTControl::~DTControl()
{

}

void DTControl::Clear()
{
    _zzmb_list.clear();
    _missionPlaneSet.clear();
    dt_uuid_list.clear();
}

void DTControl::CreateorGHBltjList(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    QJsonArray bltj = CTN["program_list"].toArray();
    std::vector<MissionPlan> missionPlane_list;
    for(QJsonValue o:bltj){
        QJsonObject obj = QJsonValue(o).toObject();
        std::string program_name = obj["program_name"].toString().toStdString();
        QJsonArray task_list = obj["task_list"].toArray();
        for(QJsonValue m: task_list){
            QJsonObject mp = QJsonValue(m).toObject();
            MissionPlan missionPlane;
            missionPlane.create(mp);
            missionPlane_list.push_back(missionPlane);
        }
        _missionPlaneSet[program_name] = missionPlane_list;
    }

}

void DTControl::CreateorDTZzmb(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonArray BlArr = root["CTN"].toArray();
    for(int i =0 ; i < BlArr.size(); i++){
        QJsonObject CTN = BlArr[i].toObject();
        MQTTMission::DTZzmb dtzzmb;

        dtzzmb.zzmb_id =  CTN["zzmb_id"].toInt();
        dtzzmb.zzmb_type  =  CTN["zzmb_type"].toInt();
        dtzzmb.zzmb_num  =  CTN["zzmb_num"].toInt();
        dtzzmb.zzmb_cxsj  = CTN["zzmb_cxsj"].toInt();
        dtzzmb.zzmb_desc = CTN["zzmb_desc"].toString().toStdString();
        dtzzmb.zzmb_zdcf =   CTN["zzmb_zdcf"].toBool();
        dtzzmb.zzmb_cfcs =  CTN["zzmb_cfcs"].toInt();
        dtzzmb.zzmb_mc =  CTN["zzmb_mc"].toString().toStdString();
        if(CTN.contains("zzmb_cftj")) dtzzmb.zzmb_cftj =  CTN["zzmb_cftj"].toBool();

        if(CTN.contains("zzmb_zzmbqy")){
            Region region;
            QJsonObject regionj = CTN["zzmb_zzmbqy"].toObject();
            QJsonArray point_array = regionj["point_list"].toArray();
            for(int i =0; i < point_array.size(); i++){
                QJsonObject p = point_array[i].toObject();
                Position           pos;
                pos.longitude = p["longitude"].toDouble();
                pos.latitude =  p["latitude"].toDouble();
                pos.altitude =  p["altitude"].toDouble();
                region.point_list.push_back(pos);
                region.area_name =  dtzzmb.zzmb_mc;
            }
            dtzzmb.zzmb_zzmbqy=  region;
        }

        if(CTN.contains("zzmb_cld")) {
            QJsonObject cld = CTN["zzmb_cld"].toObject();
            Position           pos;
            pos.longitude = cld["longitude"].toDouble();
            pos.latitude =  cld["latitude"].toDouble();
            pos.altitude =  cld["altitude"].toDouble();
            dtzzmb.zzmb_cld =  pos;
        }

        if(CTN.contains("zzmb_rychl")) dtzzmb.zzmb_rychl =  CTN["zzmb_rychl"].toInt();
        if(CTN.contains("zzmb_zbwhl")) dtzzmb.zzmb_zbwhl =  CTN["zzmb_zbwhl"].toInt();
        if(CTN.contains("zzmb_cgtj_1")) dtzzmb.zzmb_cgtj_1 =  CTN["zzmb_cgtj_1"].toInt();
        if(CTN.contains("zzmb_cgtj_2")) dtzzmb.zzmb_cgtj_2 =  CTN["zzmb_cgtj_2"].toInt();
        if(CTN.contains("zzmb_cgtj_3")) dtzzmb.zzmb_cgtj_3 =  CTN["zzmb_cgtj_3"].toInt();
        if(CTN.contains("zzmb_cgtj_4")) dtzzmb.zzmb_cgtj_4 =  CTN["zzmb_cgtj_4"].toInt();
        if(CTN.contains("zzmb_sbtj_1")) dtzzmb.zzmb_sbtj_1 =  CTN["zzmb_sbtj_1"].toInt();
        if(CTN.contains("zzmb_sbtj_2")) dtzzmb.zzmb_sbtj_2 =  CTN["zzmb_sbtj_2"].toInt();
        if(CTN.contains("zzmb_sbtj_3")) dtzzmb.zzmb_sbtj_3 =  CTN["zzmb_sbtj_3"].toInt();
        if(CTN.contains("zzmb_sbtj_4")) dtzzmb.zzmb_sbtj_4 =  CTN["zzmb_sbtj_4"].toInt();

        _zzmb_list.push_back(dtzzmb);
    }


}

void DTControl::getBltjList(std::map<string, std::vector<MissionPlan> > &missionPlaneSet)
{
    missionPlaneSet = _missionPlaneSet;
}

void DTControl::getDTZzmb(std::vector<DTZzmb>& zzmb_list)
{
    zzmb_list =  _zzmb_list;
}

string DTControl::getZzmblx(int type)
{
    if(type == 1){
        return "占领区域";
    }else if(type == 2){
        return "防守区域";
    }else if(type == 3){
        return "进攻区域";
    }else if(type == 4){
        return "撤离区域";
    }else if(type == 5){
        return "保护作战单元";
    }else if(type == 6){
        return "支援作战单元";
    }else if(type == 7){
        return "人员存活率";
    }else if(type == 8){
        return "装备完成率";
    }
}
}
