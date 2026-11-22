#include"gkd/mission.h"
#include <QJsonObject>
#include <QJsonArray>

#ifdef WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

namespace MQTTMission{

Mission::Mission()
{

}

Mission::~Mission()
{

}

// Intelligence类实现
// Intelligence::Intelligence()
// {
//     target_ID = 0;
//     data_source = 0;
//     first_time = 0;
//     latest_time = 0;
//     space_type = 0;
//     capture_count = 0;
//     scout_type = 0;
//     target_plat_type = 0;
//     health_status = 0;
// }

// Intelligence::~Intelligence()
// {
// }

// bool Intelligence::parseFromJson(const QJsonObject& root)
// {
//     if (!root.contains("CTN")) {
//         return false;
//     }
    
//     QJsonObject ctn = root["CTN"].toObject();
    
//     // 必填字段
//     if (!ctn.contains("target_ID") || !ctn.contains("battle_unit_code") ||
//         !ctn.contains("first_time") || !ctn.contains("latest_time") ||
//         !ctn.contains("space_type") || !ctn.contains("capture_count") ||
//         !ctn.contains("scout_type") || !ctn.contains("target_plat_type")) {
//         return false;
//     }
    
//     target_ID = ctn["target_ID"].toInt();
//     battle_unit_code = ctn["battle_unit_code"].toString().toStdString();
//     first_time = ctn["first_time"].toString().toLongLong();
//     latest_time = ctn["latest_time"].toString().toLongLong();
//     space_type = ctn["space_type"].toInt();
//     capture_count = ctn["capture_count"].toInt();
//     scout_type = ctn["scout_type"].toInt();
//     target_plat_type = ctn["target_plat_type"].toInt();
    
//     // 可选字段
//     if (ctn.contains("data_source")) {
//         data_source = ctn["data_source"].toInt();
//     }
//     if (ctn.contains("health_status")) {
//         health_status = ctn["health_status"].toInt();
//     }
//     if (ctn.contains("exception_msg")) {
//         exception_msg = ctn["exception_msg"].toString().toStdString();
//     }
    
//     // 频率列表
//     if (ctn.contains("freq_list")) {
//         QJsonArray freqArray = ctn["freq_list"].toArray();
//         freq_list.clear();
//         for (int i = 0; i < freqArray.size(); ++i) {
//             freq_list.push_back(freqArray[i].toDouble());
//         }
//     }
    
//     //航迹点
//     if(ctn.contains("track_info")){
//         QJsonObject track_info = ctn["track_info"].toObject();
//         QJsonObject posobj= track_info["position"].toObject();
//         Position pos;
//         bool ok;
//         QString l = posobj["longitude"].toString();
//         pos.longitude =posobj.value("longitude").toDouble();
//         pos.latitude =posobj["latitude"].toString().toDouble(&ok);
//         pos.altitude =posobj["altitude"].toString().toDouble(&ok);
//         taack_info.position = pos;
//     }
//     return true;
// }

// Intelligence_D类实现
Intelligence_D::Intelligence_D()
{
    target_ID = "0";
    disappear_state = 0;
}

Intelligence_D::~Intelligence_D()
{
}

bool Intelligence_D::parseFromJson(const QJsonObject& root)
{
    if (!root.contains("CTN")) {
        return false;
    }
    
    QJsonObject ctn = root["CTN"].toObject();
    
    // 必填字段
    if (!ctn.contains("target_ID") || !ctn.contains("disappear_state")) {
        return false;
    }
    
    target_ID = ctn["target_ID"].toString().toStdString();
    disappear_state = ctn["disappear_state"].toInt();
    
    return true;
}

// // Equiep类实现
// Equiep::Equiep()
// {
//     report_time = 0;
//     platform_type = 0;
//     velocity = 0.0f;
//     trk_direction = 0.0f;
//     roll = 0.0f;
//     pitch = 0.0f;
//     yaw = 0.0f;
//     health_status = 0;
//     plat_redidual_electricity = 0;
//     task_execute_status = 0;
// }

// Equiep::~Equiep()
// {
// }

// bool Equiep::parseFromJson(const QJsonObject& root)
// {
//     if (!root.contains("CTN")) {
//         return false;
//     }
    
//     QJsonObject ctn = root["CTN"].toObject();
    
//     // 必填字段检查
//     if (!ctn.contains("report_time") || !ctn.contains("battle_unit_code") ||
//         !ctn.contains("platform_type") || !ctn.contains("deploy_position") ||
//         !ctn.contains("velocity") || !ctn.contains("trk_direction") ||
//         !ctn.contains("task_execute_status")) {
//         return false;
//     }
    
//     report_time = ctn["report_time"].toString().toLongLong();
//     battle_unit_code = ctn["battle_unit_code"].toString().toStdString();
//     platform_type = ctn["platform_type"].toInt();
//     velocity = ctn["velocity"].toDouble();
//     trk_direction = ctn["trk_direction"].toDouble();
//     task_execute_status = ctn["task_execute_status"].toInt();
    
//     // 位置信息解析
//     if (ctn.contains("deploy_position")) {
//         QJsonObject posObj = ctn["deploy_position"].toObject();
//         if (posObj.contains("longitude")) {
//             deploy_position.longitude = posObj["longitude"].toDouble();
//         }
//         if (posObj.contains("latitude")) {
//             deploy_position.latitude = posObj["latitude"].toDouble();
//         }
//         if (posObj.contains("altitude")) {
//             deploy_position.altitude = posObj["altitude"].toDouble();
//         }
//     }
    
//     // 可选字段
//     if (ctn.contains("roll")) {
//         roll = ctn["roll"].toDouble();
//     }
//     if (ctn.contains("pitch")) {
//         pitch = ctn["pitch"].toDouble();
//     }
//     if (ctn.contains("yaw")) {
//         yaw = ctn["yaw"].toDouble();
//     }
//     if (ctn.contains("health_status")) {
//         health_status = ctn["health_status"].toInt();
//     }
//     if (ctn.contains("exception_msg")) {
//         exception_msg = ctn["exception_msg"].toString().toStdString();
//     }
//     if (ctn.contains("plat_redidual_electricity")) {
//         plat_redidual_electricity = ctn["plat_redidual_electricity"].toInt();
//     }
//     if (ctn.contains("task_number")) {
//         task_number = ctn["task_number"].toString().toStdString();
//     }
    
//     return true;
// }

MissionPlan::MissionPlan()
{

}

MissionPlan::~MissionPlan()
{
    battle_unit_code = "";
    battle_unit_name = "";
}

int MissionPlan::getModelId()
{
    if(battle_unit_code == "9") return 1001;
    return 0;  // 默认返回值
}

void MissionPlan::create(const QJsonObject &root)
{
    task_id = root["task_id"].toString().toStdString();
    task_name=root["task_name"].toString().toStdString();
    task_type = root["task_type"].toInt();
    force_num = root["force_num"].toInt();

    QJsonArray pos_list = root["route"].toArray();
    for(QJsonValue p :pos_list){
        QJsonObject pos = QJsonValue(p).toObject();
        Position position;
        position.longitude = QJsonValue(pos["longitude"]).toDouble();
        position.latitude = QJsonValue(pos["latitude"]).toDouble();
        position.altitude = QJsonValue(pos["altitude"]).toDouble();
        route.push_back(position);
    }
    if(root.contains("target_ID")) target_ID =root["target_ID"].toInt();
    battle_unit_code = root["battle_unit_code"].toString().toStdString();
    battle_unit_name =  root.contains("battle_unit_name")?   root["battle_unit_name"].toString().toStdString(): root["battle_unit_code"].toString().toStdString(); //0106
    start_time = root["start_time"].toInt();
    end_time= root["start_time"].toInt();
    auto_level = root["auto_level"].toInt();
    priority = root["priority"].toInt();
    // if(root.contains("position_lidt")){
    //     mp.position_lidt
    // }

}

PhoneElec::PhoneElec()
{

}

PhoneElec::~PhoneElec()
{

}

RecElec::RecElec()
{

}

RecElec::~RecElec()
{

}

RecEleclocation::RecEleclocation()
{

}

RecEleclocation::~RecEleclocation()
{

}

// bool OwnPlatform::parseFromJson(const QJsonObject &root)
// {
//     if (!root.contains("CTN")) {
//         return false;
//     }

//     QJsonObject ctn = root["CTN"].toObject();

//     // 必填字段检查
//     if (!ctn.contains("report_time") || !ctn.contains("battle_unit_code") ||
//         !ctn.contains("platform_type") || !ctn.contains("deploy_position") ||
//         !ctn.contains("velocity") || !ctn.contains("trk_direction") ||
//         !ctn.contains("task_execute_status")) {
//         return false;
//     }
//     basic.model_id =  ctn["battle_unit_code"].toString().toStdString();
//     basic.report_time = ctn["report_time"].toString().toLongLong();
//     basic.platform_type =ctn["platform_type"].toInt();

//     // 位置信息解析
//     if (ctn.contains("deploy_position")) {
//         QJsonObject posObj = ctn["deploy_position"].toObject();
//         if (posObj.contains("longitude")) {
//             basic.deploy_position.longitude = posObj["longitude"].toDouble();
//         }
//         if (posObj.contains("latitude")) {
//             basic.deploy_position.latitude = posObj["latitude"].toDouble();
//         }
//         if (posObj.contains("altitude")) {
//             basic.deploy_position.altitude = posObj["altitude"].toDouble();
//         }
//     }

//     basic.velocity =  ctn["velocity"].toDouble();
//     basic.trk_direction = ctn["trk_direction"].toDouble();

//     // 姿态信息
//     if (ctn.contains("roll")) {
//         basic.roll= ctn["roll"].toDouble();
//     }
//     if (ctn.contains("pitch")) {
//         basic.pitch = ctn["pitch"].toDouble();
//     }
//     if (ctn.contains("yaw")) {
//         basic.yaw = ctn["yaw"].toDouble();
//     }
//     if (ctn.contains("health_status")) {
//         business.health_status = ctn["health_status"].toInt();
//     }
//     if (ctn.contains("exception_msg")) {
//         business.exception_msg = ctn["exception_msg"].toString().toStdString();
//     }
//     if (ctn.contains("plat_redidual_electricity")) {
//         business.plat_redidual_electricity = ctn["plat_redidual_electricity"].toInt();
//     }
//     // 任务状态 (可选)
//     if(ctn.contains("task_execute_status")){
//         business.task_execute_status = ctn["task_execute_status"].toInt();
//     }
//     if(ctn.contains("task_number")){
//         business.task_number =ctn["task_number"].toString().toStdString();     // 执行的任务编号
//     }
//     // 载荷状态列表
//     if(ctn.contains("equip_state_list")){
//         QJsonArray elist = ctn["equip_state_list"].toArray();

//         for (int i = 0; i < elist.size(); ++i) {
//             EquipmentState eState;
//             QJsonObject eq = elist[i].toObject();
//             eState.equip_no = eq["equip_no"].toString();
//             eState.equip_redidual_electricity = eq["equip_redidual_electricity"].toInt();
//             business.equip_state_list.push_back(eState);
//         }

//     }
//     return true;
// }

}
