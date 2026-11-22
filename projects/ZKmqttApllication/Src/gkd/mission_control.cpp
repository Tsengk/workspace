#include"gkd/mission_control.h"
#include <QDebug>
namespace MQTTMission{
MissionControl::MissionControl()
{

}

MissionControl::~MissionControl()
{

}

void MissionControl::reSet()
{
    _missionPlanList.clear();
    _model_uuid_pair.clear();
    _missionImmediateIntPlaneList.clear();
}

bool MissionControl::hasReceiveAllForce()
{
    return _missionPlanList.size() == _tforce_num;
}

void MissionControl::packageGantt(APIBridgeSDK::SchemeInfo &info)
{
    // 常量定义
    static const int DEFAULT_DURATION = 3000;
    static const int DEFAULT_SPEED = 20;
    static const int DEFAULT_ENTITY_TYPE = 1004;
    static const double DEFAULT_SCALE = 1.0;

    // 设置方案基本信息
    QString scheme_suffix = QString("方案");
    info.schmeId = _mission.task_id + "_" + scheme_suffix.toLocal8Bit().toStdString();
    QString task_id_str = QString::fromStdString(_mission.task_id);
    int underscore_pos = task_id_str.indexOf('_');
    if (underscore_pos != -1) {
        task_id_str = task_id_str.left(underscore_pos);
    }
    info.taskId = task_id_str.toLocal8Bit().toStdString();
    QString scheme_name = QString("测试方案");
    info.schemeName = scheme_name.toLocal8Bit().toStdString();

    QJsonObject gantt;
    QJsonArray behavior_list;
    QJsonArray entity_model;

    // 只处理最后一条任务计划数据
    if (!_missionPlanList.empty()) {
        const MissionPlan& mp = _missionPlanList.back();
        QString uuid = QUuid::createUuid().toString();

        // 构建行为数据
        QJsonObject behavior_data = createBehaviorData(mp, DEFAULT_DURATION, DEFAULT_SPEED);
        QJsonObject behavior_item = createBehaviorItem(mp, behavior_data, uuid, DEFAULT_DURATION);
        behavior_list.append(behavior_item);

        // 构建实体模型数据
        QJsonObject entity_item = createEntityModel(mp, DEFAULT_ENTITY_TYPE, DEFAULT_SCALE);
        entity_model.append(entity_item);

        // 缓存实体运行状态
        EntityRunningState ers;
        ers.model_id = mp.battle_unit_code;
        ers.uuid = uuid.toStdString();
        ers.status = 0;
        _model_uuid_pair.push_back(ers);
    }

    // 组装最终JSON
    gantt["behavior_list"] = behavior_list;
    gantt["cam_list"] = QJsonArray();
    gantt["entity_model"] = entity_model;

    QJsonDocument jsonDoc(gantt);
    QByteArray jsonData = jsonDoc.toJson(QJsonDocument::Compact);
    // info.jsoninfo = jsonData.toStdString();

    QString jsonString = QString::fromUtf8(jsonData);
    auto *codec = QTextCodec::codecForName("GBK");
    QByteArray gbkData = codec->fromUnicode(jsonString);
    info.jsoninfo = std::string(gbkData.constData(), gbkData.size());
}

QJsonObject MissionControl::createBehaviorData(const MissionPlan& mp, int duration, int speed)
{
    QJsonObject data;
    QJsonArray movePath;

    for(const auto& position : mp.route) {
        QJsonArray point;
        point.append(QJsonValue(position.longitude));
        point.append(QJsonValue(position.latitude));
        point.append(QJsonValue(position.altitude));
        point.append(QJsonValue(0));
        movePath.append(point);
    }

    data["movePath"] = movePath;
    data["duration"] = duration;
    data["isRTL"] = false;
    data["limitHeight"] = 0;
    data["start_time"] = 0;

    return data;
}

QJsonObject MissionControl::createBehaviorItem(const MissionPlan& mp, const QJsonObject& data, const QString& uuid, int duration)
{
    QJsonObject item;
    item["data"] = data;
    item["duration"] = duration;
    item["model_id"] = QString::fromStdString(mp.battle_unit_code);
    QString behavior_name = QString("自主机动");
    item["name"] = behavior_name;
    item["text"] = behavior_name;
    item["time"] = 0;
    item["type"] = 1;
    item["uuid"] = uuid;

    return item;
}

QJsonObject MissionControl::createEntityModel(const MissionPlan& mp, int entityType, double scale)
{
    QJsonObject entity;

    QJsonArray alt;
    alt.append(0);
    alt.append(0);
    alt.append(0);
    entity["alt"] = alt;

    entity["entity_type"] = entityType;
    QString layer_name = QString("空基");
    entity["layer_name"] = layer_name;
    entity["model_id"] = QString::fromLocal8Bit(mp.battle_unit_code.c_str());
    entity["name"] = QString::fromLocal8Bit(mp.battle_unit_code.c_str());

    QJsonArray pos;
    pos.append(QJsonValue(_mission.init_position.longitude));
    pos.append(QJsonValue(_mission.init_position.latitude));
    pos.append(QJsonValue(_mission.init_position.altitude));
    entity["pos"] = pos;
    entity["scale"] = scale;

    return entity;
}

bool MissionControl::hasOwnPlatform(string model_id)
{
    for(auto op : _ownPlatform_list){
        if(op.basic.model_id == model_id) return true;
    }
    return false;
}

void MissionControl::addOwnPlatform(OwnPlatform &ownP)
{
    ownP.update_time = currentTimeMillisec();
     std::lock_guard<std::mutex> guard(mtx);
    _ownPlatform_list.push_back(ownP);
}

bool MissionControl::getOwnPlatform(const string model_id,  OwnPlatform &ownP)
{
    for(auto op : _ownPlatform_list){
        if(op.basic.model_id == model_id){
            ownP = op;
            return true;
        }
    }
    return false;
}

void MissionControl::removeOwnPlatform(string model_id)
{
     std::lock_guard<std::mutex> guard(mtx);
    auto it = std::find_if(_ownPlatform_list.begin(), _ownPlatform_list.end(), [&](const OwnPlatform& s) { return s.basic.model_id == model_id; });
    if (it != _ownPlatform_list.end()) {
        _ownPlatform_list.erase(it); // 删除找到的元素
    }
    return;
}

bool MissionControl::hasTargetlatform(string model_id)
{
    for(auto op : _targetPlatform_list){
        if(op.basic.model_id == model_id) return true;
    }
    return false;
}

void MissionControl::addTargetlatform(Targetlatform &tarp)
{
     tarp.update_time = currentTimeMillisec();
    _targetPlatform_list.push_back(tarp);
}

bool MissionControl::getTargetlatform(const string model_id, Targetlatform &tarp)
{
    for(auto op : _targetPlatform_list){
        if(op.basic.model_id == model_id){
            tarp = op;
            return true;
        }
    }
    return false;
}

void MissionControl::removeTargetlatform(string model_id)
{
    auto it = std::find_if(_targetPlatform_list.begin(), _targetPlatform_list.end(), [&](const Targetlatform& s) { return s.basic.model_id == model_id; });
    if (it != _targetPlatform_list.end()) {
        _targetPlatform_list.erase(it); // 删除找到的元素
    }
    return;
}
}
