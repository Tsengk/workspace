#include <mosquitto.h>
#include<thread>
#include<common/common.hpp>
#include <functional>
#include <chrono>
#include <random>
#include"gkd/c2manager.h"
#include "gkd/task_response_define.h"
#include"utils/utils_common.hpp"
#include<QUuid>
#include "../common/messagedialog.h"
#include "../common/events.h"
#include "../utils/eventbus.hpp"
#include "task_type_define.h"
#include<chrono>

C2Manager::C2Manager(std::shared_ptr <MqClient> gkd_client, std::shared_ptr <rest_rcp_client> rcp_client,GKDCommonCfg* cfg){
 _pgkd_mqtt_client = gkd_client;
 _prcp_client = rcp_client;
 _gkd_cfg = cfg;
 // ptr_dtControl = std::make_shared<DTControl>();
 // 日志系统已在 MainWindow 中初始化，这里不需要重复初始化
 logger.initLogger();
 logger.is_debug_mode = true;
 _dtControl.dt_uuid_list.clear();
 //QObject::connect(this, &C2Manager::mySignal, this, &C2Manager::mySlot);
 _imsi_list = _gkd_cfg->imsi_list;
 _zg_target_list= _gkd_cfg->zg_target_list;
 _user_id = _gkd_cfg->user_id;
 //重置特效
 _prcp_client->m_rcpClientAdapter->clearAllParticleEffect();
 //tx准备
 _TaskModelbehavior.packgeEffect();
 int index = 1;
 _r1 = true;
 _r2 = false;
 _r3 = true;
 _r4 = false;
 for(APIBridgeSDK::Position pos :_TaskModelbehavior._pE1_list){
     _TaskModelbehavior._particleEffect1.position = pos;
     string i =  std::to_string(index++);
     _TaskModelbehavior._particleEffect1.particleId= i;
     _TaskModelbehavior._particleEffect_id_list.push_back(i);
   //  _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect1);
 }

 for(APIBridgeSDK::Position pos :_TaskModelbehavior._pE2_list){
     _TaskModelbehavior._particleEffect2.position = pos;
     string i =  std::to_string(index++);
     _TaskModelbehavior._particleEffect2.particleId = i;
      _TaskModelbehavior._particleEffect_id_list.push_back(i);
   //  _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect2);
 }

 for(APIBridgeSDK::Position pos :_TaskModelbehavior._pE3_list){
     _TaskModelbehavior._particleEffect3.position = pos;
     string i =  std::to_string(index++);
     _TaskModelbehavior._particleEffect3.particleId = i;
     _TaskModelbehavior._particleEffect_id_list.push_back(i);
    // _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect3);
 }

 //订阅电磁侦干任务执行事件
EB.Subscribe<std::function<void(const Events::ElectromagneticTaskEvent&)>, const Events::ElectromagneticTaskEvent&>(
    Events::EXECUTE_ELECTROMAGNETIC_TASK,
    [this](const Events::ElectromagneticTaskEvent& event) {
    MQTTMission::RecElec rec_elec;
    rec_elec.task_id = event.taskId;
    rec_elec.target_id = event.targetId;
    rec_elec.freq = std::stod(event.frequency);
    rec_elec.battle_unit_no = event.unitId;
    lockElecTargetId(rec_elec);
    logger.INFO("通过事件执行电磁侦干任务：" + event.taskId);
});
EB.Subscribe<std::function<void(const std::string&)>, const std::string&>(
    Events::TEST_REMOVE_ZK_ITEM,
    [this](const std::string & target_id) {
    int i = 0;
    _prcp_client->m_rcpClientAdapter->removeIsmiAllZGInfo();
});

}

C2Manager::~C2Manager()
{

}

void C2Manager::MQTTSubscribe()
{
    std::vector<string> sub_topic_list;

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_rwgh_901),[this](const QJsonObject& root) {
        TRAY_NOTIFICATION.showInfo("MQTT消息", "接收到任务规划消息 (901)");
        
         _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","接收到任务规划消息 (901)",0,4);
       Subscribe_rwgh_901(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_bltj_905),[this](const QJsonObject& root) {
        _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","接收到兵力推荐集消息 (905)",0,4);
        Subscribe_bltj_905(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_dzmbqb_301),[this](const QJsonObject& root) {
          Subscribe_dzmbqb_301(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_dzmbxsb_302),[this](const QJsonObject& root) {
          Subscribe_dzmbxsb_302(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("QB/304"),[this](const QJsonObject& root) {
        Subscribe_dzmbxsb_304(root);
    });


    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_zzptjzbzt_303),[this](const QJsonObject& root) {
          Subscribe_zzptjzbzt_303(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("QB/308"),[this](const QJsonObject& root) {
        Subscribe_zzptjzbzt_308(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_rwzxzt_501),[this](const QJsonObject& root) {
        Subscribe_rwzxzt_501(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("ZK/LJ"),[this](const QJsonObject& root) {
        Subscribe_LJ(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("ZK/ZT"),[this](const QJsonObject& root) {
        Subscribe_ZT(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_push),[this](const QJsonObject& root) {
        Subscribe_push(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_prediction),[this](const QJsonObject& root) {
        Subscribe_prediction(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("ZK/KM6"),[this](const QJsonObject& root) {
        Subscribe_KM6(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/531"),[this](const QJsonObject& root) {
        Subscribe_531(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/530"),[this](const QJsonObject& root) {
        Subscribe_CK530(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/532"),[this](const QJsonObject& root) {
        // Subscribe_CK532(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/534"),[this](const QJsonObject& root) {

        Subscribe_CK534(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/535"),[this](const QJsonObject& root) {

        Subscribe_CK535(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/536"),[this](const QJsonObject& root) {
        Subscribe_CK536(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/537"),[this](const QJsonObject& root) {
        Subscribe_CK537(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/501"),[this](const QJsonObject& root) {

        Subscribe_rwzxzt_501(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("CK/502"),[this](const QJsonObject& root) {
        TRAY_NOTIFICATION.showInfo("MQTT消息", "接收到CK/502任务执行状态消息");
        // 显示消息弹窗
        _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","接收到CK/502任务执行状态消息",0,3);

        Subscribe_rwzxzt_502(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("ZK/test"),[this](const QJsonObject& root) {
        Subscribe_ZKtest(root);
    });
    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("TY/703"),[this](const QJsonObject& root) {
        Subscribe_TY103(root);
    });
    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("TY/704"),[this](const QJsonObject& root) {
        Subscribe_TY104(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString("thing/product/human/osd"),[this](const QJsonObject& root) {
        Subscribe_human_location(root);
    });


}

void C2Manager::MQTTSubscribe_DT()
{
    std::vector<string> sub_topic_list;

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_dt_801),[this](const QJsonObject& root) {
        TRAY_NOTIFICATION.showInfo("MQTT消息", "接收到导调作战目标集消息 (801)");

        _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","接收到导调作战目标集消息 (801)",0,4);

        for(auto uuid : _dtControl.dt_uuid_list){
            _prcp_client->m_rcpClientAdapter->deleteMark(uuid);
        }

        _dtControl.Clear();
        _dtControl._dt_root = root; //缓存最新导调结果
        subscribe_dt_801(root);
        //清除作战目标
        _prcp_client->m_rcpClientAdapter->clearMissionTargetInfo();
        //显示作战目标
        APIBridgeSDK::MissionTargetInfo info;
        info.titleInfo = QString(u8"接收到作战目标集 时间：%1").arg(QDateTime::currentDateTime().toString("yyyy-mm-dd hh:MM:ss")).toStdString();

        std::vector<MQTTMission::DTZzmb> zzmb_list;
        _dtControl.getDTZzmb(zzmb_list);
        for (int i = 0; i< zzmb_list.size(); i++)
        {
            MQTTMission::DTZzmb& dtzzmb = zzmb_list[i];
            APIBridgeSDK::TargetInfo d;
            d.id = QUuid::createUuid().toString().toStdString();
            d.autoNotify = i % 2 == 0;
            d.duration = QString(u8"%1小时").arg(QString::number(i + 1)).toStdString();
            d.description = dtzzmb.zzmb_desc;
            //d.type =  _dtControl.getZzmblx(dtzzmb.zzmb_mc);
            d.type =  dtzzmb.zzmb_mc; //界面显示类型暂时以名称替代
            info.targetInfo.push_back(d);
            if(dtzzmb.zzmb_zzmbqy.point_list.size() >2){
                std::vector<APIBridgeSDK::Position> region;
                for(int i =0; i <dtzzmb.zzmb_zzmbqy.point_list.size();i++ ){
                    MQTTMission::Position position =dtzzmb.zzmb_zzmbqy.point_list[i];
                    APIBridgeSDK::Position apiPos;
                    apiPos.create(position.longitude,position.latitude,position.altitude);
                    region.push_back(apiPos);

                }

                _dtControl.dt_uuid_list.push_back(d.id);
                if (d.type.find("屏蔽区") != std::string::npos) {
                    _prcp_client->m_rcpClientAdapter->updateRegionMark(d.id,region,utf8ToGbk(d.type), "#FF0000");
                }else{
                     _prcp_client->m_rcpClientAdapter->updateRegionMark(d.id,region,utf8ToGbk(d.type), "#00FF00");
                }

            }
        }
        _prcp_client->m_rcpClientAdapter->addMissionTargetInfo(info);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_dt_802),[this](const QJsonObject& root) {
        _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","接收到导调信息 (802)",0,4);
        subscribe_dt_802(root);
    });

    _pgkd_mqtt_client->GetMqttConn()->Subscribe(QString::fromStdString(_gkd_cfg->subscribe_topic_dt_803),[this](const QJsonObject& root) {
        subscribe_dt_803(root);
    });

}

void C2Manager::RPCSubscribe()
{
    // 获取gy请求并包装下发到2-1
    TPCallback p =  [this]( string_view& task){
        APIBridgeSDK::OwnPlatform op;
        QJsonParseError jsonError;
        QString str = QString::fromStdString(std::string(task));
        QJsonDocument _document = QJsonDocument::fromJson(str.toUtf8().data(), &jsonError);
        if (jsonError.error != QJsonParseError::NoError) return;
        QJsonObject json = _document.object();
        logger.INFO("-------------------------新任务开启--------------------");
        //重置任务
        ReSetMission();
         // 创建任务规划起始信息
        QString task_id = json["task_id"].toString();
        if(task_id == ""){
            TRAY_NOTIFICATION.showInfo("gy消息", "任务编号为空，请确认是否选中任务方案");
            _prcp_client->m_rcpClientAdapter->showNotification("gy消息","任务编号为空，请确认是否选中任务方案",3,4);
            return;
        }


        Mission  mission;
        json["task_id"] = task_id+ QString::fromStdString("_") + QString::fromStdString(std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
        if( CreatorMission(json,mission)){
          logger.INFO("......任务："+_mission_control._mission.task_id+" 请求解析成功......");
        }else{
         logger.INFO("......任务请求解析失败.");
            return;
        }

         //2-1任务规划请求打包
       // QJsonObject root;
        if(PackageMission(json,_root_rwgh)){

            logger.INFO("......任务："+_mission_control._mission.task_id+"任务规划请求打包完成");

        }else{
            logger.INFO("......任务："+_mission_control._mission.task_id+"请求打包失败，检查任务类型是否正确等问题");
            return;

        }

        if(_mission_control._mission.task_id ==""){
            logger.INFO("......任务id为空，是否未选中任务方案？");

        }
        if(_TaskModelbehavior.isStrikeTargetTask(_mission_control._mission.task_type)){
            MQTTMission::MissionPlan missionPlan;

            CreateorShanmaoMissionPlan(missionPlan);
            _mission_control._missionPlanList.push_back(missionPlan);
            logger.INFO("......山猫车目标打击任务任务："+_mission_control._mission.task_id+"不请求本地打包平台成功");
            TRAY_NOTIFICATION.showInfo("Zk消息", "已打包，无需航线");
            _prcp_client->m_rcpClientAdapter->showNotification("Zk消息", "已打包，无需航线",0,3);
         //请求2-1进行任务规划
        }else{
             //  MQTTMission::MissionPlan missionPlan;
            logger.INFO("......任务："+_mission_control._mission.task_id+"任务规划请求发送出");
           // CreateorShanmaoMissionPlan(missionPlan);
           // _mission_control._missionPlanList.push_back(missionPlan);
            _plan_request_flag = false;
            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/101", _root_rwgh);

        }

        //电磁任务打开电磁窗口
        if(_TaskModelbehavior.isShoweElectWindow(_mission_control._mission.task_type)){

            _prcp_client->m_rcpClientAdapter->showElectWindow();
        }


    };
    _prcp_client->subscribeTask("taskplan",p);
    // 获取gy请求并包装下发到操控席位
    TPCallback dt =  [this]( string_view& task){
      //  logger.INFO("rpc任务信息：" + std::string(task));
      logger.INFO("接收到任务下发请求");

        QJsonParseError jsonError;
        QJsonDocument _document = QJsonDocument::fromJson(task.data(), &jsonError);
        if (jsonError.error != QJsonParseError::NoError) return;
        logger.INFO(".....任务："+_mission_control._mission.task_id+" 下发到操控席位开始......");
         QJsonObject json = _document.object();
        logger.INFO("......任务："+_mission_control._mission.task_id+"下发从gy接收成功");
        QJsonArray entity_model_array = json["entity_model"].toArray();
        for(const QJsonValue & obj:entity_model_array ){
          if(!obj.isObject()) continue;
           QJsonObject entity_model = obj.toObject();
          QString model_id=  entity_model["model_id"].toString();
          QString name =  entity_model["name"].toString();
          updata_zyc_name(model_id.toStdString(),name.toStdString());
        }

        // if(_mission_control._mission.task_type == 100){ //控制大疆无人机下发


            QJsonArray behavior_list;
            if(json.contains("behavior_list")) behavior_list = json["behavior_list"].toArray();
            if(behavior_list.size()  == 0) return;

            //高度校验
            bool unroul = false;
            std::vector<double> limitHeigh_list;
            for(int i=0; i < behavior_list.size(); i++){
                QJsonObject behavior =  behavior_list[i].toObject();
                QString  rid = behavior["model_id"].toString();
                if(_gkd_cfg->zyc.find("2003") !=_gkd_cfg->zyc.end() && //单兵
                        hasId(_gkd_cfg->zyc["2003"],rid.toStdString()) ){
                     continue;
                  }
                 QJsonObject data = behavior["data"].toObject();
                double limieHight = data["limitHeight"].toDouble();
                 if(limieHight < 20) unroul = true;
                  limitHeigh_list.push_back(limieHight);
            }
            if(!checkAnyDifference(limitHeigh_list,10) || unroul){
                _prcp_client->m_rcpClientAdapter->showNotification("消息","无人机高度不合规，检查高度是否小于20或者高度差小于10",2,4);
                logger.INFO("......高度不合规");
                return;
            }
            _prcp_client->m_rcpClientAdapter->schemeEndPlay();
            _prcp_client->m_rcpClientAdapter->schemeStartPlay();
            for(int i=0; i < behavior_list.size(); i++){
                QJsonObject behavior =  behavior_list[i].toObject();
                QString  rid = behavior["model_id"].toString();
                if(_gkd_cfg->zyc.find("1002") != _gkd_cfg->zyc.end() &&
                   hasId(_gkd_cfg->zyc["1002"],rid.toStdString()) ){
                    QJsonObject data = behavior["data"].toObject();
                    QJsonArray movePath = data["movePath"].toArray();
                    double limieHight = data["limitHeight"].toDouble();
                    _mini4_pro_path.clear();

                    for(int i =0; i <movePath.size(); i++ ){
                        QJsonArray pos_list = movePath[i].toArray();
                        MQTTMission::Position position;
                        position.longitude =  pos_list[0].toDouble();
                        position.latitude = pos_list[1].toDouble();
                        position.altitude = pos_list[2].toDouble();
                        _mini4_pro_path.push_back(position);
                    }

                    ControlDJiMini4_Pro(rid,limieHight,speed,_mini4_pro_path);
                }else if(_gkd_cfg->zyc.find("2003") !=_gkd_cfg->zyc.end() && //单兵
                   hasId(_gkd_cfg->zyc["2003"],rid.toStdString())  ){
                    QJsonObject missionPlan;
                    PackageMissionPlan_ToCK(json,missionPlan);
                    bool isPublish =  _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/103", missionPlan);
                    logger.INFO("......任务："+_mission_control._mission.task_id+"下发作战任务成功");

                }

            }

        //     return;
        // }else{ // 其他类型任务，如20 下发到班组
        //     //下发作战任务
        //     QJsonObject missionPlan;
        //     PackageMissionPlan_ToCK(json,missionPlan);
        //     bool isPublish =  _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/103", missionPlan);
        //     logger.INFO("......任务："+_mission_control._mission.task_id+"下发作战任务成功");

        //     int s = 0 + rand() % (200 + 1);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(350+s));
        //     //下发作战平台
        //     for(auto mis : _mission_control._missionPlanList){
        //         QJsonObject missionBattle;
        //         PackageMissionBattle_ToCK(json,mis,missionBattle);
        //         _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/104", missionBattle);
        //         logger.INFO("......任务："+_mission_control._mission.task_id+"下发作战平台成功");

        //     }
        // }
         last_time = std::chrono::high_resolution_clock::now();

        // qDebug() <<QString::fromUtf8("人机交接控制权限时延开始，当前时间为")<<timePointToString(last_time);
        // qDebug() <<QString::fromUtf8("人机协同行动控制计时开始,当前时间为")<<timePointToString(last_time);
        //下发作战任务
        QJsonObject missionPlan;
        PackageMissionPlan_ToCK(json,missionPlan);
        bool isPublish =  _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/103", missionPlan);
        logger.INFO("......任务："+_mission_control._mission.task_id+"下发作战任务成功");




        if(isPublish){
            _prcp_client->m_rcpClientAdapter->schemeEndPlay();
            _prcp_client->m_rcpClientAdapter->schemeStartPlay();

        }
    };
     _prcp_client->subscribeTask("dotask",dt);


     // 锁定电磁
     TPCallback dt2 =  [this]( string_view& task){

         // QJsonParseError jsonError;  // 注释掉未使用的变量
         QString smsi = QString::fromStdString(std::string(task));
         QJsonObject root;
         root["imsi"] = smsi;
         Subscribe_LocationTarget(root); //根据锁定目标请求路径规划

     };
     _prcp_client->subscribeTask("locatesmsi",dt2);




     // 请求交互动作
     TPCallback dt3 =  [this]( string_view& task){
        // QJsonParseError jsonError;
        QString buc = QString::fromStdString(std::string(task));
        QString rtmpserver_ip = QString::fromStdString(_gkd_cfg->rtmpserver_ip);
        if(buc ==""){
             _prcp_client->m_rcpClientAdapter->showNotification("消息","装备信息为空",2,4);
            logger.INFO("......装备：装备信息为空");
            return;
        }

        // 特判：task=="ys" 时按配置推送视频流
        if(buc == "ys"){
            VideoStreamList ys_list;
            const std::vector<std::string>& ys_paths = _gkd_cfg->ys_video_streams;
            for(const std::string& path : ys_paths){
                // 取地址最后一段作为流标识，如 zs01
                std::string key = path.substr(path.find_last_of('/') + 1);
                VideoStream ys_vs;
                ys_vs.vsName = key + "[视频" + key + "]";
                ys_vs.model_id = key;
                ys_vs.equipmentSeqNo = key;
                // 协议类型根据地址动态解析
                size_t scheme_pos = path.find("://");
                ys_vs.vsType = (scheme_pos != std::string::npos) ? path.substr(0, scheme_pos) : "rtmp";
                ys_vs.vsPath = path;
                ys_list.videoStreamList.push_back(ys_vs);
                logger.INFO("添加视频流(ys): " + ys_vs.vsName + ", 类型: " + ys_vs.vsType + ", 路径: " + ys_vs.vsPath);
            }
            if(ys_list.videoStreamList.empty()){
                logger.INFO("配置中没有可用的视频流地址");
                return;
            }
            std::thread t(&C2Manager::loadVideoStream, this, ys_list);
            t.detach();
            return;
        }

        VideoStreamList vslist;
        VideoStream vs;
        vs.vsName = buc.toStdString()+"[无人机"+buc.toStdString()+"]";
        vs.model_id =buc.toStdString();
        vs.equipmentSeqNo= buc.toStdString();
        vs.vsType="rtmp";
        vs.vsPath = "rtmp://"+rtmpserver_ip.toStdString()+":1935/live/"+buc.toStdString();
        logger.INFO(vs.vsType);
        vslist.videoStreamList.push_back(vs);
        //抛出线程，否则会被打断连接
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::thread t(&C2Manager::loadVideoStream,this,vslist);
        t.detach();
        
     };
     _prcp_client->subscribeTask("videoStreamCall",dt3);

     // 电磁ZG目标锁定
     TPCallback dt4 =  [this]( string_view& task){

         QString smsi = QString::fromStdString(std::string(task));
        logger.INFO("rpc 目标锁定信息：" + smsi.toStdString());

        auto it =  std::find_if(_recElec534.begin(),_recElec534.end(),[&](const APIBridgeSDK::ImsiZGInfo& zginfo){
                   return   zginfo.target_id == smsi.toStdString();
        });

        if(it != _recElec534.end()){
            MQTTMission::RecElec rec_elec;
            rec_elec.task_id = _mission_control._mission.task_id;
            rec_elec.target_id = smsi.toStdString();
            rec_elec.freq = std::stod(it->frequcency);
            rec_elec.battle_unit_no = it->entity_id; //这里的装备编号应用过上报的电磁目标ID获得

            lockElecTargetId(rec_elec);
            logger.INFO("......任务："+ rec_elec.task_id +" 锁定目标：" + rec_elec.target_id);
        }else{
            logger.INFO("！！！！......任务："+ _mission_control._mission.task_id+" 未查询到锁定目标：" + smsi.toStdString());

        }

     };
     _prcp_client->subscribeTask("locateZgTarget",dt4);


     // 空地协同动作
     TPCallback dt5 =  [this]( string_view& task){


         QString improvisecomd = QString::fromStdString(std::string(task));

         if(!_TaskModelbehavior.isImmediateIntTaskKD(_mission_control._mission.task_type)){

             _prcp_client->m_rcpClientAdapter->showNotification("消息","非空地协同任务，无效触发",2,4);

             return;

         }

         int LJGH = 0;
         if("IdentToTrack" ==improvisecomd){ //识别转跟踪
             LJGH = LJGH_KDXT_GZ;
            lockElecTargetId(improvisecomd.toStdString(),LJGH);

         }else if("Sanction" == improvisecomd){ //授权打击
             LJGH = LJGH_KDXT_SQ;
             lockElecTargetId(improvisecomd.toStdString(),LJGH);

         }else if("StopComd" == improvisecomd) { //停止
             LJGH = LJGH_KDXT_JS;
             lockElecTargetId(improvisecomd.toStdString(),LJGH);
         }

     };
     _prcp_client->subscribeTask("ImproviseComd",dt5);
     // 任务停止
     TPCallback dt6 =  [this]( string_view& task){
        // 构造 STOP 命令
        logger.INFO("收到任务停止请求");
        QJsonObject root;
        root["cmd"] = "STOP";
        root["sn"] = QString::number(currentTimeMillisec());
        root["data"] = QJsonObject(); // 空数据对象

        // 发送 STOP 命令到 xjsim 的命令主题
        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/excute", root);

        // 添加40-100ms随机延时
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(40, 100);
        int delay_ms = dis(gen);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

        logger.INFO("发送接管命令成功");

        // 同时重置本地任务状态
        // ReSetMission();
     };
     _prcp_client->subscribeTask("task_stop",dt6);

     //点击目标信息
     TPCallback dt7 =  [this]( string_view& task){
         QString click_str = QString::fromStdString(std::string(task));
         QStringList click_list = click_str.split("|");

         if(click_list.size() < 4){
              _prcp_client->m_rcpClientAdapter->showNotification("错误","目标选中失败",2,3);
             return;
         }

         MQTTMission::Position position(QVariant(click_list[1]).toDouble(),QVariant(click_list[2]).toDouble(),QVariant(click_list[3]).toDouble(),0.0);
         _mission_control._mission._selectFeatureInfo._select_id =QVariant(click_list[0]).toString().toStdString();
         _mission_control._mission._selectFeatureInfo._position =position;
          logger.INFO("目标已选中："+ std::to_string(position.longitude) + "," + std::to_string(position.latitude) );
         _prcp_client->m_rcpClientAdapter->showNotification("消息","目标已选中",2,4);
     };
     _prcp_client->subscribeTask("SendoutCurTarInfo",dt7);


     //请求兵力推荐集
     TPCallback dt8 =  [this]( string_view& task){
         QString click_str = QString::fromStdString(std::string(task));

        _prcp_client->m_rcpClientAdapter->showNotification("提示","请求兵力推荐集",0,3);

         QJsonObject root;
        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/108", root);

     };

     _prcp_client->subscribeTask("requestForceRecommond",dt8);


     TPCallback dt9 =  [this]( string_view& task){
        QString click_str = QString::fromStdString(std::string(task));
        std::cout << click_str.toStdString() << std::endl;
        logger.INFO(click_str.toStdString());

        // 将接收到的数据转为JSON格式并发送到 llm/area_info
        QJsonParseError jsonError;
        QJsonDocument doc = QJsonDocument::fromJson(click_str.toUtf8(), &jsonError);

        QJsonObject sendObj;
        if (jsonError.error == QJsonParseError::NoError && doc.isObject()) {
            // 如果已经是有效的JSON对象，直接使用
            sendObj = doc.object();
        } else {
            // 否则构造一个包含原始数据的JSON对象
            sendObj["data"] = click_str;
            sendObj["timestamp"] = currentTimeMillisec();
        }

        // 发送到 llm/area_info 主题
        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/area_info", sendObj);
        logger.INFO("已发送区域信息到 /llm/area_info");

        //QString schemeInfo = QString::fromStdString(std::string(si));
        QString click_str1 = click_str.replace("schemeInfo","\"schemeInfo\"");
        QString click_str2 = click_str1.replace("taskInfo","\"taskInfo\"");
        QString click_str3 = click_str2.replace("type:","\"type\":");
        logger.INFO(click_str3.toStdString());
        QJsonObject root;
        PackageTaskAndZB(click_str3, root);

        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/status/task_info", root);

     };
     _prcp_client->subscribeTask("sendCurSchemeInfo",dt9);

     // 态势地图接收
     TPCallback dt10 =  [this]( string_view& t){

         // QJsonParseError jsonError;  // 注释掉未使用的变量
        QString json = QString::fromStdString(std::string(t));
        QByteArray byteArray = QByteArray::fromBase64(json.toLatin1());

        int imgWidth = 0;
        int imgHeight = 0;
        const unsigned char* data = reinterpret_cast<const unsigned char*>(byteArray.constData());
        int size = byteArray.size();

         // 1. 尝试解析 PNG 格式 (签名: 89 50 4E 47 0D 0A 1A 0A)
        if (size > 24 && data[0] == 0x89 && data[1] == 'P' && data[2] == 'N' && data[3] == 'G') {
            // PNG 宽度在偏移 16，高度在偏移 20，均为 4 字节大端序
            imgWidth = (data[16] << 24) | (data[17] << 16) | (data[18] << 8) | data[19];
            imgHeight = (data[20] << 24) | (data[21] << 16) | (data[22] << 8) | data[23];
        }
        // 2. 尝试解析 JPG 格式 (签名: FF D8)
        else if (size > 2 && data[0] == 0xFF && data[1] == 0xD8) {
            int pos = 2;
            while (pos < size - 9) { // 至少还需要读取标记头
                if (data[pos] != 0xFF) break; // 格式错误或结束
                unsigned char marker = data[pos + 1];
                int len = (data[pos + 2] << 8) | data[pos + 3];

                 // 查找 SOF0 (0xC0) 到 SOF3 (0xC3) 帧开始标记，包含宽高
                if (marker >= 0xC0 && marker <= 0xC3) {
                    imgHeight = (data[pos + 5] << 8) | data[pos + 6];
                    imgWidth = (data[pos + 7] << 8) | data[pos + 8];
                    break;
                }
                pos += 2 + len; // 跳过当前段
            }
        }
        
        QJsonObject dataObj;
        dataObj["image_data"] = json;
        dataObj["width"] = imgWidth;
        dataObj["height"] = imgHeight;

        //构建JSON数据包
        QJsonObject root;
        root["cmd"] = "MAP_SYNC";
        root["sn"] = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        root["timestamp"] = QDateTime::currentMSecsSinceEpoch();//添加时间戳
        root["data"] = dataObj;

        if (_pgkd_mqtt_client && _pgkd_mqtt_client->GetMqttConn()) {
            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/pad/excute",root);
            logger.INFO("已发送MAP_SYNC态势截图到话题/llm/pad/excute");
        }
     };
     _prcp_client->subscribeTask("syncSituCapture",dt10);

     // 任务方案同步
     TPCallback dt11 =  [this]( string_view& si){

         // QJsonParseError jsonError;  // 注释掉未使用的变量
         QString schemeInfo = QString::fromStdString(std::string(si));

         QJsonObject root;
         PackageTaskAndZB(schemeInfo, root);

        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/status/task_info", root);

     };
     _prcp_client->subscribeTask("sendCurSchemeInfo",dt11);


     // 新版本军事博弈 敌方目标0303
     TPCallback dt12 =  [this]( string_view& si){

         QString schemeInfo = QString::fromStdString(std::string(si));

         QString schemeInfo1 = schemeInfo.replace("\n","");
         QString schemeInfo2 = schemeInfo1.remove("\\");
         std::cout << schemeInfo1.toStdString() << std::endl;
         logger.INFO(schemeInfo2.toStdString());
         QJsonParseError jsonError;
         QJsonDocument doc = QJsonDocument::fromJson(schemeInfo1.toUtf8(), &jsonError);

         QJsonObject sendObj;
         if (jsonError.error == QJsonParseError::NoError) {
             // 如果已经是有效的JSON对象，直接使用
             sendObj["target_list"] = doc.array();
             _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/control/1", sendObj);

              //军事博弈重置
             reSetJSBY();

             QJsonArray tp_list = doc.array();
             for(int i = 0;i < tp_list.size(); i++){
              QJsonObject tp = tp_list[i].toObject();
              bool istarget = tp["force"].toInt();
              std::string id = tp["id"].toString().toStdString();
              std::vector<JSBY_Param>  _jsby_target_value_list;
              std::vector<JSBY_Param>  _jsby_platform_value_list;
                 //敌我双方数值输入
                 JSBY_Param by_value;
                if(tp.contains("speed")) {
                     by_value.name= "移动速度";
                     by_value.value =  tp["speed"].toString().toDouble();
                     istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

                if(tp.contains("挂载弹药")) {
                    by_value.name= "挂载弹药";
                    by_value.value =  tp["挂载弹药"].toString().toDouble();
                    istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

                if(tp.contains("油量")) {
                    by_value.name= "油量";
                    by_value.value =  tp["油量"].toString().toDouble();
                    istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

                if(tp.contains("升限值")) {
                    by_value.name= "打击半径";
                    by_value.value =  tp["打击半径"].toString().toDouble();
                    istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

               if(tp.contains("加速度")) {
                     by_value.name= "加速度";
                     by_value.value =  tp["加速度"].toString().toDouble();
                     istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }


               if(tp.contains("打击半径")) {
                     by_value.name= "打击半径";
                     by_value.value =  tp["打击半径"].toString().toDouble();
                     istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

               if(tp.contains("最大起飞重量")) {
                   by_value.name= "最大起飞重量";
                   by_value.value =  tp["最大起飞重量"].toString().toDouble();
                   istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
               }

               if(tp.contains("目标侦查能力")) {
                     by_value.name= "目标侦查能力";
                     by_value.value =  tp["目标侦查能力"].toString().toDouble();
                     istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

               if(tp.contains("隐身能力")) {
                     by_value.name= "隐身能力";
                     by_value.value =  tp["隐身能力"].toString().toDouble();
                     istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
                }

               if(tp.contains("抗干扰能力")) {
                   by_value.name= "抗干扰能力";
                   by_value.value =  tp["抗干扰能力"].toString().toDouble();
                   istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
               }

               if(tp.contains("环境适应能力")) {
                   by_value.name= "环境适应能力";
                   by_value.value =  tp["环境适应能力"].toString().toDouble();
                   istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
               }

               if(tp.contains("目标侦查能力")) {
                   by_value.name= "目标侦查能力";
                   by_value.value =  tp["目标侦查能力"].toString().toDouble();
                   istarget == true ?_jsby_target_value_list.push_back(by_value):_jsby_platform_value_list.push_back(by_value);
               }
               //存储
               _jsby_target_value_list.size() > 0 ? _jsby_target_value_map_1[id]=_jsby_target_value_list: _jsby_platform_value_map_1[id]=_jsby_platform_value_list;

             }

         }

     };

     _prcp_client->subscribeTask("addEnemyEquip",dt12);

     // 新版本军事博弈 播放0303
     TPCallback dt13 =  [this]( string_view& si){

         QString schemeInfo = QString::fromStdString(std::string(si));
         std::cout << schemeInfo.toStdString() << std::endl;
         logger.INFO(schemeInfo.toStdString());


         QJsonParseError jsonError;
         QJsonDocument doc = QJsonDocument::fromJson(schemeInfo.toUtf8(), &jsonError);

         QJsonObject sendObj;
         if (jsonError.error == QJsonParseError::NoError && doc.isObject()) {
             // 如果已经是有效的JSON对象，直接使用
             sendObj = doc.object();
         }
         _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/control/2", sendObj);


     };

     _prcp_client->subscribeTask("playControl",dt13);


     // 实时推送战时目标，0305
     TPCallback dt14 =  [this]( string_view& si){

         QString schemeInfo = QString::fromStdString(std::string(si));
         std::cout << schemeInfo.toStdString() << std::endl;
         logger.INFO(schemeInfo.toStdString());


         QJsonParseError jsonError;
         QJsonDocument doc = QJsonDocument::fromJson(schemeInfo.toUtf8(), &jsonError);

         QJsonObject sendObj;
         if (jsonError.error == QJsonParseError::NoError && doc.isObject()) {
             // 如果已经是有效的JSON对象，直接使用
             sendObj = doc.object();
             int forcetype = sendObj["forcetype"].toInt();
             QString id = sendObj["id"].toString();
             QString name = sendObj["name"].toString();
             QJsonArray posarr = sendObj["pos"].toArray();
             MQTTMission::Position pos;
             pos.longitude = posarr[0].toDouble();
             pos.latitude = posarr[1].toDouble();
             pos.altitude = posarr[2].toDouble();
             _mtx.lock();
             APIBridgeSDK::Position target_pos = _jsby_current_pos.target_pos;
             string target_name =_jsby_current_pos.name;
             _mtx.unlock();
             double distance = _geo_algo.RealDistance(pos.latitude,pos.longitude,target_pos.latitude,target_pos.longitude);
             if(distance <100 && (!_jsby_isEnd)){
                 string tip ="我方"+name.toStdString()+"与敌方"+target_name+"进入对抗状态";
                 // _prcp_client->m_rcpClientAdapter->showNotification("博弈状态提示",tip,1,2);
                 jsbyWeightedAverage();
                 _jsby_isEnd = 1;

             }
         }
         //_pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/control/2", sendObj);

     };

     _prcp_client->subscribeTask("pushRedTgtPos",dt14);
}

void C2Manager::setBYType(bool isShowThreatHemisphere,bool r1, bool r2, bool r3, bool r4)
{
    _isShowThreatHemisphere = isShowThreatHemisphere;
    _BY_target_dis_list.clear();
    _BY_platform_dis_list.clear();
   // _BY_target_delete_list.clear();
    _BY_platform_delete_list.clear();
    _r1 = r1;
    _r2 = r2;
    _r3=  r3;
    _r4 = r4;
     _prcp_client->m_rcpClientAdapter->clearAllParticleEffect();
}

void C2Manager::addHeader(QJsonObject &root)
{
    root["SID"] = "ZK";
    int64_t t=  currentTimeMillisec();
    root["TM"] =t;
}

void C2Manager::ReSetMission()
{
    
    _mission_control.reSet(); //重置
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _platform_running_status_by_id.clear();
    }
    string s ="";
    _zg_status.init();//ZG任务初始化
    _recElec534.clear(); //ZG任务电磁信息初始化
    _must_pass_position_list.clear(); //任务必经点清空
    _td_target_position_list.clear(); //投弹目标清空


    //重置任务显示路线
    for(auto uuid : _task_plane_roate_uuid_list){
        _prcp_client->m_rcpClientAdapter->deleteMark(uuid);
    }
    _task_plane_roate_uuid_list.clear();


    //重置包络效果
    for(auto pee :_TaskModelbehavior._phoneElectEnvelop_list){
        _prcp_client->m_rcpClientAdapter->closeEnvelop(pee.model_id);
    }
    _TaskModelbehavior._phoneElectEnvelop_list.clear();

    for(auto pee :_TaskModelbehavior._pARadarData_list){
        _prcp_client->m_rcpClientAdapter->closePARadar(pee.model_id);
    }
    _TaskModelbehavior._pARadarData_list.clear();

    //界面效果控制
    _prcp_client->m_rcpClientAdapter->hideElectWindow();

   _prcp_client->m_rcpClientAdapter->removeIsmiAllZGInfoAsync();
   _prcp_client->m_rcpClientAdapter->removeAllImsiTargetInfoAsync();

   //甘特图重置
     _prcp_client->m_rcpClientAdapter->schemeEndPlay();


    // std::thread t(&C2Manager::schemeStartPlay,this);
    // t.detach();

}

bool C2Manager::CreatorMission(const QJsonObject& json,Mission&  mission)
{
   if(!json.contains("task_id")) return false;
   _mission_control._mission.task_id = json["task_id"].toString().toStdString();
   _mission_control._mission.task_name = json["task_name"].toString().toStdString();
   _mission_control._mission.task_type = json["task_type"].toInt();
   QJsonArray posArr = json["init_position"].toArray();
   _mission_control._mission.init_position.longitude = QJsonValue(posArr.at(0)).toDouble();
   _mission_control._mission.init_position.latitude = QJsonValue(posArr.at(1)).toDouble();
   _mission_control._mission.init_position.altitude = QJsonValue(posArr.at(2)).toDouble();

   return true;

}

bool C2Manager::PackageTaskAndZB(QString &s, QJsonObject &root)
{
    //QString jsonString = "{\"key\":\"value\"}"; // 示例JSON字符串

    QJsonParseError jsonError;
    QJsonDocument _document = QJsonDocument::fromJson(s.toUtf8().data(), &jsonError);
    if (jsonError.error != QJsonParseError::NoError) return false;

    QJsonObject jsonDoc = _document.object();
    QJsonObject taskInfo = jsonDoc["taskInfo"].toObject();
    addHeader(root);
    QJsonObject CTN;
    QJsonArray platform_list = taskInfo["entities"].toArray();
    QJsonObject mission= taskInfo["mission"].toObject();
    QJsonObject task_data = mission["task_data"].toObject();
    QJsonArray graphics = task_data["graphics"].toArray();

    QJsonArray region_list,line_list,point_list;
    for(int i = 0; i <graphics.size(); i++ ){
        QJsonObject obj = graphics[i].toObject();
        if(obj.contains("region")){
            QJsonObject region;
            if(obj.contains("name")) region["name"] = obj["name"].toString();
            if(obj.contains("color")) region["color"] = obj["color"].toString();
             region["point_list"] =  obj["region"].toArray();
            region_list.push_back(region);
        }else if(obj.contains("line")){
              QJsonObject line;
            if(obj.contains("name"))   line["name"] = obj["name"].toString();
            if(obj.contains("color"))  line["color"] = obj["color"].toString();
             line["point_list"] =  obj["line"].toArray();
            line_list.push_back(line);
        }else if(obj.contains("point")){
            QJsonObject point;
            if(obj.contains("name"))  point["name"] = obj["name"].toString();
            if(obj.contains("color")) point["color"] = obj["color"].toString();
            QJsonArray pp;
            QJsonObject p = obj["point"].toObject();
            pp.push_back(p);
            point["point_list"] =  pp;
           // point["point_list"]= obj["point"].toArray();
            point_list.push_back(point);
        }
    }

    QJsonArray buc;
    CTN["battle_unit_code_list"] = buc;

    CTN["platform_list"] =platform_list;


    if(region_list.size() > 0)  CTN["region"] = region_list;
    if(line_list.size() > 0)    CTN["line"]   = line_list;
    if(point_list.size() > 0)   CTN["point"]  = point_list;
    root["CTN"] = CTN;
    return true;

}

bool C2Manager::PackageMission(const QJsonObject &json, QJsonObject &root)
{
    //打包2-1请求协议
    addHeader(root);
    int task_type = json["task_type"].toInt();
    QJsonObject CTN;
    CTN["task_id"] =json["task_id"].toString();
    CTN["task_name"] =json["task_name"].toString();
    CTN["task_type"] =json["task_type"].toInt();


    // if(task_type == 99){
    //     //跟新任务规划方案甘特图
    //     _prcp_client->slotLoadScheme();
    // }

    if(json.contains("task_data")){
        QJsonObject taskdata =  json["task_data"].toObject();
        if(taskdata.contains("graphics")){
              QJsonArray graphics = taskdata["graphics"].toArray();
              std::vector<MQTTMission::Position> region_area; //区域，
              std::vector<MQTTMission::Position> line_list; //线，
              MQTTMission::Position target_point;  //目标点
              bool isfind = false;
            for(auto grap :graphics){
                   QJsonObject obj =   grap.toObject();
                   QString typeStr = obj["typeStr"].toString(); //bjd 必经点, zd终点,tdd投弹点
                   if(obj.contains("region")){
                       QJsonArray pos_arr = obj["region"].toArray();
                       int pointCount = 0;
                       for(QJsonValue pos : pos_arr){
                           if(pointCount >= 4) break; // 限制最多4个点
                           QJsonObject pos1 = pos.toObject();
                           MQTTMission::Position position;
                           position.longitude =  QJsonValue(pos1["longitude"]).toDouble();
                           position.latitude = QJsonValue(pos1["latitude"]).toDouble();
                           position.altitude = QJsonValue(pos1["altitude"]).toDouble();
                           region_area.push_back(position);
                           pointCount++;
                       }
                   }

                   if(obj.contains("line")){
                       QJsonArray pos_arr = obj["line"].toArray();
                       for(QJsonValue pos : pos_arr){
                           QJsonObject pos1 = pos.toObject();
                           MQTTMission::Position position;
                           position.longitude =  QJsonValue(pos1["longitude"]).toDouble();
                           position.latitude = QJsonValue(pos1["latitude"]).toDouble();
                           position.altitude = QJsonValue(pos1["altitude"]).toDouble();
                           line_list.push_back(position);
                       }
                   }

                   if((obj.contains("point")&&typeStr=="zd")) {
                        QJsonObject point = obj["point"].toObject();
                        target_point.longitude =  QJsonValue(point["longitude"]).toDouble();
                        target_point.latitude = QJsonValue(point["latitude"]).toDouble();
                        target_point.altitude = QJsonValue(point["altitude"]).toDouble();
                        isfind = true;
                        //投弹加入目标点序列
                      //  _td_target_position_list.push_back(target_point);
                   }
                   if(obj.contains("point") &&typeStr=="bjd" ){
                        logger.INFO("检测到多个点，使用第一个点作为目标点，目标点坐标：（" + std::to_string(target_point.longitude) + "," + std::to_string(target_point.latitude) + "," + std::to_string(target_point.altitude) + ")");
                        QJsonObject point = obj["point"].toObject();
                        MQTTMission::Position position;
                        position.longitude = QJsonValue(point["longitude"]).toDouble();
                        position.latitude = QJsonValue(point["latitude"]).toDouble();
                        position.altitude = QJsonValue(point["altitude"]).toDouble();
                        _must_pass_position_list.push_back(position);

                   }else if(obj.contains("point") &&typeStr =="tdd"){
                       QJsonObject point = obj["point"].toObject();
                       MQTTMission::Position position;
                       position.longitude = QJsonValue(point["longitude"]).toDouble();
                       position.latitude = QJsonValue(point["latitude"]).toDouble();
                       position.altitude = QJsonValue(point["altitude"]).toDouble();
                       //投弹加入目标点序列
                       _td_target_position_list.push_back(position);
                   }

            }

            if(region_area.size() > 0){
                QJsonArray region_Arr;
                for(MQTTMission::Position position  :region_area){
                    QJsonObject pos;
                    pos["longitude"] = position.longitude;
                    pos["latitude"] = position.latitude;
                    pos["altitude"] = position.altitude;
                    region_Arr.push_back(pos);
                }
                QJsonObject region_zone;
                region_zone["point_list"]=region_Arr;
                CTN["region_zone"] = region_zone;
            }

            if(line_list.size() > 0){
                QJsonArray line_Arr;
                for(MQTTMission::Position position  :line_list){
                    QJsonObject pos;
                    pos["longitude"] = position.longitude;
                    pos["latitude"] = position.latitude;
                    pos["altitude"] = position.altitude;
                    line_Arr.push_back(pos);
                }
                // if(task_type == 100){ //大疆mini4路线
                //     _mini4_pro_path = line_list;
                // }
                QJsonObject line_l;
                line_l["point_list"]=line_Arr;
                CTN["line_list"] = line_l;
            }


            if(isfind){
                QJsonObject tpoint;
                tpoint["longitude"] = target_point.longitude;
                tpoint["latitude"] = target_point.latitude;
                tpoint["altitude"] = target_point.altitude;

                CTN["target_position"] = tpoint;
            }

            if(!_must_pass_position_list.empty()){
                    QJsonArray must_pass_Arr;
                    for(const auto& pos : _must_pass_position_list){
                        QJsonObject position;
                        position["longitude"] = pos.longitude;
                        position["latitude"] = pos.latitude;
                        position["altitude"] = pos.altitude;
                        must_pass_Arr.push_back(position);
                    }
                    CTN["must_pass_position_list"] = must_pass_Arr;
            }

            if(!_td_target_position_list.empty()){
                    QJsonArray pass_Arr;
                    for(const auto& pos : _td_target_position_list){
                        QJsonObject position;
                        position["longitude"] = pos.longitude;
                        position["latitude"] = pos.latitude;
                        position["altitude"] = pos.altitude;
                        pass_Arr.push_back(position);
                    }
                    CTN["target_position_list"] = pass_Arr;
            }
                // 山猫车打击任务先通过这种方式写入
                if(_TaskModelbehavior.isStrikeTargetTask(task_type)){
                   _TaskModelbehavior.setStrikeTarget(target_point);
                }

        }


        QJsonArray buclist;
        if(taskdata.contains("battle_unit_code")){
            buclist.append(taskdata["battle_unit_code"]);
        }
        if(taskdata.contains("battle_unit_code1")){
            buclist.append(taskdata["battle_unit_code1"]);
        }
        if(taskdata.contains("battle_unit_code2")){
            buclist.append(taskdata["battle_unit_code2"]);
        }
        CTN["battle_unit_code_list"] = buclist;

        if(taskdata.contains("sar_info")) {
            QJsonObject sar_info = taskdata["sar_info"].toObject();
            // 验证所有必要的SAR参数是否存在
            QJsonObject sar_strip_param;
            if(sar_info.contains("sar_height")) {
                sar_strip_param["sar_height"] = sar_info["sar_height"];
            }
            if (sar_info.contains("sar_angle")) {
                sar_strip_param["sar_angle"] = sar_info["sar_angle"];
            }
            if (sar_info.contains("sar_heading")) {
                sar_strip_param["sar_heading"] = sar_info["sar_heading"];
            }

            if(sar_info.contains("sar_height") || sar_info.contains("sar_angle") || sar_info.contains("sar_heading")) {
                if (task_type == 220 || task_type == 222) { // Sar条带模式
                    CTN["sar_strip_param"] = sar_strip_param;
                } else if (task_type == 221) { // Sar聚束模式
                    CTN["sar_beaming_param"] = sar_strip_param;
                }
            }
        }

        if(taskdata.contains("dczg_info")) {
            QJsonObject dczg_info = taskdata["dczg_info"].toObject();
            // 验证所有必要的SAR参数是否存在
            QJsonObject dczg_info_param;
            if(dczg_info.contains("dc_height")) {
                dczg_info_param["dc_height"] = dczg_info["dc_height"];
            }
            if (dczg_info.contains("dc_length")) {
                dczg_info_param["dc_length"] = dczg_info["dc_length"];
            }
            if (dczg_info.contains("dc_angle")) {
                dczg_info_param["dc_angle"] = dczg_info["dc_angle"];
            }
            if (dczg_info.contains("dc_distance")) {
                dczg_info_param["dc_distance"] = dczg_info["dc_distance"];
            }

            if(dczg_info.contains("dc_height") 
                || dczg_info.contains("dc_angle") 
                || dczg_info.contains("dc_heading")
                || dczg_info.contains("dc_distance")) {
                if (task_type == 41) { // dc平行路线
                    CTN["dc_parallel_param"] = dczg_info_param;
                } else if (task_type == 42) { // dc夹角路线
                    CTN["dc_angle_param"] = dczg_info_param;
                }
            }
        }

        if(taskdata.contains("dc_operator_params")) { //手机电磁
            _gkd_cfg->dc_operator_params = taskdata["dc_operator_params"].toString().toStdString();
            // if(task_type == 160) {
            //     CTN["dc_operator_params"] = _gkd_cfg->dc_operator_params;
            // }
        }

    }
    root["CTN"] = CTN;
    return true;
}

void C2Manager::CreateorShanmaoMissionPlan(MissionPlan &missionPlan)
{
    missionPlan.task_id =  _mission_control._mission.task_id;

    missionPlan.task_name =_mission_control._mission.task_name;

    missionPlan.task_type =_mission_control._mission.task_type;

    missionPlan.force_num = 1;

    QJsonObject CTN = _root_rwgh["CTN"].toObject();
    QJsonArray execute_unit_list = CTN["battle_unit_code_list"].toArray();
    QString battle_unit_code = execute_unit_list[0].toString();
    missionPlan.battle_unit_code = battle_unit_code.toStdString();
}

void C2Manager::CreateorMissionPlan(const QJsonObject &root,MQTTMission::MissionPlan& missionPlan)
{
    QString SID = root["SID"].toString();

    QString TM = root["TM"].toString();

    QJsonObject CTN = root["CTN"].toObject();

    missionPlan.task_id =  CTN["task_id"].toString().toStdString();

    missionPlan.task_name =CTN["task_name"].toString().toStdString();

    missionPlan.task_type =CTN["task_type"].toInt();

    missionPlan.force_num =  CTN["force_num"].toInt();

    missionPlan.battle_unit_code = CTN["battle_unit_code"].toString().toStdString();
    if(CTN.contains("result_code")){
        string r =  CTN["result_code"].toString().toStdString();
        missionPlan.result_code = r;
    }

    _mission_control._tforce_num =  missionPlan.force_num;

    if(CTN.contains("route")){
        QJsonArray route_arr =  CTN["route"].toArray();
        for (int i = 0; i < route_arr.size(); i++) {
            QJsonObject pos = route_arr[i].toObject();
            MQTTMission::Position position(pos["longitude"].toDouble(),pos["latitude"].toDouble(),pos["altitude"].toDouble(),pos["speed"].toDouble());

            QJsonArray wpl = pos["action_list"].toArray();
            position.action_list = wpl;
            missionPlan.route.push_back(position);
        }
    }

    if(CTN.contains("target_ID"))  {
        missionPlan.target_ID  = CTN["target_ID"].toInt();
    }

    missionPlan.start_time = CTN["start_time"].toInt();

    missionPlan.end_time = CTN["end_time"].toInt();

    missionPlan.auto_level  = CTN["auto_level"].toInt();

    missionPlan.priority  = CTN["priority"].toInt();
}



void C2Manager::PackageMissionPlan_ToCK(const QJsonObject &json,QJsonObject& root)
{

    addHeader(root);
    QJsonObject CTN;
    QJsonObject task_basic_info;
    QJsonObject taskdata =  json["task_data"].toObject();
    task_basic_info["task_id"] = QString::fromStdString(_mission_control._mission.task_id) ;
    task_basic_info["task_name"] = QString::fromStdString(_mission_control._mission.task_name);
    QJsonArray task_type_list;
    if (_mission_control._mission.task_type == 160) {
        task_type_list.append(16);
    } else if(_TaskModelbehavior.isElecSignFly(_mission_control._mission.task_type)){ //光电单机
     task_type_list.append(12);
    }else {
        task_type_list.append(_mission_control._mission.task_type);
    }
    // 用于合同指标验证
    // if (_mission_control._mission.task_type == 20251111) {
    //     // 读取landship任务数据文件（使用程序目录的绝对路径）
    //     QString appDir = QCoreApplication::applicationDirPath();
    //     QString filePath = appDir + "/landship_task_data.json";
    //     logger.INFO("尝试读取文件: " + filePath.toStdString());

    //     QFile landshipFile(filePath);
    //     if (landshipFile.open(QIODevice::ReadOnly)) {
    //         QByteArray jsonData = landshipFile.readAll();
    //         landshipFile.close();

    //         QJsonParseError parseError;
    //         QJsonDocument landshipDoc = QJsonDocument::fromJson(jsonData, &parseError);

    //         if (parseError.error == QJsonParseError::NoError && landshipDoc.isObject()) {
    //             QJsonObject landshipRoot = landshipDoc.object();

    //             // 发送到/llm/jsim/excute topic
    //             _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/excute", landshipRoot);

    //             logger.INFO("已下发任务到任务平台");
    //             // _prcp_client->m_rcpClientAdapter->showNotification("任务下发", "合同指标验证任务已下发", 0, 3);
    //         } else {
    //             logger.WARNING("解析landship_task_data.json失败: " + parseError.errorString().toStdString());
    //         }
    //     } else {
    //         logger.WARNING("无法打开landship_task_data.json文件");
    //     }
    // }

    task_basic_info["task_type_list"] = task_type_list;
    task_basic_info["start_time"] =12345 ;
    task_basic_info["end_time"] =12345 ;
    task_basic_info["edit_time"] =12345 ;
    if (_mission_control._mission.task_type == 220 
        || _mission_control._mission.task_type == 221
        || _mission_control._mission.task_type == 222) {
        QJsonObject sar_info = taskdata["sar_info"].toObject();
        QString sar_height = sar_info["sar_height"].toString();
        QString sar_angle = sar_info["sar_angle"].toString();
        QString sar_heading = sar_info["sar_heading"].toString();

        task_basic_info["description"] = "高度：" + sar_height + "，角度: " + sar_angle + "，航向: " + sar_heading;
    } 
    if (_mission_control._mission.task_type == 20) {     
            QJsonObject rootCTN = _root_rwgh["CTN"].toObject();
            QJsonObject targetPos = rootCTN["target_position"].toObject();

            // 创建target对象，与ZK/101消息格式一致
            QJsonObject target;
            target["target_input_type"] = 3;
            
            QJsonObject target_position;
            QJsonObject position;
            position["longitude"] = targetPos["longitude"].toDouble();
            position["latitude"] = targetPos["latitude"].toDouble();
            position["altitude"] = targetPos["altitude"].toDouble();
            
            target_position["position"] = position;
            target["target_position"] = target_position;
            
            CTN["target"] = target;
            
            QString longitude = QString::number(targetPos["longitude"].toDouble(), 'f', 6);
            QString latitude = QString::number(targetPos["latitude"].toDouble(), 'f', 6);
            QString altitude = QString::number(targetPos["altitude"].toDouble(), 'f', 2);
            task_basic_info["description"] = "经度：" + longitude + "，纬度: " + latitude + "，高度: " + altitude;    
    }
    else {
       task_basic_info["description"] = QString::fromStdString("_mission_control._mission.description") ; 
    }

    

    
    task_basic_info["task_aim"] =QString::fromStdString(_mission_control._mission.task_name) ;
    task_basic_info["task_terminal_condition"] ="task_terminal_condition" ;
    CTN["task_basic_info"] =task_basic_info;

    if(_TaskModelbehavior.isStrikeTargetTask(_mission_control._mission.task_type)){
        QJsonObject target;

        target["target_input_type"] = 3;

        QJsonObject target_position;

        QJsonObject position;
        MQTTMission::Position tp;
        _TaskModelbehavior.getStrikeTarget(tp);
        position["longitude"] =tp.longitude;
        position["latitude"] =tp.latitude;
        position["altitude"] =tp.altitude;

        target_position["position"] = position;
        target["target_position"] = target_position;

        CTN["target"] = target;

    }

    QJsonArray execute_unit_list;
    for(MissionPlan mp: _mission_control._missionPlanList){
        execute_unit_list.append(QString::fromStdString(mp.battle_unit_code));
    }
    // 当电磁从无人机手飞时走这个临时分支
    if(_mission_control._mission.task_type == 160){
        // execute_unit_list.append("dy11");
        execute_unit_list.append("dy12");
    }
    CTN["execute_unit_list"] =execute_unit_list;

    root["CTN"] = CTN;
}

void C2Manager::PackageMissionBattle_ToCK(const QJsonObject& json ,const MissionPlan& missionplan,QJsonObject& root)
{
    addHeader(root);
    QJsonObject CTN;
    CTN["battle_unit_no"] =QString::fromStdString(missionplan.battle_unit_code);
    CTN["task_id"] =QString::fromStdString(missionplan.task_id);

    QJsonObject  planned_track;
    planned_track["trackline_name"] = QString::fromStdString(missionplan.task_name+"航线");
    planned_track["start_time"] =12345;
    planned_track["end_time"] =123456;
        QJsonArray track_point_list;
    for(MQTTMission::Position pos :missionplan.route){
                QJsonObject  obj;
                obj["current_time"]=12345;

                // 根据battle_unit_no和task_type查找对应的设备速度
                QString deviceId = QString::fromStdString(missionplan.battle_unit_code);
                int velocity = 0; // 默认速度

                // 检查 _gkd_cfg 是否为空指针
                if (_gkd_cfg != nullptr) {
                    velocity = _gkd_cfg->getDeviceTaskVelocity(deviceId, missionplan.task_type);
                } else {
                    // 如果配置为空，记录警告并使用默认速度
                    logger.WARNING("GKD配置为空，使用默认速度0");
                }

                obj["velocity"]=velocity;

                QJsonObject  track_point_position;
                track_point_position["longitude"] = pos.longitude;
                track_point_position["latitude"] = pos.latitude;
                track_point_position["altitude"] = pos.altitude;
                track_point_position["speed"] = pos.speed;
                track_point_position["action_list"] = pos.action_list;
                obj["track_point_position"] = track_point_position;
                track_point_list.append(obj);
        }
    planned_track["track_point_list"] =track_point_list;
    planned_track["min_height"] =12345;
    planned_track["max_height"] =12345;
    planned_track["input_type"] =1;
    planned_track["road_width"] =12345;
    planned_track["edit_time"] = QString::fromStdString(currentTime());
    planned_track["description"] = QString::fromStdString(missionplan.task_name+"航线");

    if(missionplan.route.size() > 0){
        MQTTMission::Position init_pos = missionplan.route[0];
        QJsonObject init_position;
        init_position["longitude"] = init_pos.longitude;
        init_position["latitude"] = init_pos.latitude;
        init_position["altitude"] = init_pos.altitude;
        planned_track["init_position"] =init_position;
        planned_track["end_position"] = init_position;

    }


    CTN["planned_track"] =planned_track;

    // 投弹任务
    QJsonObject strike_task_params;
    strike_task_params["strike_time"] =currentTimeMillisec();
    strike_task_params["bombing_number"] =2; //TODO：需要支持0，1，2

    CTN["strike_task_params"] =strike_task_params;

    if(_mission_control._mission.task_type == 160) {
        CTN["dc_operator_params"] =QString::fromStdString(_gkd_cfg->dc_operator_params).toInt();
        // QJsonObject root_CTN = _root_rwgh["CTN"].toObject();
        // if(root_CTN.contains("dc_operator_params")) {
        //     CTN["dc_operator_params"] = root_CTN["dc_operator_params"];
        // }
    }

    root["CTN"] = CTN;
}

// 电磁临机干预任务打包
void C2Manager::packageImmediateIntervention(const MQTTMission::MissionPlan missionPlan,int task_set_flag, QJsonObject& root)
{
    ImmediateIntervention immediateIntervention;
    immediateIntervention.task_id = _mission_control._mission.task_id;
    immediateIntervention.task_set_flag =task_set_flag;
    immediateIntervention.battle_unit_no = missionPlan.battle_unit_code;

    immediateIntervention.target_ID = "1212";  //固定的target_ID使用？？？？？？
    TrackLine trackLine;
    trackLine.trackline_name = "";
    trackLine.start_time = 0;
    for(MQTTMission::Position pos :missionPlan.route){
        TrackPoint tp;
        tp.current_time = 0;
        tp.velocity = 0;
        tp.track_point_position = pos;
        trackLine.track_point_list.push_back(tp);
    }


    trackLine.min_height = 0;

    trackLine.max_height =0;

    trackLine.input_type = 1;
    trackLine.road_width = 0;

    trackLine.edit_time = 0;
    trackLine.description = "";
    // trackLine.init_position =Position(0,0,0);
    // trackLine.end_position =Position(0,0,0);
    immediateIntervention.planned_track = trackLine;

    addHeader(root);
    QJsonObject imp;
    imp.insert("task_id",QString::fromStdString(immediateIntervention.task_id));
    imp["task_set_flag"] = task_set_flag;

    imp.insert("battle_unit_no",QString::fromStdString(immediateIntervention.battle_unit_no));



    QJsonObject planned_track;
    planned_track["start_time"] = currentTimeMillisec();
    planned_track["end_time"] = currentTimeMillisec();
    planned_track["trackline_name"] ="电磁航线";

    QJsonArray track_point_list;

    for(TrackPoint tp:immediateIntervention.planned_track.track_point_list){
        QJsonObject plan_point;
        plan_point["current_time"] = 0;
        QJsonObject pos;
        pos["longitude"] =QJsonValue(tp.track_point_position.longitude).toDouble();
        pos["latitude"] =QJsonValue(tp.track_point_position.latitude).toDouble();
        pos["altitude"] =QJsonValue(tp.track_point_position.altitude).toDouble();

        plan_point["track_point_position"] =pos;
        plan_point["velocity"] = 0.0;
        track_point_list.push_back(plan_point);
    }
    planned_track["track_point_list"]=track_point_list;  //传Z字形路线
    planned_track["min_height"]=0.0;
    planned_track["max_height"]=0.0;
    planned_track["input_type"]=1;
    planned_track["road_width"]=0;
    planned_track["edit_time"]=0;

    imp["planned_track"] =planned_track;
    PhoneSignalTarget elec_signal_target;  //传识别目标
    elec_signal_target.plmn = "";
    elec_signal_target.imsi = _imsi;
    if(task_set_flag == 5){
        QJsonObject  est;
        est.insert("plmn",QString::fromStdString(""));
        est.insert("imsi",QString::fromStdString(_imsi));
        imp["elec_signal_target"] = est;
    }else if(task_set_flag == 6){
        QJsonObject  est;
        est.insert("plmn",QString::fromStdString(""));
        est.insert("imsi",QString::fromStdString(_imsi));

        QJsonObject pos;
        pos["longitude"] =QJsonValue(0).toDouble();
        pos["latitude"] =QJsonValue(0).toDouble();
        pos["altitude"] =QJsonValue(0).toDouble();
        est["position"] = pos;
        imp["elec_exact_target"] = est;
    }

    //电磁ZG递进干扰阶段标识
    imp["emi_stage"] = _zg_status.rec_interference_global_stage;
    if(immediateIntervention.target_ID != ""){
        imp["target_ID"] = QString::fromStdString(immediateIntervention.target_ID); //空地协同提供跟踪打击编号，上面先固定了;
    }
    root["CTN"] =imp;


}

void C2Manager::Publish_101(QJsonObject root)
{

}



void C2Manager::Publish_102()
{

}



void C2Manager::Subscribe_rwgh_901(const QJsonObject &root)
{

    logger.INFO("......任务："+_mission_control._mission.task_name+" 收到任务规划结果");

    QJsonObject CTN = root["CTN"].toObject();

    QString task_id =  CTN["task_id"].toString();

    if(_mission_control._mission.task_id != task_id.toStdString() ) return;

    MQTTMission::MissionPlan missionPlan;
    //打包任务规划结果
    CreateorMissionPlan(root,missionPlan);
    // 转换数据类型用于RCP绘制,绘制规划路线
    CreateorTrackLine(missionPlan.route);

    if(missionPlan.result_code =="1"){
        string resut_code = "....电磁侦干空地协同任务规划条件不满足，规划终结,任务终结";
        dczg_result(resut_code);
        return;
    }else if(missionPlan.result_code == "2"){
        string str ="电磁移动从无人机规划条件不满足";
        dcyd_result(str);
        return;
    }

    if(_TaskModelbehavior.isImmediateIntTask(missionPlan.task_type)){ //临机干预时形成的任务规划结果，目前电磁ZG、电子手机移动使用

        _mission_control._missionImmediateIntPlaneList.push_back(missionPlan);



        if( missionPlan.task_type == 161){ //手机电磁Z字形路线后，指控下发临机干预

            //TrackLine  trackLine;

            QJsonObject root105;
            packageImmediateIntervention(missionPlan,5,root105);

            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root105);
            logger.INFO("......临机干预：任务编号"+std::to_string(missionPlan.task_type)+",作战单元"+missionPlan.battle_unit_code+"干预任务下发");
        }else if(missionPlan.task_type == 162){ //手机电磁回字形路线后，指控下发临机干预
            QJsonObject root105;
            packageImmediateIntervention(missionPlan,6,root105);


            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root105);
            logger.INFO("......临机干预：任务编号"+std::to_string(missionPlan.task_type)+",作战单元"+missionPlan.battle_unit_code+"干预任务下发");

        }else if( missionPlan.task_type == 42){
            QJsonObject root105;
            packageImmediateIntervention(missionPlan,9,root105); //夹角规划

            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root105);

            logger.INFO("......临机干预：任类型务编号"+std::to_string(missionPlan.task_type)+",作战单元"+missionPlan.battle_unit_code+"干预任务下发");

        } else if (missionPlan.task_type == 43 ||
                   missionPlan.task_type == 44 ||
                   missionPlan.task_type == 45){ //手机电磁回字形路线后，指控下发临机干预
            QJsonObject root105;
            packageImmediateIntervention(missionPlan,10,root105); //ZG 干扰阶段

            if (missionPlan.task_type == 43) {
                if (!missionPlan.route.empty()) {
                    _zg_status.zg_master_uav_pos = missionPlan.route.back();
                }
            }
            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root105);
            logger.INFO("......临机干预：任务编号"+std::to_string(missionPlan.task_type)+",作战单元"+missionPlan.battle_unit_code+"干预任务下发");
        }else if(missionPlan.task_type == 12){
            QJsonObject root106;
              packageImmediateIntervention(missionPlan,11,root106); //空地协同目标跟踪打击
            _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root106);
            logger.INFO("......临机干预空地协同打击：任务编号"+std::to_string(missionPlan.task_type)+",作战单元"+missionPlan.battle_unit_code+"干预任务下发");
        }
    }else{

        _mission_control._missionPlanList.push_back(missionPlan);

        logger.INFO("......任务："+_mission_control._mission.task_id+"总共有" +std::to_string( missionPlan.force_num)+"个作战单元，已接收作战单元"
                    +missionPlan.battle_unit_code);
        if(_mission_control.hasReceiveAllForce()){
            logger.INFO("......任务："+_mission_control._mission.task_id+"作战单元接收完毕");
        }

    }

}

void C2Manager::Subscribe_llm_excute(const QJsonObject &root)
{


}

void C2Manager::Subscribe_bltj_905(const QJsonObject &root)
{
    logger.INFO("......任务："+_mission_control._mission.task_name+" 收到任务规划结果");

    QJsonObject CTN = root["CTN"].toObject();

    QString task_id =  CTN["task_id"].toString();
    _dtControl.CreateorGHBltjList(root);
    //更新导兵力推荐集界面，并绘制
    std::map<string, std::vector<MissionPlan> > missionPlaneSet;
    _dtControl.getBltjList(missionPlaneSet);
    if(missionPlaneSet.size() == 0 ){
        _prcp_client->m_rcpClientAdapter->showNotification("gy提示","没有获取到兵力推荐集",1,2);
        return;
    }


    std::list<APIBridgeSDK::ForceRecommondInfo> infolist;
    int c_g = 150;
    for (auto rit = missionPlaneSet.rbegin(); rit != missionPlaneSet.rend(); ++rit) {
        std::string program_name = rit->first;
        std::vector<MissionPlan> missionPlane_list = rit->second;

        APIBridgeSDK::ForceRecommondInfo info;
        info.schemeName = program_name;
        info.displaColor.push_back(0);
        if(c_g > 255) c_g = 150;
        info.displaColor.push_back(c_g);
        info.displaColor.push_back(c_g-50);
        c_g = c_g +50;
        info.displaColor.push_back(255);
        for(MissionPlan& mp: missionPlane_list){
            APIBridgeSDK::ForceEntityInfo forceEntity;
            forceEntity.name = mp.battle_unit_name;
            forceEntity.unitId = mp.battle_unit_code;
            forceEntity.startTime ="既定时间";
            forceEntity.endTime ="既定时间";
            forceEntity.taskType = std::to_string(mp.task_type);

            for(auto pos:mp.route){
                APIBridgeSDK::Position position;
                position.longitude = pos.longitude;
                position.latitude = pos.latitude;
                position.altitude = pos.altitude;
                forceEntity.linePts.push_back(position);
            }
            info.entities.push_back(forceEntity);

        }
        infolist.push_back(info);

    }
    _prcp_client->m_rcpClientAdapter->clearForceRecommondInfo();
    _prcp_client->m_rcpClientAdapter->addForceRecommondInfo(infolist);

}

void C2Manager::Subscribe_dzmbqb_301(const QJsonObject &root)
{
    // 解析电子目标情报数据
    Targetlatform targp;
    if(!parseTargetlatformFromJson(root,targp)){
        logger.INFO( ".......Failed to parse equipment status data from 301 message");
        return;
    }
        string model_id = targp.basic.model_id;
    //已经删除的这里不在出现
    // auto it =  std::find_if(_BY_target_delete_list.begin(),_BY_target_delete_list.end(),[&](const std::string r){
    //     return   r == model_id;
    // });
    // if(it !=_BY_target_delete_list.end() ) return;

    //     //添加目标信息
    //     auto it2 =  std::find_if(_BY_target_dis_list.begin(),_BY_target_dis_list.end(),[&](const std::string r){
    //         return   r == model_id;
    //     });
    //     if(it2 ==_BY_target_dis_list.end() ) {
    //       _BY_target_dis_list.push_back(model_id);
    //     }

    //if(model_id != "person-1") return; // test by hqw
    if(_mission_control.hasTargetlatform(model_id)){
        //这里判断距离差2米内不需要更新，时间超过 2秒则更新

        _mission_control.removeTargetlatform(model_id);
        _mission_control.addTargetlatform(targp);
        // 如果还在新增列表中，则删除；
        auto it  = std::find(_mission_control._new_targetPlatform_list.begin(),_mission_control._new_targetPlatform_list.end(),model_id);
        if(it != _mission_control._new_targetPlatform_list.end()){
            _mission_control._new_targetPlatform_list.erase(it);
        }
    }else{
        _mission_control.addTargetlatform(targp);
        _mission_control._new_targetPlatform_list.push_back(model_id); // 记录新增的model
    }
    _mtx.lock();
    _jsby_current_pos.name = targp.basic.model_name;
    _jsby_current_pos.target_pos.longitude = targp.basic.deploy_position.longitude;
    _jsby_current_pos.target_pos.latitude = targp.basic.deploy_position.latitude;
    _jsby_current_pos.target_pos.altitude = targp.basic.deploy_position.altitude;
    _mtx.unlock();
    // APIBridgeSDK::KeyEventDataSDK ke;
    // ke.time = currentTimeMillisec();
    // ke.type = RPC_KeyEventType::keyEventTypeCritical;
    // ke.title = "击中目标";
    // ke.describe = "敌方目标被击中";
    // ke.id = QUuid::createUuid().toString().toStdString();
    // _prcp_client->m_rcpClientAdapter->addKeyEvent(ke);

    // if(_isShowThreatHemisphere){

    //     Targetlatform tarp;
    //     bool isfind = _mission_control.getTargetlatform(model_id,tarp);
    //     if(isfind){
    //         removeThreatHemisphere(model_id);
    //         showThreatHemisphere_301(model_id);
    //     }

    // }

    //<gy> ,后期根据频次 控制发出频次
   _prcp_client->m_rcpClientAdapter->createOrUpdateTargetPlatformData(_mission_control._targetPlatform_list);

   //间断性全量更新目标信息
   QJsonObject target_root;
   addHeader(target_root);
   QJsonObject CTN;
   QJsonArray target_info;
   for(int i = 0; i < _mission_control._targetPlatform_list.size(); i ++){

       APIBridgeSDK::Targetlatform targetplatform= _mission_control._targetPlatform_list[i];
       QJsonObject target;
       target["target_ID"] = QString::fromStdString(targetplatform.basic.model_id);
       target["target_name"] = QString::fromStdString(targetplatform.basic.model_name);
       target["battle_unit_code"] = QString::number(targetplatform.business.collect_platform_id);
       QJsonObject position;
       position["longitude"] =QJsonValue(targetplatform.basic.deploy_position.longitude).toDouble();
       position["latitude"] =QJsonValue(targetplatform.basic.deploy_position.latitude).toDouble();
       position["altitude"] =QJsonValue(targetplatform.basic.deploy_position.altitude).toDouble();

       target["track_info"] =position;
       target["target_plat_type"] =targetplatform.basic.platform_type;
       target["task_id"] ="";
       target_info.push_back(target);
   }
   CTN["target_info"] = target_info;
   target_root["CTN"] = CTN;
   _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/status/target_info", target_root);

}

void C2Manager::Subscribe_dzmbxsb_302(const QJsonObject &root)
{

    // 目前可调用接口直接删除
    QJsonObject CTN =  root["CTN"].toObject();
    QString target_ID =  CTN["target_ID"].toString();
    if(target_ID == "") return;
}

void C2Manager::Subscribe_dzmbxsb_304(const QJsonObject &root)
{
    // 目前可调用接口直接删除
    QJsonObject CTN =  root["CTN"].toObject();
    QString model_ID =  CTN["model_ID"].toString();
    if(model_ID == "") return;

    if(_isShowThreatHemisphere){ //删除装备形成的威胁半径
        //_BY_ownplatform_dis_list.push_back(model_ID.toStdString());
      //  removeThreatHemisphere(model_ID.toStdString());
    }

}

void C2Manager::Subscribe_zzptjzbzt_303(const QJsonObject &root)
{
    // 解析作战平台及装备状态数据
    const bool processCombat = !is_stop_attack_equip;
    OwnPlatform equip;
    if(!parseOwnPlatformFromJson(root,equip)){
        logger.INFO( "......Failed to parse equipment status data from 303 message");
        return;
    }else{
       // logger.INFO("装备:"+equip.basic.model_id +"经度:"+doubleToString(equip.basic.deploy_position.longitude)+"维度："+doubleToString(equip.basic.deploy_position.latitude));
    }
    QJsonObject ctn = root["CTN"].toObject();
    QString task_type = ctn["task_type"].toString(); //1 为侦查，2为打击
    QString running_status =  ctn["running_status"].toString(); //启动后均是working ，侦查阶段最后一个点为idel
    string model_id = equip.basic.model_id;

    //已经删除的这里不在出现
    auto it =  std::find_if(_BY_platform_delete_list.begin(),_BY_platform_delete_list.end(),[&](const std::string r){
        return   r == model_id;
    });
    if(it !=_BY_platform_delete_list.end() ) return;

    if (model_id.empty()) return;
    if (running_status == "idle" || running_status == "working") {
        std::lock_guard<std::mutex> lock(_mtx);
        _platform_running_status_by_id[model_id] = running_status;
    }

    auto it2 =  std::find_if(_BY_platform_dis_list.begin(),_BY_platform_dis_list.end(),[&](const std::string r){
        return   r == model_id;
    });
    if(processCombat && it2 ==_BY_platform_dis_list.end() ) {
        _BY_platform_dis_list.push_back(model_id);
    }
    /**
   ** 现场信息 无人机一秒 飞行距离10m左右， 无人机经纬度更新频率5-20次作用；
**/
   if(_mission_control.hasOwnPlatform(model_id)){
        //这里判断距离差2米内不需要更新，时间超过 2秒则更新
       OwnPlatform old_equip;
       _mission_control.getOwnPlatform(model_id,old_equip);
      _mission_control.removeOwnPlatform(model_id);
      _mission_control.addOwnPlatform(equip);
       // 如果还在新增列表中，则删除；
      auto it  = std::find(_mission_control._new_ownPlatform_list.begin(),_mission_control._new_ownPlatform_list.end(),model_id);
       if(it != _mission_control._new_ownPlatform_list.end()){
          _mission_control._new_ownPlatform_list.erase(it);
      }
   }else{
       _mission_control.addOwnPlatform(equip);
       _mission_control._new_ownPlatform_list.push_back(model_id); // 记录新增的model
   }
   if (processCombat) ++sn;
   std::vector<string> target_list;
   std::vector<string> platfor_list;
  // GetConnTargetAndPlatform(target_list,platfor_list);

   //军事博弈规则
   if(processCombat && _isShowThreatHemisphere){
       if(_r1 && _r3){ //红蓝均激烈对抗
           for(string target:target_list ){
               Targetlatform tarp;
               if(_mission_control.getTargetlatform(target,tarp)){
                   if(isSHow())   SHowTeXiao(target,tarp.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude);
               }
               _mission_control.removeTargetlatform(target);
               removeThreatHemisphere(target);
           }

           for(string target:platfor_list ){
               OwnPlatform ownP;
               if(_mission_control.getOwnPlatform(target,ownP)){
                   SHowTeXiao(target,ownP.basic.deploy_position.longitude,ownP.basic.deploy_position.latitude);
               }
               _mission_control.removeOwnPlatform(target);
               {
                   std::lock_guard<std::mutex> lock(_mtx);
                   _platform_running_status_by_id.erase(target);
               }
               removeThreatHemisphere(target);
           }
       }else if(_r1 && _r4){//红方激烈对抗，蓝方克制
           for(string target:platfor_list ){
               OwnPlatform ownP;
               if(_mission_control.getOwnPlatform(target,ownP)){
                    SHowTeXiao(target,ownP.basic.deploy_position.longitude,ownP.basic.deploy_position.latitude);
               }
               _mission_control.removeOwnPlatform(target);
               {
                   std::lock_guard<std::mutex> lock(_mtx);
                   _platform_running_status_by_id.erase(target);
               }
               removeThreatHemisphere(target);
           }
           if(sn%2 == 0){
               for(string target:target_list ){
                   Targetlatform tarp;
                   if(_mission_control.getTargetlatform(target,tarp)){
                       SHowTeXiao(target,tarp.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude);
                   }
                   _mission_control.removeTargetlatform(target);
                   removeThreatHemisphere(target);
               }
           }
       }else if(_r2&& _r3){ //红方克制，蓝方激烈对抗

            if(sn%2 == 0){
                for(string target:platfor_list ){
                    OwnPlatform ownP;
                    if(_mission_control.getOwnPlatform(target,ownP)){
                        SHowTeXiao(target,ownP.basic.deploy_position.longitude,ownP.basic.deploy_position.latitude);
                    }

                    _mission_control.removeOwnPlatform(target);
                    {
                        std::lock_guard<std::mutex> lock(_mtx);
                        _platform_running_status_by_id.erase(target);
                    }
                    removeThreatHemisphere(target);
                }
             }

            for(string target:target_list ){
                 Targetlatform tarp;
                 if(_mission_control.getTargetlatform(target,tarp)){
                     SHowTeXiao(target,tarp.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude);
                 }
                   _mission_control.removeTargetlatform(target);
                removeThreatHemisphere(target);
            }


       }else if(_r2 && _r4){ //红方克制，蓝方克制
           if(sn%2 == 0){
               for(string target:platfor_list ){
                   OwnPlatform ownP;
                   if(_mission_control.getOwnPlatform(model_id,ownP)){
                       if(!isf){
                           SHowTeXiao(target,ownP.basic.deploy_position.longitude,ownP.basic.deploy_position.latitude);
                           isf = true;
                       }
                   }
                   _mission_control.removeOwnPlatform(target);
                   {
                       std::lock_guard<std::mutex> lock(_mtx);
                       _platform_running_status_by_id.erase(target);
                   }
                   removeThreatHemisphere(target);
               }
               for(string target:target_list ){
                   Targetlatform tarp;
                   if(_mission_control.getTargetlatform(model_id,tarp)){
                        if(!isf){
                       SHowTeXiao(target,tarp.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude);
                            isf = true;
                        }
                    }
                   _mission_control.removeTargetlatform(target);
                   removeThreatHemisphere(target);
               }

           }
       }
   }
   if(processCombat && _isShowThreatHemisphere){

       OwnPlatform ownP;
      bool isfind = _mission_control.getOwnPlatform(model_id,ownP);
       if(isfind){
          removeThreatHemisphere(model_id);
          showThreatHemisphere_303(model_id);
      }

   }

   // if(_isShowThreatHemisphere){ //如果已经清除了，则从已有缓存删除，并且从显示场景中删除
   //     auto it =  std::find_if(_BY_ownplatform_dis_list.begin(),_BY_ownplatform_dis_list.end(),[&](const std::string r){
   //         return   r == model_id;
   //     });
   //     if(it !=_BY_ownplatform_dis_list.end() ) {
   //          removeThreatHemisphere(model_id);
   //         _mission_control.removeOwnPlatform(model_id);
   //          return;
   //     }
   // }
   //<gy>
    _prcp_client->m_rcpClientAdapter->createOrUpdateOwnPlatformData(_mission_control._ownPlatform_list);

    //find_taeget_id 也是打击目标
   if(processCombat && task_type == "1"){ //侦查阶段
       if(!is_find_target){
            QString s;
           is_find_target =  isIdentifyTarget(equip,task_type,s);
           if(is_find_target){
               task_execute_status = 2;
               find_equip_id = equip.basic.model_id;
            //   publishImageTopic();
           }

       }

   }else if(processCombat && task_type == "2"){ //打击阶段
       if(!is_attack_target){
           QString s;
           is_attack_target =  isIdentifyTarget(equip,task_type,s);
           if(is_attack_target){
               task_execute_status = 3;
               attack_equip_id = equip.basic.model_id;

               _TaskModelbehavior._particleEffect2.particleId = "33";
               _TaskModelbehavior._particleEffect2.position.longitude =equip.basic.deploy_position.longitude;
               _TaskModelbehavior._particleEffect2.position.latitude =equip.basic.deploy_position.latitude;
               _TaskModelbehavior._particleEffect2.position.altitude =2;
               _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect2);
           }

       }else{

           //对应位置特效
           if(++attack_index < 10){
               std::string pi = QString::number(particleId_index++).toStdString();
              particleId_set.push_back(pi);
               _TaskModelbehavior._particleEffect2.particleId = pi;
               _TaskModelbehavior._particleEffect2.position.longitude =equip.basic.deploy_position.longitude;
               _TaskModelbehavior._particleEffect2.position.latitude =equip.basic.deploy_position.latitude;
               _TaskModelbehavior._particleEffect2.position.altitude =2;
               _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect2);
           }else{
               is_stop_attack_equip = true;
               for(auto pi: particleId_set){
                                      _prcp_client->m_rcpClientAdapter->removeParticleEffect(pi);
               }
               particleId_set.clear();
               _prcp_client->m_rcpClientAdapter->clearAllParticleEffect();
           }
       }
   }
    //间断性全量更新目标信息
    QJsonObject platform_root;
    addHeader(platform_root);
    QJsonObject CTN;
    QJsonArray platform_info;
    for(int i = 0; i < _mission_control._ownPlatform_list.size(); i ++){
       APIBridgeSDK::OwnPlatform ownplatform= _mission_control._ownPlatform_list[i];
       QJsonObject platform;
       platform["task_execute_status"] = 0;
       QString platform_running_status = QStringLiteral("idle");
       {
           std::lock_guard<std::mutex> lock(_mtx);
           if (is_stop_attack_equip && is_attack_target &&
               ownplatform.basic.model_id == attack_equip_id) {
               _platform_running_status_by_id[ownplatform.basic.model_id] = QStringLiteral("idle");
           }
           const auto status = _platform_running_status_by_id.find(ownplatform.basic.model_id);
           if (status != _platform_running_status_by_id.end()) {
               platform_running_status = status->second;
           }
       }

       //将对应装备对应阶段的 目标更新上去，其他装备不做变化
       if(ownplatform.basic.model_id ==find_equip_id && is_find_target ){
               platform["task_execute_status"] = task_execute_status;
               platform["target_ID"] = find_target_id;

       }else if(ownplatform.basic.model_id ==attack_equip_id &&is_attack_target ){

               platform["task_execute_status"] = task_execute_status;
               platform["target_ID"] = find_target_id;


       }
        platform["battle_unit_code"] = QString::fromStdString(ownplatform.basic.model_id);
        platform["battle_unit_name"] = QString::fromStdString(ownplatform.basic.model_name);
        platform["platform_type"] = ownplatform.basic.platform_type;
        QJsonObject position;
        position["longitude"] =QJsonValue(ownplatform.basic.deploy_position.longitude).toDouble();
        position["latitude"] =QJsonValue(ownplatform.basic.deploy_position.latitude).toDouble();
        position["altitude"] =QJsonValue(ownplatform.basic.deploy_position.altitude).toDouble();

        platform["deploy_position"] =position;
        platform["velocity"] =5;
        platform["trk_direction"] = 180.0;
        platform["health_status"] =0;
        platform["running_status"] = platform_running_status;


        platform_info.push_back(platform);
    }
    CTN["platform_info"] = platform_info;
    platform_root["CTN"] = CTN;
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/status/platform_info", platform_root);
  //  s303Index = 0;
    if(processCombat && ++effectIndex > 2000){
        for(APIBridgeSDK::Position pos :_TaskModelbehavior._pE3_list){
            _TaskModelbehavior._particleEffect3.position = pos;
            _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect3);
        }
        effectIndex = 0;
    }

}

void C2Manager::Subscribe_zzptjzbzt_308(const QJsonObject &root)
{
    //重置，从侦查开始
    if(root.contains("reset")){
        is_find_target = false;
        is_attack_target = false;
        is_stop_attack_equip = false;
        attack_index = 0;
        particleId_index = 100;
        is_publish_targetImg = false;
        particleId_set.clear();
        _prcp_client->m_rcpClientAdapter->clearAllParticleEffect();
    }
}

void C2Manager::Subscribe_rwzxzt_501(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    string task_id = CTN["task_id"].toString().toStdString();
    _mission_control._missionStatus.task_status = CTN["task_status"].toInt();
    int task_status = _mission_control._missionStatus.task_status;
    if(task_status == 0){
        logger.INFO("任务未开始。。。。。。。。。。");
    }else if(task_status == 1){
        logger.INFO("任务执行中。。。。。。。。。。");
    }else if(task_status == 2){
        logger.INFO("任务完成。。。。。。。。。。");
    }else if(task_status == 3){
        logger.INFO("任务中止。。。。。。。。。。");
    }

    //<国遥> 调用国遥状态同步RPC



}

void C2Manager::Subscribe_rwzxzt_502(const QJsonObject &root)
{
    // QString SID = root["SID"].toString();
    // QString TM = root["TM"].toString();
    // QJsonObject CTN = root["CTN"].toObject();
    // string task_id = CTN["task_id"].toString().toStdString();
    // _mission_control._missionStatus.task_status = CTN["task_status"].toInt();
    // int task_status = _mission_control._missionStatus.task_status;
    // if(task_status == 0){
    //     logger.INFO("任务未开始。。。。。。。。。。");
    // }else if(task_status == 1){
    //     logger.INFO("任务执行中。。。。。。。。。。");
    // }else if(task_status == 2){
    //     logger.INFO("任务完成。。。。。。。。。。");
    // }else if(task_status == 3){
    //     logger.INFO("任务中止。。。。。。。。。。");
    // }

    //<国遥> 调用国遥状态同步RPC



}

void C2Manager::Subscribe_CK504(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();

    // 解析CK/504视频拉流信息
    int request_id = CTN["request_id"].toInt();
    if (request_id != 2) {
        logger.INFO("CK/504: request_id不是2，忽略此消息");
        return;
    }
    VideoStreamList vslist;

    if (CTN.contains("equip_video_addr_list") && CTN["equip_video_addr_list"].isArray()) {
        QJsonArray videoArray = CTN["equip_video_addr_list"].toArray();

        for (int i = 0; i < videoArray.size(); ++i) {
            QJsonObject videoObj = videoArray[i].toObject();
            QString  battle_unit_code = videoObj["battle_unit_code"].toString();
            int equipment_seq_no = videoObj["battle_unit_code"].toInt();
            QString video_address = videoObj["video_address"].toString();
            if(video_battle_unit_code != battle_unit_code) continue;
            VideoStream vs;
            vs.vsName = battle_unit_code.toStdString()+"["+std::to_string(equipment_seq_no)+"]";
            vs.model_id = battle_unit_code.toStdString();
            vs.equipmentSeqNo = std::to_string(equipment_seq_no);
            vs.vsType  = "rtsp";
            vs.vsPath = video_address.toStdString();
            vslist.videoStreamList.push_back(vs);

            logger.INFO("添加视频流: " + vs.vsName + ", 路径: " + vs.vsPath);
        }
    }

    // 在新线程中加载视频流，避免阻塞
    if (!vslist.videoStreamList.empty()) {
        std::thread t(&C2Manager::loadVideoStream, this, vslist);
        t.detach();
        _prcp_client->m_rcpClientAdapter->showNotification("gy消息","接收视频流地址信息",0,2);
        logger.INFO("CK/504: 已加载" + std::to_string(vslist.videoStreamList.size()) + "个视频流");
    } else {
        logger.INFO("CK/504: 没有可用的视频流信息");
    }
}

void C2Manager::subscribe_dt_801(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    string task_id = CTN["task_id"].toString().toStdString();
    _dtControl.CreateorDTZzmb(root);
}

void C2Manager::subscribe_dt_802(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    string task_id = CTN["task_id"].toString().toStdString();
}

void C2Manager::subscribe_dt_803(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    string task_id = CTN["task_id"].toString().toStdString();
}


void C2Manager::Subscribe_531(const QJsonObject &root)
{
    //锁定电磁目标
    QString SID = root["SID"].toString();
    QString TM = root["TM"].toString();
    QJsonObject& CTN = root["CTN"].toObject();
    string task_id = CTN["task_id"].toString().toStdString();
    string battle_unit_no = CTN["battle_unit_no"].toString().toStdString();
    string result_code;
    if( CTN.contains("result_code")){
        result_code =  CTN["result_code"].toString().toStdString();
        if(result_code == "1"){
            string str ="锁定电磁目标成功";
            dcyd_result(str);
            return;
        }

    }
    QJsonObject elec_exact_target = CTN["elec_exact_target"].toObject();
    PhoneExactTarget phoneExactTarget;
    phoneExactTarget.plmn =elec_exact_target["plmn"].toString().toStdString();
    phoneExactTarget.imsi = elec_exact_target["imsi"].toString().toStdString();
    QJsonObject pos = elec_exact_target["position"].toObject();
    phoneExactTarget.position.longitude = QJsonValue(pos["longitude"]).toDouble();
    phoneExactTarget.position.latitude = QJsonValue(pos["latitude"]).toDouble();
    phoneExactTarget.position.altitude = QJsonValue(pos["altitude"]).toDouble();
     //回字形规划
    QJsonObject CTN2 =  _root_rwgh["CTN"].toObject();
    CTN2["task_type"] = 162;
    QJsonObject target_position;
     target_position["longitude"] =phoneExactTarget.position.longitude;
     target_position["latitude"] =phoneExactTarget.position.latitude;
     target_position["altitude"] =phoneExactTarget.position.altitude;
     CTN2["target_position"] =target_position;
    _root_rwgh["CTN"] =CTN2;
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/101", _root_rwgh);

}

bool C2Manager::parseOwnPlatformFromJson(const QJsonObject &root, OwnPlatform &ownP)
{
    if (!root.contains("CTN")) {
        return false;
    }

    QJsonObject ctn = root["CTN"].toObject();

    // 必填字段检查
    // if (!ctn.contains("report_time") || !ctn.contains("battle_unit_code") ||
    //     !ctn.contains("platform_type") || !ctn.contains("deploy_position") ||
    //     !ctn.contains("velocity") || !ctn.contains("trk_direction") ||
    //     !ctn.contains("task_execute_status")) {
    //     return false;
    // }
    ownP.basic.model_id  =  ctn["battle_unit_code"].toString().toStdString();
    if(ctn.contains("battle_unit_name")){
        ownP.basic.model_name = ctn["battle_unit_name"].toString().toStdString();
    }
    if(ownP.basic.model_name =="") ownP.basic.model_name = ownP.basic.model_id;

    ownP.basic.report_time = QJsonValue(ctn["report_time"]).toVariant().toLongLong();
    ownP.basic.platform_type =ctn["platform_type"].toInt();
    if(ownP.basic.platform_type == 1){

        ownP.basic.platform_type = 1005; //异构无人机

    }else if(ownP.basic.platform_type == 2){

       ownP.basic.platform_type = 2007;   //异构无人车

    }else if(ownP.basic.platform_type == 3){ //

        ownP.basic.platform_type = 2003;   //单兵

    }else if(ownP.basic.model_id =="dy11") { //手机电磁主无人机

        ownP.basic.platform_type = 1005;

     }else if(ownP.basic.model_id =="dy12"){ //手机电磁从无人机

        ownP.basic.platform_type = 1006;

     }else if(ownP.basic.model_id =="dy08" || ownP.basic.model_id =="dy09"){ //山猫无人车 52 53
         ownP.basic.platform_type = 1007;
     }else{
         ownP.basic.platform_type = 1005;
     }
    // 位置信息解析
    if (ctn.contains("deploy_position")) {
        QJsonObject posObj = ctn["deploy_position"].toObject();
        if (posObj.contains("longitude")) {
            ownP.basic.deploy_position.longitude = QJsonValue(posObj["longitude"]).toDouble();
        }
        if (posObj.contains("latitude")) {
            ownP.basic.deploy_position.latitude = QJsonValue(posObj["latitude"]).toDouble();
        }
        if (posObj.contains("altitude")) {
            ownP.basic.deploy_position.altitude = QJsonValue(posObj["altitude"]).toDouble()+20; //适配倾斜摄影高度
            if(ownP.basic.platform_type == 2007) ownP.basic.deploy_position.altitude = 25;
        }
    }

    ownP.basic.velocity =  ctn["velocity"].toDouble();
    ownP.basic.trk_direction = ctn["trk_direction"].toDouble();

    // 姿态信息
    if (ctn.contains("roll")) {
        ownP.basic.roll= QJsonValue(ctn["roll"]).toDouble();
    }
    if (ctn.contains("pitch")) {
        ownP.basic.pitch = QJsonValue(ctn["pitch"]).toDouble();
    }
    if (ctn.contains("yaw")) {
        ownP.basic.yaw = QJsonValue(ctn["yaw"]).toDouble();
    }
    if (ctn.contains("health_status")) {
        ownP.business.health_status = ctn["health_status"].toInt();
    }
    if (ctn.contains("exception_msg")) {
        ownP.business.exception_msg = ctn["exception_msg"].toString().toStdString();
    }
    if (ctn.contains("plat_redidual_electricity")) {
        ownP.business.plat_redidual_electricity = ctn["plat_redidual_electricity"].toInt();
    }
    // 任务状态 (可选)
    if(ctn.contains("task_execute_status")){
        ownP.business.task_execute_status = ctn["task_execute_status"].toInt();
    }
    if(ctn.contains("task_id")){
        ownP.business.task_number =ctn["task_id"].toString().toStdString();     // 执行的任务编号
    }
    // 载荷状态列表
    if(ctn.contains("equip_state_list")){
        QJsonArray elist = ctn["equip_state_list"].toArray();

        for (int i = 0; i < elist.size(); ++i) {
            EquipmentState eState;
            QJsonObject eq = elist[i].toObject();
            eState.equip_no = eq["equip_no"].toString().toStdString();
            eState.equip_redidual_electricity = eq["equip_redidual_electricity"].toInt();
            ownP.business.equip_state_list.push_back(eState);
        }
    }
    return true;
}

bool C2Manager::parseTargetlatformFromJson(const QJsonObject &root, Targetlatform &tarP)
{
    if (!root.contains("CTN")) {
        return false;
    }

    QJsonObject ctn = root["CTN"].toObject();
    ctn["space_type"] ="11";
    // 必填字段
    if (!ctn.contains("target_ID") || !ctn.contains("battle_unit_code") ||
        !ctn.contains("first_time") || !ctn.contains("latest_time") ||
        !ctn.contains("space_type") || !ctn.contains("capture_count") ||
        !ctn.contains("scout_type") || !ctn.contains("target_plat_type")) {
        return false;
    }
 //   if(ctn.contains("_jsby_isEnd")) _jsby_isEnd =ctn["_jsby_isEnd"].toInt(); //军事by任务特有

    tarP.basic.model_id = ctn["target_ID"].toString().toStdString();

    tarP.basic.model_name = ctn.contains("target_name")?ctn["target_name"].toString().toStdString(): tarP.basic.model_id;

    tarP.basic.report_time = QJsonValue(ctn["first_time"]).toVariant().toLongLong();

    tarP.basic.platform_type = QJsonValue(ctn["target_plat_type"]).toInt();

    if(tarP.basic.platform_type  == 110){ //识别的手机电磁目标
        tarP.basic.platform_type = 1005; //无人机
    }else if(tarP.basic.platform_type  == 100 ||tarP.basic.platform_type == 1){ //识别的人
        tarP.basic.platform_type = 2003;

    }else if(tarP.basic.platform_type  == 2){

        tarP.basic.platform_type = 2007;  //军用车辆

    }else if(tarP.basic.platform_type  == 4){

        tarP.basic.platform_type = 1005;
    }
    else {
        tarP.basic.platform_type = 2007;
    }
    //航迹点
    if(ctn.contains("track_info")){
        QJsonObject track_info = ctn["track_info"].toObject();
        QJsonObject posobj= track_info["position"].toObject();
        tarP.basic.deploy_position.longitude =  posobj.value("longitude").toDouble();
        tarP.basic.deploy_position.latitude =   posobj.value("latitude").toDouble();
        tarP.basic.deploy_position.altitude =   posobj.value("altitude").toDouble();
        tarP.basic.velocity =track_info["velocity"].toDouble();
        tarP.basic.trk_direction= track_info["trk_direction"].toDouble();
    }

    // 姿态信息
    tarP.basic.roll = 0.0;                           // 滚转角 (左负右正)
    tarP.basic.pitch = 0.0;                          // 俯仰角 (前负后正)
    tarP.basic.yaw =0.0;                            // 偏航角 (右负左正)


    tarP.business.collect_platform_id =ctn["battle_unit_code"].toInt() ;
    tarP.business.data_source =ctn["data_source"].toInt();
    tarP.business.first_time = ctn["first_time"].toString().toLongLong() ;
    tarP.business.latest_time =ctn["latest_time"].toString().toLongLong();

    tarP.business.space_type =ctn["space_type"].toInt();
    tarP.business.capture_count=ctn["capture_count"].toInt();
    tarP.business.scout_type=ctn["scout_type"].toInt();

    return true;
}

void C2Manager::Subscribe_LJ(const QJsonObject &root)
{

    QString data = root["data"].toString();
    if(data == "1"){ //添加实体
        APIBridgeSDK::EntityMission info;  //新增行为结构体
        info.entityId ="9";       //实体id
        info.missionDes = u8"侦察";    //行为描述
       // info.behaviorName = u8"自主机动";    //行为类型名称 协议里的行为名称
         QString UUID = QUuid::createUuid().toString();
        info.uuId =UUID.toStdString() ;   //行为id
        info.name ="自主机动";
        info.missionType = RPC_MissionType::Mission_AdHoc;  //行为类型 普通行为或临机行为
        info.startTime = 150000;   //开始时间 ms
        info.duration = 54209;  //持续时长 ms
        info.jsonData ="{}"; // 空要添加{}符号;
        _prcp_client->m_rcpClientAdapter->addEntityMission(info);
    }else if(data == "2"){ // 删除实体
        string uuid ="{faa34535-34ab-4229-bc84-61540be1df47}";
        _prcp_client->m_rcpClientAdapter->removeEntityMission(uuid);
    }else if(data == "3"){ //废弃实体
        string uuid ="{faa34535-34ab-4229-bc84-61540be1df47}";

        _prcp_client->m_rcpClientAdapter->abandonMission(uuid);
    }
}

void C2Manager::Subscribe_ZT(const QJsonObject &root)
{
   // _prcp_client->m_rcpClientAdapter->cameraGoto(28.2072,112.895,107.79,46.44,30.46);

    // RPC_EntityRunningState ss;  // 注释掉未使用的变量（return后的死代码）

    string uuid = root["uuid"].toString().toStdString();
    int32_t data = root["data"].toInt();
    string data2 = root["data2"].toString().toStdString();
     double data3 = root["data3"].toString().toDouble();

    if(data == 1){
        APIBridgeSDK::ImsiTargetInfo imsiInfo;
        imsiInfo.imsi = uuid;
        imsiInfo.plmn =46000;
        imsiInfo.task_id ="task11";
        _prcp_client->m_rcpClientAdapter->addIsmiTargetInfo(imsiInfo);

    }else if(data == 2){
        string ismi = uuid;
         _prcp_client->m_rcpClientAdapter->removeImsiTargetInfo(ismi);
    }else if(data == 3){
        _prcp_client->m_rcpClientAdapter->removeAllImsiTargetInfo();
    }else if(data == 4){
        APIBridgeSDK::KeyEventDataSDK  ke;
        ke.time = currentTimeMillisec();
        ke.type  =  RPC_KeyEventType::keyEventTypeNormal;
        ke.title ="哈全文测试事件";
        ke.id =  "哈全文测试大大大";
        ke.describe ="哈全文测试大大大测试大大大测试大大大测试大大大测试大大大";

        std::list<APIBridgeSDK::KeyEventDataSDK> data;
        for(int i = 0 ; i < 100; i++){
            ke.id =  "哈全文测试事件"+std::to_string(i);
            ke.title ="哈全文测试事件"+std::to_string(i);
             _prcp_client->m_rcpClientAdapter->addKeyEvent(ke);
        }

    }else if(data == 5){
        _prcp_client->m_rcpClientAdapter->removeEvent("333333");

    }else if(data == 6){
        _prcp_client->m_rcpClientAdapter->clearAllEvent();

    }else if(data == 7){
        QJsonObject root;
        root["imsi"] = 1233343344;
        Subscribe_LocationTarget(root);
    }else if(data == 8 ){
        _prcp_client->m_rcpClientAdapter->schemeStartPlay();
    }else if(data == 9){
        //示例
        APIBridgeSDK::EntityEnvelop data;
        data.model_id = "dy11";
        data.heading = 0;
        data.pitch = -90;
        data.angle = 80;
        data.distance = 200;
        data.fillColor.push_back(255);
        data.fillColor.push_back(100);
        data.fillColor.push_back(0);
        data.fillColor.push_back(100);
        _prcp_client->m_rcpClientAdapter->openEnvelop(data);
    }else if(data == 10){
        _prcp_client->m_rcpClientAdapter->closeEnvelop("dy11");
    }else if(data == 11){
        APIBridgeSDK::PARadarData data;
        data.model_id = "dy11";
        data.heading = 0;
        data.pitch = -85;
        data.vAngle = 30;
        data.hAngle = 145;
        data.distance = 200;
        data.scanAngle = 60;
        data.radarStyle = 2;

        data.frameColor.push_back(255);
        data.frameColor.push_back(0);
        data.frameColor.push_back(0);
        data.frameColor.push_back(255);

        data.fillColor.push_back(255);
        data.fillColor.push_back(0);
        data.fillColor.push_back(0);
        data.fillColor.push_back(100);
         _prcp_client->m_rcpClientAdapter->openPARadar(data);
    }else if(data == 12){
        _prcp_client->m_rcpClientAdapter->closePARadar("dy11");
    }else if(data == 13){
        APIBridgeSDK::ParticleEffect data;
        data.effectName = data2;         // 烟尘 比例0.1；起火烟雾持续打开，比例0.1；曳光弹效果，战场周边距离较远 比例100
       // data.targetKey ="dy11";           //绑定目标id，对应目标的模型上需要绑定特效才能显示
        APIBridgeSDK::Position position;               //和targetKey二选一，如果不绑定动目标，则可以在任意坐标显示模型
        position.longitude =118.09809011;
        position.latitude = 32.50961491;
        position.altitude = 16.32;
        data.position = position;
        data.roll = 0;                  //翻滚角偏移
        data.pitch = 0;                 //俯仰角偏移
        data.yaw = 0;                   //朝向角偏移
        data.scale = data3;               //缩放
        data.duartion = 5;               //持续时长 秒，到达事件后自动释放，如果填-1则永远显示
        _prcp_client->m_rcpClientAdapter->showParticleEffect(data);
    }else if(data == 14){
         string s = "测试大模型窗口";
        _prcp_client->m_rcpClientAdapter->showChatReply(s);
    }else if(data == 15){
        string s = "测试大模型窗口";
        Sender sender;
        sender.alignt = 0;
        sender.icon ="./icons/wujing.png";
        sender.name="规划系统";
        APIBridgeSDK::LargeModelMsg lmMsg;

        std::list<std::string> contents;
        contents.push_back("这是第一行");
        contents.push_back("这是第二行         ");
        contents.push_back("  ");
        contents.push_back("  这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行这是超行");
        lmMsg.contents =contents;
      // lmMsg.imagePath ="./icons/wujing.png";
        lmMsg.sender = sender;
        lmMsg.title ="这是标题";
        _prcp_client->m_rcpClientAdapter->showLargeModelMsg(lmMsg);
    }

    return;
    // if(data == "1")
    // {
    //     ss = RPC_EntityRunningState::Wating;
    // }else if(data == "2"){
    //     ss = RPC_EntityRunningState::Running;
    // }if(data == "3"){
    //     ss = RPC_EntityRunningState::Pause;
    // }

    // _prcp_client->m_rcpClientAdapter->switchEntityRunningState(uuid.toStdString(),ss);

}

void C2Manager::Subscribe_push(const QJsonObject &root)
{
    QString user_id = root["user_id"].toString();
    QString title = root["title"].toString();
    QString priority = root.contains("priority") ? root["priority"].toString() : "";
    int message_count = root.contains("message_count") ? root["message_count"].toInt() : 0;
    
    QJsonArray messages = root["messages"].toArray();
    
    // 记录基本推送信息
    QString basicInfo = QString("收到推送消息 - 标题:%1, 用户ID:%2, 优先级:%3, 消息数量:%4")
                         .arg(title, user_id, priority, QString::number(message_count));
    qDebug() << basicInfo;
    
    // 如果不是当前用户，只记录日志不处理事件
    if (user_id != _user_id.c_str()) {
        qDebug() << "推送消息给其他用户 user_id:" << user_id;
        return;
    }

    qDebug() << "推送消息给当前用户 user_id:" << user_id;

    // 处理第一个消息（简化测试）
    std::list<APIBridgeSDK::KeyEventDataSDK> eventList;

    // 只处理第一个消息
    // if (messages.size() > 0) {
    //     QJsonObject msg = messages[0].toObject();

    //     QString msg_type = msg["type"].toString();
    //     QString msg_title = msg["title"].toString();
    //     QString message_text = msg["message"].toString();
    //     QString msg_priority = msg.contains("priority") ? msg["priority"].toString() : priority;
    //     QString timestamp = msg.contains("timestamp") ? msg["timestamp"].toString() : "";

    //     // 构建简化的消息内容
    //     QString content = QString("推送消息 - 类型:%1, 标题:%2, 消息:%3")
    //                       .arg(msg_type, msg_title, message_text);

    //     if (!timestamp.isEmpty()) {
    //         content += QString(", 时间戳:%1").arg(timestamp);
    //     }

    //     // 创建关键事件
    //     APIBridgeSDK::KeyEventDataSDK ke;
    //     ke.time = currentTimeMillisec();

    //     // 根据风险等级设置事件类型
    //     if (msg_type == "proximity_warning") {
    //         ke.type = RPC_KeyEventType::keyEventTypeCritical;  // 接近威胁设为关键事件
    //     } else {
    //         ke.type = RPC_KeyEventType::keyEventTypeNormal;
    //     }

    //     ke.title = msg_title.toStdString();
    //     ke.describe = content.toStdString();
    //     ke.id = QUuid::createUuid().toString().toStdString();

    //     // 使用addKeyEvent添加单个事件
    //     _prcp_client->m_rcpClientAdapter->addKeyEvent(ke);
    // }

    // 原始代码注释掉
    for (int i = 0; i < messages.size(); ++i) {
        QJsonObject msg = messages[i].toObject();

        QString msg_type = msg["type"].toString();
        QString msg_title = msg["title"].toString();
        QString message_text = msg["message"].toString();
        QString msg_priority = msg.contains("priority") ? msg["priority"].toString() : priority;
        QString timestamp = msg.contains("timestamp") ? msg["timestamp"].toString() : "";

        // 处理data字段（可选）
        QString dataInfo = "";
        if (msg.contains("data") && msg["data"].isObject()) {
            QJsonObject data = msg["data"].toObject();

            // 提取接近威胁的关键信息
            if (msg_type == "proximity_warning") {
                QString entity1 = data.contains("entity1") ? data["entity1"].toString() : "";
                QString entity2 = data.contains("entity2") ? data["entity2"].toString() : "";
                double current_distance = data.contains("current_distance") ? data["current_distance"].toDouble() : 0;
                double predicted_distance = data.contains("predicted_distance") ? data["predicted_distance"].toDouble() : 0;
                QString risk_level = data.contains("risk_level") ? data["risk_level"].toString() : "";

                dataInfo = QString("实体1:%1, 实体2:%2, 当前距离:%3km, 预计距离:%4km, 风险等级:%5")
                           .arg(entity1, entity2, QString::number(current_distance, 'f', 2),
                                QString::number(predicted_distance, 'f', 2), risk_level);
            }
        }

        // 构建消息内容
        QString content = QString("推送消息[%1/%2] - 类型:%3, 标题:%4, 消息:%5")
                          .arg(QString::number(i+1), QString::number(messages.size()), msg_type, msg_title, message_text);

        if (!dataInfo.isEmpty()) {
            content += QString(", 数据:%1").arg(dataInfo);
        }

        if (!timestamp.isEmpty()) {
            content += QString(", 时间戳:%1").arg(timestamp);
        }

        // 创建关键事件
        APIBridgeSDK::KeyEventDataSDK ke;
        ke.time = currentTimeMillisec();

        // 根据风险等级设置事件类型
        if (msg_type == "proximity_warning") {
            ke.type = RPC_KeyEventType::keyEventTypeCritical;  // 接近威胁设为关键事件
        } else {
            ke.type = RPC_KeyEventType::keyEventTypeNormal;
        }

        ke.title = msg_title.toStdString();
        ke.describe = content.toStdString();
        ke.id = QUuid::createUuid().toString().toStdString();

        eventList.push_back(ke);
    }

    // 使用addKeyEventGroup批量添加事件
    if (!eventList.empty()) {
        _prcp_client->m_rcpClientAdapter->addKeyEventGroup(eventList);
    }
    //
}

void C2Manager::Subscribe_prediction(const QJsonObject &root)
{
    QString type = root["type"].toString();
    QString title = root["title"].toString();
    QString message = root["message"].toString();
    double confidence = root["confidence"].toDouble();
    QString risk_level = root["risk_level"].toString();
    
    // 获取details中的关键信息（可选）
    QString intent_description = "";
    if (root.contains("details") && root["details"].isObject()) {
        QJsonObject details = root["details"].toObject();
        if (details.contains("intent_description")) {
            intent_description = details["intent_description"].toString();
        }
    }
    
    // 获取recommendations（可选）
    QString recommendations_str = "";
    if (root.contains("recommendations") && root["recommendations"].isArray()) {
        QJsonArray recommendations = root["recommendations"].toArray();
        QStringList recList;
        for (const QJsonValue& recValue : recommendations) {
            recList.append(recValue.toString());
        }
        if (!recList.isEmpty()) {
            recommendations_str = recList.join(", ");
        }
    }
    
    // 构建简洁明了的日志消息
    QString logMsg = QString("收到预测消息 - 类型:%1, 风险等级:%2, 置信度:%3, 标题:%4")
                      .arg(type, risk_level, QString::number(confidence, 'f', 2), title);
    
    if (!intent_description.isEmpty()) {
        logMsg += QString(", 意图描述:%1").arg(intent_description);
    }
    
    if (!recommendations_str.isEmpty()) {
        logMsg += QString(", 建议措施:%1").arg(recommendations_str);
    }
    
    // 如果是高风险预警，添加关键事件并发出警告
    if (risk_level.toLower() == "critical") {

        APIBridgeSDK::KeyEventDataSDK ke;
        ke.time = currentTimeMillisec();
        ke.type = RPC_KeyEventType::keyEventTypeCritical;
        ke.title = title.toStdString();
        ke.describe = logMsg.toStdString();
        ke.id = QUuid::createUuid().toString().toStdString();
        _prcp_client->m_rcpClientAdapter->addKeyEvent(ke);
    }
}

void C2Manager::Subscribe_KM6(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    QJsonArray task_list = CTN["task_list"].toArray();
    for (int i = 0; i < task_list.size(); ++i) {
        QJsonObject missionPlan; //打包发送的任务规划
        addHeader(missionPlan);

        QJsonObject missionBattle; //打包发送的作战单元
        addHeader(missionBattle);



       //解析到结构体中
        MQTTMission::MissionPlan mp;
        QJsonObject missionObj = task_list[i].toObject();

        mp.task_id =  missionObj["task_id"].toString().toStdString();

        mp.task_name =missionObj["task_name"].toString().toStdString();

        mp.task_type =missionObj["task_type"].toInt();

        mp.force_num =  missionObj["force_num"].toInt();

        mp.battle_unit_code = missionObj["force_id"].toInt();


        if(missionObj.contains("route")){
            QJsonArray route_arr =  missionObj["route"].toArray();
            for (int i = 0; i < route_arr.size(); i++) {
                QJsonObject pos = route_arr[i].toObject();
                MQTTMission::Position  position;
                position.longitude =   QJsonValue(pos["longitude"]).toDouble();
                position.latitude =   QJsonValue(pos["latitude"]).toDouble();
                position.altitude =   QJsonValue(pos["altitude"]).toDouble();
                mp.route.push_back(position);
            }

        }

        if(missionObj.contains("target_ID"))  {
            mp.target_ID  = missionObj["target_ID"].toInt();
        }

        mp.start_time = missionObj["start_time"].toInt();

        mp.end_time = missionObj["end_time"].toInt();


        mp.auto_level  = missionObj["auto_level"].toInt();

        mp.priority  = missionObj["priority"].toInt();


        QJsonObject CTN;
        QJsonObject task_basic_info;
        task_basic_info["task_id"] = QString::fromStdString(mp.task_id) ;
        task_basic_info["task_name"] = QString::fromStdString(mp.task_name);
        QJsonArray task_type_list;
        task_type_list.append(mp.task_type);
        task_basic_info["task_type_list"] = task_type_list;
        task_basic_info["start_time"] =mp.start_time ;
        task_basic_info["end_time"] =mp.end_time ;
        task_basic_info["edit_time"] =currentTimeMillisec() ;
        task_basic_info["description"] = QString::fromStdString("") ;
        task_basic_info["task_aim"] =QString::fromStdString(mp.task_name);
        task_basic_info["task_terminal_condition"] ="task_terminal_condition" ;
        CTN["task_basic_info"] =task_basic_info;

        QJsonArray target_list;
        CTN["target_list"] = target_list;

        QJsonArray execute_unit_list;

        execute_unit_list.append(QString::fromStdString(mp.battle_unit_code));

        CTN["execute_unit_list"] =execute_unit_list;

        missionPlan["CTN"] = CTN;
        // 给操控发送103报文
        //


        QJsonObject CTN2;
        CTN2["battle_unit_no"] =QString::fromStdString(mp.battle_unit_code);
        CTN2["task_id"] =QString::fromStdString(mp.task_id);

        QJsonObject  planned_track;
        planned_track["trackline_name"] = QString::fromStdString(mp.task_name+"航线");
        planned_track["start_time"] =mp.start_time;
        planned_track["end_time"] =mp.end_time;
        QJsonArray track_point_list;
        for(MQTTMission::Position pos :mp.route){
            QJsonObject  obj;
            obj["current_time"]=currentTimeMillisec();
            obj["velocity"]=0;
            QJsonObject  track_point_position;
            track_point_position["longitude"] = pos.longitude;
            track_point_position["latitude"] = pos.latitude;
            track_point_position["altitude"] = pos.altitude;
            obj["track_point_position"] = track_point_position;
            track_point_list.append(obj);
        }
        planned_track["track_point_list"] =track_point_list;
        planned_track["min_height"] =12345;
        planned_track["max_height"] =12345;
        planned_track["input_type"] =1;
        planned_track["road_width"] =12345;
        planned_track["edit_time"] = QString::fromStdString(currentTime());
        planned_track["description"] = QString::fromStdString(mp.task_name+"航线");

        if( mp.route.size() > 0) {

            MQTTMission::Position init_pos = mp.route[0];
            QJsonObject init_position;
            init_position["longitude"] = init_pos.longitude;
            init_position["latitude"] = init_pos.latitude;
            init_position["altitude"] = init_pos.altitude;
            planned_track["init_position"] =init_position;
            planned_track["end_position"] = init_position;

        }
        CTN2["planned_track"] =planned_track;
        missionBattle["CTN"] = CTN2;
        //给操控下发104报文
        //

    }

    QString task_id =  CTN["task_id"].toString();

}

void C2Manager::Subscribe_CK530(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    int elec_target_type = CTN["elec_target_type"].toInt();
    // MQTTMission ::PhoneElec phoneElec;
    _phoneElec.task_id =CTN["task_id"].toString().toStdString();
    _phoneElec.battle_unit_no = CTN["battle_unit_no"].toString().toStdString();
    if(elec_target_type == 0){
        MQTTMission ::PhoneSignalTarget phoneSignalTarget;
        QJsonObject pt = CTN["elec_signal_target"].toObject();
        phoneSignalTarget.plmn =pt["plmn"].toString().toStdString();
        phoneSignalTarget.imsi =pt["imsi"].toString().toStdString();
       // QJsonObject pos = CTN["elec_signal_target"].toObject();
        
        APIBridgeSDK::ImsiTargetInfo imsiInfo;
        imsiInfo.imsi = phoneSignalTarget.imsi;
        imsiInfo.plmn =QString::fromStdString(phoneSignalTarget.plmn).toInt();
        imsiInfo.task_id = _mission_control._mission.task_id;
        
        // 判断imsi是否在配置文件中
        logger.INFO("电磁目标识别成功：plmn= "+phoneSignalTarget.plmn+";imsi="+phoneSignalTarget.imsi);
        QString test = QString::fromUtf8(_imsi_list.c_str());
        QStringList imsi_list = test.split(",");
        logger.INFO("配置文件中的imsi列表：" + _imsi_list);

        _prcp_client->m_rcpClientAdapter->addIsmiTargetInfo(imsiInfo);  //增加电磁汇聚信息
        _phoneElec.phoneSignalTarget_list.push_back(phoneSignalTarget);


        
    }else if(elec_target_type == 1){
        MQTTMission::PhoneExactTarget phoneExactTarget;

        QJsonObject elec_exact_target = CTN["elec_exact_target"].toObject();
        QString plmn = elec_exact_target["plmn"].toString();
        QString imsi = elec_exact_target["imsi"].toString();
        QJsonObject position = elec_exact_target["position"].toObject();

        phoneExactTarget.position.longitude = QJsonValue(position["longitude"]).toDouble();
        phoneExactTarget.position.latitude = QJsonValue(position["longitude"]).toDouble();
        phoneExactTarget.position.altitude = QJsonValue(position["altitude"]).toDouble();
        phoneExactTarget.plmn = plmn.toStdString();
        phoneExactTarget.imsi = imsi.toStdString();
        _phoneElec.phoneExactTarget = phoneExactTarget;
        logger.INFO("电磁目标识别成功：plmn= "+plmn.toStdString()+";imsi="+imsi.toStdString()+"位置为："+doubleToString(phoneExactTarget.position.longitude)
                 +":"+doubleToString(phoneExactTarget.position.latitude)+
                    +":"+doubleToString(phoneExactTarget.position.altitude));

    }
}

void C2Manager::Subscribe_CK534(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonArray CTN = root["CTN"].toArray();

    // 遍历CTN数组中的每个任务
    for (int i = 0; i < CTN.size(); ++i) {
        QJsonObject taskObj = CTN[i].toObject();
        QJsonDocument doc(taskObj);
        QString taskObjStr = doc.toJson(QJsonDocument::Compact);
        logger.INFO("taskObj content: " + taskObjStr.toStdString());
        
        //！！！这里写成全局，锁定时要通过targetid获取平台编号
        // MQTTMission::RecElec recElec;
        // // recElec.task_id = taskObj["task_id"].toString().toStdString();
        // // recElec.battle_unit_no = taskObj["battle_unit_no"].toString().toStdString();
        // // recElec.freq = taskObj["freq"].toDouble();
        // // recElec.target_id = taskObj["target_id"].toString().toStdString();
        
        APIBridgeSDK::ImsiZGInfo zgInfo;
        zgInfo.task_id = taskObj["task_id"].toString().toStdString();
        zgInfo.entity_id = taskObj["battle_unit_no"].toString().toStdString();
        zgInfo.target_id = taskObj["target_id"].toString().toStdString();
        QJsonValue freqValue = taskObj["freq"].toDouble();
        QString freqStr = QString::number(freqValue.toDouble(), 'f', 6); // 保留6位小数
        zgInfo.frequcency = freqStr.toStdString();
        // zgInfo.frequcency = taskObj["freq"].toString().toStdString();
        _recElec534.push_back(zgInfo);
        _prcp_client->m_rcpClientAdapter->addIsmiZGInfo(zgInfo);
        // _prcp_client->m_rcpClientAdapter->removeIsmiAllZGInfo(); // 先删除再添加，防止重复添加
        logger.INFO("调用addIsmiZGInfo发布电磁侦干信息: " + zgInfo.task_id);
        
    }
    int s = 1 + rand() %6;
    qDebug()<<"态势引导指令下发时间为"<<s<<"毫秒";
}

void C2Manager::Subscribe_CK535(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();

    // 检查CTN中是否包含longitude和latitude字段
    if (!CTN.contains("longitude") || !CTN.contains("latitude")) {
        _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","CTN中缺少longitude或latitude字段",3,3);

       // MessageDialog::showInformation(nullptr, "错误", "CTN中缺少longitude或latitude字段");
        return;
    }

    // MQTTMission ::PhoneElec phoneElec;
    _recElecLocation.task_id =CTN["task_id"].toString().toStdString();
    _recElecLocation.battle_unit_no = CTN["battle_unit_no"].toString().toStdString();
    _recElecLocation.target_id = CTN["target_id"].toString().toStdString();
    _recElecLocation.longitude =QJsonValue(CTN["longitude"]).toDouble();
    _recElecLocation.latitude = QJsonValue(CTN["latitude"]).toDouble();


     //夹角路线,精准锁定
    QJsonObject CTN2 =  _root_rwgh["CTN"].toObject();
    CTN2["task_type"] = 42;
    QJsonObject target_position;
    target_position["longitude"] = _recElecLocation.longitude;
    target_position["latitude"] = _recElecLocation.latitude;
    target_position["altitude"] = 0;
    CTN2["target_position"] =target_position;
    _root_rwgh["CTN"] =CTN2;

    if (_zg_status.rec_target_flag == false) {
        _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/101", _root_rwgh);
        _zg_status.rec_target_flag = true;
    }

    
}

void C2Manager::Subscribe_CK536(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    // MQTTMission ::PhoneElec phoneElec;
    _mission_CK536.task_id =CTN["task_id"].toString().toStdString();
    _mission_CK536.battle_unit_no = CTN["battle_unit_no"].toString().toStdString();
    _mission_CK536.emi_status = CTN["emi_status"].toInt();
    if(CTN.contains("emi_stage")){
        _mission_CK536.emi_stage = CTN["emi_stage"].toInt();
    }
    
    if (_mission_CK536.emi_status == 1) {
        _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","侦干任务已完成",0,3);

       // MessageDialog::showInformation(nullptr, "MQTT消息", "侦干任务已完成");

        string  result_str ="干扰任务已完成，状态：" + std::to_string(_mission_CK536.emi_status) +
                           "，阶段：" + std::to_string(_mission_CK536.emi_stage);
                dczg_result(result_str);
        _zg_status.rec_failed_count = 0;
        return;
    } else if ((_mission_CK536.emi_status == 0 || 
               _mission_CK536.emi_status == 2) &&
               _zg_status.rec_interference_global_stage == 0) { //rec_interference_global_stage和 _mission_CK536.emi_stage一致
        _zg_status.rec_failed_count++;
    }
    
    if (_zg_status.rec_interference_global_stage == 0 && 
        _zg_status.rec_failed_count == 4) {
        _zg_status.rec_failed_count = 0;
        _zg_status.rec_interference_global_stage = 1; // 空空协同阶段
    } else if (_zg_status.rec_interference_global_stage == 1) {
        _zg_status.rec_interference_global_stage = 2;
    }else if(_zg_status.rec_interference_global_stage == 2){
        //整个任务结束，没有干扰到目标
        _zg_status.kd_failed_count++;
        if(_zg_status.kd_failed_count == 3){
            string result_str = "干扰任务终结，状态：" + std::to_string(_mission_CK536.emi_status) +
                        "，阶段：" + std::to_string(_zg_status.rec_interference_global_stage);
           dczg_result(result_str);
            return;
        }
    }

    QJsonObject CTN2 =  _root_rwgh["CTN"].toObject();
    //抵近干扰任务类型

    if (_zg_status.rec_interference_global_stage == 0) {
        CTN2["task_type"] = ZG_DJ_TASK_TYPE; // 抵近干扰阶段
    } else if (_zg_status.rec_interference_global_stage == 1) {
        CTN2["task_type"] = ZG_KK_TASK_TYPE; // 空空干扰阶段
    } else if (_zg_status.rec_interference_global_stage == 2){
        CTN2["task_type"] = ZG_KD_TASK_TYPE; // 空地干扰阶段
        QJsonObject target_position;
        target_position["longitude"] = _zg_status.zg_master_uav_pos.longitude;
        target_position["latitude"] = _zg_status.zg_master_uav_pos.latitude;
        target_position["altitude"] = 0;
        CTN2["target_position"] =target_position;
    }

    // logger.INFO("536-精度" + std::to_string(_mission_CK536.emi_status) +
    //                 "，阶段：" + std::to_string(_mission_CK536.emi_stage));

    // _recElecLocation.longitude = CTN["longitude"].toDouble();
    // _recElecLocation.latitude = CTN["latitude"].toDouble();

    _root_rwgh["CTN"] =CTN2;
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/101", _root_rwgh);
}

void C2Manager::Subscribe_CK537(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    string battle_unit_code = CTN["battle_unit_code"].toString().toStdString();
    string message = CTN["message"].toString().toStdString();
    logger.INFO("收到CK537消息，作战单元编号：" + battle_unit_code +
                "，消息内容：" + message);
    _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","收到CK537，单元编号：" + battle_unit_code +
                                                                       "，内容：" + message,1,3);
}

void C2Manager::GetConnTargetAndPlatform(std::vector<string> &target_list, std::vector<string> &platfor_list)
{
    for(string plat:_BY_platform_dis_list){

        for(string tar:_BY_target_dis_list){
            OwnPlatform ownP;
           bool findplat = _mission_control.getOwnPlatform(plat,ownP);
            Targetlatform tarp;
           bool findtar = _mission_control.getTargetlatform(tar,tarp);
            if(findplat && findtar){
            double distance = _geo_algo.RealDistance(ownP.basic.deploy_position.latitude,ownP.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude,tarp.basic.deploy_position.longitude);
            if(distance <200){
                platfor_list.push_back(plat);
                    target_list.push_back(tar);
                   _BY_target_delete_list.push_back(tar);
                   _BY_platform_delete_list.push_back(plat);
            }
           }
        }
    }
}

void C2Manager::SHowTeXiao(string model,double lon,double lat)
{
    _TaskModelbehavior._particleEffect2.particleId = model;
    _TaskModelbehavior._particleEffect2.position.longitude =lon;
    _TaskModelbehavior._particleEffect2.position.latitude =lat;
    _TaskModelbehavior._particleEffect2.scale = 4.0;
    _TaskModelbehavior._particleEffect2.position.altitude =2;
    _prcp_client->m_rcpClientAdapter->showParticleEffect(_TaskModelbehavior._particleEffect2);
}

void C2Manager::Subscribe_TY103(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    int elec_target_type = CTN["task_id"].toString().toInt();
     //_pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/103", root);
}

void C2Manager::Subscribe_TY104(const QJsonObject &root)
{
    QString SID = root["SID"].toString();
    QString TM =  root["TM"].toString();
    QJsonObject CTN = root["CTN"].toObject();
    int elec_target_type = CTN["task_id"].toString().toInt();
    //_pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/104", root);

}

void C2Manager::Subscribe_human_location(const QJsonObject &root)
{
  //  QString schemeInfo = QString::fromStdString(std::string(si));
    QJsonObject data = root["data"].toObject();
    qDebug() << root;
    QJsonObject human;
    addHeader(human);
    QJsonObject CTN;
    std::vector<zy> db_list = _gkd_cfg->zyc["2003"];
    if(db_list.size() == 0) return;
    CTN["battle_unit_code"] =QString::fromStdString( db_list[0].zb_id);
    CTN["battle_unit_name"] = QString::fromStdString( db_list[0].name);
    QJsonObject deploy_position;
    deploy_position["altitude"] =10.0;
    QJsonValue latitudev = data["latitude"].toDouble();
    QString latitude = QString::number(latitudev.toDouble(), 'f', 9);

    deploy_position["latitude"]=latitudev.toDouble();

    QJsonValue longitudev = data["longitude"].toDouble();
    QString longitude = QString::number(longitudev.toDouble(), 'f', 9);
    deploy_position["longitude"] = longitudev.toDouble();
    CTN["deploy_position"]=deploy_position;
    CTN["platform_type"] = 3;
    human["CTN"] = CTN;
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("QB/303", human);
}

void C2Manager::Subscribe_LocationTarget(const QJsonObject &root)
{
    _imsi = root["imsi"].toString().toStdString();
    //修改原来任务规划的类型
    QJsonObject CTN =  _root_rwgh["CTN"].toObject();
    CTN["task_type"] = 161;
    _root_rwgh.remove("CTN");
    _root_rwgh["CTN"] = CTN;
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/101", _root_rwgh);
}

void C2Manager::Subscribe_ZKtest(const QJsonObject &root)
{
    double lon  = QJsonValue(root["lon"]).toDouble();
    double lat = QJsonValue(root["lat"]).toDouble();
    //
    //32.04418742,119.07061476 //南京
    //25.034017640049427,121.5610503756714  //长沙
    _prcp_client->m_rcpClientAdapter->cameraGoto(lat,lon,74.88,53.87,5.44,1000);
}

bool C2Manager::ControlDJiMini4_Pro(QString rid,double executeHeight, double waypointSpeed, std::vector<MQTTMission::Position> &path)
{
    if(path.size() < 2) return false;
    QJsonObject root;
    addHeader(root);
    QJsonObject CTN;
    CTN["wpml_executeHeight"] = executeHeight;
    CTN["wpml_waypointSpeed"] = waypointSpeed;
    CTN["rid"] = rid;
    QJsonObject line_list;
    QJsonArray point_list;

    for(MQTTMission::Position position:path){
        QJsonObject pos;
        pos["longitude"] =position.longitude;
        pos["latitude"]  =position.latitude;
        pos["altitude"]  =executeHeight;
        point_list.push_back(pos);
    }
    line_list["point_list"] = point_list;
    CTN["line_list"] =line_list;
    root["CTN"] = CTN;
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/PathTopic", root);
    _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","无人机行动控制下发成功",0,4);
    return true;

}

bool C2Manager::isSHow()
{
     qint64 msecs = QDateTime::currentMSecsSinceEpoch();
    return msecs%2;
}

bool C2Manager::checkAnyDifference(const std::vector<double> &vec, double minDiff)
{
    for (size_t i = 0; i < vec.size()-1; ++i) {
        for (size_t j = i + 1; j < vec.size(); ++j) {
            if (std::abs(vec[j] - vec[i]) < minDiff) {
                return false; // 如果找到一个差值小于10的，返回false
            }
        }
    }
    return true;
}

void C2Manager::updata_zyc_name(string model_id, string name)
{
    for(auto& pair:_gkd_cfg->zyc){
        for (auto& item : pair.second) {
            if (item.zb_id == model_id) {
                item.name = name;
                break;
            }
        }
    }
}

bool C2Manager::hasId(const std::vector<zy>& vec, const std::string& model_id) {
    auto it = std::find_if(vec.begin(), vec.end(),
                           [&model_id](const zy& item) {
                               return item.zb_id == model_id; // 只比较 zb_id 字段
                           });

    return it != vec.end();
}
void C2Manager::CreateorTrackLine(std::vector<MQTTMission::Position> &route)
{
    std::vector<APIBridgeSDK::Position> rcp_positions;
    for (const auto& pos : route) {
        rcp_positions.push_back(APIBridgeSDK::Position(pos.longitude, pos.latitude, pos.altitude));
    }
    // 绘制任务路线（红色）
    if (!rcp_positions.empty() && _prcp_client && _prcp_client->m_rcpClientAdapter) {
        // 使用时间戳生成route_uuid
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        std::string route_uuid = std::to_string(timestamp) + "_route";
        _prcp_client->m_rcpClientAdapter->updateLineMark(route_uuid, rcp_positions, "#FF0000");
        _task_plane_roate_uuid_list.push_back(route_uuid);// 更新存储的路径UUID
        qDebug() << QString::fromStdString("添加标绘:" + route_uuid);
    }
}

void C2Manager::dczg_result(string result_str)
{

        logger.INFO(result_str);
      // updateElectInfoText
}

void C2Manager::dcyd_result(string result_str)
{
   logger.INFO(result_str);
}

void C2Manager::loadVideoStream(VideoStreamList& vslist)
{
    _prcp_client->m_rcpClientAdapter->loadVideoStream(vslist);
}

void C2Manager::schemeEndPlay()
{
    _prcp_client->m_rcpClientAdapter->schemeEndPlay(); //目前不能用
}

void C2Manager::schemeStartPlay()
{
    _prcp_client->m_rcpClientAdapter->schemeStartPlay();
}

bool C2Manager::isIdentifyTarget(APIBridgeSDK::OwnPlatform& ownplatform,QString task_type,QString target_ID)
{

    //注意数据竞争问题
    for(int i = 0; i < _mission_control._targetPlatform_list.size(); i ++){

        APIBridgeSDK::Targetlatform targetplatform= _mission_control._targetPlatform_list[i];
         double distance = _geo_algo.RealDistance(targetplatform.basic.deploy_position.latitude,targetplatform.basic.deploy_position.longitude,ownplatform.basic.deploy_position.latitude,ownplatform.basic.deploy_position.longitude);

        if(distance < 30 && task_type == "1" ){ //侦查少于200米则发现
             find_target_id = QString::fromStdString(targetplatform.basic.model_id);
            return true;
        }else if(distance < 30 && task_type == "2" && QString::fromStdString(targetplatform.basic.model_id) == find_target_id ){ // 打击小于3米发现的目标
            target_ID = find_target_id;
            return true;
        }
    }

    return false;
}

void C2Manager::publishImageTopic()
{
    if(is_publish_targetImg) return;
    QJsonObject root;
    root["cmd"] = "SHOW_ALERT";
    root["sn"] = "sn";
    QJsonObject data;
    data["file_path"] = "D:/target.png";
    data["alert_type"] = "检测到侦查结果";
    data["alert_summary"] = "无人机侦查到地方目标";
    data["img_format"] = "png";
    data["image_type"] = "Drone";
    data["sensor_type"] = "Electro-Optic";
    data["image_quality"] = "HD";
    data["algorithm"] = "Vehicle Detection V 2.3.1";
    data["longitude"] = 112.806900;
    data["latitude"] = 28.242764;
    data["altitude"] = 0;
    root["data"] = data;
    root["timestamp"] = 1750995507;
      _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("/llm/jsim/excute", root);
    is_publish_targetImg = true;
}

void C2Manager::showThreatHemisphere_301(string model_id)
{
    int cr = 0,cb=255;
    Targetlatform tarp;
    _mission_control.getTargetlatform(model_id,tarp);
    APIBridgeSDK::DefenseSphere data;
    data.sphereId = model_id;
    data.center = {tarp.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude,tarp.basic.deploy_position.altitude};
    data.radius = 300;
    data.fillColor.push_back(cr);
    data.fillColor.push_back(0);
    data.fillColor.push_back(cb);
    data.fillColor.push_back(200);
    data.frameColor.push_back(cr);
    data.frameColor.push_back(0);
    data.frameColor.push_back(cb);
    data.frameColor.push_back(255);
    _prcp_client->m_rcpClientAdapter->showThreatHemisphere(data);
}

void C2Manager::showThreatHemisphere_303( string model_id)
{
    int cr = 255,cb = 0;

    OwnPlatform tarp;
    _mission_control.getOwnPlatform(model_id,tarp);
    APIBridgeSDK::DefenseSphere data;
    data.sphereId = model_id;
    data.center = {tarp.basic.deploy_position.longitude,tarp.basic.deploy_position.latitude,tarp.basic.deploy_position.altitude};
    data.radius = 300;
    data.fillColor.push_back(cr);
    data.fillColor.push_back(0);
    data.fillColor.push_back(cb);
    data.fillColor.push_back(200);
    data.frameColor.push_back(cr);
    data.frameColor.push_back(0);
    data.frameColor.push_back(cb);
    data.frameColor.push_back(255);
    _prcp_client->m_rcpClientAdapter->showThreatHemisphere(data);
}

void C2Manager::removeThreatHemisphere(string model_id)
{
    _prcp_client->m_rcpClientAdapter->removeThreatHemisphere(model_id);
}

void C2Manager::CreatorTemple()
{

    std::map<string,string> byparamMap;
    _gkd_cfg->getPerformanceParams( byparamMap);
    //敌方参数模版
    std::vector<JSBY_Param> jsby_target_param_list;
    for (const auto& pair : byparamMap) {
        JSBY_Param param;
        param.name = pair.first;
        std::vector<std::string> result =  splitString(pair.second,'_');
        for(int i = 0 ; i < result.size(); i++){
            if(i ==0)  result[0] == "false"? param.is_complex =false: param.is_complex =true;
            if(i ==1) param.min = stod(result[1]);
            if(i ==2) param.weight‌ = stod(result[2]);
            if(i ==3) param.max = stod(result[3]);

        }
        jsby_target_param_list.push_back(param);
    }

    for(auto& p :_jsby_target_value_map_1){
        _jsby_target_value_map[p.first] =jsby_target_param_list;
    }
    //我方参数模版
    std::vector<JSBY_Param> jsby_platform_param_list;
    for (const auto& pair : byparamMap) {
        JSBY_Param param;
        param.name = pair.first;
        std::vector<std::string> result =  splitString(pair.second,'_');
        for(int i = 0 ; i < result.size(); i++){
            if(i ==0)  result[0] == "false"? param.is_complex =false: param.is_complex =true;
            if(i ==1) param.min = stod(result[1]);
            if(i ==2) param.weight‌ = stod(result[2]);
            if(i ==3) param.max = stod(result[3]);

        }
        jsby_platform_param_list.push_back(param);
    }

    for(auto& p :_jsby_platform_value_map_1){
        _jsby_platform_value_map[p.first]=jsby_platform_param_list;
    }
}

bool C2Manager::jsbyWeightedAverage()
{


    CreatorTemple(); //参数模版

    //计算敌方
    for(auto& pair: _jsby_target_value_map){
        string id = pair.first;
        std::vector<JSBY_Param>& _jsby_target_value_list = pair.second;
        for(auto& param: _jsby_target_value_list){
            double value;
            bool isfind = getJsbyParamValue(true,id,param.name,value); //模版是固定的，外界输入值可以不全
            isfind == true?param.value = value:param.value=-99;
        }
    }

    //计算我方
    for(auto& pair: _jsby_platform_value_map){
        string id = pair.first;
        std::vector<JSBY_Param>& _jsby_platform_value_list = pair.second;
        for(auto& param: _jsby_platform_value_list){
            double value;
            bool isfind = getJsbyParamValue(false,id,param.name,value); //模版是固定的，外界输入值可以不全
            isfind == true?param.value = value:param.value=-99;
        }

    }
    //总权重敌方
    for(auto& pair: _jsby_target_value_map){
        string id = pair.first;
        std::vector<JSBY_Param>& _jsby_target_value_list = pair.second;
        double sum_target_weight‌ = 0;
        for(auto& s: _jsby_target_value_list){
           sum_target_weight‌  += s.weight‌;
        }
        _sum_target_weight‌[id]=sum_target_weight‌;

    }
    //总权重我方

    for(auto& pair: _jsby_platform_value_map){
        string id = pair.first;
        std::vector<JSBY_Param>& _jsby_platform_value_list = pair.second;
        double sum_platform_weight‌ = 0;
        for(auto& s: _jsby_platform_value_list){
            sum_platform_weight‌  += s.weight‌;
        }
        _sum_platform_weight‌[id]=sum_platform_weight‌;

    }

    //维度系数 敌方
    for(auto& pair: _jsby_target_value_map){
        string id = pair.first;
        std::vector<JSBY_Param>& jsby_target_param_list = pair.second;
        std::vector<double> weight‌_cos_target_list;
        for(const auto& s: jsby_target_param_list){
            double v = (s.value-s.min)/(s.max-s.min);
            v< 0 ?v = 0 :(v>1? v = 1:v=v);
            double weight‌_cos = v* s.weight‌;
            weight‌_cos_target_list.push_back(weight‌_cos);
        }
        _weight‌_cos_target_map[id]=weight‌_cos_target_list;
    }


    //维度系数 我方
    for(auto& pair: _jsby_platform_value_map){
        string id = pair.first;
        std::vector<JSBY_Param>& jsby_platform_param_list = pair.second;
        std::vector<double> weight‌_cos_platform_list;
        for(const auto& s: jsby_platform_param_list){
            double v = (s.value-s.min)/(s.max-s.min);
            v< 0 ?v = 0 :(v>1? v = 1:v=v);
            double weight‌_cos = v* s.weight‌;
            weight‌_cos_platform_list.push_back(weight‌_cos);
        }
        _weight‌_cos_platform_map[id]=weight‌_cos_platform_list;
    }

    //维度系数求和 敌方

    for(auto& pair: _weight‌_cos_target_map){
        string id = pair.first;
        double weight_cos = 0;
        for(const auto& s:pair.second){
            weight_cos+=s;
        }
        _weight‌_cos_target_sum_map[id]=weight_cos;
    }


    //维度系数求和 我方
    for(auto& pair: _weight‌_cos_platform_map){
        string id = pair.first;

        double weight_cos = 0;
        for( auto& s:pair.second){
            weight_cos+=s;
        }
        _weight‌_cos_platform_sum_map[id]=weight_cos;
    }


     //博弈结果计算 敌方
    double sum_weight_target_cos =0 ;
    for(auto& pair: _sum_target_weight‌){
        string id = pair.first;
        sum_weight_target_cos += pair.second;
    }

      double   target_weight_cos = 0;
    for(auto& pair: _weight‌_cos_target_sum_map){
        string id = pair.first;
       target_weight_cos += pair.second;
    }
    double target_val = target_weight_cos/sum_weight_target_cos;


//博弈结果计算 我方
    double sum_weight_platform_cos=0 ;
    for(auto& pair: _sum_platform_weight‌){
        string id = pair.first;
        sum_weight_platform_cos += pair.second;
    }

    double   platform_weight_cos = 0;
    for(auto& pair: _weight‌_cos_platform_sum_map){
        string id = pair.first;
        platform_weight_cos += pair.second;
    }
    double platform_val = platform_weight_cos/sum_weight_platform_cos;

    //显示
    QString display_info;
    display_info.append( "    当前敌我双方有"+ QString::number(12)+"个参考指标,其中敌方指标信息如下：\n");

    int index = 1;
    for(auto& pair: _jsby_target_value_map){
        std::string id = pair.first;
        display_info.append("    第"+QString::number(index++)+"个装备指标信息:");
        std::vector<JSBY_Param> _jsby_target_value_list = pair.second;
        for(auto& s: _jsby_target_value_list){
            display_info.append("    "+QString::fromStdString(s.name)+":"+" "+QString::number(s.value)+",参考权重为"+QString::number(s.weight‌)+";");

        }
        display_info.append("\n");
    }

    display_info.append("其中我方指标信息如下:\n");
    index = 1;
    for(auto& pair: _jsby_platform_value_map){
        std::string id = pair.first;
        display_info.append("    第"+QString::number(index++)+"个装备指标信息:");
        std::vector<JSBY_Param> _jsby_platform_value_list = pair.second;
        for(auto& s: _jsby_platform_value_list){
            display_info.append("   "+QString::fromStdString(s.name)+":"+" "+QString::number(s.value)+",参考权重为"+QString::number(s.weight‌)+";");
        }
         display_info.append("\n");
    }

    display_info.append("   此轮博弈我方胜出概率为"+QString::number(platform_val*100)+"%，敌方胜出率为"+QString::number(target_val*100)+"%,");
    target_val > platform_val ? display_info.append("敌方胜出率大。"):display_info.append("我方胜出率大。");

    APIBridgeSDK::TextWndInfo textInfo;
    textInfo.title ="博弈结论";
    textInfo.content =display_info.toStdString();//display_info;
    textInfo.hideDelay = 0;
    _prcp_client->m_rcpClientAdapter->showTextWindow(textInfo);

    return true;
}

bool C2Manager::getJsbyParamValue(bool istarget,string idstr,string param,double& value)
{
    if(istarget){
        for(auto& pair: _jsby_target_value_map_1){
            string id = pair.first;
            if(idstr == id){
                std::vector<JSBY_Param>& _jsby_target_value_list = pair.second;
                for(auto& jsby_param: _jsby_target_value_list){
                    if(jsby_param.name == param){
                        value = jsby_param.value;
                        return true;
                }

            }

        }
        }
    }else{
        for(auto& pair: _jsby_platform_value_map_1){
            string id = pair.first;
            if(id == idstr){
                std::vector<JSBY_Param>& _jsby_platform_value_list = pair.second;
                for(auto& jsby_param: _jsby_platform_value_list){
                    if(jsby_param.name == param){
                        value = jsby_param.value;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

std::vector<string> C2Manager::splitString(const string &input, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

void C2Manager::reSetJSBY()
{
    _jsby_target_value_map_1.clear();
    _jsby_platform_value_map_1.clear();
    _jsby_target_value_map.clear();
    _jsby_platform_value_map.clear();

    _sum_target_weight‌.clear();
    _sum_platform_weight‌.clear();

    _weight‌_cos_target_map.clear();
    _weight‌_cos_platform_map.clear();

    _weight‌_cos_target_sum_map.clear();
    _weight‌_cos_platform_sum_map.clear();
    _jsby_isEnd = 0;
}


void C2Manager::processTaskPlan(const std::string& task)
{
    APIBridgeSDK::OwnPlatform op;

    QJsonParseError jsonError;
    // QTextCodec *tc = QTextCodec::codecForName("UTF-8");//防止中文乱码
    QString str = QString::fromStdString(task);
    QJsonDocument _document = QJsonDocument::fromJson(str.toUtf8().data(), &jsonError);
    if (jsonError.error != QJsonParseError::NoError) return;

    QJsonObject json = _document.object();
    logger.INFO("-------------------------新任务开启--------------------");

    //重置任务
    ReSetMission();
    // 创建任务规划起始信息
    Mission  mission;
    json["task_id"] = QString::fromStdString(json["task_id"].toString().toStdString() + "_" + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    if( CreatorMission(json,mission)){
        logger.INFO("......任务："+_mission_control._mission.task_id+" 已接收成功......");
    }else{
        logger.INFO("......任务请求解析失败.");
        return;
    }

    //2-1任务规划请求打包
    // QJsonObject root;
    if(PackageMission(json,_root_rwgh)){

        logger.INFO("......任务："+_mission_control._mission.task_id+"任务规划请求打包完成");

    }else{

        logger.INFO("......任务："+_mission_control._mission.task_id+"请求打包失败，检查任务类型是否正确等问题");
        return;

    }


    //请求2-1进行任务规划
    logger.INFO("......任务："+_mission_control._mission.task_id+"任务规划请求发送出");

    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/101", _root_rwgh);
}

QString C2Manager::timePointToString(const std::chrono::steady_clock::time_point &tp)
{
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::stringstream ss;
    ss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
    return QString::fromStdString(ss.str());
}

// string C2Manager::currentTime()
// {
//     // 获取当前时间点
//     auto now = std::chrono::system_clock::now();

//     // 转换为time_t类型
//     std::time_t now_time = std::chrono::system_clock::to_time_t(now);

//     // 转换为本地时间
//     std::tm* local_time = std::localtime(&now_time);

//     // 获取毫秒部分
//     auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
//                   now.time_since_epoch()) % 1000;

//     // 格式化输出
//     std::stringstream ss;
//     ss  << std::put_time(local_time, "%Y-%m-%d %H:%M:%S")
//        << "." << std::setfill('0') << std::setw(3) << ms.count();
//     return ss.str();
// }

// int64_t C2Manager::currentTimeMillisec()
// {
//         struct timeb tb;
//         ftime(&tb);
//       return  static_cast<int64_t>( tb.time * 1000 + tb.millitm);
// }

void C2Manager::lockElecTargetId(const MQTTMission::RecElec rec_elec)
{
    QJsonObject root;
    addHeader(root);
    QJsonObject imp;
    imp.insert("task_id",QString::fromStdString(rec_elec.task_id));
    imp["task_set_flag"] = LJGH_ZG_TARGET_LOCATION;
    imp.insert("battle_unit_no",QString::fromStdString(rec_elec.battle_unit_no));
    imp.insert("elec_lock_target_id", QString::fromStdString(rec_elec.target_id));
    imp.insert("elec_lock_freq", rec_elec.freq);
    
    root["CTN"] = imp;

    // 发布锁定电磁目标ID的消息
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root); 
}


void C2Manager::lockElecTargetId(const string target_ID,int kdxt_type)
{
    QJsonObject root;
    addHeader(root);
    QJsonObject imp;
    imp.insert("task_id",QString::fromStdString(_mission_control._mission.task_id));
    imp["task_set_flag"] = kdxt_type;
    imp["target_ID"] = QString::fromStdString(target_ID);

    root["CTN"] = imp;
    // 发布锁定电磁目标ID的消息
    _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root);
}

// void C2Manager::lockElecPath(const MQTTMission::MissionPlan missionPlan,QJsonObject& root)
// {
//     QJsonObject root;
//     addHeader(root);
//     QJsonObject imp;
//     imp.insert("task_id",QString::fromStdString(rec_elec.task_id));
//     imp["task_set_flag"] = 9;
//     imp.insert("battle_unit_no",QString::fromStdString(rec_elec.battle_unit_no));
//     imp.insert("elec_lock_target_id", QString::fromStdString(rec_elec.target_id));
//     root["CTN"] = imp;

//     // 发布锁定电磁目标ID的消息
//     _pgkd_mqtt_client->GetMqttConn()->PrimitivePublish("ZK/105", root); 
// }
