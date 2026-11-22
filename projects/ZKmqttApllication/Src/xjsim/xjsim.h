#pragma once

#include <QDebug>
#include <QTimer>
#include <QThread>
#include <memory>
#include <mutex>
#include <map>
#include <QSet>
#include <QImage>
#include <QByteArray>
#include <QDir>
#include "data_define.h"
#include "utils/json.hpp"
#include <QUuid>
#include <utm/projection_utm.h>
#include<iomanip>
#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include "mqtt/mq_client.h"
#include "../rest_rcp_client.h"
#include "../RCPServerBridge.h"
#include "SDKStructDefine.h"
#include <QBuffer>
#include <QDateTime>
#include "utils/uav_data_parser.h"
#include "common/linearInterpolate.h"
#include "common/gkd_config.h"
//任务执行实体信息
struct excuteUAVOrUGV{
    string uid;
    string uname;
    string cmd;  //无人机 or 无人车
    string task_type;
    int platform_type;
    double executeHeight;
    double waypointSpeed;
    double roll;// 滚转角 (左负右正)  optional
    double pitch;// 俯仰角 (前负后正)  optional
    double yaw;// 偏航角 (右负左正)  optional
    std::vector<MQTTMission::Position> path;
    int timestamp;
};

//新军事博弈 敌方目标结构体
struct JSBY_Target{
    string id;
    std::vector<Position> route;
    int type;
    int speed;
};
class XjSim : public QObject {
    Q_OBJECT

public:
    explicit XjSim(QObject* parent = nullptr);
    ~XjSim() override;

    // 公共接口
    void startTimer();
    void initDevice();
    //接收数据
    ApiResponse revMqttData(const string& topic,const string& data);
    //接收任务数据
    ApiResponse revTaskData(const string& data);
    // 获取智能体列表
    const std::map<std::string, SimObject>& getTotalObjects() const;
    const SimObject& getSelfObject() const;

    // 命令接口
    ApiResponse startUav();
    ApiResponse stopUav();
    ApiResponse resetCommand();
    ApiResponse openCabin();
    ApiResponse closeCabin();
    ApiResponse stopFpv();
    ApiResponse releaseFpv(int number);

    // 移动控制
    ApiResponse testSelfMove(MoveDirection direction, const std::string& guid = "");

    // 快捷命令接口
    ApiResponse startCommand(const std::string& cmd,int type, const std::string& guid, const std::vector<LatLon>& positions);
    ApiResponse executeCommand(CommandType cmdType, const std::string& guid = "");

    void initUav();
    //    void SetFunctionCall_Command();
    void InitMqttAndRcp(MqClient* mqClient,rest_rcp_client* _rcp_client,const CommonCfg& _cfg,const GKDCommonCfg&_gkd_cfg);
    ApiResponse runFpv(bool isDj);
    ApiResponse runFpv(bool isDj,string height);

    // 相机控制
    ApiResponse cameraGoto(double latitude, double longitude, double altitude, double tilt = 0, double heading = 0, int time_ms = 1000);

    // 火力点显示控制
    ApiResponse showFirePoints();

    // 安全区显示控制
    ApiResponse showSafeZone();

    // 规避面显示控制
    ApiResponse showAvoidanceArea();

    // 避让点显示控制
    ApiResponse showYieldPoints();
    //加载态势数据
    ApiResponse loadVechileMode();
    void PublishMD(QString name,int type,double lon,double lat);
    bool PackageDzmbqb_301(QString name,int type,QJsonObject& root,MQTTMission::Position& pos); //电子目标情报
    bool PackageZzptjzbzt_303(QJsonObject& root,QString runing_status,excuteUAVOrUGV& ab,MQTTMission::Position& pos); // 作战平台,飞机
    bool PackageJSBY_dzmbpt_301(QJsonObject& root,QString id,APIBridgeSDK::Position& point,int speed,int type); // 新版本军事博弈

    //发送装备信息
    void planZB(excuteUAVOrUGV& zb);
    void slotLoadScheme();
private slots:
    void checkDeviceStatus();
    void updateTacticalInformation();

private:
    QSet<QString> working_aircraft_uids_;
    qint64 last_task_info_tm_ = 0;
    bool task_info_tm_initialized_ = false;
    std::mutex aircraft_status_mutex_;
    void recordAircraftCommand(const QJsonObject& root);
    void resetAircraftStates(const QJsonObject& root);
    QString aircraftRunningStatus(const QString& uid);

    // 私有方法
    void sendRegister();
    void updateObject(bool isOnline, SimObject* obj,
                      const string& guid, const string& name,
                      int type, const RealTimeStatusDataEx& pos);
    void convertToOwnPlatforms(std::vector<APIBridgeSDK::OwnPlatform>& ownPlatforms);
    void convertToTargetPlatforms(std::vector<APIBridgeSDK::OwnPlatform>& ownPlatforms);
    qreal convertToNorthHeading(qreal heading);
    void cleanup();
    bool hasId(const std::vector<zy>& vec, const std::string& mode_id);
    // 命令相关
    std::string getCommandString(CommandType cmdType) const;
    ApiResponse sendCommand(const std::string& cmd, const std::string& guid);

    //地图添加区域
    ApiResponse addAreaToMap(nlohmann::json jsonData);
    ApiResponse addPointToMap(nlohmann::json jsonData);
    ApiResponse removePlotToMap(nlohmann::json jsonData);
    ApiResponse addRouteToMap(nlohmann::json jsonData);
    ApiResponse runUav(nlohmann::json jsonData);
    ApiResponse run_uav_uid(nlohmann::json jsonData);
    //更新态势数据
    ApiResponse addOrUpdateVechile(nlohmann::json jsonData);
    void runEnemy();
    ApiResponse showAlert(nlohmann::json jsonData);
    //显示固定的警告
    void showAlert();
    ApiResponse createScheme(nlohmann::json jsonData);
    ApiResponse addEntity(nlohmann::json jsonData);
    ApiResponse addHoc(nlohmann::json jsonData);
    //废弃实体行为
    ApiResponse abortEntity(nlohmann::json jsonData);
    ApiResponse runLandship(nlohmann::json jsonData);
    //删除方案
    ApiResponse removeScheme(nlohmann::json jsonData);
    //删除实体行为(任务)
    ApiResponse removeEntityMission(nlohmann::json jsonData);
    //加载任务
    ApiResponse loadTask(nlohmann::json jsonData);
    ApiResponse revImage(nlohmann::json jsonData);
    bool resetCommand(const std::string& guid);
    bool exCommand(CommandSetDataEx cmdData, const std::string& guid);
    void showChatReply(const nlohmann::json& revData);
    std::string utf8ToGbk(const std::string& data);
    void autoShowOrTj();
    void initGYVideo();
    void loadVideoStream(VideoStreamList& vslist);
    void jsbyPlay();//军事博弈敌方装备播放
    // 处理设备状态数据（来自 llm/701/jsim/status 主题）
    void processJsimStatusData(const std::string& data);

    void addHeader(QJsonObject& root); // 加入报文头 SID TM
    bool ControlDJiMini4_Pro(const string& rid, double executeHeight,double waypointSpeed,std::vector<MQTTMission::Position>& path); // 控制大疆mini4_pro无人机
    void ControlDJiMini4_Pro_Gimbal_pitch(const QString& uid);// 控制大疆mini4_pro无人机云台

private:
    int command_id_{10};
    bool is_initialized_{false};
    int test_id = 0;

    //是否开舱检测
    bool is_open_{false};

    double _height = 10;//补偿的高度

    // 坐标转换
    ProjectionUTM projection_utm_;
    UTMPoint utm_point_;

    // 对象状态
    SimObject self_object_;
    SimObject enemy_object_;
    SimObject uav_object_;

    std::map<std::string, SimObject> total_objects_;

    // 线程安全
    mutable std::mutex objects_mutex_;
    //    //默认侦察的UAV
    //    std::string uavGuid="";

    ApiResponse apiRes;

    nlohmann::json revData;

    QTimer* timer_ = nullptr;
    QTimer* status_timer_ = nullptr;
    QThread* timer_thread_ = nullptr;
    QThread* status_thread_ = nullptr;
    ExCmd exCmd_;

    MqClient* mqClient = nullptr;
    rest_rcp_client* rcp_ = nullptr;

    QString status_topic = "/llm/jsim/status";
    QString cmd_topic = "/llm/jsim/excute";
    QString cmd_respose_topic = "/llm/jsim/respose";
    std::string qB_303 = "{\"CTN\":{\"battle_unit_code\":\"dy03\",\"deploy_position\":{\"altitude\":222.6340298595398,\"latitude\":32.5129934,\"longitude\":118.0979694},\"equip_state_list\":[{\"equip_no\":\"yc34h011\",\"hyperspectral_equip_status\":{\"device_id\":1,\"modal\":\"1\",\"sensor_id\":\"SENSOR-2\"},\"light_infrared_equip_status\":{\"device_id\":1,\"modal\":\"rgb\",\"sensor_id\":\"yc34h011\",\"type\":1}}],\"health_status\":1,\"is_external\":0,\"pitch\":-0.6343631039533421,\"plat_redidual_electricity\":44,\"platform_type\":1,\"report_time\":1761127714542,\"roll\":-5.907120266423616,\"trk_direction\":1.631076455116272,\"velocity\":0,\"velocityX\":0.005572739522904158,\"velocityY\":-0.027356108650565147,\"velocityZ\":-0.0013774753315374255,\"yaw\":1.631076455116272},\"SID\":\"2\",\"TM\":1761127714551}";
    CommonCfg _cfg;
    GKDCommonCfg _gkd_cfg;
    DroneDataParser data_parser;
    std::vector<string> _task_plane_roate_uuid_list;// 存储所有路线UUID

    QJsonArray _jsby_targetObj;
    int _jsby_playstatus = 0;
    int _jsby_rate = 0;
    int _jsby_totaltime =0;
    int _jsby_currenttime=0;


    ///////////////大模型相关参数/////////////////////////////////////////
    QJsonObject _llm_excute; //行动信息
    std::map<QString, QString> platforms_name_by_id_;//装备id和name映射
    std::mutex platforms_name_mutex_;//互斥锁
    std::atomic<int> task_type;
    std::atomic<bool>   is_zc_dj = false; //这里区分是侦查打击任务还是普通的飞行任务

};

inline const std::map<string, SimObject> &XjSim::getTotalObjects() const { return total_objects_; }

inline const SimObject &XjSim::getSelfObject() const { return self_object_; }

inline ApiResponse XjSim::startUav() {
    return executeCommand(CommandType::START_UAV);
}

inline ApiResponse XjSim::stopUav() {
    //        bool isWorking = false;
    //       for(const auto& pair : getTotalObjects()) {
    //           const auto& obj = pair.second;
    //           if(1 == obj.type) {
    //               if(HasTaskExecuting(obj.uid)){
    //                   isWorking = true;
    //               }
    //               break;
    //           }
    //       }
    //       if(isWorking){
    //           return apiRes.Failure(ErrorCode::ERR,"有正在进行任务的无人机，请等无人机执行完任务");
    //       }
    return executeCommand(CommandType::STOP_UAV);
}

inline ApiResponse XjSim::resetCommand() {
    exCmd_.cmd_type = ExCommandType::EX_NONE;
    exCmd_.isNeedDj = false;
    exCmd_.isWorking = false;
    exCmd_.isNeedShowTargent = false;
    exCmd_.isNeedTj = false;
    exCmd_.task_list.clear();
    return executeCommand(CommandType::RESET_CMD);
}

inline ApiResponse XjSim::openCabin() {
    if(1 == exCmd_.isOpenCabin){
        return apiRes.Failure(ErrorCode::ERR,"舱门已经打开");
    }
    exCmd_.isOpenCabin = 1;
    return executeCommand(CommandType::OPEN_CABIN);
}

inline ApiResponse XjSim::closeCabin() {
    // this->rcp_->m_rcpClientAdapter->cameraGoto(0,0,0,0,0,0); // test by hqw
    if(2 == exCmd_.isOpenCabin){
        return apiRes.Failure(ErrorCode::ERR,"舱门已经关闭");
    }
    exCmd_.isOpenCabin = 2;
    return executeCommand(CommandType::CLOSE_CABIN);
}

inline ApiResponse XjSim::stopFpv() { return executeCommand(CommandType::STOP_FPV); }
