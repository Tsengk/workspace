#ifndef C2Manager_H
#define C2Manager_H
#include<memory>
#include"common/gkd_config.h"
#include "mqtt/mq_client.h"
#include "../rest_rcp_client.h"
#include"gkd/task_response_define.h"
 #include "utm/geo_algo.h"
#include <initializer_list>
#include "gkd/mission_control.h"
#include<gkd/mission.h>
#include <vector>
#include<QObject>
#include<QJsonArray>
#include<SDKStructDefine.h>
#include"utils/log_sys.h"
#include"QJsonValue"
#include <functional>
#include<map>
#include"taskmodelbehavior.h"
#include"DtTaskTarget.h"
#include"gkd/dt_control.h"
#include<QDateTime>
#include <chrono>
#include <iostream>
#include <cmath> // 用于abs函数
/**
*现场联调管理类
*by hqw
**/
struct mosquitto;  // 与mosquitto库保持一致
using namespace MQTTMission;
//军事博弈参数模型结构
struct JSBY_Param{
    string name;
    bool is_complex; //是否为复合参数
    double min;
    double weight‌;  //当前参数权重
    double max;
    double value; //当前参数值
    bool is_target; //是否为敌方
};

//军事博弈蓝方信息
struct JSBY_Target1{
    string id;
    string name;
    APIBridgeSDK::Position target_pos;

};

class C2Manager  {
//Q_OBJECT

public:
    C2Manager(std::shared_ptr<MqClient> gkd_client = nullptr,std::shared_ptr<rest_rcp_client> rcp_client = nullptr,GKDCommonCfg* gkd_cfg = nullptr);
   ~C2Manager();

   void MQTTSubscribe(); //订阅现场mqtt协议
   void MQTTSubscribe_DT(); //订阅导调系统mqtt协议

   void RPCSubscribe(); //订阅国遥RPC协议

   void setBYType(bool _isShowThreatHemisphere,bool r1,bool r2, bool r3, bool r4); //军事博弈状态类型设定 基于零和模型

   void processTaskPlan(const std::string& task); //任务规划处理
   QString timePointToString(const std::chrono::steady_clock::time_point& tp);
   
   void lockElecTargetId(const MQTTMission::RecElec rec_elec);
   void lockElecTargetId(const std::string target_ID,int kdxt_type); // 空地协同锁定 task_type = 12;
   private:
   void addHeader(QJsonObject& root); // 加入报文头 SID TM
   void ReSetMission(); // 重置任务
   bool CreatorMission(const QJsonObject& task,Mission&  mission); // 2-1任务规划协议组装
   bool PackageTaskAndZB(QString &s, QJsonObject &root);
   bool PackageMission(const QJsonObject& task,QJsonObject& root); // 2-1任务规划请求协议打包

   //山猫车基于目标打击，不需要2-1进行任务规划，操控需要平台信息，这里打包实现
   void CreateorShanmaoMissionPlan(MQTTMission::MissionPlan& missionPlan); //2山猫车打包平台信息
   void CreateorMissionPlan(const QJsonObject& task,MQTTMission::MissionPlan& missionPlan); //2-1任务规划结果

   void PackageMissionPlan_ToCK(const QJsonObject& json ,QJsonObject& root);  //指控对操控的作战任务计划103
   void PackageMissionBattle_ToCK(const QJsonObject& json ,const MissionPlan& missionplan,QJsonObject& root);  //指控对操控的作战平台计划104
   void packageImmediateIntervention(const MQTTMission::MissionPlan missionplan,int task_set_flag,QJsonObject& json); //临机干预， 手机电磁
   // signals:
   //  void mySignal(const QString topic, QJsonObject root);
   // public slots:
   //  void mySlot(const QString topic, QJsonObject root);

private:
   /**
    *协议校验规则： optional 和 repeated 均需要校验，其他不校验
    ***/
   // void aa(const QString &topic, QJsonObject& root);
   void Publish_101( QJsonObject root); //指控对操控的直播拉流请求
   void Publish_102(); //指控对操控的直播拉流请求
   void Subscribe_rwgh_901(const QJsonObject& root); //获取作战任务规划
   void Subscribe_llm_excute(const QJsonObject& root); //无人车行动控制协议
   void Subscribe_bltj_905(const QJsonObject& root); //兵力推荐集905
   void Subscribe_dzmbqb_301(const QJsonObject& root); //获取电子目标情报
   void Subscribe_dzmbxsb_302(const QJsonObject& root); //获取电子目标消失报
   void Subscribe_dzmbxsb_304(const QJsonObject& root); //获取装备消失报
   void Subscribe_zzptjzbzt_303(const QJsonObject& root); //作战平台及装备状态
   void Subscribe_zzptjzbzt_308(const QJsonObject& root); //重置任务流 长沙
   void Subscribe_rwzxzt_501(const QJsonObject& root); //操控反馈任务执行状态
   void Subscribe_rwzxzt_502(const QJsonObject& root); //操控反馈任务接收状态
   void Subscribe_CK504(const QJsonObject& root); //接收视频流

   void subscribe_dt_801(const QJsonObject& root); //作战目标
   void subscribe_dt_802(const QJsonObject& root); //下发导调
   void subscribe_dt_803(const QJsonObject& root); //巡飞弹

    bool parseOwnPlatformFromJson(const QJsonObject& root,OwnPlatform& ownPlatform); //解析装备状态
   // 解析JSON数据
   bool parseTargetlatformFromJson(const QJsonObject& root,Targetlatform& targetlatform);  //解析电子目标情报

   void Subscribe_LJ(const QJsonObject& root); //test 临机决策
   void Subscribe_ZT(const QJsonObject& root); //test 任务状态反馈
   void Subscribe_push(const QJsonObject& root); //接收态势个性化推送
   void Subscribe_prediction(const QJsonObject& root); //接收态势预测
   void Subscribe_KM6(const QJsonObject& root); //科目6测试、验证;
   void Subscribe_CK530(const QJsonObject& root); //科目 电磁验证; 接收CK/530
   void Subscribe_531(const QJsonObject& root); //电磁目标锁定请求，从机
   void Subscribe_TY103(const QJsonObject& root); // 接收2-3推演结果/703
   void Subscribe_TY104(const QJsonObject& root); // 接收2-3TY/704
   void Subscribe_human_location(const QJsonObject& root);
   void Subscribe_CK532(const QJsonObject& root); //科目 电磁验证; 接收CK/532
   void Subscribe_CK534(const QJsonObject& root); //科目 侦察干扰-目标列表上报; 接收CK/534
   void Subscribe_CK535(const QJsonObject& root); //科目 侦察干扰-精确目标上报; 接收CK/535
   void Subscribe_CK536(const QJsonObject& root); //科目 侦察干扰-干扰结果上报; 接收CK/536
   void Subscribe_CK537(const QJsonObject& root); //科目 导航干扰-消息上报; 接收CK/537

   void GetConnTargetAndPlatform(std::vector<string>& target_list,std::vector<string>& platfor_list);//军事博弈相碰的红蓝双方
   void  SHowTeXiao(string model,double lon,double lat);
   void Subscribe_LocationTarget(const QJsonObject& root); //指控下发目标定位报文
    void Subscribe_ZKtest(const QJsonObject& root); /*test */
   bool ControlDJiMini4_Pro(QString rid,double executeHeight,double waypointSpeed,std::vector<MQTTMission::Position>& path); // rid 写入装备的身份编号，控制大疆mini4_pro无人机
    bool isSHow();
   bool checkAnyDifference(const std::vector<double>& vec, double minDiff);
    void updata_zyc_name(std::string model_id,string name);
   bool hasId(const std::vector<zy>& vec, const std::string& mode_id);
private:
    void CreateorTrackLine(std::vector<MQTTMission::Position>& route); //任务规划路线和临机干预路线绘制
    void dczg_result(string result_str);
    void dcyd_result(string result_str); //电磁移动结果输出
    void loadVideoStream(VideoStreamList& vslist);
    void schemeEndPlay();
    void schemeStartPlay();
    void packgeEffect(); //特效准备
    bool isIdentifyTarget(APIBridgeSDK::OwnPlatform& ownplatform,QString task_type,QString target_ID);
    void publishImageTopic();

    void showThreatHemisphere_301(string model_id);
    void showThreatHemisphere_303(string model_id);
    void removeThreatHemisphere(string model_id);
    void CreatorTemple();
    bool jsbyWeightedAverage(); //军事博弈维度系数加加权平均法
    bool getJsbyParamValue(bool istarget,string id,std::string param,double& value); //敌我双发实际值获取
    std::vector<std::string> splitString(const std::string& input, char delimiter);// 将字符串按指定分隔符分割的函数

    void reSetJSBY();
private:
   // string currentTime();
   // int64_t currentTimeMillisec();

bool _isShowThreatHemisphere = false;
std::vector<std::string> _BY_target_dis_list;// 军事博中的缓存的所有目标目标
std::vector<std::string> _BY_platform_dis_list;// 军事博中缓存的所有装备的信息
std::vector<std::string> _BY_target_delete_list; //军事博弈本轮彻底删除的目标
std::vector<std::string> _BY_platform_delete_list; //军事博弈本轮彻底删除的装备
private:
QString video_battle_unit_code;
GEO_ALGO _geo_algo;
Loggers logger;
int sn = 0;
bool _r1,_r2,_r3,_r4;
//bool error_pg = false;
std::chrono::steady_clock::time_point last_time;
QJsonObject _root_rwgh; //任务规划请求包装
MQTTMission::MissionControl _mission_control;
std::mutex _mtx;
std::map<std::string, QString> _platform_running_status_by_id;
JSBY_Target1 _jsby_current_pos;
bool isf = false;
MQTTMission::PhoneElec _phoneElec;
std::vector<APIBridgeSDK::ImsiZGInfo> _recElec534; //电磁zg列表
MQTTMission::RecEleclocation _recElecLocation;
MQTTMission::MissionCK536 _mission_CK536;
string _imsi; //电磁任务，锁定手机imsi号
//int i =0;
MQTTMission::ZgStatus _zg_status;
string _imsi_list;
std::vector<string> _task_plane_roate_uuid_list;// 单个任务 存储当前路径UUID
std::vector<MQTTMission::Position> _must_pass_position_list; //任务必经点位置序列
std::vector<MQTTMission::Position> _td_target_position_list; //投弹任务23目标点列表
string _zg_target_list;
bool _plan_request_flag = false;
string _user_id; //态势推送用户ID
GKDCommonCfg* _gkd_cfg;
int effectIndex = 0; // 计数
std::shared_ptr <MqClient> _pgkd_mqtt_client;
std::shared_ptr <rest_rcp_client> _prcp_client;

 int _jsby_isEnd = 0; // 本轮军事博弈结束
std::map<std::string,std::vector<JSBY_Param> > _jsby_target_value_map_1;// 计算参数值原始值存储，敌方
std::map<std::string,std::vector<JSBY_Param> > _jsby_platform_value_map_1;// 计算参数值原始值存储，我方

std::map<std::string,std::vector<JSBY_Param>> _jsby_target_value_map;// 计算参数值存储，敌方
 std::map<std::string,std::vector<JSBY_Param>> _jsby_platform_value_map;// 计算参数值存储，我方
std::map<std::string,double> _sum_target_weight‌; //总权重敌方
std::map<std::string,double> _sum_platform_weight‌; //总权重我方

std::map<std::string,std::vector<double>> _weight‌_cos_target_map; //维度系数 敌方
std::map<std::string,std::vector<double>> _weight‌_cos_platform_map; //维度系数 我方

std::map<std::string,double> _weight‌_cos_target_sum_map; //单车博弈胜算系数 敌方，
std::map<std::string,double> _weight‌_cos_platform_sum_map; //单车博弈胜算系数 我方
// std::vector<JSBY_Param> _jsby_target_value_list;
// std::vector<JSBY_Param> _jsby_platform_value_list;


///////////////大模型相关参数/////////////////////////////////////////
 QJsonObject _llm_excute; //行动信息
int s301Index = 0;
int s303Index = 0;
int task_execute_status = 0;
QString find_target_id;
bool is_find_target = false;
string find_equip_id; //侦查装备id
bool is_attack_target = false;
int attack_index =0;
bool is_stop_attack_equip = false;
string attack_equip_id; //打击装备id
bool is_publish_targetImg = false;

std::vector<MQTTMission::Position> _mini4_pro_path; //大疆mini4无人机飞行路线
TaskModelbehavior _TaskModelbehavior;
DtTaskTarget    _dtTaskTarget;
DTControl          _dtControl; // 导调控制类
double speed = 8;

int  particleId_index = 100;
std::vector<string> particleId_set;

};
#endif // C2Manager_H
