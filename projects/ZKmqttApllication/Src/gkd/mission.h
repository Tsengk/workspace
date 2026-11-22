#ifndef Mission_H
#define Mission_H
#include <QJsonObject>
#include<QJsonArray>
#include <vector>
#include <string>
#include <cstdint>

// 前向声明
class QJsonObject;

/**
*现场联调管理类
*by hqw
**/
struct mosquitto;  // 与mosquitto库保持一致
namespace MQTTMission{
using namespace std;

//节点任务规划信息
struct WaypointAction
{

    WaypointAction()
    {
        // is_hover = false;
        // hover_time = 0.0;
        // is_drop = false;
        // drop_number = 0;
    }

    // WaypointAction (bool ih, double ht,bool isd,int dn){
    //     is_hover = ih;
    //     hover_time = ht;
    //     is_drop = isd;
    //     drop_number = dn;
    // }
    // void toJsonObject(QJsonObject& obj){
    //     obj["is_hover"] = is_hover;
    //     obj["hover_time"] = hover_time;
    //     obj["is_drop"] = is_drop;
    //     obj["drop_number"] = drop_number;
    // }
    // bool is_hover; // 是否悬停
    // double hover_time; // 悬停时间
    // bool is_drop; //是否投弹
    // int drop_number; // 投弹个数
    QString  key;
    QJsonValue value;
};

class Position{
public:
    Position() : longitude(0.0f), latitude(0.0f), altitude(0.0f),speed(0.0f) {}
    Position(double lon,double lat ,double al,double sp){
        longitude = lon;
        latitude = lat;
        altitude = al;
        speed = sp;
    }
    double longitude;
    double latitude;
    double altitude;
    //速度
    double speed;
    //航点动作
    QJsonArray  action_list;
};
//选中目标信息
struct selectFeatureInfo{
    string _select_id;
    Position _position;
};
/***4.2.15  航线/路径点(复合结构)**/
struct TrackPoint{
    int64_t current_time;
    Position track_point_position;
    float velocity;
};
/***4.2.164.2.16  航线/路径(复合结构)****/
struct TrackLine{
    string trackline_name;
    int64_t start_time;
    int64_t end_time;
    std::vector<TrackPoint> track_point_list;
    float min_height;
    float max_height;
    int input_type;
    float road_width;
    int64_t edit_time;
    string description;
    Position init_position;
    Position end_position;

};



// 任务载荷状态
// struct EquipmentState {
//     std::string equip_no;                 // 载荷编号
//     uint8_t equip_redidual_electricity;   // 载荷剩余电量 (0-100%)
// };

// 通用平台结构体
struct BasicPlatform {
    std::string model_id;                 // 作战单元编号
    int64_t report_time;                  // 上报时间 (UTC毫秒时间戳)
    int32_t platform_type;                //  仿真无人机: 1001,
        // 仿真无人车: 2001,
        // 三维建图无人机: 1003,
        // 二维建图无人机 : 1002,
        // 目标识别无人机 : 1004
        // 目标-人员:5001
        // 目标-军用车辆:5002
        //目标-民用车辆：5003
        // 目标-飞机：5004
        // ......
    Position deploy_position;             // 平台位置

    float velocity;                       // 速度 (km/h)
    float trk_direction;                  // 航向 (北偏东角度)

    // 姿态信息
    float roll;                           // 滚转角 (左负右正)
    float pitch;                          // 俯仰角 (前负后正)
    float yaw;                            // 偏航角 (右负左正)
};

// // 我方装备业务信息结构体
// struct OwnPlatformBusiness {
//     // 系统状态 (可选)
//     uint32_t health_status; // 0:正常,1:故障
//     std::string exception_msg; // 异常消息

//     // 能源状态 (可选)
//     uint8_t plat_redidual_electricity; // 平台剩余电量 (0-100%)

//     // 任务状态 (可选)
//     uint32_t task_execute_status; // 0:空闲,1:执行中
//     std::string task_number;     // 执行的任务编号

//     // 载荷状态列表
//     std::vector<EquipmentState> equip_state_list;


// };

// // 目标装备业务信息结构体
// struct TargetPlatformBusiness {
//     uint64_t collect_platform_id;  // 发现该目标的设备id
//     int32_t data_source;           // 侦察来源标识
//     int64_t first_time;            // 首次发现时间 (UTC毫秒时间戳)
//     int64_t latest_time;           // 最后发现时间 (UTC毫秒时间戳)

//     uint8_t space_type;            // 空间信息类型（1:航迹点, 2:测向线, 3:无空间信息）
//     uint32_t capture_count;        // 截获次数

//     int32_t scout_type;            // 侦察设备类型（字典值）
// };

// // 我方装备结构体
// struct OwnPlatform {
//     BasicPlatform basic;
//     OwnPlatformBusiness business;
//     // 解析JSON数据
//   //  bool parseFromJson(const QJsonObject& root);
// };

// // 目标装备结构体
// struct Targetlatform {
//     BasicPlatform basic;
//     TargetPlatformBusiness business;

// };
// //任务状态
// enum MissionStatus {
//     IDLE = 1,      // 空闲状态
//     PAUSED,    // 暂停状态
//     RUNNING    // 进行中状态
// };

struct TrackInfo{
public:
    Position position;
    double speed;
    double track_direction;
};


//对应协议101，直接转换为json请求

class Mission  {

public:
    Mission();
   ~Mission();

public:
   string task_id;
   string task_name;
   string task_start_time;
   string task_end_time;
  // string mission_batch;

   int  task_type;
   Position init_position;
   selectFeatureInfo _selectFeatureInfo;
   string description; //任务描述
   // int force_num;
   // std::vector<Position>route;
   // int target_ID;
   //MissionStatus mission_status;
};

//对应2-1任务规划结果 901
//对应单个兵力
class MissionPlan  {

public:
    MissionPlan();
    ~MissionPlan();

    int getModelId(); //force_id 映射model_id
    void create(const QJsonObject &root);
public:
    string task_id;
    string task_name;
    int  task_type;
    int  force_num;
    std::vector<Position>route;
    int target_ID;
    string battle_unit_code;
    string battle_unit_name;
    int64_t start_time;
    int64_t end_time;
    int32_t   auto_level;//自主登记
    int32_t priority;// 优先等级
    string result_code;
    //MissionStatus mission_status;
};


/***对应手机电磁目标信息***/
//电磁信号目标
struct PhoneSignalTarget{
    string plmn; // 运营商
    string imsi; // 手机编号
};
//电磁准确目标
struct PhoneExactTarget{
    string plmn; // 运营商
    string imsi; // 手机编号
    Position position; //目标位置
};
struct PhoneElec  {

public:
    PhoneElec();
    ~PhoneElec();

    //int getModelId(); //force_id 映射model_id
public:
    string task_id;
    string battle_unit_no;
    int elec_target_type;
    std::vector<PhoneSignalTarget> phoneSignalTarget_list;
    PhoneExactTarget phoneExactTarget;
};
//对应临机干预（ZK/105）
struct ImmediateIntervention{
    string task_id;
    int task_set_flag;
    string battle_unit_no;
    TrackLine planned_track;
    PhoneSignalTarget elec_target;  // 电磁侦查目标 task_set_flag 为5有效
    PhoneExactTarget elec_elect_target; //电磁准确目标 task_set_flag 为6 有效
    string task_set_desc;
    string target_ID;

};


//对应协议301电子目标情报

// class Intelligence{
// public:
//     Intelligence();
//     ~Intelligence();
    
//     // 解析JSON数据
//     bool parseFromJson(const QJsonObject& root);
    
// public:
//     int32_t target_ID;                    // 目标批号
//     std::string battle_unit_code;         // 作战单元编号（侦察来源）
//     int32_t data_source;                  // 情报信息来源 (optional)
//     int64_t first_time;                   // 首次时间
//     int64_t latest_time;                  // 最新时间
//     int32_t space_type;                   // 空间信息类型：1-航迹点，2-测向线，3-无空间信息
//     // Position track_info;               // 航迹信息 (optional)
//     // DOAInfo doa_info;                  // 测向信息 (optional)
//     int32_t capture_count;                // 截获次数
//     int32_t scout_type;                   // 侦察设备类型
//     int32_t target_plat_type;             // 作战目标类型
//     std::vector<float> freq_list;         // 频率列表，单位：MHz
//     uint32_t health_status;               // 健康状态：0-正常，1-故障
//     std::string exception_msg;            // 异常消息
//     TrackInfo  taack_info;               //航迹点
// };

//对应协议302电子目标消失报
class Intelligence_D{
public:
    Intelligence_D();
    ~Intelligence_D();
    
    // 解析JSON数据
    bool parseFromJson(const QJsonObject& root);
    
public:
    string target_ID;                    // 目标批号
    int32_t disappear_state;         // 消失状态：1-消失
};


//对应协议303作战平台及装备状态

// class Equiep{
// public:
//     Equiep();
//     ~Equiep();
    
//     // 解析JSON数据
//     bool parseFromJson(const QJsonObject& root);
    
// public:
//     int64_t report_time;                  // 上报时间
//     std::string battle_unit_code;         // 作战单元编号
//     int32_t platform_type;                // 作战平台类别：1-无人机，2-无人车，3-单兵
//     Position deploy_position;             // 平台位置
//     float velocity;                       // 速度，单位：km/h
//     float trk_direction;                  // 航向，北偏东的角度，单位：度
//     float roll;                           // 滚转角，单位：度 (optional)
//     float pitch;                          // 俯仰角，单位：度 (optional)
//     float yaw;                            // 偏航角，单位：度 (optional)
//     uint32_t health_status;               // 健康状态：0-正常，1-故障 (optional)
//     std::string exception_msg;            // 异常消息 (optional)
//     uint32_t plat_redidual_electricity;   // 平台剩余电量，百分比[0,100] (optional)
//     uint32_t task_execute_status;         // 任务执行状态：0-空闲，1-任务执行中
//     std::string task_number;              // 在执行的任务编号 (optional)
//     // std::vector<EquipState> equip_state_list; // 载荷状态列表
// };
//对应501任务执行状态
struct MissionExeStatus {
    string task_exec_progress;
    int task_status;
};

struct RecElec  {
public:
    RecElec();
    ~RecElec();
public:
    string task_id;
    string battle_unit_no;
    string target_id;
    double freq;
};

struct RecEleclocation  {
public:
    RecEleclocation();
    ~RecEleclocation();
public:
    string task_id;
    string battle_unit_no;
    string target_id;
    double longitude;
    double latitude;
};

struct MissionCK536 {
    string task_id;
    string battle_unit_no;
    int emi_status; //干扰评估结果状态 0 未侦测到信号，1干扰有效；2，干扰无效
    int emi_stage; //协调类型 0 表示抵进侦查，1空空协同，2空地协同阶段
};

struct ZgStatus {
    int rec_interference_global_stage = 0; //Z干任务阶段
    int rec_failed_count = 0; // 抵近侦察失败次数
    int kd_failed_count = 0; // 空地干扰阶段失败次数
    bool rec_target_flag = false; //是否发现目标
    MQTTMission::Position zg_master_uav_pos; //递近状况
    bool zg_kdxt_plane_error = false; //空地协同车规划不满足条件
    // 初始化函数
    void init() {
        rec_interference_global_stage = 0;
        rec_failed_count = 0;
        kd_failed_count = 0;
        rec_target_flag = false;
        zg_kdxt_plane_error = false;
        zg_master_uav_pos = MQTTMission::Position();
    }
};

}
#endif // Mission_H

