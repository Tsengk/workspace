#ifndef TASKRESPONSEDEFINE_H
#define TASKRESPONSEDEFINE_H
#include "../common/common.hpp"
#include "gkd/mission_control.h"
#include<functional>
#include<QJsonDocument>
#include <sys/timeb.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace std;
// //地理坐标
// struct Position{
//   double lon;
//   double lat;
//   double z;
//   Position(double lon_,double lat_,double z_){
//       lon = lon_;
//       lat = lat_;
//       z = z_;
//   }
//   MSGPACK_DEFINE(lon,lat,z)
// };

// //区域
// struct Area{
//     string area_name;
//     std::vector<Position> area_list;  //点序列不需要闭合
//     MSGPACK_DEFINE(area_name,area_list)
// };

// //任务结构
// struct TaskPlan {
//     string task_id; // 任务编号
//     string  mission_data; //任务内容
//     string task_batch; //任务下发批次，保留字段
//     string task_name; // 任务名称
//     string task_start_time;  //任务开始时间
//     string task_end_time;  //任务结束时间
//     string task_type; //任务类型
//     int target_ID; // 目标id，optional
//     Position target_position; // 目标位置,optional
//     Area    area;// 目标位置,optional
//    MSGPACK_DEFINE(task_id,task_batch,task_name,task_start_time,task_end_time, task_type, target_ID,target_position,area)

//    bool verificate() const{  //根据任务类型校验请求参数合法性
//        if(task_id =="" ||
//           task_start_time =="" ||
//           task_end_time == "" ) return false;

//        if(task_type == "11"){

//            return true;
//        }else if(task_type =="2"){
//           return true;
//        }
//        std::cout<<"任务请求参数失败,任务类型为"<<task_type<<std::endl;
//        return true;
//    }

//    void packageJson( QJsonObject &root)
//    {
//        //正确方式： toJsonObject
//        QJsonObject ss;
//        ss.insert("task_id",QString::fromStdString(task_id));
//        ss.insert("task_name",QString::fromStdString(task_id));

//        root = {
//                {"SID", "ZK"},
//                {"TM", 1},
//                {"CTN", ss}};
//        qDebug()<<QString(QJsonDocument(root).toJson());
//    }


// };



//任务执行结构
struct DoTask {
    string task_id; // 任务编号
    string task_batch; //任务下发批次，保留字段
    string task_type; //任务类型
 MSGPACK_DEFINE(task_id,task_batch, task_type)
    bool verificate() const{

        return true;
    }

    void packageJson( QJsonObject& root) const {
        //正确方式： toJsonObject
        QJsonObject ss;
        ss.insert("task_id",QString::fromStdString(task_id));

        root = {
                {"SID", "ZK"},
                {"TM", 1},
                {"CTN", ss}};

    }
};
typedef std::function<void(string_view& root)> TPCallback;

//// 网络状态结构
//struct NetworkState {
//    int type{0};
//    int quality{0};
//    float rate{0.0f};
//};

//// 充电状态结构
//struct DroneChargeState {
//    int state{0};
//    int capacity_percent{0};
//};

//// 备用降落点结构
//struct AlternateLandPoint {
//    double latitude{0.0};
//    double longitude{0.0};
//    double safe_land_height{0.0};
//    int is_configured{0};
//};

//// 位置状态结构
//struct PositionState {
//    int is_calibration{0};
//    int is_fixed{0};
//    int quality{0};
//    int gps_number{0};
//    int rtk_number{0};
//};

//// 存储状态结构
//struct Storage {
//    qint64 total{0};
//    qint64 used{0};
//};

//// 子设备结构
//struct SubDevice {
//    QString device_sn;
//    QString device_model_key;
//    int device_online_status{-1};
//    int device_paired{0};
//};


////飞行数据
//struct plannedpathpoint
//{
//    double lat;
//    double lon;
//    float  height;
//};

//struct flyResult
//{
//    QString fly_to_id;
//    QString status;    //如果是飞向目标点接口：{"wayline_cancel":"取消飞向目标点","wayline_failed":"执行失败","wayline_ok":"执行成功，已飞向目标点","wayline_progress":"执行中"}
//    //如果是一键起飞的状态 {"task_finish":"一键起飞任务完成","task_ready":"准备起飞","wayline_cancel":"取消飞向目标点","wayline_failed":"执行失败","wayline_ok":"执行成功，已飞向目标点","wayline_progress":"执行中"}
//    int     result;  //值行结果，0 是成功  非0是失败
//    int     way_point_index;
//    float   remaining_distance;  //剩余任务距离
//    float   remaining_time;      //剩余任务时间
//    QString method;   //fly_to_point_progress 飞向目标点 // takeoff_to_point_progress 一键起飞
//    plannedpathpoint info;
//    flyResult() {
//        fly_to_id = "";
//        status = "";
//    }
//};
//struct FlyManager
//{
//    QString fly_to_id;
//    QString status;
//    QString method;
//};



//typedef struct DJUavStatus{
//    double lon;
//    double lat;
//    float elevation;
//    int heading;
//    float pitch;
//    float roll;
//    DJUavStatus(){
//        lon = 0;
//        lat = 0;
//        elevation = 0;
//        heading = 0;
//        pitch = 0;
//        roll = 0;
//    }
//}DJUAV;
//// 主数据结构
//class DroneData : public QObject
//{
//    Q_OBJECT

//public:
//    explicit DroneData(QObject *parent = nullptr){

//    }
//    ~DroneData() = default;

//    // 基本信息
//    QString tid;
//    QString bid;
//    qint64 timestamp{0};
//    QString gateway;

//    // 复合数据结构
//    NetworkState network_state;
//    DroneChargeState drone_charge_state;
//    AlternateLandPoint alternate_land_point;
//    PositionState position_state;
//    Storage storage;
//    SubDevice sub_device;

//    // 基本状态数据
//    int drone_in_dock{-1};
//    int rainfall{0};
//    int wind_speed{0};
//    int environment_temperature{0};
//    double temperature{0.0};
//    int humidity{0};
//    double latitude{0.0};
//    double longitude{0.0};
//    double height{0.0};
//    qint64 first_power_on{0};
//    int mode_code{0};
//    int cover_state{0};
//    int supplement_light_state{0};
//    int emergency_stop_state{0};
//    int air_conditioner_mode{0};
//    int battery_store_mode{0};
//    int alarm_state{0};
//    int putter_state{0};


//    flyResult fly_result_data;

//    RawData  uav_status_data;
//signals:
//    void dataChanged();
//    void batteryLevelChanged(int level);
//    void positionChanged(double lat, double lon);
//    void temperatureChanged(double temp);
//};

#endif // TASKRESPONSEDEFINE_H
