#ifndef UAVSTATUSDEFINE_H
#define UAVSTATUSDEFINE_H
#pragma warning (disable:4819)

#include <QString>
#include <QObject>

// 网络状态结构
struct NetworkState {
    int type{0};
    int quality{0};
    float rate{0.0f};
};

// 充电状态结构
struct DroneChargeState {
    int state{0};
    int capacity_percent{0};
};

// 备用降落点结构
struct AlternateLandPoint {
    double latitude{0.0};
    double longitude{0.0};
    double safe_land_height{0.0};
    int is_configured{0};
};

// 位置状态结构
struct PositionState {
    int is_calibration{0};
    int is_fixed{0};
    int quality{0};
    int gps_number{0};
    int rtk_number{0};
};

// 存储状态结构
struct Storage {
    qint64 total{0};
    qint64 used{0};
};

// 子设备结构
struct SubDevice {
    QString device_sn;
    QString device_model_key;
    int device_online_status{-1};
    int device_paired{0};
};


//飞行数据
struct plannedpathpoint
{
    double lat;
    double lon;
    float  height;
};

struct flyResult
{
    QString fly_to_id;
    QString status;    //如果是飞向目标点接口：{"wayline_cancel":"取消飞向目标点","wayline_failed":"执行失败","wayline_ok":"执行成功，已飞向目标点","wayline_progress":"执行中"}
    //如果是一键起飞的状态 {"task_finish":"一键起飞任务完成","task_ready":"准备起飞","wayline_cancel":"取消飞向目标点","wayline_failed":"执行失败","wayline_ok":"执行成功，已飞向目标点","wayline_progress":"执行中"}
    int     result;  //值行结果，0 是成功  非0是失败
    int     way_point_index;
    float   remaining_distance;  //剩余任务距离
    float   remaining_time;      //剩余任务时间
    QString method;   //fly_to_point_progress 飞向目标点 // takeoff_to_point_progress 一键起飞
    plannedpathpoint info;
    flyResult() {
        fly_to_id = "";
        status = "";
    }
};
struct FlyManager
{
    QString fly_to_id;
    QString status;
    QString method;
};



typedef struct DJUavStatus{
    double lon;
    double lat;
    float elevation;
    int heading;
    float pitch;
    float roll;
    DJUavStatus(){
        lon = 0;
        lat = 0;
        elevation = 0;
        heading = 0;
        pitch = 0;
        roll = 0;
    }
}DJUAV;
// 主数据结构
class DroneData : public QObject
{
    Q_OBJECT

public:
    explicit DroneData(QObject *parent = nullptr){

    }
    ~DroneData() = default;

    // 基本信息
    QString tid;
    QString bid;
    qint64 timestamp{0};
    QString gateway;

    // 复合数据结构
    NetworkState network_state;
    DroneChargeState drone_charge_state;
    AlternateLandPoint alternate_land_point;
    PositionState position_state;
    Storage storage;
    SubDevice sub_device;

    // 基本状态数据
    int drone_in_dock{-1};
    int rainfall{0};
    int wind_speed{0};
    int environment_temperature{0};
    double temperature{0.0};
    int humidity{0};
    double latitude{0.0};
    double longitude{0.0};
    double height{0.0};
    qint64 first_power_on{0};
    int mode_code{0};
    int cover_state{0};
    int supplement_light_state{0};
    int emergency_stop_state{0};
    int air_conditioner_mode{0};
    int battery_store_mode{0};
    int alarm_state{0};
    int putter_state{0};


    flyResult fly_result_data;


 //   RawData  uav_status_data;
signals:
    void dataChanged();
    void batteryLevelChanged(int level);
    void positionChanged(double lat, double lon);
    void temperatureChanged(double temp);
};

#endif // UAVSTATUSDEFINE_H
