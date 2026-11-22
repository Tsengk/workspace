
#pragma once
#include "Vector3Ex.h"
#include <string>
#include <vector>
using namespace std;

struct RTTaskCommandItemEx
{
    int iTaskID;
    string strCommandSourceGUID;  //记录谁发送的消息，查询的时候也是只返回自己发送的和查询全部通过这个区分
    string strTaskCommand;  //当前正在执行的指令Key的列表，这里一定是在注册信息中的指令列表中的指令
    int iTaskProcess;  //当前正在执行的指令的进度，0-100，一个指令对应一个
    bool bNotice=false;//是否上报 true或iTaskProcess=100 都会上报
    string strCurDesc;//当前描述信息 用户自定义
};
struct RTTaskCommandListEx
{
    vector<RTTaskCommandItemEx> vecTaskCommandList;
};

struct RTHealthStatusItemEx
{
    int   iHealthCode;  //  设备健康码，0 代表无已知问题，具体含义的设定，每个设备不一样，这是一个开放接口， 错误码100以上的认为是需要立即反馈的事件类型
    string  strHealthDescription;  //问题描述
    int iTaskID;  //不是任务相关的就用-1
    string strSourceGUID;
};

struct RTHealthStatusListEx
{
    vector<RTHealthStatusItemEx> vctHealthStatusList;
};

struct RTConnectionItemEx
{
    string strGUID;
    int iConType;   // -1 接收指令的 和 1 发送指令的
    string strDescription;  
};

struct RTConnectionListEx
{
    vector<RTConnectionItemEx> vecConnCollection;
};

struct RealTimeStatusDataEx
{
    long long timeStamp;
    double dLongitude; // 经度，
    double dLatitude;    // 纬度， 
    double dHeight;    // 高度 ，高度为海平面绝对高度
    double dPosUTMX;    // UTM坐标系下的位置X
    double dPosUTMY;     //UTM坐标系下的位置Y
    double dPosHeight;   //距离地面的高度
    Vector3Ex v3Posture;   //x roll, 参照车辆坐标系，设备正前方为正半轴，向右旋转为正， y pitch, 设备左方为正半轴，向下旋转为正， z heading，世界坐标系，东向北为正
                         // 单位都是角度，取值范围(-180, 180]
    Vector3Ex v3Velocity;  // 世界坐标系下的三轴速度，世界坐标系，正东为x轴正，正北为y轴正，向上为z轴正
    Vector3Ex v3Acc;       // 世界坐标系下的三轴加速度
    float fSpeed;       // 标量速度值，heading方向（径向）的
    float fAccel;       // 标量加速度值，heading方向（径向）的
    bool  bHealth;      // 设备是否工作正常
    RTHealthStatusListEx curHealthStatusCollection;
    bool bTaskExec;    // 是否当前有正在执行的指令   
    RTTaskCommandListEx curTaskCommandCollection;
    
    RTConnectionListEx curConnectionCollection;  // 当前逻辑连接信息 

    string strOther;     //其它信息，用Json
};
