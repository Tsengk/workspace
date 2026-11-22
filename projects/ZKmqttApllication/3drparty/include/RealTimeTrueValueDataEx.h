#pragma once
#include "Vector3Ex.h"
#include <string>
#include <vector>
using namespace std;
struct trueValueItemEx
{
    int iObjType;   // 0 车， 1 无人机，2 人， 3 动物，4其它
    Vector3Ex v3Position;   //在车辆坐标系下的相对位置
    Vector3Ex v3Size;       //目标的大小
    Vector3Ex v3Posture;   //x roll, 参照车辆坐标系，设备正前方为正半轴，向右旋转为正， y pitch, 设备左方为正半轴，向下旋转为正， z heading，车辆坐标系下的相对朝向
                         // 单位都是角度，取值范围(-180, 180]
    Vector3Ex v3Velocity;  // 车辆坐标系下的相对速度
    Vector3Ex v3Acc;       // 车辆坐标系下的相对加速度
    float fSpeed;       // 标量速度值，heading方向（径向）的
    float fAccel;       // 标量加速度值，heading方向（径向）的
    vector<Vector3Ex> cellList;   //目标的轮廓边缘点，车辆坐标系下的点
};

//TrueValue 的结果数据
struct RealTimeTrueValueDataEx
{
    long long timeStamp;
    int iObjCount;
    vector<trueValueItemEx> vecTrueValueItemList;
};