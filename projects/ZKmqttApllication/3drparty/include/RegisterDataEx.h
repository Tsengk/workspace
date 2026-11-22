#pragma once
#include "Vector3Ex.h"
#include <string>
#include <vector>
using namespace std;
// 设备的基本信息，静态的描述
struct DeviceBaseEx
{
	string strName;
	int iDeviceType;   //-2 纯传感器, -1, UI(不具备智能体功能发送传感数据和状态), 0 车，1 无人机， 2 单兵（人）， 3 单兵（机器狗）
	float fLength;  //设备前后的距离
	float fWidth;   //设备左右的距离
	float fHeight;  //高度
	float fWeight;  //重量
	string strProductionDate;  //生产日期
	string strFactory;  //生产厂家
	string strDescription;   //描述信息
	string strOSName;     //设备操作系统的名字
	string strBoardInfo;  //主板信息
	string strOther;       //其它信息，用Json格式
};

struct CommandParamentEx   //这是一个参数的描述
{
	string strName;           // 参数名字
	string strDescription;    //解释参数含义
	int iType;    //基础类型，只能是0 int, 1 float ,2 bool,3 double，4 string 
	bool isRepeat; //此参数是否可重复，也就是不一定长的意思
};

struct CommandItemEx //一个指令的描述
{
	string strCmdKey;   //指令名称
	string strDescription;
	int iFixedParaCount;  //固定的参数个数，放在前面的，后面的可以是不定长，一个指令只能有一个不定长的参数，并且是放在最后的
	                      //如果有两个不定长的，指令分开！！
	vector<CommandParamentEx> vecParaList;
};

struct SensorLiderEx
{
	string strName;   //在一台设备商不能重复
	string strProductionDate;
	string strDescription;
	string strOther;
	Vector3Ex v3Position;  //安装外参
	Vector3Ex v3Posture;

	float dFar; //有效监测距离
	int iHeightCount; //线束
	float fFovX;   //水平观察角度
	float fFovY;  //垂直观察角度
	float fHoriInterval;   //水平分辨率
	float fFarInterval; //距离分辨率
};

struct SensorImageEx
{
	string strName;   //在一台设备商不能重复
	string strProductionDate;
	string strDescription;
	string strOther;
	Vector3Ex v3Position;  //安装外参
	Vector3Ex v3Posture;

	float fFovX;  //水平观察角度
	float fFovY;  //垂直观察角度
	float fFocus;  //焦距
	int iXPixel;    //输出结果横向像素数
	int iYPixel;    //输出结果竖向像素数

	bool isSupportVideo;
	string strVideoURL;
};

struct SensorTrueValueEx  //描述一个传感器
{
	string strName;     //在一台设备商不能重复
	string strOther;
};

struct SensorInfoEx
{
	vector<SensorLiderEx> vecLidarList;
	vector<SensorImageEx> vecImageList;
	vector<SensorTrueValueEx> vecTrueValueList;
};

struct CommandInfoEx
{
	vector<CommandItemEx> vecCommandList;
};

struct RegisterDataEx
{
	long long timeStamp;
	string strCustormName; //用来过滤逻辑分组
	int iPermission;      // 默认赋0，以后实现再说
	int iLevel;  // 0优先级最高，越往上权利越小
	string strGUID;   //系统生成的，独一无二的
	DeviceBaseEx objDeviceBase;
	SensorInfoEx sensorInfo;
	CommandInfoEx commandInfo;
};

struct DeviceChangedDataEx
{
	string strDeviceGUID;  //设备的GUID
	bool isOn;     //上线还是下线
};

struct DeviceItemEx
{
	string strName;
	string strGUID;
	string strCustormName; //用来过滤逻辑分组
	int iType;
};

struct DeviceItemListEx
{
	vector<DeviceItemEx> vctDeviceCollection;
};