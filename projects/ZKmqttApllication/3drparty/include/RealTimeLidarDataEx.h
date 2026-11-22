#pragma once
#include <string>
#include <vector>
using namespace std;
struct lidarPointEx
{
	float fPosX;
	float fPosY;
	float fPosZ;
	float fReflect;
};
//Lidar 的结果数据
struct RealTimeLidarDataEx
{
	long long timeStamp;
	int iPointCount;
	vector<lidarPointEx> vecLidarPointList;
};