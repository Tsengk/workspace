#pragma once
#include <string>
#include <vector>
using namespace std;
struct imageColorEx
{
	uint8_t cR;
	uint8_t cG;
	uint8_t cB;
};
//Image 的结果数据
struct RealTimeImageDataEx
{
	long long timeStamp;
	int iWidth;
	int iHeight;
	vector<imageColorEx> vecImageColorList;
	int iColorCount;
};