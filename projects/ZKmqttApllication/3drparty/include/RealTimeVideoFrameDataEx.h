#pragma once 
#include <vector>
using namespace std;

struct RealTimeVideoFrameDataEx 
{
	long long timeStamp;
	uint32_t iWidth;
	uint32_t iHeight;
	uint32_t iFPS;
	double dDTS;
	double dPTS;
	uint32_t iSSR;
	vector<uint8_t> vecFrameData;
	int iType;
};