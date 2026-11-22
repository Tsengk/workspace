#pragma once
#include <string>
#include <vector>
using namespace std;
struct CommandParamentSetEx   //这是一个参数的描述
{
	string strName;           // 参数名字
	int iType;    //基础类型，只能是0 int, 1 float ,2 bool,3 double，4 string 
	string strValue;  //参数值，根据上面的类型进行转换
};

struct CommandSetDataEx //一个指令的描述
{
	string strCmdKey;   //指令名称
	string strCommandSourceGUID;   //由下层的通讯赋值
	int iParaCount;
	vector<CommandParamentSetEx> vecParaList;
};