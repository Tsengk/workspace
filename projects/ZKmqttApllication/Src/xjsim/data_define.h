#ifndef DATA_DEFINE_H
#define DATA_DEFINE_H
#include "qobject.h"
#include <iostream>
#include <xTakLib.h>
#include "common/common_config.h"
#include <utils/json.hpp>

// 错误码定义
enum class ErrorCode {
    SUCCESS = 0,                  // 成功
    ERR = 1000,                // 错误
    TASK_RUNNING = 1006,         // 任务运行中
    UNKNOWN_ERROR = 9999         // 未知错误
};

//响应结构体
struct ApiResponse {
    bool success{false};                 // 是否成功
    ErrorCode code{ErrorCode::SUCCESS};  // 错误码
    std::string message;                 // 错误描述
    std::string sn;                      // 序号
    std::string topic;                   // 请求的topic，
    std::string cmd;                     // 请求的cmd
    std::string data;                    // 响应数据(JSON字符串)
    
    // 构造函数
    ApiResponse() = default;
    
    // 成功响应的快捷构造
    ApiResponse Success(const std::string& data = "") {
        ApiResponse response;
        response.sn = this->sn;
        response.cmd = this->cmd;
        response.topic = this->topic;
        response.success = true;
        response.code = ErrorCode::SUCCESS;
        response.message = "操作成功";
        response.data = data;
        return response;
    }
    
    // 失败响应的快捷构造
    ApiResponse Failure(ErrorCode code, const std::string& msg) {
        ApiResponse response;
        response.sn = this->sn;
        response.cmd = this->cmd;
        response.topic = this->topic;
        response.success = false;
        response.code = code;
        response.message = msg;
        return response;
    }
    
    // 转换为JSON字符串
    std::string toJson() const {
        nlohmann::json j;
        j["success"] = success;
        j["code"] = static_cast<int>(code);
        j["message"] = message;
        j["data"] = data;
        j["sn"] = sn;
        j["topic"] = topic;
        j["cmd"] = cmd;
        return j.dump();
    }
};

struct SimObject   //仿真数据实体
{
    double utm_x;
    double utm_y;
    double utm_z;
    double lat = 0;
    double lon = 0;
    double height = 0;
    double heading = 0;
    double roll=0;
    double pitch=0;
    int type;//类型
    string name;//名字
    bool isZc;//是否作为侦察机
    int isOnline = 0;//是否正常获取设备信息
    string uid="";//uid
    bool isTasking = false; //是否正在执行任务
    float speed;       // 标量速度值，heading方向（径向）的
    float accel;       // 标量加速度值，heading方向（径向）的
    bool  health = false;      // 设备是否工作正常
    bool isWorking = false;
//    double run_utm_x;//自主任务开始那一刻的x值
//    double run_utm_y;//自主任务开始那一刻的y值
//    double run_utm_z;//自主任务开始那一刻的z值
    int exCmdTimes = 0;

};
// 命令类型枚举
enum class CommandType {
    START_UAV,
    STOP_UAV,
    RESET_CMD,
    OPEN_CABIN,
    CLOSE_CABIN,
    STOP_FPV,
    RELEASE_FPV,
    RUN
};

// 移动方向枚举
enum class MoveDirection {
    LEFT = 1,
    RIGHT = 2,
    TOP = 3,
    BOTTOM = 4
};

// 命令类型枚举
enum class ExCommandType {
    EX_NONE,
    EX_UAV,
    EX_UAV_RUN,
    EX_UAV_RUNNING,
    EX_FPV_OPEN,
    EX_FPV,
    EX_FPV_RUN,
    EX_FPV_END,
    EX_FPV_CLOSE
};

struct ExTask//当前执行指令的实体
{
    ExCommandType ex_cmd = ExCommandType::EX_NONE;//执行的命令
};
struct ExCmd//当前执行指令的实体
{
    string guid;//
    int cmd_id;//
    CommandSetDataEx cmdData;
    bool isSendSuccess;//是否发送成功
    int sendTimes = 0;
    string strCmdKey;
    int times = 2;//用于延时处理
    string uav_fly_height = "300";//无人机飞行的高度
    ExCommandType cmd_type = ExCommandType::EX_NONE;
    bool isNeedShowTargent{false};//是否需要显示目标
    bool isNeedTj{false};//敌方车辆是否需要突击
    bool isNeedDj{false};//敌方车辆是否需要Dj
    bool isWorking;//是否在执行任务
    std::vector<ExTask> task_list;
    int isOpenCabin=0;//1:打开 2:关闭
};

#endif // DATA_DEFINE_H

