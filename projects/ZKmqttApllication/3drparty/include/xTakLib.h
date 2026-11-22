#pragma once
#include "include/RealTimeStatusDataEx.h"
#include "include/RegisterDataEx.h"
#include "include/RealTimeImageDataEx.h"
#include "include/RealTimeVideoFrameDataEx.h"
#include "include/RealTimeLidarDataEx.h"
#include "include/RealTimeTrueValueDataEx.h"
#include "include/CommandSetDataEx.h"

//extern "C" {
#ifdef _WIN32  
    #define EXPORT_API __declspec(dllexport)  
#else  
    #define EXPORT_API __attribute__((visibility("default")))  
#endif 

    EXPORT_API int Init();
    EXPORT_API void Finalization();
    //返回所有智能体的个数, 返回-1表示失败，此返回数值不包括自己
    EXPORT_API int GetObjCount();
    //返回所有智能体设备的列表
    EXPORT_API bool GetObjList(DeviceItemListEx* pDeviceList);
    //获取自己的编组名称
    EXPORT_API string GetSelfGroupName();
    //获取其他智能体的编组名称
    EXPORT_API string GetObjGroupName(string strObjGUID);
    //修改自己的编组名称
    EXPORT_API bool SetSelfGroupName(string strObjGroupName);
    //修改其他智能体的编组名称
    EXPORT_API bool SetObjGroupName(string strObjGUID,string strObjGroupName);

    //获取单个智能体的所有基础信息
    EXPORT_API bool GetObjAllBasicInfo(DeviceBaseEx* pDeviceBase, string strObjGUID);
    //获取单个智能体的尺寸
    EXPORT_API bool GetObjSize(Vector3Ex* pV3Size, string strObjGUID);
    //获取单个智能体的重量
    EXPORT_API float GetObjWeight(string strObjGUID);

    //获取单个智能体的所有状态信息
    EXPORT_API bool GetObjAllStatus(RealTimeStatusDataEx* pStatus, string strObjGUID);
    //获取单个智能体的位置信息，经纬高表示方法
    EXPORT_API bool GetObjPositionLLH(Vector3PositionEx* pV3PosLLH, bool isAbsoluteH, string strObjGUID);
    //获取单个智能体的位置信息，UTM坐标系表示方法
    EXPORT_API bool GetObjPositionUTM(Vector3PositionEx* pV3PosUTM, bool isAbsoluteH, string strObjGUID);
    //获取单个智能体的姿态信息
    EXPORT_API bool GetObjPosture(Vector3Ex* pV3Posture, string strObjGUID);
    //获取单个智能体的三轴速度信息，世界坐标系，正东为x轴正，正北为y轴正，向上为z轴正
    EXPORT_API bool GetObjVelocity3D(Vector3Ex* pV3Velocity, string strObjGUID);
    //获取单个智能体的三轴加速度信息，世界坐标系，正东为x轴正，正北为y轴正，向上为z轴正
    EXPORT_API bool GetObjAccel3D(Vector3Ex* pV3Accel, string strObjGUID);
    //获取单个智能体的朝向方向的速度数值，标量
    EXPORT_API float GetObjHeadingSpeed(string strObjGUID);
    //获取单个智能体的朝向方向的加速度数值，标量
    EXPORT_API float GetObjHeadingAcc(string strObjGUID);
    //获取单个智能体的other部分
    EXPORT_API string GetObjOtherState(string strObjGUID);

    //返回当前智能体的所有传感器描述,带有传感器名字
    EXPORT_API bool GetObjSensorInfo(SensorInfoEx* pSensorInfo, string strObjGUID);
    //获取设备中的某个ImgSensor的数据结果
    EXPORT_API bool GetSensorImgData(RealTimeImageDataEx* pRTImage, string strObjGUID, string strImgSensorName);
    //获取设备中某个图像的视频帧数据
    EXPORT_API bool GetSensorVideoFrameData(RealTimeVideoFrameDataEx* pRTImage, string strObjGUID, string strImgSensorName);
    //获取设备中的某个LidarSensor的数据结果
    EXPORT_API bool GetSensorLidarData(RealTimeLidarDataEx* pRTLidar, string strObjGUID, string strLidarSensorName);
    //获取设备中的某个TrueValueSensor的数据结果
    EXPORT_API bool GetSensorTrueValueData(RealTimeTrueValueDataEx* pRTTrueValue, string strObjGUID, string strTrueValueSensorName);


    //查询某设备的逻辑连接信息，正在发生的
    EXPORT_API bool GetConnectionList(RTConnectionListEx* pConnList, string strObjGUID);

    //查询某设备的健康状态
    EXPORT_API bool IsHealth(string strObjGUID);
    //获取某设备的健康状态信息
    EXPORT_API bool GetCurHealthStatus(RTHealthStatusListEx* pCurHealthList, string strObjGUID);

    //获取设备的所有指令描述信息
    EXPORT_API bool GetObjCommandInfo(CommandInfoEx* pCommandInfo, string strObjGUID);
    //查询某设备是否正在执行任务
    EXPORT_API bool HasTaskExecuting(string strObjGUID);
    //获取某设备当前执行的所有任务信息
    EXPORT_API bool GetCurTaskExecuting(RTTaskCommandListEx* pCurTaskList, string strObjGUID,  bool isSelf);
    //为某个设备下发指令
    EXPORT_API bool SetCommand(CommandSetDataEx* pCmd, string strTargetGUID, int cmdID);
    // 查询某个设备的某个指令的执行进度，如果没有查询到或不是当前正在执行的任务，则返回-1
    EXPORT_API int GetCommandProcess(string strObjGUID, int cmdID);

    //**********************************************************************
    //**********************************************************************
    //**********************************************************************
    EXPORT_API bool RegisterDeviceSelf(RegisterDataEx* pRegi);
    EXPORT_API string RegisterDeviceSelfFromFile();
    EXPORT_API bool UnRegisterDeviceSelf();
    EXPORT_API bool WriteStatus(RealTimeStatusDataEx* pStatus);
    EXPORT_API bool WriteImgSensor(RealTimeImageDataEx* pRTImage, string strImgSensorName);
    EXPORT_API bool WriteVideoFrameSensor(RealTimeVideoFrameDataEx* pRTVideoData, string strVideoSensorName);
    EXPORT_API bool WriteLidarSensor(RealTimeLidarDataEx* pRTLidar, string strLidarSensorName);
    EXPORT_API bool WriteTrueValueSensor(RealTimeTrueValueDataEx* pRTTrueValue, string strTrueValueSensorName);

    EXPORT_API bool SendFile(string strfilePath, string guid);
    //**********************************************************************
    //**********************************************************************
    //**********************************************************************
    typedef void (*GetCommandFunc)(CommandSetDataEx* pCmd, int cmdID);
    EXPORT_API bool SetFunctionCall_GetCommand(GetCommandFunc callback);

    /// <summary>
    /// 执行命令回调  不用返回执行结果和状态
    /// </summary>
    typedef void (*GetCommandNoBackFunc)(CommandSetDataEx* pCmd);
    EXPORT_API bool SetFunctionCall_GetCommandNoBack(GetCommandNoBackFunc callback);

    typedef void (*CommandFinishedFunc)(int& cmdID,string strDesc);
    EXPORT_API bool SetFunctionCall_CommandFinished(CommandFinishedFunc callback);

    typedef void (*DeviceChangedFunc)(DeviceChangedDataEx* pDeviceChanged);
    EXPORT_API bool SetFunctionCall_DeviceChanged(DeviceChangedFunc callback);

    typedef void (*EventOccurFunc)(RTHealthStatusItemEx* pRTHealthStatusItem, string strGUID);
    EXPORT_API bool SetFunctionCall_EventOccur(EventOccurFunc callback);

    typedef void (*SendFileFinishedFunc)(bool isOK,string filePath,string target_guid);
    EXPORT_API bool SetFunctionCall_SendFileFinishedFunc(SendFileFinishedFunc callback);

    typedef void (*ReceiveFileFinishedFunc)(bool isOK,string filePath,string from_guid);
    EXPORT_API bool SetFunctionCall_ReceiveFileFinishedFunc(ReceiveFileFinishedFunc callback);

    typedef void (*NewVideoFrameDataFunc)(string strGUID,string strSensorName, RealTimeVideoFrameDataEx* pRTVideoData);
    EXPORT_API bool SetFunctionCall_NewVideoFrameDataFunc(NewVideoFrameDataFunc callback);

//}
