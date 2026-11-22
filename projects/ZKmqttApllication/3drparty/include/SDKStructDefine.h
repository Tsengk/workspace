#pragma once
#include <string>
#include <vector>
#include "rest_rcp/rest_rpc.hpp"
//对外暴露接口的结构体定义在此文件中添加
//对外结构体与内部实现结构体可以相同也可以不同，我们内部做数据转换，满足外部调用方需求即可

//任务类型
enum RPC_MissionType
{
	Mission_Common = 0,			//通用类型 有具体开始时间和持续时长
	Mission_AdHoc,				//临机任务 只有开始时间，持续时长未知
};
MSGPACK_ADD_ENUM(RPC_MissionType);

enum RPC_EntityRunningState
{
	Wating = 0,					//闲置
	Running,					//运行中
	Pause,					//已完成
};
MSGPACK_ADD_ENUM(RPC_EntityRunningState);


enum  RPC_KeyEventType
{
	keyEventTypeNormal,
	keyEventTypeCritical,
	keyEventTypeInformation,
	keyEventTypeError
};
MSGPACK_ADD_ENUM(RPC_KeyEventType);

namespace APIBridgeSDK
{
	// 地理位置
	struct Position
	{
		double longitude;	// 经度
		double latitude;	// 纬度
		double altitude;	// 高度
		Position(double _lon = 0, double _lat = 0, double _alt = 0)
			: longitude(_lon), latitude(_lat), altitude(_alt) {}
        void create(double _lon = 0, double _lat = 0, double _alt = 0);

		MSGPACK_DEFINE(longitude, latitude, altitude);
	};

	struct MilitaryAlertData {
		// 警报基本信息
		std::string alertType;        // 警报类型，例如："异常军事活动检测"
		// 图像数据
		struct {
			std::vector<unsigned char> imageData; // 原始图像二进制数据
			int width;                            // 图像宽度
			int height;                           // 图像高度
			int channels;                         // 图像通道数（如RGB为3）
			std::string format;                   // 图像格式（如"jpg"、"png"等）
			MSGPACK_DEFINE(imageData, width, height, channels, format);
		} image;
		// 概要信息
		std::string summary;          // 警报概要描述
		// 详细信息
		struct {
			std::time_t timestamp;    // 时间戳
			std::string imageType;    // 图像类型（例如："Drone"）
			std::string sensorType;   // 传感器类型（例如："Electro-Optic"）
			std::string imageQuality; // 图像质量（例如："HD"）
			std::string algorithm;    // 使用的算法（例如："Vehicle Detection V 2.3.1"）
									  // 位置坐标（经纬度）
			struct {
				double latitude;      // 纬度
				double longitude;     // 经度
				double altitude;      // 高度/海拔（可选）
				MSGPACK_DEFINE(latitude, longitude, altitude);
			} location;
			MSGPACK_DEFINE(timestamp, imageType, sensorType, imageQuality, algorithm, location);
		} details;

		MSGPACK_DEFINE(alertType, image, summary, details);
	};

	//方案信息
	struct SchemeInfo
	{
		std::string schmeId;
		std::string schemeName;
		std::string taskId;
		std::string jsoninfo;
		MSGPACK_DEFINE(schmeId, schemeName, taskId, jsoninfo);
	};

    struct ActionInfo
    {
        std::string actiontype;
        std::string jsoninfo;
        MSGPACK_DEFINE(actiontype, jsoninfo);
    };

	//实体行为(任务)
	struct EntityMission
	{
		std::string entityId;				//实体（资产）id
		std::string uuId;				//行为id
		std::string name;			//行为类型名称 (路径机动、打开无人车舱体、关闭无人车舱体......)
		std::string missionDes;				//描述文字
		int startTime;						//开始时间 毫秒，以方案开始时间为起始
		int duration;						//持续时长 毫秒
		RPC_MissionType missionType;		//行为类型
		std::string jsonData;				//行为参数

		MSGPACK_DEFINE(entityId, uuId, name, missionDes, startTime, duration, missionType, jsonData)
	};
	
	// 任务载荷状态
	struct EquipmentState {
		std::string equip_no;                 // 载荷编号
		uint8_t equip_redidual_electricity;   // 载荷剩余电量 (0-100%)
		MSGPACK_DEFINE(equip_no, equip_redidual_electricity)
	};

	// 通用平台结构体
	struct BasicPlatform {
		std::string model_id;                 // 作战单元编号
        std::string model_name;				  // 作战单元名称
        int64_t report_time;                  // 上报时间 (UTC毫秒时间戳)
		int32_t platform_type;                //  仿真无人机: 1001,
											  // 仿真无人车: 2001,
											  // 三维建图无人机: 1003,
											  // 二维建图无人机 : 1002,
											  // 目标识别无人机 : 1004
											  // 目标-人员:5001
											  // 目标-军用车辆:5002
											  //目标-民用车辆：5003
											  // 目标-飞机：5004
											  // ......

		Position deploy_position;             // 平台位置
		float velocity;                       // 速度 (km/h)
		float trk_direction;                  // 航向 (北偏东角度)

											  // 姿态信息
		float roll;                           // 滚转角 (左负右正)  optional
		float pitch;                          // 俯仰角 (前负后正)  optional
		float yaw;                            // 偏航角 (右负左正)  optional
        MSGPACK_DEFINE(model_id,model_name, report_time, platform_type, deploy_position, velocity, trk_direction, roll, pitch, yaw)
	};

	// 我方装备业务信息结构体
	struct OwnPlatformBusiness {
		// 系统状态 (可选)
		uint32_t health_status; // 0:正常,1:故障  optional
		std::string exception_msg; // 异常消息     optional

								   // 能源状态 (可选)
		uint8_t plat_redidual_electricity; // 平台剩余电量 (0-100%)

										   // 任务状态 (可选)
		uint32_t task_execute_status; // 0:空闲,1:执行中
		std::string task_number;     // 执行的任务编号

									 // 载荷状态列表
		std::vector<EquipmentState> equip_state_list;
		MSGPACK_DEFINE(health_status, exception_msg, plat_redidual_electricity, task_execute_status, task_number, equip_state_list)
	};

	// 目标装备业务信息结构体
	struct TargetPlatformBusiness {
		uint64_t collect_platform_id;  // 发现该目标的设备id
		int32_t data_source;           // 侦察来源标识
		int64_t first_time;            // 首次发现时间 (UTC毫秒时间戳)
		int64_t latest_time;           // 最后发现时间 (UTC毫秒时间戳)

		uint8_t space_type;            // 空间信息类型（1:航迹点, 2:测向线, 3:无空间信息）
		uint32_t capture_count;        // 截获次数

		int32_t scout_type;            // 侦察设备类型（字典值）

		MSGPACK_DEFINE(collect_platform_id, data_source, first_time, latest_time, space_type, capture_count, scout_type)
	};

	// 我方装备结构体
	struct OwnPlatform {
		BasicPlatform basic;
		OwnPlatformBusiness business;
        int64_t update_time;           // 最后更新时间 (UTC毫秒时间戳) by hqw

		MSGPACK_DEFINE(basic, business)
	};

	// 目标装备结构体
	struct Targetlatform {
		BasicPlatform basic;
		TargetPlatformBusiness business;
        int64_t update_time;

		MSGPACK_DEFINE(basic, business)
	};

	//电磁目标信息结构体
	struct ImsiTargetInfo
	{
		std::string task_id;
		int plmn;		//运营商 46000移动 46001联通 46011电信
		std::string imsi;		//无人机smsi号

		MSGPACK_DEFINE(task_id, plmn, imsi)
	};

	//电磁侦干信息结构体
	struct ImsiZGInfo
	{
		std::string task_id;			//任务编号
		std::string entity_id;			//作战单元编号
		std::string target_id;			//目标id
		std::string frequcency;			//频率
		MSGPACK_DEFINE(task_id, entity_id, target_id, frequcency);
	};

	//关键事件结构体
	struct KeyEventDataSDK
	{
		int64_t						time;				//消息时间
		RPC_KeyEventType			type;				//消息类型
		std::string					title;				//消息标题
		std::string					describe;			//消息描述
		std::string					id;					//消息唯一id

		MSGPACK_DEFINE(time, type, title, describe, id)
	};

	struct VideoStream
	{
		std::string vsName;				//视频流显示标签内容“作战单元名称[装备名称]”
		std::string model_id;			//作战单元编号
		std::string equipmentSeqNo;		//装备编号
		std::string vsType;				//视频流类型 trsp or rtmp
		std::string vsPath;				//视频流地址
		MSGPACK_DEFINE(vsName, model_id, equipmentSeqNo, vsType, vsPath);
	};

	struct VideoStreamList
	{
		std::vector<VideoStream> videoStreamList;
		MSGPACK_DEFINE(videoStreamList);
	};
    //目标雷达包络参数
    struct EntityEnvelop
    {
        std::string model_id;
        double heading;				//朝向角
        double pitch;				//俯仰角 0度为平视，-90度为垂直向下
        double angle;				//张角
        double distance;			//距离
        std::list<short> fillColor;	//填充颜色 四个元素 rgba[255,255,255,255];
        MSGPACK_DEFINE(model_id, heading, pitch, angle, distance, fillColor);
    };

    //扫描雷达参数
    struct PARadarData
    {
        std::string model_id;
        double heading;					//朝向角
        double pitch;					//俯仰角 0度为平视，-90度为垂直向下
        double vAngle;					//纵向扫描范围
        double hAngle;					//横向扫描范围
        double distance;				//扫描距离
        double scanAngle;				//扫描体角度
        int    radarStyle;				//雷达样式 0线框 1填充面 2线框和填充面
        std::list<short> frameColor;	//线框颜色
        std::list<short> fillColor;		//填充颜色

        MSGPACK_DEFINE(model_id, heading, pitch, vAngle, hAngle, distance, scanAngle, radarStyle, frameColor, fillColor);
    };

    //威胁半球参数
    struct DefenseSphere
    {
        std::string sphereId;			//威胁半球id
        double radius;					//半径
        Position center;			//中心坐标
        std::list<short> frameColor;	//线框颜色
        std::list<short> fillColor;		//填充颜色

        MSGPACK_DEFINE(sphereId, radius, center, frameColor, fillColor);
    };

    //特效显示参数
    struct ParticleEffect
    {
         std::string particleId;          //特效唯一id，根据id更新或删
        std::string effectName;			//特效名称 对应evd中的特效名称
        std::string targetKey;			//绑定目标id，对应目标的模型上需要绑定特效才能显示
        Position position;				//和targetKey二选一，如果不绑定动目标，则可以在任意坐标显示模型
        float roll = 0;					//翻滚角偏移
        float pitch = 0;				//俯仰角偏移
        float yaw = 0;					//朝向角偏移
        float scale = 1.0;				//缩放
        int duartion = -1;				//持续时长 秒，到达事件后自动释放，如果填-1则永远显示
        MSGPACK_DEFINE(particleId,effectName, targetKey, position, roll, pitch, yaw, scale, duartion);
    };

    struct TextWndInfo
    {
        std::string title;                        // 标题
        std::string content;                // 内容
        std::string titleFontName;        // 标题字体
        int titleFontSize;                        // 标题字号
        std::string titleColor;                // 标题颜色
        std::string contentFontName;// 标题字体
        int contentFontSize;                // 标题字号
        std::string contentColor;        // 标题颜色
        int hideDelay;                                // 窗口渐隐时长（0：不渐隐）

        TextWndInfo() : titleFontName("Microsoft YaHei"), titleFontSize(16), titleColor("255,255,0"),
            contentFontName("Microsoft YaHei"), contentFontSize(12), contentColor("255,255,255"),
            hideDelay(0){}
        MSGPACK_DEFINE(title, content, titleFontName, titleFontSize, titleColor,
                       contentFontName, contentFontSize, contentColor, hideDelay);
    };
    //大模型消息发送者结构体
        struct Sender
    {
        std::string icon;			//图标.png格式
        std::string name;			//名称
        int alignt;				//对齐方式 0左对齐 1右对齐
        MSGPACK_DEFINE(icon, name, alignt);
    };
    //大模型消息 格式化文本、文本+图片
    struct LargeModelMsg
    {
        Sender sender;
        std::string title;					//文本标题
        std::list<std::string> contents;	//内容
        std::string imagePath;				//图片路径，没有则为空
        MSGPACK_DEFINE(sender, title, contents, imagePath);
    };
    //目标信息结构体
        struct TargetInfo
    {
        std::string id;
        std::string type;
        bool autoNotify;
        std::string duration;
        std::string description;
        MSGPACK_DEFINE(id, type, autoNotify, duration, description);
    };

    //作战目标结构体
    struct MissionTargetInfo
    {
        std::string titleInfo;
        std::list<APIBridgeSDK::TargetInfo> targetInfo;
        MSGPACK_DEFINE(titleInfo, targetInfo);
    };

    //兵力推荐信息
    struct ForceEntityInfo
    {
        std::string name;
        std::string taskType;
        std::string unitId;
        std::string startTime;
        std::string endTime;
        std::list<APIBridgeSDK::Position> linePts;
        MSGPACK_DEFINE(name, taskType, unitId, startTime, endTime, linePts);
    };

    //兵力推荐结构体
    struct ForceRecommondInfo
    {
        std::string schemeName;
        std::list<short> displaColor;
        std::list<APIBridgeSDK::ForceEntityInfo> entities;
        MSGPACK_DEFINE(schemeName, displaColor, entities);
    };

    inline void Position::create(double _lon, double _lat, double _alt)
    {
        longitude =_lon;
        latitude = _lat;
        altitude = _alt;
    }

}
