将UAVAndFPV放在D盘根目录,启动xTak_LandShip.exe,场景内就增加了一辆主车
利用主车的StartUAV指令,可以动态增加一个无人机智能体.




无人车指令: 
    RUN: 执行自主机动任务   
    参数:double集合,参数不定长,行走路径点的坐标的x值或y值,UTM绝对坐标系  

    OpenCabin:打开舱体
    参数:无

    CloseCabin:关闭舱体
    参数:无

    StartUAV:释放一个侦查无人机
    参数:无

    StopUAV:回收侦查无人机

    参数:无

    ReleaseFPV:释放穿越无人攻击机
    参数:int,释放的攻击机个数(1-6)

    StopFPV:回收穿越无人攻击机
    参数:无

    ResetCMD:终止所有此设备上运行的任务
    参数:无


无人机指令:
    RUN:执行自主机动任务
    参数: 1高度double,2不定长double集合,行走路径点的坐标的x值或y值,UTM绝对坐标系

    Move:移动
    参数:6个double
        1:double,deltaX,自身向右移动,单位米
        2:double,deltaY,自身向上移动,
        3:double,deltaZ,自身向前移动
        4,double,rotateY,航向旋转角度,单位度
        5,double,CamerafocusCamerarotateX,镜头俯仰角度
        6,double,Camerafocus,镜头焦距

    ResetCMD:终止所有此设备上运行的任务
    参数:无
（1）经常性的出现不能控制，比如可以释放穿越无人机，但不能回收无人机了（基本都是在执行任务后发送出现的问题较大，但是执行终止任务还是不行）；
（2）释放穿越无人机后，会在前端显示出终端界面，从而遮挡仿真界面；
（3）终止任务的指令未生效。
（4）不能切换到无人机的视角
（5）无人机回收后，获取的设备接口还能收到无人机的uid，但是没有utm信息（重启仿真软件也未生效，重新释放正常了）
（6）GetObjAllStatus接口中获取的dPosHeight;   //距离地面的高度 数据一直为0（无人机起飞后也是为0）
（7）无人机执行任务，查询的是无人机的uid，出现还在跑任务，实际任务状态为false，或者相反。

1、国摇软件界面在态势数据接入时概率奔溃
2、车头朝向不对
3、打击无人机未到达敌方车辆位置
4、3联屏电脑仿真打开奔溃（仿真电脑msc地址改变，需要重新注册）
5、国摇软件使用icaos网联sdk卡住，不能启动

异常警告显示示例
{
	"cmd": "SHOW_ALERT",
	"sn": "sn",
	"data": {
		"file_path": "C:/Users/53007/Pictures/Screenshots/screen.png",
		"alert_type": "异常军事活动检测",
		"alert_summary": "警报概要描述",
		"img_format": "png",
		"image_type": "Drone",
		"sensor_type": "Electro-Optic",
		"image_quality": "HD",
		"algorithm": "Vehicle Detection V 2.3.1",
		"latitude": 29.730017189201778,
		"longitude": 106.5173764921907,
		"altitude": 180.0
	},
	"timestamp": 1750995507
}

对话内容显示
{
	"cmd": "SHOW_CHAT",
	"sn": "sn",
	"data": "hello 你好呀",
	"timestamp": 1750995507
}

仿真当前智能体
name:UAV0,type:1,uid:15161ad9-366b-4f05-8e64-bc6492425171
name:enemy,type:0,uid:2a860ab1-8cba-4b76-b919-31f2269a937c
name:LandShip,type:0,uid:bfdfa56e-0f6a-45a2-9b0d-324d0e4e1538
