#include"taskmodelbehavior.h"



TaskModelbehavior::TaskModelbehavior()
{
    //电磁任务显示包络
    electEnvelopList.insert(pair<int,string>(41,"dy13"));
    electEnvelopList.insert(pair<int,string>(41,"dy14")); //ZG

    electEnvelopList.insert(pair<int,string>(12,"dy04")); //光电
  //  electEnvelopList.insert(pair<int,string>(23,"dy30")); //投弹

    pRaadarList.insert(pair<int,string>(13,"dy07")); //三维建图
    pRaadarList.insert(pair<int,string>(11,"dy06")); //二维建图
    pRaadarList.insert(pair<int,string>(160,"dy11"));
    pRaadarList.insert(pair<int,string>(160,"dy12"));
    //显示
}

void TaskModelbehavior::packgeEffect()
{
    //手机电磁识别包络，圆形
    _phoneElectEnvelop;
    _phoneElectEnvelop.model_id = "dy11";
    _phoneElectEnvelop.heading = 0;
    _phoneElectEnvelop.pitch = -90;
    _phoneElectEnvelop.angle = 45;
    _phoneElectEnvelop.distance = 200;
    _phoneElectEnvelop.fillColor.push_back(255);
    _phoneElectEnvelop.fillColor.push_back(100);
    _phoneElectEnvelop.fillColor.push_back(0);
    _phoneElectEnvelop.fillColor.push_back(100);
    //手机侦干电磁包络 扇形

    _phoneElectPAR.model_id = "dy11";
    _phoneElectPAR.heading = 0;
    _phoneElectPAR.pitch = -85;
    _phoneElectPAR.vAngle = 30;
    _phoneElectPAR.hAngle = 145;
    _phoneElectPAR.distance = 200;
    _phoneElectPAR.scanAngle = 60;
    _phoneElectPAR.radarStyle = 2;

    _phoneElectPAR.frameColor.push_back(255);
    _phoneElectPAR.frameColor.push_back(0);
    _phoneElectPAR.frameColor.push_back(0);
    _phoneElectPAR.frameColor.push_back(255);

    _phoneElectPAR.fillColor.push_back(255);
    _phoneElectPAR.fillColor.push_back(0);
    _phoneElectPAR.fillColor.push_back(0);
    _phoneElectPAR.fillColor.push_back(100);


    _particleEffect1.effectName = "烟尘";

    _particleEffect1.roll = 0;
    _particleEffect1.pitch = 0;
    _particleEffect1.yaw = 0;
    _particleEffect1.scale = 1;
    _particleEffect1.duartion = -1;

     APIBridgeSDK::Position position1;
     position1.longitude =118.09610449;
     position1.latitude = 32.50770464;
     position1.altitude = 19.52;
   //  data.position = position;
    _pE1_list.push_back(position1);

     position1.longitude =118.10277134;
     position1.latitude = 32.51302928;
     position1.altitude = 10.52;
     _pE1_list.push_back(position1);

     position1.longitude =118.10227701;
     position1.latitude = 32.51117143;
     position1.altitude = 14.03;
     _pE1_list.push_back(position1);

     position1.longitude =118.09641017;
     position1.latitude = 32.51295071;
     position1.altitude = 16.30;
     _pE1_list.push_back(position1);


    _particleEffect2.effectName = "起火烟雾";

    _particleEffect2.roll = 0;
    _particleEffect2.pitch = 0;
    _particleEffect2.yaw = 0;
    _particleEffect2.scale = 0.1;
    _particleEffect2.duartion = -1;

    APIBridgeSDK::Position position2;
    position2.longitude =118.10043722;
    position2.latitude = 32.51062561;
    position2.altitude = 14.45;
    _pE2_list.push_back(position2);


    position2.longitude =118.09988806;
    position2.latitude = 32.51029755;
    position2.altitude = 12.65;
    _pE2_list.push_back(position2);


    position2.longitude =118.10034808;
    position2.latitude = 32.50983055;
    position2.altitude = 12.50;
    _pE2_list.push_back(position2);


    position2.longitude =118.09995828;
    position2.latitude = 32.50925139;
    position2.altitude = 11.17;
    _pE2_list.push_back(position2);

    position2.longitude =118.10010634;
    position2.latitude = 32.50851536;
    position2.altitude = 10.17;
    _pE2_list.push_back(position2);


    position2.longitude =118.09807439;
    position2.latitude = 32.51048390;
    position2.altitude = 15.68;
    _pE2_list.push_back(position2);

    position2.longitude =118.09839092;
    position2.latitude = 32.51003070;
    position2.altitude = 15.03;
    _pE2_list.push_back(position2);

    position2.longitude =118.09783580;
    position2.latitude = 32.50957586;
    position2.altitude = 15.18;
    _pE2_list.push_back(position2);

    position2.longitude =118.09836611;
    position2.latitude = 32.50925079;
    position2.altitude = 14.74;
    _pE2_list.push_back(position2);


    _particleEffect3.effectName = "曳光弹效果";
    _particleEffect3.roll = 0;
    _particleEffect3.pitch = 0;
    _particleEffect3.yaw = 0;
    _particleEffect3.scale = 0100;
    _particleEffect3.duartion = 2;

    APIBridgeSDK::Position position3;
    position3.longitude =118.10280150;
    position3.latitude = 32.51111990;
    position3.altitude = 10.36;

    _pE3_list.push_back(position3);

    position3.longitude =118.09504929;
    position3.latitude = 32.50653208;
    position3.altitude = 16.79;

    _pE3_list.push_back(position3);

}

TaskModelbehavior::~TaskModelbehavior()
{

}

bool TaskModelbehavior::isElecSignFly(int tasktype)
{
    return tasktype == 120|| tasktype == 121;
}

bool TaskModelbehavior::isPhoneElectLocation(int tasktype)
{
    return  tasktype == 162;
}

bool TaskModelbehavior::isStrikeTargetTask(int tasktype)
{
    return tasktype == 210; // 山猫无人车提供目标后自主机动打击,
}

bool TaskModelbehavior::isImmediateIntTask(int tasktype)
{
    //42,43,44,45 电磁ZG临机任务， 161,162 电磁手机侦查任务
     return tasktype == 42 ||tasktype == 43||
           tasktype == 44 ||tasktype == 45 ||
           tasktype == 161 || tasktype== 162;
}

bool TaskModelbehavior::isImmediateIntTaskKD(int tasktype)
{
    return tasktype == 12 ||tasktype == 121||tasktype == 120||tasktype == 19;  //光电单机， 空地协同任务识别转跟踪
}

void TaskModelbehavior::getStrikeTarget(MQTTMission::Position &target)
{
    target = _strikeTarget;
}

void TaskModelbehavior::setStrikeTarget(MQTTMission::Position &target)
{
    _strikeTarget = target;
}

bool TaskModelbehavior::isShowelEctEnvelop(int tasktype, string model_id)
{

    for(auto& pair : electEnvelopList) {
        if(pair.first == tasktype && pair.second ==model_id )  return true;
    }
    return false;

}

bool TaskModelbehavior::isShowPARadar(int tasktype, string model_id)
{

    for(auto& pair : pRaadarList) {
        if(pair.first == tasktype && pair.second ==model_id )  return true;
    }
    return false;
    // auto it = pRaadarList.find(tasktype);
    // if (it != pRaadarList.end() && it->second == model_id) {
    //     // std::cout << "Found: " << taskid << " -> " << value << std::endl;
    //     return true;
    // } else {
    //     return false;
    // }

}

bool TaskModelbehavior::isShoweElectWindow(int taskid)
{
    return taskid == 160||
        taskid == 41; // 电磁手机任务、电磁侦干任务打开电磁窗口
}
