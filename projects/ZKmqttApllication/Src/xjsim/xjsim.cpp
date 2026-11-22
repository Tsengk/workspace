#include "xjsim.h"
#include <utils/json.hpp>
#include <ctime>
#include <QDebug>
#include <set>
#include <QTextCodec>
#include <QString>
#include<gkd/mission.h>
#include<vector>
#include <cmath>

using namespace MQTTMission;
XjSim::XjSim(QObject* parent) : QObject(parent) {
    qDebug() << "XjSim constructor";
    task_type.store(0);
}

XjSim::~XjSim() {
    if (timer_) {
        timer_->stop();
        delete timer_;
    }
    if (status_timer_) {
        status_timer_->stop();
        delete status_timer_;
    }
    if (timer_thread_) {
        timer_thread_->quit();
        timer_thread_->wait();
        delete timer_thread_;
    }
    if (status_thread_) {
        status_thread_->quit();
        status_thread_->wait();
        delete status_thread_;
    }
    qDebug() << "XjSim destructor";
}

void XjSim::startTimer() {
    qDebug() << "Starting timer";

    // 创建并启动战术态势更新定时器线程
    timer_thread_ = new QThread(this);
    timer_ = new QTimer();
    timer_->moveToThread(timer_thread_);

    connect(timer_thread_, &QThread::started, timer_, [this]() {
        timer_->start(this->_cfg.status_interval);
    });
    connect(timer_thread_, &QThread::finished, timer_, &QTimer::deleteLater);
    connect(timer_, &QTimer::timeout, this, &XjSim::updateTacticalInformation, Qt::DirectConnection);

    timer_thread_->start();

    // 创建并启动状态检查定时器线程
    status_thread_ = new QThread(this);
    status_timer_ = new QTimer();
    status_timer_->moveToThread(status_thread_);

    connect(status_thread_, &QThread::started, status_timer_, [this]() {
        status_timer_->start(this->_cfg.check_interval);
    });
    connect(status_thread_, &QThread::finished, status_timer_, &QTimer::deleteLater);
    connect(status_timer_, &QTimer::timeout, this, &XjSim::checkDeviceStatus, Qt::DirectConnection);

    status_thread_->start();
    // initGYVideo();
}

void XjSim::sendRegister() {
    RegisterDataEx rde;
    rde.strCustormName = "liuhaibo";
    rde.iPermission = 0;
    rde.iLevel = 0;

    // 设置设备基本信息
    auto& device = rde.objDeviceBase;
    device.strName = "CarWWJ";
    device.iDeviceType = 0;
    device.fLength = 5;
    device.fWidth = 3;
    device.fHeight = 2.5f;
    device.fWeight = 3000;
    device.strProductionDate = "2020-12-20";
    device.strFactory = "XXX";
    device.strDescription = "FD,8Wheels";
    device.strOSName = "Ubuntu20.04";
    device.strBoardInfo = "BoardInfoWhat";

    // 配置激光雷达传感器
    auto configureLidar = [](SensorLiderEx& lidar, const std::string& name,
                             float x, float y, float z, float angle) {
        lidar.strName = name;
        lidar.strProductionDate = "2020-06-03";
        lidar.strDescription = "Lidar 16Lines";
        lidar.strOther = "\"GuoChan\":true";
        lidar.v3Position = {x, y, z};
        lidar.v3Posture = {0, 0, angle};
        lidar.dFar = 120;
        lidar.iHeightCount = 16;
        lidar.fFovX = 120;
        lidar.fFovY = 16;
        lidar.fHoriInterval = 0.2f;
        lidar.fFarInterval = 0.05f;
    };

    SensorLiderEx lidar1, lidar2;
    configureLidar(lidar1, "Lidar1", 4, 1.3f, 1.2f, 30);
    configureLidar(lidar2, "Lidar2", 4, -1.3f, 1.2f, -30);

    rde.sensorInfo.vecLidarList.push_back(lidar1);
    rde.sensorInfo.vecLidarList.push_back(lidar2);

    // 配置图像传感器
    SensorImageEx camera;
    camera.strName = "sImg1";
    camera.strProductionDate = "2020-06-16";
    camera.strDescription = "Image Sensor";
    camera.strOther = "\"GuoChan\":false";
    camera.v3Position = {4, 0, 1.2f};
    camera.v3Posture = {0, 0, 0};
    camera.fFovX = 120;
    camera.fFovY = 60;
    camera.fFocus = 0.3f;
    camera.iXPixel = 800;
    camera.iYPixel = 600;
    camera.isSupportVideo = false;

    rde.sensorInfo.vecImageList.push_back(camera);

    // 添加真值传感器
    SensorTrueValueEx trueValue;
    trueValue.strName = "TV1";
    rde.sensorInfo.vecTrueValueList.push_back(trueValue);

    // 配置命令信息
    auto addCommand = [&rde](const std::string& key, const std::string& desc) {
        CommandItemEx cmd;
        cmd.strCmdKey = key;
        cmd.strDescription = desc;
        cmd.iFixedParaCount = 0;
        rde.commandInfo.vecCommandList.push_back(cmd);
    };

    // 添加RUN命令
    {
        CommandItemEx runCmd;
        runCmd.strCmdKey = "RUN";
        runCmd.strDescription = "执行自主机动任务";
        runCmd.iFixedParaCount = 0;

        CommandParamentEx param;
        param.strName = "startPointXOrY";
        param.strDescription = "行走路径点的坐标的x值或y值，UTM绝对坐标系";
        param.iType = 1;
        param.isRepeat = true;
        runCmd.vecParaList.push_back(param);

        rde.commandInfo.vecCommandList.push_back(runCmd);
    }

    // 添加其他命令
    addCommand("Load", "执行上装任务");
    addCommand("UnLoad", "执行下装任务");

    RegisterDeviceSelf(&rde);
    UTMPoint utm_point_;
    LatLon latLng;
    latLng.lat =29.73010975037474;
    latLng.lon = 106.5178665518761;
    projection_utm_.convertLatLngToXYZ(latLng,utm_point_);
    qDebug().noquote().nospace() << "utm_point_: x=" << qSetRealNumberPrecision(12) << utm_point_.x << ", y=" << qSetRealNumberPrecision(12) << utm_point_.y;
    latLng.lat =29.728854329274995;
    latLng.lon = 106.51636451482774;
    projection_utm_.convertLatLngToXYZ(latLng,utm_point_);
    qDebug().noquote().nospace() << "utm_point_2: x=" << qSetRealNumberPrecision(12) << utm_point_.x << ", y=" << qSetRealNumberPrecision(12) << utm_point_.y;

}

void XjSim::initDevice() {
    if (is_initialized_) return;
    is_initialized_ = true;
    Init();
    sendRegister();

    qDebug() << "initDevice";
}


void XjSim::initUav() {
    startUav();
    qDebug() << "initUav";
}

void XjSim::updateTacticalInformation() {
    try {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        initDevice();
        DeviceItemListEx deviceList;
        nlohmann::json situationData;
        situationData["timestamp"] = time(nullptr);
        situationData["devices"] = nlohmann::json::array();
        // 创建一个临时集合存储当前在线的设备GUID
        std::set<std::string> current_guids;
        if (!GetObjList(&deviceList)) {
            qDebug() << "Failed to get object list";
            return;
        }
        //    qDebug() << "updateTacticalInformation:"<<deviceList.vctDeviceCollection.size();

        for (const auto& device : deviceList.vctDeviceCollection) {
            RealTimeStatusDataEx position;
            const std::string& guid = device.strGUID;
            const std::string& name = device.strName;
            int type = device.iType;

            // 记录当前设备的GUID
            current_guids.insert(guid);

            nlohmann::json deviceInfo;
            deviceInfo["guid"] = guid;
            deviceInfo["name"] = name;
            deviceInfo["type"] = type;

            bool isOnline = GetObjAllStatus(&position, guid);
            //            qDebug() << "name:"<<QString::fromStdString(name)<<"GetObjAllStatus:"<<position.dPosUTMX<<",dPosUTMY:"<<position.dPosUTMY;
            if (type == 0) {
                if (name == "LandShip") {
                    updateObject(isOnline, &self_object_, guid, name, type, position);
                    deviceInfo["position"] = {
                        {"lat", self_object_.lat},
                        {"lon", self_object_.lon},
                        {"height", self_object_.height},
                        {"speed", self_object_.speed},
                        {"roll", self_object_.roll},
                        {"heading", self_object_.heading},
                        {"pitch", self_object_.pitch},
                        {"accel", self_object_.accel},
                        {"health", self_object_.health},
                        {"is_working", self_object_.isWorking},
                        {"is_tasking", self_object_.isTasking}
                    };

                } else if (name == "enemy") {

                    bool isFirst = enemy_object_.uid.empty() ? true : false;
                    updateObject(isOnline, &enemy_object_, guid, name, type, position);
                    total_objects_[guid] = enemy_object_;
                    deviceInfo["position"] = {
                        {"lat", enemy_object_.lat},
                        {"lon", enemy_object_.lon},
                        {"height", enemy_object_.height},
                        {"speed", enemy_object_.speed},
                        {"roll", enemy_object_.roll},
                        {"heading", enemy_object_.heading},
                        {"pitch", enemy_object_.pitch},
                        {"accel", enemy_object_.accel},
                        {"health", enemy_object_.health},
                        {"is_working", enemy_object_.isWorking},
                        {"is_tasking", enemy_object_.isTasking}
                    };
                    //每次启动隐藏目标
                    // if(isFirst && nullptr != this->rcp_){
                    //     this->rcp_->m_rcpClientAdapter->setTargetVisible(enemy_object_.uid,false);
                    // }
                }
            } else if (type == 1) {
                SimObject uav;
                updateObject(isOnline, &uav, guid, name, type, position);
                total_objects_[guid] = uav;
                deviceInfo["position"] = {
                    {"lat", uav.lat},
                    {"lon", uav.lon},
                    {"height", uav.height},
                    {"speed", uav.speed},
                    {"roll", uav.roll},
                    {"heading", uav.heading},
                    {"pitch", uav.pitch},
                    {"accel", uav.accel},
                    {"health", uav.health},
                    {"is_working", uav.isWorking},
                    {"is_tasking", uav.isTasking}
                };
                if(uav_object_.uid.empty() ||
                    uav_object_.uid == guid) {
                    //取一个在线的作为侦察机
                    uav_object_.name = uav.name;
                    uav_object_.uid = uav.uid;
                    uav_object_.type = uav.type;
                    uav_object_.isOnline = uav.isOnline;
                    uav_object_.isTasking = uav.isTasking;
                    uav_object_.speed = uav.speed;
                    uav_object_.accel = uav.accel;

                    uav_object_.health = uav.health;

                    uav_object_.lat = uav.lat;
                    uav_object_.lon = uav.lon;
                    uav_object_.height =uav.height;

                }
            }

            if (!isOnline) {
                qDebug() << "Device offline:" << QString::fromStdString(guid);
            }
            else{
                //                cout << "name:" << name << ",type:" << type << ",uid:" << guid << std::endl;
                //                cout << "Position UTM - X:" << position.dPosUTMX << ", Y:" << position.dPosUTMY << ", Height:" << position.dHeight << std::endl;
                //            cout << "----------------------------------------" << std::endl;
            }

            situationData["devices"].push_back(deviceInfo);
        }
        if(situationData["devices"].size() > 0){
            QString update_status = QString::fromStdString(situationData.dump());
            if(nullptr != this->mqClient){
                qDebug() << "status_topic publish";
                this->mqClient->GetMqttConn()->NormalPublish(status_topic,update_status.toStdString());
            }

            // 同步我方装备信息到RCP服务器
            if(nullptr != this->rcp_){

                std::vector<APIBridgeSDK::OwnPlatform> ownPlatforms;
                convertToOwnPlatforms(ownPlatforms);
                qDebug() << "current platform nums:" << ownPlatforms.size();

                if(!ownPlatforms.empty()){
                    this->rcp_->m_rcpClientAdapter->createOrUpdateOwnPlatformData(ownPlatforms);
                    qDebug() << "Successfully called createOrUpdateOwnPlatformData, synced" << ownPlatforms.size() << "platforms (including enemy targets)";
                } else {
                    qDebug() << "No platforms to sync";
                }
            } else {
                qDebug() << "RCP client is null, cannot sync platform data";
            }
        }
        // 清除不在当前设备列表中的对象
        for (auto it = total_objects_.begin(); it != total_objects_.end();) {
            if (current_guids.find(it->first) == current_guids.end()) {
                if(it->second.uid == uav_object_.uid){// 如果要删除的对象是当前的侦察机，则清空 uavGuid
                    uav_object_.uid.clear();
                }
                it = total_objects_.erase(it);
            } else {
                ++it;
            }
        }
    }  catch (std::exception& ex) {
        qDebug() << "exception error:" << ex.what();
    }
}
ApiResponse XjSim:: revTaskData(const string& data) {
    try {
        qDebug()<<"revTaskData:"<<QString::fromStdString(data);
        nlohmann::json taskData = nlohmann::json::parse(data);
        // 检查必要的字段
        if (!taskData.contains("data")) {
            qDebug() << "Invalid data format: missing 'data' field";
            return apiRes.Failure(ErrorCode::ERR,"缺少data字段");
        }

        const std::string t_data = taskData["data"];
        if(t_data == "dj_finish"){//任务结束
            //重置任务显示路线
            if(nullptr != this->rcp_){
                for(auto uuid : _task_plane_roate_uuid_list){
                    this->rcp_->m_rcpClientAdapter->deleteMark(uuid);
                }
            }
            qDebug()<<"revTaskData: dj_finish";
            _task_plane_roate_uuid_list.clear();
        }

        return apiRes.Success();
    } catch (const nlohmann::json::exception& e) {
        qDebug() << "JSON parsing error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR,"JSON parsing error");
    } catch (const std::exception& e) {
        qDebug() << "Error processing command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR,"JSON parsing command");
    }
}
//接收到的数据解析
ApiResponse XjSim:: revMqttData(const string& topic,const string& data) {
    try {
        qDebug()<<"rev data:"<<QString::fromStdString(data);
        if (!revData.empty()) {
            revData.clear();
        }
        revData = nlohmann::json::parse(data);
        // 检查必要的字段
        if (!revData.contains("cmd")) {
            qDebug() << "Invalid command format: missing 'cmd' field";
            return apiRes.Failure(ErrorCode::ERR,"缺少cmd字段");
        }
        if (!revData.contains("sn")) {
            qDebug() << "Invalid command format: missing 'sn' field";
            return apiRes.Failure(ErrorCode::ERR,"缺少sn字段");
        }
        if(!revData.contains("data")){
            qDebug() << "Invalid command format: missing 'data' field";
            return apiRes.Failure(ErrorCode::ERR,"缺少data字段");
        }
        const std::string cmd = revData["cmd"];
        const std::string sn = revData["sn"];
        apiRes.sn = sn;
        apiRes.topic = topic;
        apiRes.cmd = cmd;
        if(cmd == "RUN_LANDSHIP"){//自车任务执行
            exCmd_.isNeedTj = true;
            exCmd_.isWorking = true;
            exCmd_.isNeedShowTargent = false;
            return runLandship(revData);
        }
        else if(cmd == "RUN_UAV"){//侦察机任务执行
            uav_object_.exCmdTimes = 0;
            exCmd_.isNeedShowTargent = true;
            exCmd_.isNeedTj = false;
            exCmd_.isWorking = true;
            return runUav(revData);
        }else if(cmd == "RUN_UAV_UID"){//侦察机任务执行（使用指定UID）
            uav_object_.exCmdTimes = 0;
            exCmd_.isNeedShowTargent = true;
            exCmd_.isNeedTj = false;
            exCmd_.isWorking = true;
            return run_uav_uid(revData);
        }else if(cmd == "RUN_FPV"){//打击任务执行
            exCmd_.isWorking = true;
            openCabin();
            exCmd_.cmd_type = ExCommandType::EX_FPV_OPEN;
        }else if(cmd == "UPATE_POINT"){//地图添加目标点
            return addPointToMap(revData);
        }else if(cmd == "REV_IMAGE"){//地图添加目标点
            return revImage(revData);
        }else if(cmd == "UPATE_REGION"){//地图添加区域
            return addAreaToMap(revData);
        }else if(cmd == "UPATE_ROUTE"){//添加路线
            return addRouteToMap(revData);
        }else if(cmd == "DELET_PLOT"){//删除地图
            return removePlotToMap(revData);
        }else if(cmd == "UPATE_VECHILE"){//更新实体vechile
            return addOrUpdateVechile(revData);
        }else if(cmd == "TARGET_SHOW"){//目标显示隐藏
            if(nullptr != this->rcp_){
                this->rcp_->m_rcpClientAdapter->setTargetVisible(enemy_object_.uid,false);
            }
        }else if(cmd == "SHOW_CHAT"){//显示大模型对话内容
            showChatReply(revData);
        }else if(cmd == "SHOW_ALERT"){//显示警告及图片
            if(nullptr != this->rcp_){
                return showAlert(revData);
                //                  showAlert();
            }
        }else if(cmd == "CREATE_SCHEME"){//创建方案
            if(nullptr != this->rcp_){
                return createScheme(revData);
            }
        }else if(cmd == "ADD_ENTITY"){
            if(nullptr != this->rcp_){
                return addEntity(revData);
            }
        }else if(cmd == "ADD_HOC"){
            if(nullptr != this->rcp_){
                return addHoc(revData);
            }
        }else if(cmd == "ABORT_ENTITY"){
            if(nullptr != this->rcp_){
                return abortEntity(revData);
            }
        }else if(cmd == "REMOVE_SCHEME"){
            if(nullptr != this->rcp_){
                return removeScheme(revData);
            }
        }else if(cmd == "REMOVE_ENTITY"){
            if(nullptr != this->rcp_){
                return removeEntityMission(revData);
            }
        }else if(cmd == "STOP"){
            if(nullptr != this->rcp_){
                return resetCommand();
            }
        }else if(cmd == "LOAD_TASK"){
            if(nullptr != this->rcp_){
                return loadTask(revData);
            } else {
                qDebug() << "LOAD_TASK error: RCP client is null";
                return apiRes.Failure(ErrorCode::ERR, "RCP客户端未初始化");
            }
        }
        return apiRes.Success();
    } catch (const nlohmann::json::exception& e) {
        qDebug() << "JSON parsing error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR,"JSON parsing error");
    } catch (const std::exception& e) {
        qDebug() << "Error processing command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR,"JSON parsing command");
    }
}

//地图添加区域
ApiResponse XjSim::addAreaToMap(nlohmann::json jsonData){
    try {
        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "data必须是数组格式";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        std::vector<APIBridgeSDK::Position> vecPos;
        for (const auto& point : data) {
            if (!point.contains("lat")
                || !point.contains("lon")
                || !point.contains("height")) {
                qDebug() << "Invalid point format: missing lat height or lon";
                continue;
            }
            vecPos.push_back(APIBridgeSDK::Position(point["lon"].get<double>(), point["lat"].get<double>(), point["height"].get<double>()));
        }
        if (vecPos.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "没有有效的位置点");
        }
        if (vecPos.size() < 3) {
            return apiRes.Failure(ErrorCode::ERR, "绘制区域所需点需大于2");
        }
        if(nullptr != this->rcp_){
            QUuid uuid = QUuid::createUuid();  // 生成随机UUID
            string uid_ = uuid.toString().toStdString(); // 低版本适配 update by hqw
            const string plot_name =revData.contains("plot_name") ?
                                         jsonData["plot_name"] : "区域1";
            this->rcp_->m_rcpClientAdapter->updateRegionMark(uid_, vecPos, utf8ToGbk(plot_name), "#00FF00");
        }
        return apiRes.Success();
    } catch (const std::exception& e) {
        qDebug() << "Error processing addAreaToMap command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "地图添加区域失败");
    }
}
//删除地图
ApiResponse XjSim::removePlotToMap(nlohmann::json jsonData){
    try {
        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "data必须是数组格式";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        for (const auto& item : data) {
            if (!item.is_string()) {
                qDebug() << "UUID必须是字符串类型";
                continue; // 跳过无效项，继续处理其他项
            }

            std::string uuid = item.get<std::string>();
            qDebug() << "removePlotToMap, UUID:" << QString::fromStdString(uuid);

            if(nullptr != this->rcp_){
                this->rcp_->m_rcpClientAdapter->deleteMark(uuid);
            }
        }
        return apiRes.Success("删除地图元素成功");
    } catch (const std::exception& e) {
        qDebug() << "Error processing UAV command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "删除地图元素失败");
    }
}
//添加路线到地图
ApiResponse XjSim::addRouteToMap(nlohmann::json jsonData){
    try {
        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "data必须是数组格式";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }
        std::vector<APIBridgeSDK::Position> vecPos;
        for (const auto& point : data) {
            if (!point.contains("lat")
                || !point.contains("lon")
                || !point.contains("height")) {
                qDebug() << "Invalid point format: missing lat height or lon";
                continue;
            }
            //            vecPos.push_back(APIBridgeSDK::Position(point["lon"].get<double>(), point["lat"].get<double>(), point["height"].get<double>()));
            vecPos.push_back(APIBridgeSDK::Position(point["lon"].get<double>(), point["lat"].get<double>(), point["height"].get<double>()));

        }
        if (vecPos.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "没有有效的位置点");
        }
        if (vecPos.size() < 2) {
            return apiRes.Failure(ErrorCode::ERR, "路点数量必须大于或等于2");
        }
        if(nullptr != this->rcp_){
            const string plot_name = jsonData.contains("plot_name") ?
                                         jsonData["plot_name"].get<string>() : "route1";
            const string plot_color = jsonData.contains("plot_color") ?
                                          jsonData["plot_color"].get<string>() : "#1e31ff";
            qDebug() << plot_name.c_str();
            this->rcp_->m_rcpClientAdapter->updateLineMark(plot_name, vecPos, plot_color);
            _task_plane_roate_uuid_list.push_back(plot_name);// 更新存储的路径UUID
        }
        return apiRes.Success("路线添加成功");
    } catch (const std::exception& e) {
        qDebug() << "Error processing UAV command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "地图添加路线失败");
    }
}
//截图数据
ApiResponse XjSim::revImage(nlohmann::json jsonData) {
    try {
        // 检查data字段是否为字符串
        const auto& data_obj = jsonData["data"];
        double lat = data_obj["lat"].get<double>();
        double lon = data_obj["lon"].get<double>();
        double height = data_obj["height"].get<double>();
        string base64_image = data_obj["image_data"].get<string>();
        QByteArray image_data = QByteArray::fromBase64(QString::fromStdString(base64_image).toUtf8());
        QImage restoreImage;

        if (!restoreImage.loadFromData(image_data)) {
            qDebug() << "Failed to load image from base64 data";
            return apiRes.Failure(ErrorCode::ERR, "图片数据加载失败");
        }

        // 创建保存目录（如果不存在）
        QDir saveDir("./saved_images");
        if (!saveDir.exists()) {
            if (!saveDir.mkpath(".")) {
                qDebug() << "Failed to create save directory";
                return apiRes.Failure(ErrorCode::ERR, "创建保存目录失败");
            }
        }
        // QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        QString filePath = saveDir.filePath("image_save.png");
        // 保存图片到本地
        if (restoreImage.save(filePath, "PNG")) {
            qDebug() << "Image2 saved successfully:" << filePath;
        } else {
            qDebug() << "Failed to save image";
            return apiRes.Failure(ErrorCode::ERR, "图片保存失败");
        }
        QImage img(filePath);
        //    QImage img(QString("%1/screen.png").arg(QApplication::applicationDirPath()));

        QByteArray byteArray;
        QBuffer buffer(&byteArray);
        buffer.open(QIODevice::WriteOnly);
        img.save(&buffer, "png");
        buffer.close();
        APIBridgeSDK::MilitaryAlertData data;
        data.alertType = utf8ToGbk("异常军事活动检测");//"异常军事活动检测";
        data.image.imageData.assign(byteArray.begin(), byteArray.end());
        data.image.width = img.width();
        data.image.height = img.height();
        data.image.channels = img.hasAlphaChannel() ? 4 : 3;
        data.image.format = "png";//"png";
        data.summary = utf8ToGbk("警报概要描述");//"警报概要描述";
        data.details.timestamp = static_cast<std::time_t>(QDateTime::currentMSecsSinceEpoch());
        data.details.imageType = utf8ToGbk("Drone");//"Drone";
        qDebug() << "showAlert11:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;

        data.details.sensorType = utf8ToGbk("Electro-Optic");//"Electro-Optic";
        data.details.imageQuality = utf8ToGbk("HD");// "HD";
        data.details.algorithm = utf8ToGbk("Vehicle Detection V 2.3.1");//"Vehicle Detection V 2.3.1";
        data.details.location.longitude = lon;
        data.details.location.latitude = lat;
        data.details.location.altitude = height;
        qDebug() << "showAlert22:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;
        this->rcp_->m_rcpClientAdapter->showAlertData(data);
        qDebug() << "showAlert:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;
        return apiRes.Success();
    } catch (const std::exception& e) {
        qDebug() << "revImage:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "图片处理失败");
    }
}
//地图添加点
ApiResponse XjSim::addPointToMap(nlohmann::json jsonData) {
    try {
        // 检查data字段是否为数组
        const auto& points = jsonData["data"];
        if (!points.is_array()) {
            qDebug() << "addPointToMap error: data should be an array";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        // 遍历所有点
        for (const auto& point : points) {
            if (!point.contains("lon") || !point.contains("lat") ||
                !point.contains("height") || !point.contains("point_name")) {
                qDebug() << "addPointToMap error: missing required fields";
                continue;
            }

            try {
                double lon = point["lon"].get<double>();
                double lat = point["lat"].get<double>();
                double height = point["height"].get<double>();
                std::string point_name = point["point_name"].get<std::string>();

                // 使用point_name作为唯一标识
                if (this->rcp_ && this->rcp_->m_rcpClientAdapter) {
                    this->rcp_->m_rcpClientAdapter->updatePointMark(
                        point_name,
                        APIBridgeSDK::Position(lon, lat, height),
                        10,
                        utf8ToGbk(point_name),
                        "#FF0000"
                        );
                    qDebug() << "Added point:" << QString::fromStdString(point_name)
                             << "at" << lon << "," << lat << "," << height;
                }
            } catch (const std::exception& e) {
                qDebug() << "Error processing point:" << e.what();
                continue;  // 继续处理下一个点
            }
        }

        return apiRes.Success();
    } catch (const std::exception& e) {
        qDebug() << "addPointToMap:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "地图添加点失败");
    }
}
//侦察机执行自主任务
ApiResponse XjSim::runUav(nlohmann::json jsonData){
    try {
        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "Invalid data format: data should be an array";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        std::vector<LatLon> positions;
        for (const auto& point : data) {
            if (!point.contains("lat") || !point.contains("lon")) {
                qDebug() << "Invalid point format: missing lat or lon";
                continue;
            }

            LatLon pos;
            pos.lat = point["lat"].get<double>();
            pos.lon = point["lon"].get<double>();
            positions.push_back(pos);
        }

        if (positions.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "没有有效的位置点");
        }


        if(uav_object_.uid.empty()){
            //先释放无人机
            apiRes = startUav();
            exCmd_.cmd_type = ExCommandType::EX_UAV;
            return apiRes;
        }else{
            auto it = total_objects_.find(uav_object_.uid);
            if (it != total_objects_.end()) {
                //有这个uid的无人机
                SimObject obj = it->second;
                exCmd_.uav_fly_height = "300";
                if(uav_object_.exCmdTimes > 1){
                    qDebug()<<"侦察机多次执行自主任务失败";
                    exCmd_.isWorking = false;
                    return apiRes.Failure(ErrorCode::ERR,"多次执行失败");
                }
                uav_object_.exCmdTimes++;
                apiRes = startCommand("RUN", 1, uav_object_.uid, positions);
                exCmd_.cmd_type = ExCommandType::EX_UAV_RUN;
                return apiRes;
            } else {
                //先释放无人机
                apiRes = startUav();
                exCmd_.cmd_type = ExCommandType::EX_UAV;
                return apiRes;
            }
        }
    } catch (const std::exception& e) {
        qDebug() << "Error processing UAV command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "侦察机执行命令失败");
    }
}
//侦察机执行自主任务（使用指定UID）
ApiResponse XjSim::run_uav_uid(nlohmann::json jsonData){
    try {
        // 检查是否包含uid字段
        if (!jsonData.contains("uid")) {
            qDebug() << "Invalid command format: missing 'uid' field";
            return apiRes.Failure(ErrorCode::ERR, "缺少uid字段");
        }

        // 从MQTT数据中获取uid
        std::string mqtt_uid = jsonData["uid"].get<std::string>();

        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "Invalid data format: data should be an array";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        std::vector<LatLon> positions;
        for (const auto& point : data) {
            if (!point.contains("lat") || !point.contains("lon")) {
                qDebug() << "Invalid point format: missing lat or lon";
                continue;
            }

            LatLon pos;
            pos.lat = point["lat"].get<double>();
            pos.lon = point["lon"].get<double>();
            positions.push_back(pos);
        }

        if (positions.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "没有有效的位置点");
        }

        // 检查指定的uid是否存在于total_objects_中
        // auto it = total_objects_.find(mqtt_uid);
        // if (it != total_objects_.end()) {
        // 找到了指定uid的无人机
        // SimObject obj = it->second;
        exCmd_.uav_fly_height = "300";

        // if(uav_object_.exCmdTimes > 1){
        //     qDebug()<<"侦察机多次执行自主任务失败";
        //     exCmd_.isWorking = false;
        //     return apiRes.Failure(ErrorCode::ERR,"多次执行失败");
        // }
        // uav_object_.exCmdTimes++;
        apiRes = startCommand("RUN", 1, mqtt_uid, positions);
        // exCmd_.cmd_type = ExCommandType::EX_UAV_RUN;

        qDebug() << "run_uav_uid: Using uid from MQTT:" << QString::fromStdString(mqtt_uid);
        return apiRes;
        // } else {
        //     qDebug() << "run_uav_uid: UID not found in total_objects_:" << QString::fromStdString(mqtt_uid);
        //     return apiRes.Failure(ErrorCode::ERR, "指定的无人机UID不存在");
        // }
    } catch (const std::exception& e) {
        qDebug() << "Error processing run_uav_uid command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "侦察机执行命令失败");
    }
}
//执行打击
ApiResponse XjSim::runFpv(bool isDj){
    try {
        exCmd_.isWorking = true;
        SimObject fpvObiect;
        for(const auto& pair : getTotalObjects()) {
            const auto& obj = pair.second;
            if(1 == obj.type &&
                obj.isOnline &&
                uav_object_.uid != obj.uid) {
                fpvObiect = obj;
                break;
            }
        }
        if (fpvObiect.uid.empty()) {//没有可用的侦察机，先释放无人机
            exCmd_.cmd_type = ExCommandType::EX_FPV;
            return startUav();
        }

        std::vector<LatLon> positions;
        LatLon pos;
        pos.lat = enemy_object_.lat;
        pos.lon = enemy_object_.lon;
        positions.push_back(pos);
        exCmd_.cmd_type = isDj ? ExCommandType::EX_FPV_END : ExCommandType::EX_FPV_RUN;
        exCmd_.uav_fly_height = isDj ? QString::number((enemy_object_.utm_z), 'f', 7).toStdString() : "300";
        return startCommand("RUN", 1, fpvObiect.uid, positions);
    } catch (const std::exception& e) {
        qDebug() << "Error processing UAV command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "执行打击机命令失败");
    }
}
ApiResponse XjSim::runFpv(bool isDj,string height){
    try {
        SimObject fpvObiect;
        for(const auto& pair : getTotalObjects()) {
            const auto& obj = pair.second;
            if(1 == obj.type &&
                obj.isOnline &&
                uav_object_.uid != obj.uid) {
                fpvObiect = obj;
                break;
            }
        }
        if (fpvObiect.uid.empty()) {//没有可用的侦察机，先释放无人机
            exCmd_.cmd_type = ExCommandType::EX_FPV;
            return startUav();
        }

        std::vector<LatLon> positions;
        LatLon pos;
        pos.lat = enemy_object_.lat;
        pos.lon = enemy_object_.lon;
        positions.push_back(pos);
        exCmd_.cmd_type = ExCommandType::EX_NONE;
        exCmd_.uav_fly_height = isDj ? QString::number(enemy_object_.utm_z, 'f', 7).toStdString() : height;
        return startCommand("RUN", 1, fpvObiect.uid, positions);
    } catch (const std::exception& e) {
        qDebug() << "Error processing UAV command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "执行打击机命令失败");
    }
}

// 相机控制方法
ApiResponse XjSim::cameraGoto(double latitude, double longitude, double altitude, double tilt, double heading, int time_ms) {
    try {
        // 检查RCP客户端是否有效
        if (this->rcp_ == nullptr || this->rcp_->m_rcpClientAdapter == nullptr) {
            qDebug() << "cameraGoto error: RCP client or adapter is null";
            return apiRes.Failure(ErrorCode::ERR, "RCP客户端未正确初始化");
        }

        qDebug() << "cameraGoto calling with parameters:"
                 << "lat=" << latitude
                 << ", lon=" << longitude
                 << ", alt=" << altitude
                 << ", tilt=" << tilt
                 << ", heading=" << heading
                 << ", time=" << time_ms;

        // 调用RCP客户端的cameraGoto方法
        this->rcp_->m_rcpClientAdapter->cameraGoto(latitude, longitude, altitude, tilt, heading, time_ms);

        qDebug() << "cameraGoto completed successfully";
        return apiRes.Success();
    } catch (const std::exception& e) {
        qDebug() << "Error in cameraGoto:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "相机定位失败");
    }
}
//无人车执行自主任务
ApiResponse XjSim::runLandship(nlohmann::json jsonData){
    try {
        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "Invalid data format: data should be an array";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        std::vector<LatLon> positions;
        for (const auto& point : data) {
            if (!point.contains("lat") || !point.contains("lon")) {
                qDebug() << "Invalid point format: missing lat or lon";
                continue;
            }

            LatLon pos;
            pos.lat = point["lat"].get<double>();
            pos.lon = point["lon"].get<double>();
            positions.push_back(pos);
        }

        if (positions.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "没有有效的位置点");
        }
        if(self_object_.uid.empty()){
            return apiRes.Failure(ErrorCode::ERR,"无人车初始化失败");
        }
        return startCommand("RUN", 0, self_object_.uid, positions);
    } catch (const std::exception& e) {
        qDebug() << "Error processing UAV command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "处理侦察机命令失败");
    }
}
//显示固定的警告
void XjSim::showAlert(){
    string file_path = _cfg.target_file;
    string alert_type = "检测到目标";
    string alert_summary = "发现敌方车辆";
    string img_format = "png";
    string image_type = "Drone";
    string sensor_type = "Electro-Optic";
    string image_quality = "HD";
    string algorithm = "Vehicle Detection V 2.3.1";

    double longitude = enemy_object_.lat;
    double latitude = enemy_object_.lon;
    double altitude = enemy_object_.height;

    QImage img(QString::fromStdString(file_path));
    //    QImage img(QString("%1/screen.png").arg(QApplication::applicationDirPath()));

    QByteArray byteArray;
    QBuffer buffer(&byteArray);
    buffer.open(QIODevice::WriteOnly);
    img.save(&buffer, "png");
    buffer.close();
    APIBridgeSDK::MilitaryAlertData data;
    data.alertType = utf8ToGbk(alert_type);//"异常军事活动检测";
    data.image.imageData.assign(byteArray.begin(), byteArray.end());
    data.image.width = img.width();
    data.image.height = img.height();
    data.image.channels = img.hasAlphaChannel() ? 4 : 3;
    data.image.format = img_format;//"png";
    data.summary = utf8ToGbk(alert_summary);//"警报概要描述";
    data.details.timestamp = static_cast<std::time_t>(QDateTime::currentMSecsSinceEpoch());
    data.details.imageType = image_type;//"Drone";
    qDebug() << "showAlert11:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;
    data.details.sensorType = sensor_type;//"Electro-Optic";
    data.details.imageQuality = image_quality;// "HD";
    data.details.algorithm = algorithm;//"Vehicle Detection V 2.3.1";
    data.details.location.longitude = longitude;
    data.details.location.latitude = latitude;
    data.details.location.altitude = altitude;
    this->rcp_->m_rcpClientAdapter->showAlertData(data);
}
//创建方案
ApiResponse XjSim::createScheme(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string schmeId = json_data["scheme_id"];
    string taskId = json_data["task_id"];
    string schemeName = json_data["scheme_name"];
    string jsoninfo = json_data["json_info"];
    APIBridgeSDK::SchemeInfo info;
    info.schmeId = utf8ToGbk(schmeId);
    info.taskId = utf8ToGbk(taskId);
    info.schemeName = utf8ToGbk(schemeName);
    info.jsoninfo = utf8ToGbk(jsoninfo);
    qDebug() << "createScheme   start " << QString::fromStdString( schmeId)
             << ","<<QString::fromStdString(schemeName)
             << ","<<QString::fromStdString(taskId);
    this->rcp_->m_rcpClientAdapter->loadScheme(info);
    //    qDebug() << "createScheme   end ";
    return apiRes.Success();
}
//新增实体行为
ApiResponse XjSim::addEntity(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string entityId = json_data["entityId"];
    string missionDes = json_data["missionDes"];
    string missionId = json_data["missionId"];
    int startTime = json_data["startTime"].get<int>();
    int duration = json_data["duration"].get<int>();

    APIBridgeSDK::EntityMission info;
    info.entityId = utf8ToGbk(entityId);				//
    info.missionDes = utf8ToGbk(missionDes);
    info.uuId = utf8ToGbk(missionId);
    info.missionType = RPC_MissionType::Mission_Common;
    info.startTime = startTime;
    info.duration = duration;
    this->rcp_->m_rcpClientAdapter->addEntityMission(info);
    return apiRes.Success();
}
//新增临机行为
ApiResponse XjSim::addHoc(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string entityId = json_data["entityId"];
    string missionDes = json_data["missionDes"];
    string missionId = json_data["missionId"];
    int startTime = json_data["startTime"].get<int>();

    APIBridgeSDK::EntityMission info;
    info.entityId = utf8ToGbk(entityId);
    info.missionDes = utf8ToGbk(missionDes);
    info.uuId = utf8ToGbk(missionId);
    info.missionType = RPC_MissionType::Mission_AdHoc;
    info.startTime = startTime;
    this->rcp_->m_rcpClientAdapter->addEntityMission(info);
    return apiRes.Success();
}
//废弃实体行为
ApiResponse XjSim::abortEntity(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string missionId = json_data["id"];
    this->rcp_->m_rcpClientAdapter->abandonMission(utf8ToGbk(missionId));
    return apiRes.Success();
}
//删除方案
ApiResponse XjSim::removeScheme(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string schemeID = json_data["id"];
    this->rcp_->m_rcpClientAdapter->removeScheme(utf8ToGbk(schemeID));
    return apiRes.Success();
}
//删除实体行为(任务)
ApiResponse XjSim::removeEntityMission(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string missionId = json_data["id"];
    this->rcp_->m_rcpClientAdapter->removeEntityMission(utf8ToGbk(missionId));
    return apiRes.Success();
}

// //加载任务
// ApiResponse XjSim::loadTask(nlohmann::json jsonData){
//     try {
//         // 检查是否包含data字段
//         if (!jsonData.contains("data")) {
//             qDebug() << "loadTask error: missing 'data' field";
//             return apiRes.Failure(ErrorCode::ERR, "缺少data字段");
//         }

//         const auto& json_data = jsonData["data"];
//         if (!json_data.is_object()) {
//             qDebug() << "loadTask error: data should be an object";
//             return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
//         }

//         // 从JSON数据中提取任务信息
//         string taskJsonStr = json_data.dump(); // 将整个data对象转换为JSON字符串

//         // 检查RCP客户端是否有效
//         if (this->rcp_ == nullptr || this->rcp_->m_rcpClientAdapter == nullptr) {
//             qDebug() << "loadTask error: RCP client or adapter is null";
//             return apiRes.Failure(ErrorCode::ERR, "RCP客户端未正确初始化");
//         }

//         qDebug() << "loadTask calling with data:" << QString::fromStdString(taskJsonStr);

//         // 调用RCP客户端的loadTask方法
//         this->rcp_->m_rcpClientAdapter->loadTask(taskJsonStr);
//         // this->rcp_->m_rcpClientAdapter->
//         qDebug() << "loadTask completed successfully";
//         return apiRes.Success();

//     } catch (const nlohmann::json::exception& e) {
//         qDebug() << "loadTask JSON error:" << e.what();
//         return apiRes.Failure(ErrorCode::ERR, "JSON处理失败: " + string(e.what()));
//     } catch (const std::exception& e) {
//         qDebug() << "loadTask error:" << e.what();
//         return apiRes.Failure(ErrorCode::ERR, "加载任务失败: " + string(e.what()));
//     }
// }

//加载任务
ApiResponse XjSim::loadTask(nlohmann::json jsonData){

    // 检查是否包含data字段
    if (!jsonData.contains("data")) {
        qDebug() << "loadTask error: missing 'data' field";
        return apiRes.Failure(ErrorCode::ERR, "缺少data字段");
    }

    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "loadTask error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }

    // 从JSON数据中提取任务信息
    string taskJsonStr = json_data.dump(); // 将整个data对象转换为JSON字符串

    // 检查RCP客户端是否有效
    if (this->rcp_ == nullptr || this->rcp_->m_rcpClientAdapter == nullptr) {
        qDebug() << "loadTask error: RCP client or adapter is null";
        return apiRes.Failure(ErrorCode::ERR, "RCP客户端未正确初始化");
    }

    qDebug() << "loadTask calling with data:" << QString::fromStdString(taskJsonStr);

    // 调用RCP客户端的loadTask方法
    //    this->rcp_->m_rcpClientAdapter->loadTask(taskJsonStr);
    this->rcp_->m_rcpClientAdapter->m_rcpClient->call("loadTaskInfomation", taskJsonStr);
    // this->rcp_->m_rcpClientAdapter->
    qDebug() << "loadTask completed successfully";
    return apiRes.Success();


}

//异常活动检测
ApiResponse XjSim::showAlert(nlohmann::json jsonData){
    const auto& json_data = jsonData["data"];
    if (!json_data.is_object()) {
        qDebug() << "showAlert error: data should be an object";
        return apiRes.Failure(ErrorCode::ERR, "data必须是对象格式");
    }
    string file_path = json_data["file_path"];
    string alert_type = json_data["alert_type"];
    string alert_summary = json_data["alert_summary"];
    string img_format = json_data["img_format"];
    string image_type = json_data["image_type"];
    string sensor_type = json_data["sensor_type"];
    string image_quality = json_data["image_quality"];
    string algorithm = json_data["algorithm"];

    double longitude = json_data["longitude"];
    double latitude = json_data["latitude"];
    double altitude = json_data["altitude"];

    QImage img(QString::fromStdString(file_path));

    QByteArray byteArray;
    QBuffer buffer(&byteArray);
    buffer.open(QIODevice::WriteOnly);
    img.save(&buffer, "png");
    buffer.close();
    APIBridgeSDK::MilitaryAlertData data;
    data.alertType = utf8ToGbk(alert_type);//"异常军事活动检测";
    data.image.imageData.assign(byteArray.begin(), byteArray.end());
    data.image.width = img.width();
    data.image.height = img.height();
    data.image.channels = img.hasAlphaChannel() ? 4 : 3;
    data.image.format = img_format;//"png";
    data.summary = utf8ToGbk(alert_summary);//"警报概要描述";
    data.details.timestamp = static_cast<std::time_t>(QDateTime::currentMSecsSinceEpoch());
    data.details.imageType = utf8ToGbk(image_type);//"Drone";
    qDebug() << "showAlert11:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;

    data.details.sensorType = utf8ToGbk(sensor_type);//"Electro-Optic";
    data.details.imageQuality = utf8ToGbk(image_quality);// "HD";
    data.details.algorithm = utf8ToGbk(algorithm);//"Vehicle Detection V 2.3.1";
    data.details.location.longitude = longitude;
    data.details.location.latitude = latitude;
    data.details.location.altitude = altitude;
    qDebug() << "showAlert22:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;
    this->rcp_->m_rcpClientAdapter->showAlertData(data);
    qDebug() << "showAlert:"<<data.image.width<<",height:"<<data.image.height<<",channels:"<<data.image.channels;
    return apiRes.Success();
}
qreal XjSim::convertToNorthHeading(qreal heading) {
    qreal northHeading = fmod(90 - heading + 360, 360);
    return northHeading;
}

bool XjSim::hasId(const std::vector<zy>& vec, const std::string& model_id)
{
    auto it = std::find_if(vec.begin(), vec.end(),
                           [&model_id](const zy& item) {
                               return item.zb_id == model_id; // 只比较 zb_id 字段
                           });

    return it != vec.end();
}

//更新智能体信息
void XjSim::updateObject(bool isOnline, SimObject* obj,
                         const string& guid, const string& name,
                         int type, const RealTimeStatusDataEx& pos) {
    obj->name = name;
    // 添加GUID长度检查，防止std::length_error异常
    if (guid.length() > 1000) {
        qDebug() << "Warning: GUID too long, truncating. Original length:" << guid.length();
        obj->uid = guid.substr(0, 1000);
    } else {
        obj->uid = guid;
    }
    obj->type = type;
    obj->isOnline = isOnline ? 1 : 0;
    if(exCmd_.isWorking){
        obj->isWorking = pos.bTaskExec;
    }else{
        obj->isWorking = exCmd_.isWorking;
    }
    obj->speed = pos.fSpeed;
    obj->roll = pos.v3Posture.x;
    obj->pitch = pos.v3Posture.y;
    double heading = pos.v3Posture.z;
    //    if(heading < 0){
    //        heading += 360;
    //    }
    //    obj->heading = heading + 120;
    obj->heading = convertToNorthHeading(heading);
    obj->accel = pos.fAccel;
    obj->health = pos.bHealth;
    if (isOnline) {
        LatLon outLatLon;
        //        projection_utm_.CartesianToLatLon(pos.dPosX, pos.dPosY, pos.dPosZ, false, outLatLon);
        // 仿真坐标转WGS84
        //        LatLon wgs84;
        projection_utm_.SimToWGS84(pos.dPosUTMX, pos.dPosUTMY, outLatLon);
        cout <<std::fixed<< "！！！！！！！lat:" << std::setprecision(9)<< outLatLon.lat<<",lon:"<<outLatLon.lon<< std::endl;
        obj->lat = outLatLon.lat;
        obj->lon = outLatLon.lon;
        obj->height = pos.dHeight + _height;
        obj->utm_x = pos.dPosUTMX;
        obj->utm_y = pos.dPosUTMY;
        obj->utm_z = pos.dHeight;

    } else {
        obj->lat = 0;
        obj->lon = 0;
        obj->height = 0;
        obj->utm_x = 0;
        obj->utm_y = 0;
        obj->utm_z = 0;
    }
}

ApiResponse XjSim::startCommand(const std::string& cmd,int type, const std::string& guid,
                                const std::vector<LatLon>& positions) {
    CommandSetDataEx cmdData;
    cmdData.strCmdKey = cmd;
    cmdData.iParaCount = 0;
    std::vector<CommandParamentSetEx> params;
    if(type == 1){//无人机
        CommandParamentSetEx param;
        param.iType = 3;
        param.strValue = exCmd_.uav_fly_height;
        params.push_back(param);
    }
    for (const auto& pos : positions) {
        CommandParamentSetEx param;
        param.iType = 3;

        projection_utm_.convertLatLngToXYZ(pos,utm_point_);

        // Add X coordinate
        param.strValue = QString::number(utm_point_.x, 'f', 7).toStdString();
        params.push_back(param);

        // Add Y coordinate
        param.strValue = QString::number(utm_point_.y, 'f', 7).toStdString();
        params.push_back(param);
        cout <<std::fixed<< "X:" << std::setprecision(9)<<utm_point_.x << ",Y:" << utm_point_.y
             << ",zone:" << utm_point_.zone << ",pos.lat:"<<pos.lat<<",pos.lon"<<pos.lon<< std::endl;
    }

    cmdData.vecParaList = params;
    bool success = exCommand(cmdData, guid);
    qDebug() << "Command" << QString::fromStdString(cmd)
             << " uid:"<<QString::fromStdString(guid)
             <<" type:"<<type
             << "executed with status:" << success;
    return apiRes.Success();
}

std::string XjSim::getCommandString(CommandType cmdType) const {
    switch (cmdType) {
    case CommandType::START_UAV: return "StartUAV";
    case CommandType::STOP_UAV: return "StopUAV";
    case CommandType::RESET_CMD: return "ResetCMD";
    case CommandType::OPEN_CABIN: return "OpenCabin";
    case CommandType::CLOSE_CABIN: return "CloseCabin";
    case CommandType::STOP_FPV: return "StopFPV";
    case CommandType::RELEASE_FPV: return "ReleaseFPV";
    case CommandType::RUN: return "RUN";
    default: return "";
    }
}

ApiResponse XjSim::sendCommand(const std::string& cmd, const std::string& guid) {
    CommandSetDataEx cmdData;
    cmdData.strCmdKey = cmd;
    cmdData.iParaCount = 0;
    bool success = exCommand(cmdData, guid);
    return success ? apiRes.Success() : apiRes.Failure(ErrorCode::ERR,"执行失败");
}

bool XjSim::exCommand(CommandSetDataEx cmdData, const std::string& guid){
    exCmd_.guid = guid;
    const string g_uid = exCmd_.guid;
    //先调用清理指令
    //    if(cmdData.strCmdKey == "RUN"){
    //       resetCommand(g_uid);
    //    }
    exCmd_.cmd_id = command_id_;
    exCmd_.cmdData = cmdData;
    exCmd_.isSendSuccess = false;
    exCmd_.sendTimes = 1;
    exCmd_.strCmdKey = cmdData.strCmdKey;
    exCmd_.times = this->_cfg.check_times;
    bool success = SetCommand(&cmdData, g_uid, command_id_);
    ++command_id_;
    qDebug() << "Command " << QString::fromStdString(exCmd_.strCmdKey)
             << " exCmd_.guid "<<QString::fromStdString(g_uid)
             << " exCmd_.cmd_id"<<exCmd_.cmd_id
             << "executed with status:" << success;
    return success;
}
bool XjSim::resetCommand(const std::string& guid){
    CommandSetDataEx cmdData;
    cmdData.strCmdKey = getCommandString(CommandType::RESET_CMD);
    cmdData.iParaCount = 0;
    bool success = SetCommand(&cmdData, guid, command_id_);
    qDebug() << "resetCommand " << QString::fromStdString(cmdData.strCmdKey)
             << " exCmd_.guid "<<QString::fromStdString(guid)
             << " exCmd_.cmd_id"<<command_id_
             << "executed with status:" << success;
    ++command_id_;
    return success;
}
ApiResponse XjSim::executeCommand(CommandType cmdType, const std::string& guid) {
    std::string cmd = getCommandString(cmdType);
    if (!cmd.empty()) {
        std::string target_guid = guid.empty() ? self_object_.uid : guid;
        // 添加空值检查，防止异常
        if (target_guid.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "目标GUID为空，请先初始化系统");
        }
        return sendCommand(cmd, target_guid);
    }else{
        return apiRes.Failure(ErrorCode::ERR,"未知指令");
    }
}

ApiResponse XjSim::releaseFpv(int number) {
    CommandSetDataEx cmdData;
    cmdData.strCmdKey = "ReleaseFPV";
    cmdData.iParaCount = 0;

    CommandParamentSetEx param;
    param.iType = 0;
    param.strValue = QString::number(number).toStdString();
    cmdData.vecParaList.push_back(param);
    //    if(HasTaskExecuting(self_object_.uid)){
    //        qDebug() << "releaseFpv"<< "正在执行任务";
    //        return apiRes.Failure(ErrorCode::TASK_RUNNING,"正在执行任务");
    //    }
    bool success = exCommand(cmdData, self_object_.uid);
    qDebug() << "ReleaseFPV command executed with status:" << success;
    return success ? apiRes.Success() : apiRes.Failure(ErrorCode::ERR,"执行失败");
}

ApiResponse XjSim::testSelfMove(MoveDirection direction, const std::string& guid) {
    std::vector<LatLon> position;
    LatLon target;
    // 使用传入的guid，如果为空则使用self_object的uid
    std::string target_guid = guid.empty() ? self_object_.uid : guid;

    // 获取目标对象的当前位置
    auto target_it = total_objects_.find(target_guid);
    if(target_it == total_objects_.end()) {
        return apiRes.Failure(ErrorCode::ERR, "未找到目标对象");
    }
    const SimObject& target_obj = target_it->second;

    const double MOVE_DISTANCE = 0.002;

    switch (direction) {
    case MoveDirection::LEFT:
        target.lat = target_obj.lat - MOVE_DISTANCE;
        target.lon = target_obj.lon;
        break;
    case MoveDirection::RIGHT:
        target.lat = target_obj.lat + MOVE_DISTANCE;
        target.lon = target_obj.lon;
        break;
    case MoveDirection::TOP:
        target.lat = 29.7299444;
        target.lon = 106.5186095;
        position.push_back(target);
        target.lat = 29.7301121;
        target.lon = 106.5176439;
        position.push_back(target);
        target.lat = 29.7295321;
        target.lon = 106.5174428;
        break;
    case MoveDirection::BOTTOM:
        target.lat = target_obj.lat - MOVE_DISTANCE;
        target.lon = target_obj.lon - MOVE_DISTANCE;
        break;
    }

    position.push_back(target);
    return startCommand("RUN",target_obj.type ,target_guid, position);
}

void XjSim::recordAircraftCommand(const QJsonObject& root)
{
    const QString uid = root["uid"].toString().trimmed();
    std::lock_guard<std::mutex> lock(aircraft_status_mutex_);
    working_aircraft_uids_.insert(uid);
}

void XjSim::resetAircraftStates(const QJsonObject& root)
{
    if (!root["CTN"].toObject()["platform_list"].isArray() || !root["TM"].isDouble()) return;
    const double timestamp = root["TM"].toDouble();
    const qint64 taskInfoTime = static_cast<qint64>(timestamp);
    std::lock_guard<std::mutex> lock(aircraft_status_mutex_);
    if (!task_info_tm_initialized_) {
        last_task_info_tm_ = taskInfoTime;
        task_info_tm_initialized_ = true;
    } else if (taskInfoTime > last_task_info_tm_) {
        working_aircraft_uids_.clear();
        last_task_info_tm_ = taskInfoTime;
    }
}

QString XjSim::aircraftRunningStatus(const QString& uid)
{
    std::lock_guard<std::mutex> lock(aircraft_status_mutex_);
    return working_aircraft_uids_.contains(uid) ? "working" : "idle";
}

void XjSim::InitMqttAndRcp(MqClient* mqClient,rest_rcp_client* _rcp_client,const CommonCfg& _cfg, const GKDCommonCfg& gkd_cfg){
    this->mqClient = mqClient;
    this->rcp_ = _rcp_client;
    this->_cfg = _cfg;
    this->_gkd_cfg = gkd_cfg;
    projection_utm_.InitTransformParameters(_cfg);
    status_topic = QString::fromStdString(_cfg.status_topic);
    cmd_topic = QString::fromStdString(_cfg.cmd_topic);

    //js博弈 敌方目标
    this->mqClient->GetMqttConn()->Subscribe(QString::fromStdString("/control/1"),[this](const QJsonObject& root) {
        _jsby_targetObj = root["target_list"].toArray();
    });



    //js博弈，播放
    this->mqClient->GetMqttConn()->Subscribe(QString::fromStdString("/control/2"),[this](  QJsonObject& root) {
        QJsonObject  playControl = root;
        _jsby_playstatus = playControl["playstatus"].toInt();
        _jsby_rate = playControl["rate"].toInt();
        _jsby_totaltime = playControl["totaltime"].toInt();
        _jsby_totaltime = _jsby_totaltime*1000;
        _jsby_currenttime = playControl["currenttime"].toInt();
        if(_jsby_playstatus == 0){
            std::thread t(&XjSim::jsbyPlay,this);
            t.detach();
        }
    });
    //无人平台（无人机、无人车）行动控制接口(mqtt)协议如下
    this->mqClient->GetMqttConn()->Subscribe(QString::fromStdString("/llm/jsim/excute"),[this](const QJsonObject& root) {
        recordAircraftCommand(root);
        _llm_excute =root;  //行动控制信息
        QString uid = _llm_excute["uid"].toString();
        if(_gkd_cfg.zyc.find("1002") != _gkd_cfg.zyc.end() &&
                    hasId(_gkd_cfg.zyc["1002"],uid.toStdString()) ){
        // if(uid == "dy99" || uid == "dy100" || uid == "dy101"){ //控制实体无人机
            if((!_llm_excute.contains("executeHeight"))|| (!_llm_excute.contains("waypointSpeed"))
            ||(!_llm_excute.contains("task_type"))) return;  // task_type 0 为飞控， 1为，侦查，2 为打击
            QString tt = _llm_excute["task_type"].toString();

            if(tt == "0"){
                is_zc_dj = false; // 普通飞行任务
                task_type.store(1);
            }else{
                is_zc_dj = true; //侦查打击任务
                if(tt == "2" && task_type.load() == 1){
                    QJsonObject root;
                    root["reset"] =1;
                    this->mqClient->GetMqttConn()->PrimitivePublish("QB/308", root);
                    task_type.store(tt.toInt());
                }else {
                    task_type.store(tt.toInt());
                }
            }

            std::vector<MQTTMission::Position> path;
            QJsonArray data = _llm_excute["data"].toArray();
            for(QJsonValue p: data){
                QJsonObject point =  p.toObject();
                MQTTMission::Position position;
                position.longitude = QJsonValue(point["lon"]).toDouble();
                position.latitude = QJsonValue(point["lat"]).toDouble();
                position.altitude = QJsonValue(point["height"]).toDouble();
                path.push_back(position);
            }
            if(path.size() < 2) return ;
            double executeHeight = _llm_excute["executeHeight"].toDouble();
            double waypointSpeed = _llm_excute["waypointSpeed"].toDouble();
            ControlDJiMini4_Pro_Gimbal_pitch(uid);
            ControlDJiMini4_Pro(uid.toStdString(), executeHeight, waypointSpeed, path);

        }else if(_llm_excute["cmd"]=="RUN_UAV_UID" || _llm_excute["cmd"]=="RUN_UGV_UID"){
            excuteUAVOrUGV ag{};
            ag.roll = 0.0;
            QString uid = _llm_excute["uid"].toString();
            QString uname = _llm_excute["uname"].toString();
            ag.uid = uid.toStdString();
            ag.uname = uname.toStdString();
            ag.cmd = _llm_excute["cmd"].toString().toStdString();
            if(_llm_excute.contains("roll")) ag.roll = _llm_excute["roll"].toDouble();
            if(_llm_excute.contains("pitch")) ag.pitch = _llm_excute["pitch"].toDouble();
            if(_llm_excute.contains("yaw")) ag.yaw = _llm_excute["yaw"].toDouble();
            QJsonArray path = _llm_excute["data"].toArray();
            int platform_type = _llm_excute["platform_type"].toInt();
            ag.platform_type = platform_type;
            QString tt = _llm_excute["task_type"].toString(); //，1为侦查，2为打击
            //阶段控制
            if(tt == "2" &&( task_type == 1)){
                QJsonObject root;
                root["reset"] =1;
                this->mqClient->GetMqttConn()->PrimitivePublish("QB/308", root);
                task_type.store(tt.toInt());

            }else {
                task_type.store(tt.toInt());
            }
            ag.task_type =std::to_string( task_type.load());
            for(auto pos: path){
                QJsonObject pos1 = pos.toObject();
                MQTTMission::Position position;
                position.longitude = QJsonValue(pos1["lon"]).toDouble();
                position.latitude = QJsonValue(pos1["lat"]).toDouble();
                position.altitude = QJsonValue(pos1["height"]).toDouble();
                ag.path.push_back(position);
            }
            ag.executeHeight = _llm_excute.contains("executeHeight") ? _llm_excute["executeHeight"].toDouble() : 80.0;
            ag.waypointSpeed = _llm_excute.contains("waypointSpeed") ? _llm_excute["waypointSpeed"].toDouble() : 5.0;

            //插值
            interpolateRouteLinear(ag.path,2);
            //发送
            std::thread t(&XjSim::planZB,this,ag);
            t.detach();
        }


    });
    //订阅任务方案装备信息
    this->mqClient->GetMqttConn()->Subscribe(QString::fromStdString("/llm/jsim/status/task_info"), [this](const QJsonObject& root) {
        resetAircraftStates(root);
        const QJsonValue platforms = root["CTN"].toObject()["platform_list"];
        if (!platforms.isArray()) return;

        std::map<QString, QString> names;
        for (const auto& value : platforms.toArray()) {
            const QJsonObject platform = value.toObject();
            const QString id = platform["id"].toString().trimmed();
            const QString name = platform["name"].toString().trimmed();
            if (!id.isEmpty() && !name.isEmpty()) {
                names[id] = name;
            }
        }
        std::lock_guard<std::mutex> lock(platforms_name_mutex_);//创建自动加锁，保护下面执行的代码
        platforms_name_by_id_.swap(names);//交换指针地址，新的映射关系更新到platforms_name_by_id_
    });

    //订阅DJImini4 实时位置信息
    this->mqClient->GetMqttConn()->Subscribe(QString::fromStdString("/aircraft/location"),[this](const QString& loc) {
        QStringList parts = loc.split(",");
        if(parts.size() < 4) return;

        QString uid   = parts.at(0).trimmed();
        double lon    = parts.at(1).toDouble();
        double lat    = parts.at(2).toDouble();
        double alt    = parts.at(3).toDouble();

        QString uname = "无人机";
        {
            std::lock_guard<std::mutex> lock(platforms_name_mutex_);
            const auto name = platforms_name_by_id_.find(uid);
            if (name != platforms_name_by_id_.end()) {
                uname = name->second;
            }
        }

        MQTTMission::Position pos;
        pos.longitude = lon;
        pos.latitude  = lat;
        pos.altitude  = alt;

        QString runing_status = aircraftRunningStatus(uid);
        QJsonObject root;
        excuteUAVOrUGV zb{};
        zb.cmd = "RUN_UAV_UID";
        zb.uid = uid.toStdString();
        zb.uname = uname.toStdString();
        zb.task_type = std::to_string( task_type.load());
        zb.executeHeight = 10.0 + alt;
        zb.platform_type = 1002;
        zb.waypointSpeed = 0.0;
        zb.roll = 0.0;
        zb.pitch = 0.0;
        zb.yaw = 0.0;
        PackageZzptjzbzt_303(root, runing_status, zb, pos);
        this->mqClient->GetMqttConn()->PrimitivePublish("QB/303", root);
    });

    this->mqClient->GetMqttConn()->Subscribe(cmd_topic,[this](const QJsonObject& root) {
        QJsonDocument doc(root);
        QString jsonStr = doc.toJson(QJsonDocument::Compact);
        auto respose = revMqttData(cmd_topic.toStdString(),jsonStr.toStdString());
        this->mqClient->GetMqttConn()->NormalPublish(cmd_respose_topic,respose.toJson());
    });
    QString dj_finish = "llm/task/zc/command";
    this->mqClient->GetMqttConn()->Subscribe(dj_finish,[this](const QJsonObject& root) {
        QJsonDocument doc(root);
        QString jsonStr = doc.toJson(QJsonDocument::Compact);
        auto respose = revTaskData(jsonStr.toStdString());
    });
    // 订阅 jsim 设备状态主题，用于接收外部设备态势数据
    QString jsim_status_topic = "llm/701/jsim/status";
    this->mqClient->GetMqttConn()->Subscribe(jsim_status_topic, [this](const QJsonObject& root) {
        QJsonDocument doc(root);
        QString jsonStr = doc.toJson(QJsonDocument::Compact);
        this->processJsimStatusData(jsonStr.toStdString());
    });
}

void XjSim::initGYVideo(){

    VideoStreamList vslist;
    VideoStream vs;
    vs.vsName = "dy0[无人机视频]";
    vs.model_id ="dy0";
    vs.equipmentSeqNo= "";
    vs.vsType = "rtmp";
    // vs.vsPath = "rtsp://127.0.0.1:8554/loop_stream";
    vs.vsPath = "rtmp://192.168.2.109:1935/rtmplive/live";
    vslist.videoStreamList.push_back(vs);
    vs.vsPath = "rtsp://192.168.2.112:8554/v4l2";
    vs.vsName = "dy0[无人车视频]";
    vslist.videoStreamList.push_back(vs);

    qDebug() << "c2manager dt3";
    vs.vsName = "dy0[侧向视频]";
    vslist.videoStreamList.push_back(vs);


    //抛出线程，否则会被打断连接
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::thread t(&XjSim::loadVideoStream,this,vslist);
    t.detach();
}
void XjSim::loadVideoStream(VideoStreamList& vslist)
{
    if (this->rcp_->m_rcpClientAdapter) {
        this->rcp_->m_rcpClientAdapter->loadVideoStream(vslist);
        qDebug() << "loadVideoStream success:";

    }
}

void XjSim::jsbyPlay()
{
    if(_jsby_playstatus !=0 ) return;
    std::vector<JSBY_Target> _jsby_target_list;
    for(QJsonValue value:_jsby_targetObj){
        QJsonObject obj =  value.toObject();
        QString id =  obj["id"].toString();
        QJsonArray path = obj["path"].toArray();
        double speed = obj["speed"].toDouble();
        int type = obj["type"].toInt();

        JSBY_Target target;
        target.route.clear();
        target.id = id.toStdString();
        target.speed = speed;
        target.type = type;
        for(QJsonValue value : path){
            QJsonObject point = value.toObject();
            APIBridgeSDK::Position pos;
            pos.longitude = QJsonValue(point["lon"]).toDouble();
            pos.latitude = QJsonValue(point["lat"]).toDouble();
            pos.altitude = QJsonValue(point["alt"]).toDouble();
            target.route.push_back(pos);
        }

        std::vector<APIBridgeSDK::Position> route1 = target.route;
        // target.route = interpolateRouteLinear(route1,5);
        _jsby_target_list.push_back(target);
    }

    int length = 0;
    for(JSBY_Target& target: _jsby_target_list){
        if(length < target.route.size()) length = target.route.size();
    }

    int sleep = _jsby_totaltime/(length-1);
    for(int i = 0; i <length; i++){
        Sleep(sleep);
        for(JSBY_Target& obj:_jsby_target_list){
            QString id = QString::fromStdString(obj.id);
            if(_jsby_playstatus != 0) return;//线程停止
            if(i> obj.route.size()-1) continue;
            APIBridgeSDK::Position point = obj.route[i];
            double speed = obj.speed;
            int type = obj.type;
            QJsonObject root;
            PackageJSBY_dzmbpt_301(root,id,point,speed,type);
            this->mqClient->GetMqttConn()->PrimitivePublish("QB/301", root);
        }
    }





}

void XjSim::showChatReply(const nlohmann::json& revData) {
    try {
        const std::string& data = revData["data"];
        if (this->rcp_->m_rcpClientAdapter) {
            this->rcp_->m_rcpClientAdapter->showChatReply(utf8ToGbk(data));
        }
    } catch (const std::exception& e) {
        qDebug() << "showChatReply error:" << e.what();
    }
}
std::string XjSim::utf8ToGbk(const std::string& data) {
    if (data.empty()) {
        return "";
    }

    try {
        static QTextCodec* gbkCodec = QTextCodec::codecForName("GBK");
        if (!gbkCodec) {
            qDebug() << "Failed to get GBK codec";
            return data;
        }

        // 检查输入是否为有效的UTF-8
        QString qstr = QString::fromUtf8(data.c_str());
        if (qstr.isEmpty() && !data.empty()) {
            qDebug() << "Invalid UTF-8 input, returning original data";
            return data;
        }

        // 转换为GBK
        QByteArray gbk_bytes = gbkCodec->fromUnicode(qstr);
        if (gbk_bytes.isEmpty()) {
            qDebug() << "Failed to convert to GBK, returning original data";
            return data;
        }

        // 验证转换结果不包含无效字符
        std::string result(gbk_bytes.constData(), gbk_bytes.size());
        if (result.empty() && !data.empty()) {
            qDebug() << "Empty result after conversion, returning original data";
            return data;
        }

        return result;
    } catch (const std::exception& e) {
        qDebug() << "utf8ToGbk error:" << e.what() << ", returning original data";
        return data;
    } catch (...) {
        qDebug() << "utf8ToGbk unknown error, returning original data";
        return data;
    }
}

void XjSim::autoShowOrTj(){
    //    if(!exCmd_.isWorking)return;
    //需要显示目标
    if(exCmd_.isNeedShowTargent){
        bool need_show = data_parser.isWithinHorizontalDistance(uav_object_.lat,uav_object_.lon,
                                                                enemy_object_.lat,enemy_object_.lon,_cfg.zc_distance);
        qDebug()<<"need_show :"<<need_show;
        if(need_show){
            exCmd_.isNeedShowTargent = false;
            try {
                if(nullptr != this->rcp_){
                    //                   try {
                    this->rcp_->m_rcpClientAdapter->setTargetVisible(enemy_object_.uid,false);
                    //                   } catch (const std::exception& e) {
                    //                       qDebug() << "setTargetVisible:" << e.what();
                    //                   }
                    //showAlert();  暂时注释 hqw
                }
            }  catch (std::exception& ex) {
                qDebug()<<"error :"<<ex.what();
            }

        }
    }else if(exCmd_.isNeedTj){//是否需要突击
        bool need_tj = data_parser.isWithinHorizontalDistance(self_object_.lat,self_object_.lon,
                                                              enemy_object_.lat,enemy_object_.lon,_cfg.dj_distance);
        qDebug()<<"need_tj :"<<need_tj;
        if(need_tj){
            enemy_object_.exCmdTimes = 1;
            exCmd_.isNeedTj = false;
            exCmd_.isNeedDj = true;
            std::vector<LatLon> positions;
            LatLon pos;
            pos.lat = _cfg.fpv_lat;
            pos.lon = _cfg.fpv_lon;
            positions.push_back(pos);
            startCommand("RUN", 0, enemy_object_.uid, positions);
        }
    }
}
// 检查是否有序列化的任务需要执行
void XjSim::checkDeviceStatus() {
    try {
        if(!exCmd_.guid.empty()){
            const string guid = exCmd_.guid;
            bool _process = HasTaskExecuting(guid);
            if(!_process){
                if(exCmd_.times< 1){
                    qDebug()<<"exCmd_:"<<QString::fromStdString(exCmd_.strCmdKey)<<",查询的uid:"<<QString::fromStdString(guid) <<",执行完毕";
                    if(!exCmd_.isSendSuccess && exCmd_.sendTimes < 3){
                        if(exCmd_.cmdData.strCmdKey == "RUN"){
                            bool success = SetCommand(&exCmd_.cmdData, exCmd_.guid, command_id_);
                            ++command_id_;
                            qDebug() <<"第"<<exCmd_.sendTimes<<"次执行"
                                     << "RUN Command " << QString::fromStdString(exCmd_.strCmdKey)
                                     << " exCmd_.guid "<<QString::fromStdString(exCmd_.guid)
                                     << " exCmd_.cmd_id"<<exCmd_.cmd_id
                                     << "executed with status:" << success;
                            exCmd_.sendTimes++;
                        }
                    }
                    if(ExCommandType::EX_UAV == exCmd_.cmd_type ||
                        ExCommandType::EX_UAV_RUN == exCmd_.cmd_type){
                        exCmd_.guid = "";
                        if(exCmd_.isWorking)runUav(revData);
                    }else if(ExCommandType::EX_FPV == exCmd_.cmd_type){
                        exCmd_.guid = "";
                        if(exCmd_.isWorking){
                            closeCabin();
                            runFpv(false);
                        }
                    }else if(ExCommandType::EX_FPV_RUN == exCmd_.cmd_type){
                        exCmd_.guid = "";
                        if(exCmd_.isWorking){
                            runFpv(true);
                        }
                    }else if(ExCommandType::EX_FPV_END == exCmd_.cmd_type){
                        exCmd_.guid = "";
                        exCmd_.isWorking = false;
                    }else if(exCmd_.isNeedDj){
                        exCmd_.guid = "";
                        if(exCmd_.isWorking){
                            openCabin();
                            exCmd_.cmd_type = ExCommandType::EX_FPV_OPEN;
                        }
                    }else if(ExCommandType::EX_UAV_RUNNING == exCmd_.cmd_type){
                        exCmd_.guid = "";
                        exCmd_.isWorking = false;
                    }else{
                        exCmd_.guid = "";
                    }
                }else{
                    exCmd_.times--;
                    qDebug()<<"exCmd_:"<<QString::fromStdString(exCmd_.strCmdKey)<<",执行完毕:"<<exCmd_.times;
                }
            }
            else{
                exCmd_.isSendSuccess = true;
                if(ExCommandType::EX_FPV_OPEN == exCmd_.cmd_type){
                    if(-6 > exCmd_.times){
                        exCmd_.guid = "";
                        runFpv(false);
                    }
                    exCmd_.times--;
                }else if(ExCommandType::EX_UAV_RUN == exCmd_.cmd_type){
                    exCmd_.cmd_type = ExCommandType::EX_UAV_RUNNING;
                }
                qDebug()<<"exCmd_:"<<QString::fromStdString(exCmd_.strCmdKey)<<",查询的uid:"<<QString::fromStdString(guid)<<",执行中";
            }
        }
        autoShowOrTj();
    }  catch (std::exception& ex) {
        qDebug()<<"checkDeviceStatus:"<<ex.what();
    }

}
//接收到MQ的3D态势模型数据
ApiResponse XjSim::addOrUpdateVechile(nlohmann::json jsonData) {

    try {
        const auto& data = jsonData["data"];
        if (!data.is_array()) {
            qDebug() << "data必须是数组格式";
            return apiRes.Failure(ErrorCode::ERR, "data必须是数组格式");
        }

        std::vector<APIBridgeSDK::OwnPlatform> ownPlatforms;
        for (const auto& point : data) {
            if (!point.contains("guid")
                || !point.contains("name")
                || !point.contains("type")
                || !point.contains("position")) {
                qDebug() << "Invalid point format: guid name type or position";
                continue;
            }
            auto position = point["position"];
            if (!position.contains("accel")
                || !position.contains("health")
                || !position.contains("is_tasking")
                || !position.contains("lat")
                || !position.contains("lon")
                || !position.contains("speed")
                || !position.contains("roll")
                || !position.contains("pitch")
                || !position.contains("heading")
                || !position.contains("height")) {
                qDebug() << "Invalid point format: lat lon or height";
                continue;
            }
            string guid = point["guid"];
            string name = point["name"];
            int type = point["type"];
            int accel = position["accel"];
            bool health = position["health"];
            bool is_tasking = position["is_tasking"];
            double lat = position["lat"];
            double lon = position["lon"];
            double speed = position["speed"];
            double height = position["height"];
            double roll = position["roll"];
            double pitch = position["pitch"];
            double heading = position["heading"];

            APIBridgeSDK::OwnPlatform ownPlatform;

            // 基础平台信息
            ownPlatform.basic.model_id = guid;
            ownPlatform.basic.report_time = QDateTime::currentMSecsSinceEpoch();
            // int report_type = type == 1  ? 1001 : 2001;
            ownPlatform.basic.platform_type = type; // 仿真无人机
            ownPlatform.basic.deploy_position.longitude = lon;
            ownPlatform.basic.deploy_position.latitude = lat;
            ownPlatform.basic.deploy_position.altitude = height;
            ownPlatform.basic.velocity = speed;
            ownPlatform.basic.trk_direction = heading;
            ownPlatform.basic.roll = roll;
            ownPlatform.basic.pitch = pitch;
            ownPlatform.basic.yaw = heading;

            // 业务信息
            ownPlatform.business.health_status = health ? 0 : 1; // 0:正常,1:故障
            ownPlatform.business.exception_msg = "";
            ownPlatform.business.plat_redidual_electricity = 100; // 默认100%电量
            ownPlatform.business.task_execute_status = is_tasking ? 1 : 0; // 0:空闲,1:执行中
            ownPlatform.business.task_number = "";

            ownPlatforms.push_back(ownPlatform);
        }
        if (ownPlatforms.empty()) {
            return apiRes.Failure(ErrorCode::ERR, "没有有效的实体");
        }
        if(nullptr != this->rcp_){
            this->rcp_->m_rcpClientAdapter->createOrUpdateOwnPlatformData(ownPlatforms);
            qDebug() << "Successfully called createOrUpdateOwnPlatformData, synced" << ownPlatforms.size();

        }
        return apiRes.Success();
    } catch (const std::exception& e) {
        qDebug() << "Error processing addOrUpdateVechile command:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "更新实体失败");
    }
}
void XjSim::convertToOwnPlatforms(std::vector<APIBridgeSDK::OwnPlatform>& ownPlatforms) {
    qDebug() << "Starting to convert own platform data";

    // 转换自身平台（LandShip）
    if (!self_object_.uid.empty() && self_object_.isOnline) {
        qDebug() << "Converting self platform (LandShip):" << QString::fromStdString(self_object_.uid)
                 << "position:(" << self_object_.lat << "," << self_object_.lon << "," << self_object_.height << ")";
        APIBridgeSDK::OwnPlatform ownPlatform;

        // 基础平台信息
        ownPlatform.basic.model_id = self_object_.uid;
        ownPlatform.basic.report_time = QDateTime::currentMSecsSinceEpoch();
        ownPlatform.basic.platform_type = 2001; // 仿真无人车
        ownPlatform.basic.deploy_position.longitude = self_object_.lon;
        ownPlatform.basic.deploy_position.latitude = self_object_.lat;
        ownPlatform.basic.deploy_position.altitude = self_object_.height;
        ownPlatform.basic.velocity = self_object_.speed;
        ownPlatform.basic.trk_direction = self_object_.heading;
        ownPlatform.basic.roll = self_object_.roll;
        ownPlatform.basic.pitch = self_object_.pitch;
        ownPlatform.basic.yaw = self_object_.heading;

        // 业务信息
        ownPlatform.business.health_status = self_object_.health ? 0 : 1; // 0:正常,1:故障
        ownPlatform.business.exception_msg = "";
        ownPlatform.business.plat_redidual_electricity = 100; // 默认100%电量
        ownPlatform.business.task_execute_status = self_object_.isTasking ? 1 : 0; // 0:空闲,1:执行中
        ownPlatform.business.task_number = "";

        ownPlatforms.push_back(ownPlatform);
        qDebug() << "Successfully added self platform to own platform list";
    }

    // 转换无人机平台
    qDebug() << "Starting to convert UAV platforms, total objects count:" << total_objects_.size();
    for (const auto& pair : total_objects_) {
        const SimObject& obj = pair.second;

        // 只处理无人机类型且在线的设备，排除敌方目标
        if (obj.type == 1 && obj.isOnline && obj.name != "enemy") {
            qDebug() << "Converting UAV:" << QString::fromStdString(obj.uid)
                     << "name:" << QString::fromStdString(obj.name)
                     << "position:(" << obj.lat << "," << obj.lon << "," << obj.height << ")";
            APIBridgeSDK::OwnPlatform ownPlatform;

            // 基础平台信息
            ownPlatform.basic.model_id = obj.uid;
            ownPlatform.basic.report_time = QDateTime::currentMSecsSinceEpoch();
            ownPlatform.basic.platform_type = 1001; // 仿真无人机
            ownPlatform.basic.deploy_position.longitude = obj.lon;
            ownPlatform.basic.deploy_position.latitude = obj.lat;
            ownPlatform.basic.deploy_position.altitude = obj.height;
            ownPlatform.basic.velocity = obj.speed;
            ownPlatform.basic.trk_direction = obj.heading;
            ownPlatform.basic.roll = obj.roll;
            ownPlatform.basic.pitch = obj.pitch;
            ownPlatform.basic.yaw = obj.heading;

            // 业务信息
            ownPlatform.business.health_status = obj.health ? 0 : 1; // 0:正常,1:故障
            ownPlatform.business.exception_msg = "";
            ownPlatform.business.plat_redidual_electricity = 100; // 默认100%电量
            ownPlatform.business.task_execute_status = obj.isTasking ? 1 : 0; // 0:空闲,1:执行中
            ownPlatform.business.task_number = "";

            ownPlatforms.push_back(ownPlatform);
            qDebug() << "Successfully added UAV to own platform list";
        } else if (obj.type == 1) {
            qDebug() << "Skipping UAV:" << QString::fromStdString(obj.uid)
                     << "reason: isOnline=" << obj.isOnline << ", name=" << QString::fromStdString(obj.name);
        }
    }
    qDebug() << "Conversion completed, total own platforms:" << ownPlatforms.size();

    // 转换敌方目标为我方平台数据（敌方仿真无人车）
    convertToTargetPlatforms(ownPlatforms);
}
void XjSim::convertToTargetPlatforms(std::vector<APIBridgeSDK::OwnPlatform>& ownPlatforms) {
    qDebug() << "Starting to convert enemy target platform data";

    // 转换敌方目标（enemy）为我方平台数据
    if (!enemy_object_.uid.empty() && enemy_object_.isOnline && enemy_object_.name == "enemy") {
        qDebug() << "Converting enemy platform:" << QString::fromStdString(enemy_object_.uid)
                 << "name:" << QString::fromStdString(enemy_object_.name)
                 << "position:(" << enemy_object_.lat << "," << enemy_object_.lon << "," << enemy_object_.height << ")";
        APIBridgeSDK::OwnPlatform ownPlatform;

        // 基础平台信息
        ownPlatform.basic.model_id = enemy_object_.uid;
        ownPlatform.basic.report_time = QDateTime::currentMSecsSinceEpoch();
        ownPlatform.basic.platform_type = 2001; // 仿真无人车（敌方）
        ownPlatform.basic.deploy_position.longitude = enemy_object_.lon;
        ownPlatform.basic.deploy_position.latitude = enemy_object_.lat;
        ownPlatform.basic.deploy_position.altitude = enemy_object_.height;
        ownPlatform.basic.velocity = enemy_object_.speed;
        ownPlatform.basic.trk_direction = enemy_object_.heading;
        ownPlatform.basic.roll = enemy_object_.roll;
        ownPlatform.basic.pitch = enemy_object_.pitch;
        ownPlatform.basic.yaw = enemy_object_.heading;

        // 业务信息
        ownPlatform.business.health_status = enemy_object_.health ? 0 : 1; // 0:正常,1:故障
        ownPlatform.business.exception_msg = "";
        ownPlatform.business.plat_redidual_electricity = 100; // 默认100%电量
        ownPlatform.business.task_execute_status = enemy_object_.isTasking ? 1 : 0; // 0:空闲,1:执行中
        ownPlatform.business.task_number = "";

        ownPlatforms.push_back(ownPlatform);
        qDebug() << "Successfully added enemy platform to own platform list";
    }

    qDebug() << "Enemy target conversion completed";
}

// 火力点显示功能
ApiResponse XjSim::showFirePoints() {
    try {
        if (!this->rcp_ || !this->rcp_->m_rcpClientAdapter) {
            qDebug() << "showFirePoints error: RCP client not initialized";
            return apiRes.Failure(ErrorCode::ERR, "RCP客户端未初始化");
        }

        // 使用提供的经纬度显示火力点
        double longitude = 106.518684;  // 经度
        double latitude = 29.728331;    // 纬度
        double altitude = 400.0;        // 默认高度100米

        std::string firePointId = "fire_point_001";
        std::string note = "火力点";
        std::string color = "#FF0000";  // 红色
        int pixelRadius = 30;           // 像素半径

        // 安全处理字符串转换
        std::string convertedNote;
        try {
            convertedNote = utf8ToGbk(note);
            if (convertedNote.empty()) {
                convertedNote = "FirePoint"; // 备用英文名称
            }
        } catch (...) {
            convertedNote = "FirePoint"; // 发生异常时使用备用名称
        }

        this->rcp_->m_rcpClientAdapter->updatePointMark(
            firePointId,
            APIBridgeSDK::Position(longitude, latitude, altitude),
            pixelRadius,
            convertedNote,
            color
            );

        qDebug() << "Fire point displayed at:" << longitude << "," << latitude << "," << altitude;
        return apiRes.Success("火力点显示成功");

    } catch (const std::exception& e) {
        qDebug() << "showFirePoints error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "火力点显示失败");
    }
}

// 安全区显示功能
ApiResponse XjSim::showSafeZone() {
    try {
        if (!this->rcp_ || !this->rcp_->m_rcpClientAdapter) {
            qDebug() << "showSafeZone error: RCP client not initialized";
            return apiRes.Failure(ErrorCode::ERR, "RCP客户端未初始化");
        }

        // 使用提供的经纬度作为安全区圆心
        double longitude = 106.520170;  // 经度
        double latitude = 29.729343;    // 纬度
        double altitude = 0.0;          // 高度设为0（地面级别）

        std::string safeZoneId = "safe_zone_001";
        std::string note = "安全区";
        std::string color = "#00FF00";  // 绿色表示安全区
        int distanceRadius = 20;       // 500米半径的安全区

        // 安全处理字符串转换
        std::string convertedNote;
        try {
            convertedNote = utf8ToGbk(note);
            if (convertedNote.empty()) {
                convertedNote = "SafeZone"; // 备用英文名称
            }
        } catch (...) {
            convertedNote = "SafeZone"; // 发生异常时使用备用名称
        }

        this->rcp_->m_rcpClientAdapter->updateCircleMark(
            safeZoneId,
            APIBridgeSDK::Position(longitude, latitude, altitude),
            distanceRadius,
            convertedNote,
            color
            );

        qDebug() << "Safe zone displayed at:" << longitude << "," << latitude << "," << altitude
                 << "with radius:" << distanceRadius << "meters";
        return apiRes.Success("安全区显示成功");

    } catch (const std::exception& e) {
        qDebug() << "showSafeZone error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "安全区显示失败");
    }
}

// 规避面显示功能
ApiResponse XjSim::showAvoidanceArea() {
    try {
        if (!this->rcp_ || !this->rcp_->m_rcpClientAdapter) {
            qDebug() << "showAvoidanceArea error: RCP client not initialized";
            return apiRes.Failure(ErrorCode::ERR, "RCP客户端未初始化");
        }

        // 定义规避面区域的四个顶点坐标
        std::vector<APIBridgeSDK::Position> regionVertices;

        // 添加区域顶点（按照提供的数据）
        regionVertices.push_back(APIBridgeSDK::Position(106.51349580722064, 29.73104415147467, 276.4618486734107));
        regionVertices.push_back(APIBridgeSDK::Position(106.51521004665922, 29.730708440156192, 269.70953365415335));
        regionVertices.push_back(APIBridgeSDK::Position(106.51474466415453, 29.729389299884115, 272.48939736001194));
        regionVertices.push_back(APIBridgeSDK::Position(106.51302768290212, 29.729698481523197, 279.1792241455987));

        std::string avoidanceAreaId = "avoidance_area_001";
        std::string note = "规避面";
        std::string color = "#FFA500";  // 橙色表示警告区域

        // 安全处理字符串转换
        std::string convertedNote;
        try {
            convertedNote = utf8ToGbk(note);
            if (convertedNote.empty()) {
                convertedNote = "AvoidanceArea"; // 备用英文名称
            }
        } catch (...) {
            convertedNote = "AvoidanceArea"; // 发生异常时使用备用名称
        }

        this->rcp_->m_rcpClientAdapter->updateRegionMark(
            avoidanceAreaId,
            regionVertices,
            convertedNote,
            color
            );

        qDebug() << "Avoidance area displayed with" << regionVertices.size() << "vertices";
        return apiRes.Success("规避面显示成功");

    } catch (const std::exception& e) {
        qDebug() << "showAvoidanceArea error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "规避面显示失败");
    }
}

// 避让点显示功能
ApiResponse XjSim::showYieldPoints() {
    try {
        if (!this->rcp_ || !this->rcp_->m_rcpClientAdapter) {
            qDebug() << "showYieldPoints error: RCP client not initialized";
            return apiRes.Failure(ErrorCode::ERR, "RCP客户端未初始化");
        }

        // 使用提供的经纬度显示避让点
        double longitude = 106.514915;  // 经度
        double latitude = 29.728642;    // 纬度
        double altitude = 400.0;        // 默认高度400米

        std::string yieldPointId = "yield_point_001";
        std::string note = "避让点";
        std::string color = "#FFFF00";  // 黄色表示避让点
        int pixelRadius = 25;           // 像素半径

        // 安全处理字符串转换
        std::string convertedNote;
        try {
            convertedNote = utf8ToGbk(note);
            if (convertedNote.empty()) {
                convertedNote = "YieldPoint"; // 备用英文名称
            }
        } catch (...) {
            convertedNote = "YieldPoint"; // 发生异常时使用备用名称
        }

        this->rcp_->m_rcpClientAdapter->updatePointMark(
            yieldPointId,
            APIBridgeSDK::Position(longitude, latitude, altitude),
            pixelRadius,
            convertedNote,
            color
            );

        qDebug() << "Yield point displayed at:" << longitude << "," << latitude << "," << altitude;
        return apiRes.Success("避让点显示成功");

    } catch (const std::exception& e) {
        qDebug() << "showYieldPoints error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "避让点显示失败");
    }
}
//加载态势数据
ApiResponse XjSim::loadVechileMode() {
    try {
        if (!this->rcp_ || !this->rcp_->m_rcpClientAdapter) {
            qDebug() << "showYieldPoints error: RCP client not initialized";
            return apiRes.Failure(ErrorCode::ERR, "RCP客户端未初始化");
        }
        if(_cfg.areas.empty()){
            qDebug() << "loadVechileMode error:没有态势数据";
            return apiRes.Failure(ErrorCode::ERR, "没有态势数据");
        }

        for(AreaInfo info:_cfg.areas){
            QJsonObject jsonData;
            jsonData["cmd"] = "UPATE_REGION";
            QString name =  QString::fromStdString(info.name);
            jsonData["sn"] = name;
            jsonData["plot_name"] = name;
            QJsonArray data;
            for(LatLonPoint point :info.lat_lons){
                QJsonObject jsonPoint;
                jsonPoint["lat"] = point.lat;
                jsonPoint["lon"] = point.lon;
                jsonPoint["height"] = point.height;
                data.append(jsonPoint);
            }
            jsonData["data"] = data;
            QJsonDocument doc(jsonData);
            qDebug() << "loadVechileMode :" <<name;
            QString jsonString = doc.toJson(QJsonDocument::Compact);
            if (!revData.empty()) {
                revData.clear();
            }
            revData = (nlohmann::json::parse(jsonString.toStdString()));
            addAreaToMap(revData);
        }
        std::string speak_msg = "在区域1发现可疑目标，请指挥人员针对该区域开展作战任务";

        if (this->rcp_->m_rcpClientAdapter) {
            this->rcp_->m_rcpClientAdapter->showChatReply(utf8ToGbk(speak_msg));
        }
        nlohmann::json speak_data;
        speak_data["data"] = speak_msg;
        this->mqClient->GetMqttConn()->NormalPublish("root/llm/app/speak",speak_data.dump());
        this->mqClient->GetMqttConn()->NormalPublish("QB/303",qB_303);
        return apiRes.Success("加载态势数据成功");
    } catch (const std::exception& e) {
        qDebug() << "showYieldPoints error:" << e.what();
        return apiRes.Failure(ErrorCode::ERR, "加载态势数据失败");
    }
}

void XjSim::PublishMD(QString name, int type, double lon, double lat)
{
    QJsonObject root;
    MQTTMission::Position pos;
    pos.longitude = lon;
    pos.latitude = lat;
    PackageDzmbqb_301(name,type,root,pos);
    this->mqClient->GetMqttConn()->PrimitivePublish("QB/301", root);
}

bool XjSim::PackageDzmbqb_301(QString name,int type, QJsonObject &root,MQTTMission::Position &pos)
{
    addHeader(root);
    QJsonObject CTN;
    // if(test_id++ > 150){
    //      return false;
    //  }
    CTN["target_ID"]="dyy"+name+QString(test_id);
    CTN["target_name"]=name;
    CTN["battle_unit_code"]="dyy"+name+QString(test_id);
    CTN["data_source"]="dy04";
    CTN["first_time"]=currentTimeMillisec();
    CTN["latest_time"]=currentTimeMillisec();

    QJsonObject track_info;
    QJsonObject position;

    position["longitude"] =pos.longitude;
    position["latitude"] =pos.latitude;
    position["altitude"] =pos.altitude;

    track_info["position"] =position;
    track_info["speed"] =30;
    track_info["track_direction"] =74;

    CTN["track_info"]=track_info;
    CTN["capture_count"]=1;
    CTN["threat_level"]=-1;
    CTN["scout_type"]=1;
    CTN["target_plat_type"] = type; //4飞机，2 军用车辆
    root["CTN"] =CTN;
    test_id++;
    return true;
}

bool XjSim::PackageZzptjzbzt_303(QJsonObject &root,QString running_status,excuteUAVOrUGV& ab,MQTTMission::Position &pos)
{
    addHeader(root);
    QJsonObject CTN;
    CTN["report_time"] = currentTimeMillisec();
    CTN["battle_unit_code"]  =QString::fromStdString(ab.uid);
    CTN["battle_unit_name"] = QString::fromStdString(ab.uname);
    CTN["task_type"] = QString::fromStdString(ab.task_type);
    CTN["roll"] = ab.roll;
    CTN["pitch"] = ab.pitch;
    CTN["yaw"] = ab.yaw;
    CTN["running_status"] =running_status;
    CTN["platform_type"] =ab.platform_type;
    QString cmd = QString::fromStdString(ab.cmd);
    // if(cmd == "RUN_UAV_UID"){

    //    CTN["platform_type"]  =1;

    // }else if(cmd == "RUN_UGV_UID"){
    //     CTN["platform_type"]  =2; // 1无人机 ，2 无人车，3 单兵
    // }

    QJsonObject deploy_position;

    deploy_position["longitude"] =pos.longitude;
    deploy_position["latitude"] =pos.latitude;
    deploy_position["altitude"] =ab.executeHeight;
    CTN["deploy_position"]  =deploy_position;
    CTN["velocity"]  =ab.waypointSpeed;
    root["CTN"] =CTN;
    return true;
}

bool XjSim::PackageJSBY_dzmbpt_301(QJsonObject &root, QString id, APIBridgeSDK::Position& point, int speed, int type)
{
    addHeader(root);
    QJsonObject CTN;
    CTN["target_ID"]=id;
    CTN["target_name"]="博弈_"+id;
    CTN["battle_unit_code"]="dyy";
    CTN["data_source"]="dy04";
    CTN["first_time"]=currentTimeMillisec();
    CTN["latest_time"]=currentTimeMillisec();

    QJsonObject track_info;
    QJsonObject position;

    position["longitude"] =point.longitude;
    position["latitude"] =point.latitude;
    position["altitude"] =point.altitude;

    track_info["position"] =position;
    track_info["speed"] =30;
    track_info["track_direction"] =74;

    CTN["track_info"]=track_info;
    CTN["capture_count"]=1;
    CTN["threat_level"]=-1;
    CTN["scout_type"]=1;
    CTN["target_plat_type"]=4; //4飞机，2 军用车辆
    root["CTN"] =CTN;

    return true;
}

void XjSim::planZB(excuteUAVOrUGV &zb)
{
    if (!std::isfinite(zb.waypointSpeed) || zb.waypointSpeed <= 0) return;
    for (std::size_t index = 0; index < zb.path.size(); ++index) {
        MQTTMission::Position pos = zb.path[index];
        QJsonObject root;
        const bool lastPoint = index + 1 == zb.path.size();
        QString runing_status = lastPoint ? "idle" : "working";
        PackageZzptjzbzt_303(root,runing_status,zb,pos);

        if (index == 0) Sleep(30000);
        this->mqClient->GetMqttConn()->PrimitivePublish("QB/303", root);

        if (!lastPoint) {
            const auto& next = zb.path[index + 1];
            const double distanceMeters = haversineDistance(
                GeoLocation{pos.latitude, pos.longitude},
                GeoLocation{next.latitude, next.longitude}) * 1000.0;
            const double intervalSeconds = distanceMeters / zb.waypointSpeed;
            if (intervalSeconds > 0.0) {
                Sleep(static_cast<int>(intervalSeconds * 1000));
            }
        } else {
            const auto& next = zb.path[0];
            const double distanceMeters = haversineDistance(
                GeoLocation{pos.latitude, pos.longitude},
                GeoLocation{next.latitude, next.longitude}) * 1000.0;
            const double intervalSeconds = distanceMeters / zb.waypointSpeed;
            if (intervalSeconds > 0.0) {
                Sleep(static_cast<int>(intervalSeconds * 1000));
            }
            MQTTMission::Position pos = zb.path[0];
            QJsonObject root;
            QString runing_status = "idle";
            PackageZzptjzbzt_303(root,runing_status,zb,pos);
            this->mqClient->GetMqttConn()->PrimitivePublish("QB/303", root);
        }
    }
}

void XjSim::slotLoadScheme()
{
    this->rcp_->slotLoadScheme();
}

// 处理 llm/701/jsim/status 主题的设备状态数据
void XjSim::processJsimStatusData(const std::string& data) {
    try {
        nlohmann::json jsonData = nlohmann::json::parse(data);

        // 验证必需字段
        if (!jsonData.contains("devices") || !jsonData["devices"].is_array()) {
            qDebug() << "processJsimStatusData error: missing 'devices' array";
            return;
        }

        // 创建 OwnPlatform 向量来存储所有设备信息
        std::vector<APIBridgeSDK::OwnPlatform> ownPlatforms;

        const auto& devices = jsonData["devices"];
        for (const auto& device : devices) {
            try {
                // 验证必需字段
                if (!device.contains("UAVplatid") || !device.contains("type") ||
                    !device.contains("lat") || !device.contains("lon") ||
                    !device.contains("alt")) {
                    qDebug() << "Device missing required fields, skipping";
                    continue;
                }

                // 提取设备字段
                // UAVplatid 可能是整数，需要转换为字符串
                std::string platid;
                if (device["UAVplatid"].is_number()) {
                    platid = std::to_string(device["UAVplatid"].get<int>());
                } else {
                    platid = device["UAVplatid"].get<std::string>();
                }

                // type 是字符串 "uav" 或 "ugv"
                std::string typeStr = device["type"].get<std::string>();
                int platformType = (typeStr == "uav") ? 1001 : 2001;

                double lat = device["lat"].get<double>();
                double lon = device["lon"].get<double>();
                double altitude = device["alt"].get<double>();
                double heading = device.contains("head") ? device["head"].get<double>() : 0.0;
                double speed = device.contains("speed") ? device["speed"].get<double>() : 0.0;
                double yaw = device.contains("yaw") ? device["yaw"].get<double>() : heading;
                int taskstatus = device.contains("taskstatus") ? device["taskstatus"].get<int>() : 0;

                // 创建 OwnPlatform 对象
                APIBridgeSDK::OwnPlatform ownPlatform;

                // 基础平台信息
                ownPlatform.basic.model_id = platid;
                ownPlatform.basic.report_time = QDateTime::currentMSecsSinceEpoch();
                ownPlatform.basic.platform_type = platformType;
                ownPlatform.basic.deploy_position.longitude = lon;
                ownPlatform.basic.deploy_position.latitude = lat;
                ownPlatform.basic.deploy_position.altitude = altitude;
                ownPlatform.basic.velocity = static_cast<float>(speed);
                ownPlatform.basic.trk_direction = static_cast<float>(heading);
                ownPlatform.basic.yaw = static_cast<float>(yaw);
                ownPlatform.basic.roll = 0.0f;
                ownPlatform.basic.pitch = 0.0f;

                // 业务信息
                ownPlatform.business.health_status = 0;  // 0:正常
                ownPlatform.business.exception_msg = "";
                ownPlatform.business.plat_redidual_electricity = 100;  // 默认100%电量
                ownPlatform.business.task_execute_status = taskstatus;  // 任务执行状态
                ownPlatform.business.task_number = "";

                ownPlatforms.push_back(ownPlatform);

                qDebug() << "processJsimStatusData: Processed device:" << QString::fromStdString(platid)
                         << "Type:" << QString::fromStdString(typeStr)
                         << "Position:(" << lat << "," << lon << "," << altitude << ")";

            } catch (const std::exception& e) {
                qDebug() << "processJsimStatusData: Error processing device:" << e.what();
                continue;
            }
        }

        // 批量更新我方平台数据
        if (!ownPlatforms.empty()) {
            if (nullptr != this->rcp_ && nullptr != this->rcp_->m_rcpClientAdapter) {
                this->rcp_->m_rcpClientAdapter->createOrUpdateOwnPlatformData(ownPlatforms);
                qDebug() << "processJsimStatusData: Successfully updated" << ownPlatforms.size() << "platforms";
            } else {
                qDebug() << "processJsimStatusData: RCP client not initialized";
            }
        }

    } catch (const nlohmann::json::exception& e) {
        qDebug() << "processJsimStatusData: JSON parsing error:" << e.what();
    } catch (const std::exception& e) {
        qDebug() << "processJsimStatusData: Error:" << e.what();
    }
}
void XjSim::addHeader(QJsonObject& root){
    root["SID"] = "ZK";
    int64_t t=  currentTimeMillisec();
    root["TM"] =t;
}
bool XjSim::ControlDJiMini4_Pro(const string& rid, double executeHeight, double waypointSpeed, std::vector<MQTTMission::Position> &path)
{
    if(path.size() < 2) return false;
    QJsonObject root;
    addHeader(root);
    QJsonObject CTN;
    CTN["rid"] = QString::fromStdString(rid);
    CTN["wpml_executeHeight"] = executeHeight;
    CTN["wpml_waypointSpeed"] = waypointSpeed;
    QJsonObject line_list;
    QJsonArray point_list;

    for(MQTTMission::Position position:path){
        QJsonObject pos;
        pos["longitude"] =position.longitude;
        pos["latitude"]  =position.latitude;
        pos["altitude"]  =position.altitude;
        point_list.push_back(pos);
    }
    line_list["point_list"] = point_list;
    CTN["line_list"] =line_list;
    root["CTN"] = CTN;
    this->mqClient->GetMqttConn()->PrimitivePublish("ZK/PathTopic", root);
    //  _prcp_client->m_rcpClientAdapter->showNotification("MQTT消息","无人机行动控制下发成功",0,4);
    return true;

}

void XjSim::ControlDJiMini4_Pro_Gimbal_pitch(const QString& uid)
{
    const QString rid = uid.trimmed();
    if (rid.isEmpty()) return;
    const QString commandId = QStringLiteral("gimbal-%1-%2").arg(uid.trimmed()).arg(QDateTime::currentMSecsSinceEpoch());
    QJsonObject command;
    command["type"] = "gimbal_pitch";
    command["rid"] = rid;
    command["command_id"] = commandId;
    command["mode"] = "absolute";
    command["pitch"] = -90;
    command["gimbal_index"] = "LEFT_OR_MAIN";
    mqClient->GetMqttConn()->PrimitivePublish(QStringLiteral("ZK/GimbalTopic"), command);
}