#ifndef COMMONCFG_H
#define COMMONCFG_H
#include <string>
#include <QSettings>
#include <QDebug>
#include <QCoreApplication>
#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QTextStream>
#include<iostream>
#include "utm/map_define.h"

using namespace std;

// 定义坐标点结构体
struct LatLonPoint {
    double lat;
    double lon;
    double height;
    
    LatLonPoint() : lat(0), lon(0), height(0) {}
    LatLonPoint(double latitude, double longitude, double h) 
        : lat(latitude), lon(longitude), height(h) {}
};

// 定义区域结构体
struct AreaInfo {
    string name;
    string color;
    vector<LatLonPoint> lat_lons;
    
    AreaInfo() {}
    AreaInfo(const string& n, const string& c) : name(n), color(c) {}
};
struct CommonCfg
{

    double SIM_X1 = 450.30860000000001;   // 仿真坐标点1 X
    double SIM_Y1 = -58.724089999999997;   // 仿真坐标点1 Y
    double LAT1 = 29.730049483533421; // 对应的WGS84纬度1
    double LON1 = 106.51781357824804; // 对应的WGS84经度1

    double SIM_X2 = 476.06959999999998;   // 仿真坐标点2 X
    double SIM_Y2 = -60.565480000000001;   // 仿真坐标点2 Y
    double LAT2 = 29.73003201487699; // 对应的WGS84纬度2
    double LON2 = 106.51808112859727; // 对应的WGS84经度2


    string ip = "8.137.149.129";
    string username = "";
    string passwrd ="";
    int16_t port = 1883;
    int16_t keepalive = 60;

    //rpc
    string rpc_ip = "192.168.100.9";
    int rpc_port = 9000;
    //仿真
    int status_interval = 1000;//上传频率ms
    int check_interval = 300;//检查频率ms
    int check_times = 4;//状态检测

    string status_topic="/llm/jsim/status";
    string cmd_topic="/llm/jsim/excute";
    string cmd_respose_topic="/llm/jsim/respose";
    //目标经纬度
    double fpv_lat = 29.72915013491906;
    double fpv_lon = 106.51487052440643;
    double fpv_alt = 0;
    //识别到多少米之后敌方车辆开始自主任务
    double dj_distance = 55;
    double zc_distance = 35;
    string target_file = "C:/Users/53007/Pictures/Screenshots/screen.png";
    string areas_json_file = "./jl2.json";  // JSON文件路径
    vector<AreaInfo> areas;
    CommonCfg() {}

    // 从ini文件加载配置
    bool loadFromIni(const QString& filePath = QString("%1/config.ini").arg(QCoreApplication::applicationDirPath())) {
        QSettings settings(filePath, QSettings::IniFormat);

        if (!settings.contains("MQTT/ip")) {
            qDebug() << "配置文件不存在或格式错误:" << filePath;
            
            // 如果文件存在，先删除
            QFile file(filePath);
            if (file.exists()) {
                file.remove();
            }

            // 手动创建并写入配置文件
            QFile newFile(filePath);
            if (!newFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                qDebug() << "无法创建配置文件:" << filePath;
                return false;
            }

            QTextStream out(&newFile);
            out.setCodec("UTF-8");  // 设置UTF-8编码

            // 写入配置内容
            out << "[CoordinateTransform]\n"
                << QString("SIM_X1=%1\n").arg(SIM_X1)
                << QString("SIM_Y1=%1\n").arg(SIM_Y1)
                << QString("LAT1=%1\n").arg(LAT1)
                << QString("LON1=%1\n").arg(LON1)
                << QString("SIM_X2=%1\n").arg(SIM_X2)
                << QString("SIM_Y2=%1\n").arg(SIM_Y2)
                << QString("LAT2=%1\n").arg(LAT2)
                << QString("LON2=%1\n\n").arg(LON2)

                << "[MQTT]\n"
                << QString("ip=%1\n").arg(QString::fromStdString(ip))
                << QString("username=%1\n").arg(QString::fromStdString(username))
                << QString("password=%1\n").arg(QString::fromStdString(passwrd))
                << QString("port=%1\n").arg(port)
                << QString("keepalive=%1\n\n").arg(keepalive)


                << "[RPC]\n"
                << QString("ip=%1\n").arg(QString::fromStdString(rpc_ip))
                << QString("port=%1\n\n").arg(rpc_port)

                << "[Simulation]\n"
//                << ";上传频率ms\n"
                << QString("status_interval=%1\n").arg(status_interval)
//                << ";检查频率ms\n"
                << QString("check_interval=%1\n").arg(check_interval)
//                << ";状态检测\n"
                << QString("check_times=%1\n\n").arg(check_times)

                << "[Topics]\n"
//                << ";上传topic\n"
                << QString("status_topic=%1\n").arg(QString::fromStdString(status_topic))
//                << ";命令topic\n"
                << QString("cmd_topic=%1\n").arg(QString::fromStdString(cmd_topic))
//                << ";命令响应topic\n"
                << QString("cmd_response_topic=%1\n\n").arg(QString::fromStdString(cmd_respose_topic))

                << "[FPV]\n"
//                << ";纬度\n"
                << QString("latitude=%1\n").arg(fpv_lat)
//                << ";经度\n"
                << QString("longitude=%1\n").arg(fpv_lon)
//                << ";高度\n"
                << QString("altitude=%1\n").arg(fpv_alt)
                << "[Area]\n"
                << QString("areas=%1\n").arg(QString::fromStdString(areas_json_file))
                
                << "[distance]\n"
                << QString("dj_distance=%1\n").arg(dj_distance)
                << QString("zc_distance=%1\n").arg(zc_distance)
                << "\n[path]\n"
                << QString("target_file=%1\n").arg(QString::fromStdString(target_file));
            newFile.close();
            qDebug() << "已创建新的配置文件并写入默认值:" << filePath;
            
            // 重新加载设置
            settings.sync();
        }

        //仿真坐标点
        SIM_X1 = settings.value("CoordinateTransform/SIM_X1", SIM_X1).toDouble();
        SIM_Y1 = settings.value("CoordinateTransform/SIM_Y1", SIM_Y1).toDouble();
        LAT1 = settings.value("CoordinateTransform/LAT1", LAT1).toDouble();
        LON1 = settings.value("CoordinateTransform/LON1", LON1).toDouble();

        SIM_X2 = settings.value("CoordinateTransform/SIM_X2", SIM_X2).toDouble();
        SIM_Y2 = settings.value("CoordinateTransform/SIM_Y2", SIM_Y2).toDouble();
        LAT2 = settings.value("CoordinateTransform/LAT2", LAT2).toDouble();
        LON2 = settings.value("CoordinateTransform/LON2", LON2).toDouble();

        // MQTT配置
        ip = settings.value("MQTT/ip", ip.c_str()).toString().toStdString();
        username = settings.value("MQTT/username", username.c_str()).toString().toStdString();
        passwrd = settings.value("MQTT/password", passwrd.c_str()).toString().toStdString();
        port = settings.value("MQTT/port", port).toInt();
        keepalive = settings.value("MQTT/keepalive", keepalive).toInt();
        // 读取Area JSON文件路径
        areas_json_file = settings.value("Area/areas", areas_json_file.c_str()).toString().toStdString();
        qDebug() << "Area JSON文件路径:" << QString::fromStdString(areas_json_file);

        // 加载JSON文件数据
        if (!loadAreasFromJsonFile(QString::fromStdString(areas_json_file))) {
            qDebug() << "无法加载Area JSON文件:" << QString::fromStdString(areas_json_file);
        }

        // RPC配置
        rpc_ip = settings.value("RPC/ip", rpc_ip.c_str()).toString().toStdString();
        rpc_port = settings.value("RPC/port", rpc_port).toInt();

        // 仿真配置
        status_interval = settings.value("Simulation/status_interval", status_interval).toInt();
        check_interval = settings.value("Simulation/check_interval", check_interval).toInt();
        check_times = settings.value("Simulation/check_times", check_times).toInt();

        // Topic配置
        status_topic = settings.value("Topics/status_topic", status_topic.c_str()).toString().toStdString();
        cmd_topic = settings.value("Topics/cmd_topic", cmd_topic.c_str()).toString().toStdString();
        cmd_respose_topic = settings.value("Topics/cmd_response_topic", cmd_respose_topic.c_str()).toString().toStdString();

        // FPV配置
        fpv_lat = settings.value("FPV/latitude", fpv_lat).toDouble();
        fpv_lon = settings.value("FPV/longitude", fpv_lon).toDouble();
        fpv_alt = settings.value("FPV/altitude", fpv_alt).toDouble();

        // 距离配置
        dj_distance = settings.value("distance/dj_distance", dj_distance).toDouble();
        zc_distance = settings.value("distance/zc_distance", zc_distance).toDouble();
        
        // 路径配置
        target_file = settings.value("path/target_file", target_file.c_str()).toString().toStdString();
        qDebug() << "配置文件加载成功:" << filePath<<",target_file:"<<QString::fromStdString(target_file);
        return true;
    }
    // 从JSON文件加载区域数据
    bool loadAreasFromJsonFile(const QString& jsonFilePath) {
        QFile file(jsonFilePath);
        if (!file.exists()) {
            qDebug() << "JSON文件不存在:" << jsonFilePath;
            return false;
        }
        
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qDebug() << "无法打开JSON文件:" << jsonFilePath;
            return false;
        }
        
        QByteArray jsonData = file.readAll();
        file.close();
        
        return parseAreaJsonConfig(jsonData);
    }

    // 解析JSON配置数据
    bool parseAreaJsonConfig(const QByteArray& jsonData) {
        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(jsonData, &parseError);
        
        if (parseError.error != QJsonParseError::NoError) {
            qDebug() << "JSON解析错误:" << parseError.errorString();
            return false;
        }
        
        if (doc.isNull() || !doc.isObject()) {
            qDebug() << "JSON文档格式错误";
            return false;
        }
        
        QJsonObject rootObj = doc.object();
        areas.clear();
        
        // 解析polygons数组
        if (rootObj.contains("polygons") && rootObj["polygons"].isArray()) {
            QJsonArray polygonsArray = rootObj["polygons"].toArray();
            
            for (const QJsonValue& polygonValue : polygonsArray) {
                if (polygonValue.isObject()) {
                    QJsonObject polygonObj = polygonValue.toObject();
                    AreaInfo area;
                    
                    // 解析区域名称
                    area.name = polygonObj["name"].toString().toStdString();
                    
                    // 解析颜色（从r,g,b分量计算）
                    int r = polygonObj["r"].toInt(255);
                    int g = polygonObj["g"].toInt(255);
                    int b = polygonObj["b"].toInt(255);
                    area.color = QString("#%1%2%3")
                        .arg(r, 2, 16, QLatin1Char('0'))
                        .arg(g, 2, 16, QLatin1Char('0'))
                        .arg(b, 2, 16, QLatin1Char('0'))
                        .toStdString();
                    
                    // 解析坐标点数组
                    if (polygonObj.contains("lat_lons") && polygonObj["lat_lons"].isArray()) {
                        QJsonArray pointsArray = polygonObj["lat_lons"].toArray();
                        
                        for (const QJsonValue& pointValue : pointsArray) {
                            if (pointValue.isObject()) {
                                QJsonObject pointObj = pointValue.toObject();
                                LatLonPoint point;
                                
                                point.lat = pointObj["lat"].toDouble();
                                point.lon = pointObj["lon"].toDouble();
                                point.height = pointObj["height"].toDouble();  // 默认高度，可根据需要调整
                                area.lat_lons.push_back(point);
                            }
                        }
                    }
                    
                    areas.push_back(area);
                }
            }
        }
        qDebug() << "成功从JSON文件加载" << areas.size() << "个区域配置";
        return true;
    }
};
#endif //COMMON_HPP
