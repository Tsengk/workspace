#ifndef DRONE_DATA_PARSER_H
#define DRONE_DATA_PARSER_H

#include <QObject>
#include"uavstatusdefine.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <memory>
class DroneDataParser : public QObject
{
    Q_OBJECT

public:
    explicit DroneDataParser(QObject *parent = nullptr);
    ~DroneDataParser() = default;

    static  std::shared_ptr<DroneData> parseJson(const QJsonObject& jsonStr);
    static  std::shared_ptr<DroneData> parseFlyJson(const QJsonObject& jsonStr);
    //static QString serializeToJson(const std::shared_ptr<DroneData>& data);
    static void printDroneData(const std::shared_ptr<DroneData>& data);
    static bool validateData(const std::shared_ptr<DroneData>& data);
    // 判断两个经纬度点之间的距离是否小于指定值（包含高程）
    bool isWithinDistance(double lat1, double lon1, double alt1,
                               double lat2, double lon2, double alt2, 
                               double maxDistanceMeters);
    // 判断两个经纬度点之间的水平距离是否小于指定值（不考虑高程）
    bool isWithinHorizontalDistance(double lat1, double lon1,
                                         double lat2, double lon2,
                                         double maxDistanceMeters);

private:
    static bool parseNetworkState(const QJsonObject& obj, NetworkState& state);
    static bool parseDroneChargeState(const QJsonObject& obj, DroneChargeState& state);
    static bool parseAlternateLandPoint(const QJsonObject& obj, AlternateLandPoint& point);
    static bool parsePositionState(const QJsonObject& obj, PositionState& state);
    static bool parseStorage(const QJsonObject& obj, Storage& storage);
    static bool parseSubDevice(const QJsonObject& obj, SubDevice& device);
private:
     double EARTHRADIUS = 6371000.0; // 地球平均半径（米）
};

#endif // DRONE_DATA_PARSER_H
