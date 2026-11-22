#ifndef DRONE_DATA_PARSER_H
#define DRONE_DATA_PARSER_H
#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <memory>
//class DroneDataParser : public QObject
//{
//    Q_OBJECT

//public:
//    explicit DroneDataParser(QObject *parent = nullptr);
//    ~DroneDataParser() = default;

   // static  std::shared_ptr<DroneData> parseJson(const QJsonObject& jsonStr);
   // static  std::shared_ptr<DroneData> parseFlyJson(const QJsonObject& jsonStr);
    //static QString serializeToJson(const std::shared_ptr<DroneData>& data);
    //static void printDroneData(const std::shared_ptr<DroneData>& data);
//    static bool validateData(const std::shared_ptr<DroneData>& data);

//private:
//    static bool parseNetworkState(const QJsonObject& obj, NetworkState& state);
//    static bool parseDroneChargeState(const QJsonObject& obj, DroneChargeState& state);
//    static bool parseAlternateLandPoint(const QJsonObject& obj, AlternateLandPoint& point);
//    static bool parsePositionState(const QJsonObject& obj, PositionState& state);
//    static bool parseStorage(const QJsonObject& obj, Storage& storage);
//    static bool parseSubDevice(const QJsonObject& obj, SubDevice& device);
//};

#endif // DRONE_DATA_PARSER_H
