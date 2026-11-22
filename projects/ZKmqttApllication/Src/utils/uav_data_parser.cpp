#include "uav_data_parser.h"
#include <cmath>
#include <QtMath>

DroneDataParser::DroneDataParser(QObject *parent) : QObject(parent)
{
}

std::shared_ptr<DroneData> DroneDataParser::parseJson(const QJsonObject& jsonObj)
{
    auto droneData = std::make_shared<DroneData>();

    // 解析顶层数据
    droneData->tid = jsonObj["tid"].toString();
    droneData->bid = jsonObj["bid"].toString();
    droneData->timestamp = jsonObj["timestamp"].toVariant().toLongLong();
    droneData->gateway = jsonObj["gateway"].toString();

    // 解析data对象
    QJsonObject data = jsonObj["data"].toObject();

    // 使用辅助函数解析各个部分
    parseNetworkState(data["network_state"].toObject(), droneData->network_state);
    parseDroneChargeState(data["drone_charge_state"].toObject(), droneData->drone_charge_state);
    parseAlternateLandPoint(data["alternate_land_point"].toObject(), droneData->alternate_land_point);
    parsePositionState(data["position_state"].toObject(), droneData->position_state);
    parseStorage(data["storage"].toObject(), droneData->storage);
    parseSubDevice(data["sub_device"].toObject(), droneData->sub_device);

    // 解析基本数据
    droneData->drone_in_dock = data["drone_in_dock"].toInt();
    droneData->rainfall = data["rainfall"].toInt();
    droneData->wind_speed = data["wind_speed"].toInt();
    droneData->environment_temperature = data["environment_temperature"].toInt();
    droneData->temperature = data["temperature"].toDouble();
    droneData->humidity = data["humidity"].toInt();
    droneData->latitude = data["latitude"].toDouble();
    droneData->longitude = data["longitude"].toDouble();
    droneData->height = data["height"].toDouble();
    droneData->first_power_on = data["first_power_on"].toVariant().toLongLong();
    droneData->mode_code = data["mode_code"].toInt();
    droneData->cover_state = data["cover_state"].toInt();
    droneData->supplement_light_state = data["supplement_light_state"].toInt();
    droneData->emergency_stop_state = data["emergency_stop_state"].toInt();
    droneData->air_conditioner_mode = data["air_conditioner_mode"].toInt();
    droneData->battery_store_mode = data["battery_store_mode"].toInt();
    droneData->alarm_state = data["alarm_state"].toInt();
    droneData->putter_state = data["putter_state"].toInt();

    return droneData;
}

std::shared_ptr<DroneData> DroneDataParser::parseFlyJson(const QJsonObject& jsonObj){
    auto droneData = std::make_shared<DroneData>();


    QJsonObject rootObj = jsonObj;

    // 解析顶层数据
    QString method = rootObj["method"].toString();

    // 解析data对象
    if (!rootObj.contains("data") || !rootObj["data"].isObject()) {
        qDebug() << "缺少data字段或格式错误";
        return droneData;
    }

    QJsonObject dataObj = rootObj["data"].toObject();
    droneData->fly_result_data.fly_to_id = dataObj["fly_to_id"].toString();
    droneData->fly_result_data.remaining_distance = dataObj["remaining_distance"].toDouble();
    droneData->fly_result_data.remaining_time = dataObj["remaining_time"].toDouble();
    droneData->fly_result_data.result = dataObj["result"].toInt();
    droneData->fly_result_data.status = dataObj["status"].toString();
    droneData->fly_result_data.way_point_index = dataObj["way_point_index"].toInt();
    droneData->fly_result_data.method = method;

    // 解析planned_path_points数组
    if (!dataObj.contains("planned_path_points") || !dataObj["planned_path_points"].isArray()) {
        qDebug() << "缺少planned_path_points字段或格式错误";
        return droneData;
    }

    QJsonArray pointsArray = dataObj["planned_path_points"].toArray();
    for (const QJsonValue& pointValue : pointsArray) {
        if (!pointValue.isObject()) {
            continue;
        }

        QJsonObject pointObj = pointValue.toObject();

        droneData->fly_result_data.info.height = pointObj["height"].toDouble();
        droneData->fly_result_data.info.lat = pointObj["latitude"].toDouble();
        droneData->fly_result_data.info.lon = pointObj["longitude"].toDouble();
    }

    return droneData;
}


bool DroneDataParser::parseNetworkState(const QJsonObject& obj, NetworkState& state)
{
    if (obj.isEmpty()) return false;

    state.type = obj["type"].toInt();
    state.quality = obj["quality"].toInt();
    state.rate = obj["rate"].toDouble();
    return true;
}

bool DroneDataParser::parseDroneChargeState(const QJsonObject& obj, DroneChargeState& state)
{
    if (obj.isEmpty()) {
        return false;
    }

    state.state = obj["state"].toInt();
    state.capacity_percent = obj["capacity_percent"].toInt();
    return true;
}

bool DroneDataParser::parseAlternateLandPoint(const QJsonObject& obj, AlternateLandPoint& point)
{
    if (obj.isEmpty()) {
        return false;
    }

    point.latitude = obj["latitude"].toDouble();
    point.longitude = obj["longitude"].toDouble();
    point.safe_land_height = obj["safe_land_height"].toDouble();
    point.is_configured = obj["is_configured"].toInt();
    return true;
}

bool DroneDataParser::parsePositionState(const QJsonObject& obj, PositionState& state)
{
    if (obj.isEmpty()) {
        return false;
    }

    state.is_calibration = obj["is_calibration"].toInt();
    state.is_fixed = obj["is_fixed"].toInt();
    state.quality = obj["quality"].toInt();
    state.gps_number = obj["gps_number"].toInt();
    state.rtk_number = obj["rtk_number"].toInt();
    return true;
}

bool DroneDataParser::parseStorage(const QJsonObject& obj, Storage& storage)
{
    if (obj.isEmpty()) {
        return false;
    }

    storage.total = obj["total"].toVariant().toLongLong();
    storage.used = obj["used"].toVariant().toLongLong();
    return true;
}

bool DroneDataParser::parseSubDevice(const QJsonObject& obj, SubDevice& device)
{
    if (obj.isEmpty()) {
        return false;
    }

    device.device_sn = obj["device_sn"].toString();
    device.device_model_key = obj["device_model_key"].toString();
    device.device_online_status = obj["device_online_status"].toInt();
    device.device_paired = obj["device_paired"].toInt();
    return true;
}

void DroneDataParser::printDroneData(const  std::shared_ptr<DroneData>& data)
{
    if (!data) return;

    qDebug() << "Drone Data:";
    qDebug() << "TID:" << data->tid;
    qDebug() << "BID:" << data->bid;
    qDebug() << "Timestamp:" << data->timestamp;
    // ... 打印其他数据
}

bool DroneDataParser::validateData(const std::shared_ptr<DroneData>& data)
{
    if (!data){
        return false;
    }
    //验证数据正确性

    return true;
}

bool DroneDataParser::isWithinDistance(double lat1, double lon1, double alt1, 
                                     double lat2, double lon2, double alt2, 
                                     double maxDistanceMeters)
{
    
    // 将经纬度转换为弧度
    double lat1Rad = qDegreesToRadians(lat1);
    double lon1Rad = qDegreesToRadians(lon1);
    double lat2Rad = qDegreesToRadians(lat2);
    double lon2Rad = qDegreesToRadians(lon2);
    
    // 计算经纬度差值
    double deltaLat = lat2Rad - lat1Rad;
    double deltaLon = lon2Rad - lon1Rad;
    
    // Haversine公式计算水平距离
    double a = qSin(deltaLat/2) * qSin(deltaLat/2) +
               qCos(lat1Rad) * qCos(lat2Rad) *
               qSin(deltaLon/2) * qSin(deltaLon/2);
    double c = 2 * qAtan2(qSqrt(a), qSqrt(1-a));
    double horizontalDistance = EARTHRADIUS * c;
    
    // 计算高度差
    double verticalDistance = qAbs(alt2 - alt1);
    
    // 计算三维空间中的直线距离（毕达哥拉斯定理）
    double distance = qSqrt(horizontalDistance * horizontalDistance + 
                          verticalDistance * verticalDistance);
    
    // 判断距离是否小于最大允许距离
    return distance <= maxDistanceMeters;
}

bool DroneDataParser::isWithinHorizontalDistance(double lat1, double lon1,
                                               double lat2, double lon2,
                                               double maxDistanceMeters)
{
    
    // 将经纬度转换为弧度
    double lat1Rad = qDegreesToRadians(lat1);
    double lon1Rad = qDegreesToRadians(lon1);
    double lat2Rad = qDegreesToRadians(lat2);
    double lon2Rad = qDegreesToRadians(lon2);
    
    // 计算经纬度差值
    double deltaLat = lat2Rad - lat1Rad;
    double deltaLon = lon2Rad - lon1Rad;
    
    // Haversine公式计算水平距离
    double a = qSin(deltaLat/2) * qSin(deltaLat/2) +
               qCos(lat1Rad) * qCos(lat2Rad) *
               qSin(deltaLon/2) * qSin(deltaLon/2);
    double c = 2 * qAtan2(qSqrt(a), qSqrt(1-a));
    double horizontalDistance = EARTHRADIUS * c;
//    qDebug()<<"isWithinHorizontalDistance:"<<horizontalDistance;
    // 判断距离是否小于最大允许距离
    return horizontalDistance <= maxDistanceMeters;
}
