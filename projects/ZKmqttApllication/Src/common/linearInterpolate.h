#ifndef linearInterpolate_H
#define linearInterpolate_H

#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>
#include "gkd/mission.h"

#include <iostream>

#include <vector>

struct GeoLocation {
    double latitude;
    double longitude;
};

// 将角度转换为弧度
static double toRadians(double degrees) {
    //return degrees * M_PI / 180;
    return degrees * acos(-1.0) / 180;
}

// 计算两个经纬度点之间的距离（单位：公里）
static double haversineDistance(const GeoLocation& loc1, const GeoLocation& loc2) {
    double lat1 = toRadians(loc1.latitude);
    double lon1 = toRadians(loc1.longitude);
    double lat2 = toRadians(loc2.latitude);
    double lon2 = toRadians(loc2.longitude);

    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;

    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double R = 6371; // 地球半径，单位：公里
    return R * c;
}

// 计算一系列经纬度点构成的路径长度
static double calculatePathLength(const std::vector<GeoLocation>& locations) {
    if (locations.size() < 2) {
        return 0; // 如果少于两个点，则路径长度为0
    }
    double totalDistance = 0.0;
    for (size_t i = 0; i < locations.size() - 1; ++i) {
        totalDistance += haversineDistance(locations[i], locations[i + 1]);
    }
    return totalDistance;
}


// 定义点结构
// struct Point {
//     double lon; // 经度
//     double lat; // 纬度
//     Point(double l = 0, double t = 0) : lon(l), lat(t) {}
// };

// 线性插值函数：在两个点之间按比例插值
static MQTTMission::Position linearInterpolate(const MQTTMission::Position& p1, const MQTTMission::Position& p2, double t) {
    // t ∈ [0, 1]，0表示p1，1表示p2
    MQTTMission::Position position;
     position.longitude = p1.longitude + t * (p2.longitude - p1.longitude);
     position.latitude  = p1.latitude + t * (p2.latitude - p1.latitude);
     position.altitude = p1.altitude;
     position.speed = p1.speed;
     return position;
}

// 对整个路线进行线性插值，生成等间距的新点
static std::vector<MQTTMission::Position> interpolateRouteLinear(const std::vector<MQTTMission::Position>& route, int numPoints) {
    if (route.size() < 2 || numPoints < 2) return {};

    // 计算总路径长度
    double totalLength = 0.0;
    std::vector<double> segmentLengths;
    for (size_t i = 1; i < route.size(); ++i) {
        double dx = route[i].longitude - route[i-1].longitude;
        double dy = route[i].latitude - route[i-1].latitude;
        double len = std::sqrt(dx*dx + dy*dy);
        segmentLengths.push_back(len);
        totalLength += len;
    }
    numPoints = totalLength; //一米一个
    std::vector<MQTTMission::Position> interpolated;
    interpolated.reserve(numPoints);
    // 按等距分割总路径
    double step = totalLength / (numPoints - 1);
    double currentDistance = 0.0;
    size_t currentSegment = 0;

    interpolated.push_back(route.front()); // 添加起点

    for (int i = 1; i < numPoints - 1; ++i) {
        currentDistance += step;

        // 找到当前距离所在的线段
        while (currentSegment < segmentLengths.size() &&
               currentDistance > segmentLengths[currentSegment]) {
            currentDistance -= segmentLengths[currentSegment];
            ++currentSegment;
        }

        // 在当前线段上进行线性插值
        if (currentSegment < route.size() - 1) {
            double t = currentDistance / segmentLengths[currentSegment];
            interpolated.push_back(linearInterpolate(route[currentSegment], route[currentSegment + 1], t));
        }
    }

    interpolated.push_back(route.back()); // 添加终点
    return interpolated;
}


#endif // linearInterpolate_H
