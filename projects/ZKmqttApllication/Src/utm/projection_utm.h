//#ifndef PROJECTION_UTM_H
//#define PROJECTION_UTM_H
/**
* UTM 算法类   by hqw
**/
#include "stdafx.h"
#include <string>
#include "common/common_config.h"
//struct UTMPoint {
//    double x;
//    double y;
//    double z;
//    int zone;
//};

//struct LatLon {
//    double lat;
//    double lon;
//    double height{0};
//};

class ProjectionUTM {
public:
    ProjectionUTM();
    ~ProjectionUTM();

    void CartesianToLatLon(double x, double y, int zone, bool southhemi, LatLon &latlon);

    void LatLonToCartesian(double lat, double lon, UTMPoint &xy);
    void LatLonToCartesian(double lat, double lon,int zone,UTMPoint &xy); // by hqw
    int LonToZone(double lon);

    // 新增的仿真坐标转换函数
    void SimToWGS84(double sim_x, double sim_y, LatLon& wgs84);
    void WGS84ToSim(double lat, double lon, double& sim_x, double& sim_y);
    void ProjectionUTM::convertLatLngToXYZ(const LatLon& latLng,UTMPoint& point);
    // 初始化转换参数
    void InitTransformParameters(const CommonCfg& _cfg);
private:
    void MapXYToLatLon(double x, double y, double lambda0, LatLon &philambda);

    void MapLatLonToXY(double phi, double lambda, double lambda0, UTMPoint &xy);

    double ArcLengthOfMeridian(double phi);

    double UTMCentralMeridian(int zone);

    double FootpointLatitude(double y);

    // 仿真坐标系参数
    double SIM_X1 = 450.30860000000001;   // 仿真坐标点1 X
    double SIM_Y1 = -58.724089999999997;   // 仿真坐标点1 Y
    double LAT1 = 29.730049483533421; // 对应的WGS84纬度1
    double LON1 = 106.51781357824804; // 对应的WGS84经度1
    
    double SIM_X2 = 476.06959999999998;   // 仿真坐标点2 X
    double SIM_Y2 = -60.565480000000001;   // 仿真坐标点2 Y
    double LAT2 = 29.73003201487699; // 对应的WGS84纬度2
    double LON2 = 106.51808112859727; // 对应的WGS84经度2

    // 计算的转换参数
    double scale_x_;  // X方向比例因子
    double scale_y_;  // Y方向比例因子
    double offset_x_; // X方向偏移
    double offset_y_; // Y方向偏移



private:
    static double pi;

    static double sm_a;
    static double sm_b;
    static double sm_EccSquared;
    static double UTMScaleFactor;

public:
    static int zone;
};


