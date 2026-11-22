

#include "stdafx.h"
#include <vector>
#include <cmath>


#define ZXZ_ERROR(a, ...)
#define ZXZ_WARN(a, ...)

struct LatLon {
    double lon = 0; //经度
    double lat = 0; //纬度

    LatLon() : lon(0.0), lat(0.0) {}

    LatLon(double lat, double lon) : lon(lon), lat(lat) {}

    LatLon(const LatLon &latlon) : lon(latlon.lon), lat(latlon.lat) {}
    bool operator ==(LatLon& ll){
        return lat == ll.lat && lon == ll.lon;

    }
};


struct UTMPoint {
    double x = 0;
    double y = 0;
    int zone = 0;

    UTMPoint() : x(0.0), y(0.0), zone(0) {}

    UTMPoint(double dx, double dy) : x(dx), y(dy) {}

    double Distance2D(const UTMPoint& pt){

        double dlength2 = (pt.x - x) * (pt.x - x) + (pt.y - y) * (pt.y - y);
        double disance = sqrt(dlength2);
        return disance;
    }
};


