 #ifndef GEO_ALGO_H
 #define GEO_ALGO_H
/**
*  地理算法类 by hqw
**/
#define MIN_VALUE 1e-8
#define IS_DOUBLE_ZERO(d)  (abs(d) < MIN_VALUE)
#define EARTH_RADIUS 6378.137 //地球半径 KM
#define M_PI_SEFDEF  3.14159265358979323846

class GEO_ALGO{

public:
    double rad(double d)
    {
        return d * M_PI_SEFDEF /180.0;
    }
    double RealDistance(double lat1,double lng1,double lat2,double lng2)//lat1第一个点纬度,lng1第一个点经度,lat2第二个点纬度,lng2第二个点经度
    {
        double a;
        double b;
        double radLat1 = rad(lat1);
        double radLat2 = rad(lat2);
        a = radLat1 - radLat2;
        b = rad(lng1) - rad(lng2);
        double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
        s = s * EARTH_RADIUS;
        s = s * 1000;
        return s;
    }
};
 #endif // GEO_ALGO_H
