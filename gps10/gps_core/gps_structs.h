#ifndef GPS_STRUCTS_H
#define GPS_STRUCTS_H


//关于时间的结构体
typedef struct{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
}DATE_TIME;

//卫星的信息，包括卫星的编号，卫星仰角，卫星方位角，卫星信号的信噪比
typedef struct{
    int prn;
    float elevation;
    float azimuth;
    float snr;
}SATELLITES_INFO;
//卫星的模式，定位类型，12个信道的prn编码，综合位置精度因子，水平精度因子，垂直精度因子
typedef struct {
    int sig;
    int fix;
    int prn[24];
    float pdop;
    float hdop;
    float vdop;
    int satnum;
}GPS_GSA_INFO;
//设备的位置，包括度、分、秒
typedef struct{
    int degree;
    int min;
    int sec;
}POSITION_T;
//可见卫星信息 ，包括卫星总数，和最多12个卫星的信息
typedef struct {
    int num;
    SATELLITES_INFO sat[24];
}GPS_GSV_INFO;
//GPS的信息，包括卫星信息和设备位置等信息
typedef struct{
    double latitude;
    double longitude;
    POSITION_T lati;
    POSITION_T longi;
    float speed;
    float direction;
    float elv;
    float height;
    float hdop;
    int signal;
    int dgps_age;
    int dgps_id;
    int satellites;

    char height_unit;
    char elv_unit;
    char NS;
    char EW;
    //SATELLITES_INFO sat[12];
    GPS_GSA_INFO gsa;
    GPS_GSV_INFO gsv;
    DATE_TIME d;
}GPS_INFO;

typedef struct{
    double lat;
    double lng;
    double height;
}GPS_G_T;

typedef struct{
    float slope;
    float presnr;
    int satnum;
    SATELLITES_INFO sat;
}SAT_SNR_TREND;


#endif // GPS_STRUCTS_H
