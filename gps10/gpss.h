#ifndef GPSS_H
#define GPSS_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <ctype.h>
#include "gps_structs.h"
#include "draw.h"
#include <vector>
#include <map>

extern Draw draw;
extern Mat sat_image;

class GPSS
{
public:
    GPSS(); //构造函数
    ~GPSS(); //析构函数
public:
    //function
     int process_gps(char *buff,GPS_INFO *gps);
     int show_gps_info(GPS_INFO *gps);
     int gps2ned(GPS_INFO *gps);
     int positionjudge(vector<SATELLITES_INFO> satinfo);
     int positionjudge2(vector<SAT_SNR_TREND> trend);//
     int writeSatsIntoSatsvector(vector<SATELLITES_INFO> satinfo);//put 5 sets of vector <SATELLITES_INFO> into satsvector
     int satsnrstat(vector<vector<SATELLITES_INFO> > satsinfo); //put every sat infomation into vector<pair<int, vector<float> > > satsnrs
     void LineFitLeastSquares(vector<float> snrs, int satnub, int i);//least square method fit line
     int openfieldjudge(vector<SATELLITES_INFO> satinfo);//make a open field as init moment
     int writeinitsatsnub(vector<SATELLITES_INFO> satinfo);//put the init sats'number into vector<pair<int, vector<float> > > satsnrs
     int FirstFrameSatPosDisStat(vector<SATELLITES_INFO> satinfo,vector <pair<vector<int>, vector<int> > > firstframetemp);//statistics frist frame sats'position
     int PreFrameSatPosDisStat(vector <SAT_SNR_TREND> trend, vector <vector<SAT_SNR_TREND> > preframetemp);//statistics pre frame sats'position
     void BinoFitLeastSquares(vector<float> snrs, int satnub, int i);//least square method fit binomial
     int WriteToInitsatsnrs(vector<SATELLITES_INFO> satinfo);
     int SatisInitState(vector <pair<int,vector<float> > > initsatsnrsTEMP);
public:
     //variable
     vector <SATELLITES_INFO> sats;//sats 是当前帧卫星信息容器
     vector <vector<SATELLITES_INFO> >satsvector;//satsvector 是连续5帧卫星信息容器
     vector <pair<int,vector<float> > > satsnrs;//satsnrs 是所有卫星连续5帧snrs数据，是对satsvector中数据的重新统计
     vector <pair<int,vector<float> > > initsatsnrs;//initsatsnrs 是初始化空旷地带连续5帧的1-4象限的强卫星强度容器
     int flag;//flag 判断初始化成功的参数
     vector <SAT_SNR_TREND> satstrend;//satstrend是卫星变化趋势结构体
     vector <pair<vector<int>, vector<int> > > firstframe; //初始帧卫星分布
//     vector <vector<SAT_SNR_TREND> > preframe;//预测帧卫星分布
private:
     int gpgsa(char *buff,GPS_INFO *gps);
     int gpgsv(char *buff);
     int gprmc(char *buff,GPS_INFO *gps);
     int gpgga(char *buff,GPS_INFO *gps);
     int gpvtg(char *buff,GPS_INFO *gps);
     int glgsv(char *buff,GPS_INFO *gps);
     int gngll(char *buff,GPS_INFO *gps);
     int calu_checksum(char *msg);
     int g2ecef(GPS_G_T *g_position,double *pe);
     int ecef2ned(GPS_G_T *current_position,GPS_G_T *ref_position,double *pn);
     int gps2g(GPS_INFO *gps,GPS_G_T *g_position);
     int date_convert(DATE_TIME *src,DATE_TIME *dst);
     int position_cal(double data,POSITION_T *pos);

};

#endif // GPSS_H
