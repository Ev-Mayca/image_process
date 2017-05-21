#include "gpss.h"


GPS_INFO gps;
GPS_GSV_INFO gps_gsv;
GPS_GSV_INFO gps_gsv1;
GPS_GSV_INFO gps_gsv_total;
//GPS_GSA_INFO gps_gngsa;
GPS_GSA_INFO gps_gsa[2];
GPS_GSA_INFO gps_gsa_total;


#define FORMAT_NUM 7
double L_AXIS = 6378137;
#define F 0.003325810664
#define PI 3.141592653
#define S_AXIS L_AXIS*(1-F)


//--------------------------- Veriable ---------------------------------//

double position[3]={0};

int a = 0;

///数据缓存
char buf_temp[1024];
extern FILE *fpsat;
extern FILE *fppos;
extern FILE *fpdop;
extern FILE *fpned;
extern int line_num;

double lattemp = 0;
double lngtemp = 0;

float static_total[4] = {0};
//------------------------- Function -----------------------------------//


GPSS::GPSS()
{
    firstframe.resize(6);
    //   preframe.resize(5);
    flag = 0;
}

GPSS::~GPSS()
{

}



int GPSS::process_gps(char *buff,GPS_INFO *gps)
{
    const char *name_buff[] = {
        "GGA",
        "GSA",
        "GPGSV",
        "RMC",
        "VTG",
        "GLGSV",
        "GLL"
    };

    //   int (*oper_func[])(char *buff,GPS_INFO *gps) = {
    //      gpgga,
    //       gpgsa,
    //       gpgsv,
    //       gprmc,
    //      gpvtg
    //
    //   };
    int i;
    //	int error = 0;
    //检查数据格式
    if(calu_checksum(buff)){
        //printf("format error !\t");
        return -1;
    }
    //判断是哪种格式数据
    for(i = 0; i < FORMAT_NUM; i ++){
        if(strstr(buff,name_buff[i]) != 0){
            break;
        }
    }
    //判断该行数据是否有相应的函数进行处理
    if(i == FORMAT_NUM){
        //printf("unknow format");
        return -1;
    }else{
        if(i==0) gpgga(buff,gps);
        if(i==1) gpgsa(buff,gps);
        if(i==2) gpgsv(buff);
        if(i==3) gprmc(buff,gps);
        if(i==4) gpvtg(buff,gps);
        if(i==5) glgsv(buff,gps);
        if(i==6) gngll(buff,gps);
    }

    return i;

}


int GPSS::gprmc(char *buff,GPS_INFO *gps)
{

    char *p = buff;

    DATE_TIME d_temp;

    d_temp.hour = gps->d.hour;
    d_temp.minute = gps->d.minute;
    d_temp.second = gps->d.second;

    p = strstr(p,",");
    if(p == 0){
        //printf("gprmc format error !\t");
        return -1;
    }
    //得到utc时间
    p ++;
    if(*p == ',')return -1;

    gps->d.hour = (*p - '0') * 10 + *(p + 1) - '0';
    p += 2;
    gps->d.minute = (*p - '0') * 10 + *(p + 1) - '0';
    p += 2;
    gps->d.second = (*p - '0') * 10 + *(p + 1) - '0';

    p = strstr(p,",");
    //判断本行数据是否有效
    if(*(++p) == 'A'){
        //longitude
        p = strstr(p,",");
        p ++;
        gps->latitude= atof(p);
        //N or S
        p = strstr(p,",");
        p ++;
        if(*p == ',')gps->NS = ' ';
        else gps->NS = *p;
        //latitude
        p = strstr(p,",");
        p ++;
        gps->longitude= atof(p);
        // W or E
        p = strstr(p,",");
        p ++;
        if(*p == ',')gps->EW = ' ';
        else gps->EW = *p;
        //speed
        p = strstr(p,",");
        p ++;
        gps->speed = 1.85 * atof(p);
        //方位角
        p = strstr(p,",");
        p ++;
        gps->direction = atof(p);

        //utc日期
        p = strstr(p,",");
        p ++;
        gps->d.day = (*p - '0') * 10 + *(p + 1) - '0';
        p += 2;
        gps->d.month = (*p - '0') * 10 + *(p + 1) - '0';
        p += 2;
        gps->d.year = (*p - '0') * 10 + *(p + 1) - '0';
        p ++;
        //进行日期转换，将UTC时间转换为北京时间
        date_convert(&gps->d,&gps->d);
        //将经纬度数据转换为相应的度、分、秒
        position_cal(gps->longitude,&gps->longi);
        position_cal(gps->latitude,&gps->lati);
        return 0;
    }
    gps->d.hour = d_temp.hour;
    gps->d.minute = d_temp.minute;
    gps->d.second = d_temp.second;

    return -1;
}

int GPSS::gpgga(char *buff,GPS_INFO *gps)
{
    char *p = buff;
    p = strstr(p,",");
    p ++;
    /*
    gps->d.hour = (*p - '0') * 10 + *(p + 1) - '0';
    p += 2;
    gps->d.minute = (*p - '0') * 10 + *(p + 1) - '0';
    p += 2;
    gps->d.second = (*p - '0') * 10 + *(p + 1) - '0';
    */
    //date to beijing time
    //	date_convert(&gps->d,&gps->d);
    // latitude
    p = strstr(p,",");
    p ++;
    gps->latitude= atof(p);
    //N or S
    p = strstr(p,",");
    p ++;
    if(*p == ',')gps->NS = ' ';
    else gps->NS = *p;
    //longitude
    p = strstr(p,",");
    p ++;
    gps->longitude= atof(p);
    // W or E
    p = strstr(p,",");
    p ++;
    if(*p == ',')gps->EW = ' ';
    else gps->EW = *p;
    //定位质量指标
    p = strstr(p,",");
    p ++;
    gps->signal = *p - '0';
    //使用卫星数量
    p = strstr(p,",");
    p ++;
    gps->satellites = atoi(p);
    //水平精度
    p = strstr(p,",");
    p ++;
    gps->hdop = atof(p);
    //天线离海平面的高度
    p = strstr(p,",");
    p ++;
    gps->elv = atof(p);
    //高度单位
    p = strstr(p,",");
    p ++;
    if(*p == ',')gps->EW = ' ';
    else gps->elv_unit = *p;
    //大地椭球面相对海平面的高度
    p = strstr(p,",");
    p ++;
    gps->height = atof(p);
    //高度单位
    p = strstr(p,",");
    p ++;
    gps->height_unit = *p;
    //差分GPS数据期限
    p = strstr(p,",");
    p ++;
    gps->dgps_age = atoi(p);
    //差分参考基站标号
    p = strstr(p,",");
    p ++;
    gps->dgps_id = atoi(p);

    //将经纬度数据转换为相应的度、分、秒
    position_cal(gps->longitude,&gps->longi);
    position_cal(gps->latitude,&gps->lati);

    return 0;
}

int GPSS::gpgsa(char *buff,GPS_INFO *gps)
{
    char *p = buff;
    int i;
    //模式
    p = strstr(p,",");
    p ++;
    gps_gsa[a].sig = *p;

    //定位类型
    p = strstr(p,",");
    p ++;
    gps_gsa[a].fix = *p - '0';

    //得到最多12个卫星数据的prn编码
    for(i = 0; i < 12; i ++){
        p = strstr(p,",");
        p ++;
        if(isdigit(*p)){
            gps_gsa[a].prn[i] = atoi(p);
        }else{
            gps_gsa[a].prn[i] = -1;
        }
    }

    for(i=0;i<12;i++){
        if(gps_gsa[a].prn[i] == -1){
            gps_gsa[a].satnum = i;
            break;
        }
    }


    //综合位置精度因子
    p = strstr(p,",");
    p ++;
    gps_gsa[a].pdop = atof(p);

    //水平精度因子
    p = strstr(p,",");
    p ++;
    gps_gsa[a].hdop = atof(p);

    //垂直精度因子
    p = strstr(p,",");
    p ++;
    gps_gsa[a].vdop = atof(p);

    a++;

    if(a == 2){
        a = 0;
        memset(&gps_gsa_total,0,sizeof(gps_gsa_total));
        memcpy(&gps_gsa_total,&gps_gsa[0],sizeof(gps_gsa_total));
        gps_gsa_total.satnum = gps_gsa[0].satnum + gps_gsa[1].satnum;

        for( int q = 0;q < gps_gsa[1].satnum;q++)
        {
            gps_gsa_total.prn[gps_gsa[0].satnum + q] = gps_gsa[1].prn[q];
        }

        //       if(gps_gsa[0].satnum - gps_gsa[1].satnum){
        //           memcpy(&gps_gngsa,&gps_gsa[0],sizeof(gps_gsa[0]));
        //           gps_gngsa = gps_gsa[0];
        //      }else{
        //           memcpy(&gps_gngsa,&gps_gsa[0],sizeof(gps_gsa[0]));
        //           gps_gngsa = gps_gsa[1];
        //       }

        //    if(gps_gngsa.fix == 1){
        //        memset(buf_temp,0,sizeof(buf_temp));
        //        sprintf(buf_temp,"未定位!\t");
        //        fwrite(buf_temp,1,strlen(buf_temp),fp);

        //    }else {
        //		memset(buf_temp,0,sizeof(buf_temp));
        //		sprintf(buf_temp,"%d维定位\t",gps_gsa.fix);
        //		fwrite(buf_temp,1,strlen(buf_temp),fp);

        //        for(i = 0; i < 12; i ++){
        //            if(gps_gngsa.prn[i] != -1){
        //				memset(buf_temp,0,sizeof(buf_temp));
        //				sprintf(buf_temp,"%02d ",gps_gsa.prn[i]);
        //				fwrite(buf_temp,1,strlen(buf_temp),fp);
        //            }
        //        }
        //		memset(buf_temp,0,sizeof(buf_temp));
        //		sprintf(buf_temp,"p:%0.2f h:%.2f v:%.2f\n",gps_gsa.pdop,gps_gsa.hdop,gps_gsa.vdop );
        //		fwrite(buf_temp,1,strlen(buf_temp),fp);
        //    }
        //将得到的数据放入gps中去
        memcpy(&gps->gsa,&gps_gsa_total,sizeof(GPS_GSA_INFO));
    }

    return 0;
}

int GPSS::gpgsv(char *buff)
{
    char *p = buff;
    //    int total_num = 0;
    int gsv_num = 0;
    int current_gsv_num = 0;
    int num_temp = 0;
    int i = 0;

    p = strstr(p,",");
    if(p == 0){
        //printf("gpgsv format error !\t");
        return -1;
    }
    p ++;
    gsv_num = *p - '0';
    //当前GSV语句号
    p = strstr(p,",");
    p ++ ;
    current_gsv_num = *p - '0';

    if(current_gsv_num == 1){
        memset(&gps_gsv,0,sizeof(gps_gsv));
    }
    //可视卫星总数
    p = strstr(p,",");
    p ++;

    gps_gsv.num = atoi(p);

    //获取卫星的信息
    for(i = 0; i < 4; i ++){
        num_temp = (current_gsv_num * 4 - 4 + i );
        if(num_temp >= gps_gsv.num)break;
        //卫星编号
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv.sat[num_temp].prn = 0;
        else gps_gsv.sat[num_temp].prn = atoi(p);
        //卫星仰角
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv.sat[num_temp].elevation = 0;
        else gps_gsv.sat[num_temp].elevation = atof(p);
        //卫星方位角
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv.sat[num_temp].azimuth = 0;
        else gps_gsv.sat[num_temp].azimuth = atof(p);
        //卫星信号的信噪比
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv.sat[num_temp].snr = 0;
        else gps_gsv.sat[num_temp].snr = atof(p);
        //printf("%d,%.2f,%.2f,%.2f\t",gps_gsv.sat[num_temp].prn,gps_gsv.sat[num_temp].elevation,gps_gsv.sat[num_temp].azimuth,gps_gsv.sat[num_temp].snr);
    }

    return 0;
}

int GPSS::glgsv(char *buff, GPS_INFO *gps)
{
    char *p = buff;
    //    int total_num = 0;
    int gsv_num = 0;
    int current_gsv_num = 0;
    int num_temp = 0;
    int i = 0;

    p = strstr(p,",");
    if(p == 0){
        //printf("gpgsv format error !\t");
        return -1;
    }
    p ++;
    gsv_num = *p - '0';
    //当前GSV语句号
    p = strstr(p,",");
    p ++ ;
    current_gsv_num = *p - '0';

    if(current_gsv_num == 1){
        memset(&gps_gsv1,0,sizeof(gps_gsv1));
    }
    //可视卫星总数
    p = strstr(p,",");
    p ++;

    gps_gsv1.num = atoi(p);

    //获取卫星的信息
    for(i = 0; i < 4; i ++){
        num_temp = (current_gsv_num * 4 - 4 + i );
        if(num_temp >= gps_gsv1.num)break;
        //卫星编号
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv1.sat[num_temp].prn = 0;
        else gps_gsv1.sat[num_temp].prn = atoi(p);
        //卫星仰角
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv1.sat[num_temp].elevation = 0;
        else gps_gsv1.sat[num_temp].elevation = atof(p);
        //卫星方位角
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv1.sat[num_temp].azimuth = 0;
        else gps_gsv1.sat[num_temp].azimuth = atof(p);
        //卫星信号的信噪比
        p = strstr(p,",");
        p ++;
        if(!isdigit(*p))gps_gsv1.sat[num_temp].snr = 0;
        else gps_gsv1.sat[num_temp].snr = atof(p);
        //printf("%d,%.2f,%.2f,%.2f\t",gps_gsv.sat[num_temp].prn,gps_gsv.sat[num_temp].elevation,gps_gsv.sat[num_temp].azimuth,gps_gsv.sat[num_temp].snr);
    }

    if(current_gsv_num == gsv_num){

        memset(&gps_gsv_total,0,sizeof(SATELLITES_INFO));//clear gps_gsv_total
        gps_gsv_total.num = gps_gsv.num + gps_gsv1.num;
        SATELLITES_INFO temp;
        for(int a = 0; a < gps_gsv.num;a++)
        {
            memset(&temp,0,sizeof(SATELLITES_INFO));
            memcpy(&temp,&gps_gsv.sat[a],sizeof(SATELLITES_INFO));
            memcpy(&gps_gsv_total.sat[a],&temp,sizeof(SATELLITES_INFO));
        }
        for(int a = 0 ; a < gps_gsv1.num; a++)
        {
            memset(&temp,0,sizeof(SATELLITES_INFO));
            memcpy(&temp,&gps_gsv1.sat[a],sizeof(SATELLITES_INFO));
            memcpy(&gps_gsv_total.sat[a+gps_gsv.num],&temp,sizeof(SATELLITES_INFO));
        }
        memcpy(&gps->gsv,&gps_gsv_total,sizeof(GPS_GSV_INFO));
    }

    return 0;
}


int GPSS::gpvtg(char *buff,GPS_INFO *gps)
{

    //printf("function not aviable !\t");
    return 0;
}

int GPSS::gngll(char *buff, GPS_INFO *gps)
{
    return 0;
}


// date to beijing time
int GPSS::date_convert(DATE_TIME *src,DATE_TIME *dst)
{
    //日期转换函数，在UTC的基础上加上8小时，即为北京时间

    dst->minute = src->minute;
    dst->second = src->second;
    dst->hour = src->hour + 8;
    if(src->month ==  1 || src->month == 3 || src->month == 5 || src->month == 7 || src->month == 8 || src->month == 10 || src->month == 12){//31天的月份
        if(dst->hour > 24){
            dst->hour = dst->hour - 24;
            dst->day = src->day + 1;
            if(dst->day > 31){
                dst->day -= 31;
                dst->month = src->month + 1;
                if(dst->month > 12){
                    dst->month -= 12;
                    dst->year = src->year + 1;
                }
            }
        }
    }else if(src->month == 4 || src->month == 6 || src->month == 9 || src->month == 11){
        //30天的月份
        if(dst->hour > 24){
            dst->hour = dst->hour - 24;
            dst->day = src->day + 1;
            if(dst->day > 30){
                dst->day -= 30;
                dst->month = src->month + 1;
            }
        }
    }else{
        //2月
        if(dst->hour > 24){
            dst->hour = dst->hour - 24;
            dst->day = src->day + 1;
            if((src->year%400 == 0)||(src->year%4 == 0 && src->year%100 != 0)){
                if(dst->day > 29){
                    dst->day -= 29;
                    dst->month = src->month + 1;
                }
            }else{
                if(dst->day > 28){
                    dst->day -= 28;
                    dst->month = src->month + 1;
                }
            }
        }
    }
    return 0;

}

//每行数据的校验
int GPSS::calu_checksum(char *msg)
{
    int i = 0;
    int c = 0;
    char *sumstr;
    int parsedsum = 0;

    if(msg[i] != '$'){
        //printf("不正确的数据格式");
        return -1;
    }else{
        for(c = msg[1],i = 2; msg[i] != '*'; i ++){
            c ^= msg[i];
        }
    }
    sumstr = &msg[i + 1];
    sscanf(sumstr,"%x",&parsedsum);

    return (c - parsedsum)?-1:0;
}

//位置数据转换
int GPSS::position_cal(double data,POSITION_T *pos)
{
    float temp = 0;

    pos->degree = data/100;
    pos->min = data - pos->degree * 100;
    //pos->sec = (data - pos->degree * 100 - pos->min) * 60;
    pos->sec = (data - (int)data) * 60;
    return 0;
}

int GPSS::show_gps_info(GPS_INFO *gps)
{
    int i,j;
    char temp[10] = {
        0
    };

    //    char buf_temp[512];
    //清屏
    printf("\033c");

    printf("\033[32mDate&Time | position |    PRN   |Elevation | azimuth  | snr(dB)  | satellites\n");

    printf("\033[39m%04d/%02d/%02d|%03d%c%02d'%02d\"|    %02d    | %07.3f  | %07.3f  | %07.3f  | %d \n",gps->d.year,gps->d.month,gps->d.day,gps->longi.degree,gps->EW,gps->longi.min,gps->longi.sec,gps->gsv.sat[0].prn,gps->gsv.sat[0].elevation,gps->gsv.sat[0].azimuth,gps->gsv.sat[0].snr,gps->gsv.num);

    printf(" %02d:%02d:%02d |%03d%c%02d'%02d\"|    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->d.hour,gps->d.minute,gps->d.second,gps->lati.degree,gps->NS,gps->lati.min,gps->lati.sec,gps->gsv.sat[1].prn, gps->gsv.sat[1].elevation,gps->gsv.sat[1].azimuth,gps->gsv.sat[1].snr);

    printf("\033[32m  speed   |direction \033[39m|    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->gsv.sat[2].prn, gps->gsv.sat[2].elevation,gps->gsv.sat[2].azimuth,gps->gsv.sat[2].snr);

    printf(" %7.3f  | %7.3f  |    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->speed,gps->direction,gps->gsv.sat[3].prn, gps->gsv.sat[3].elevation,gps->gsv.sat[3].azimuth,gps->gsv.sat[3].snr);

    printf("\033[32m  Height  |   elv    \033[39m|    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->gsv.sat[4].prn, gps->gsv.sat[4].elevation,gps->gsv.sat[4].azimuth,gps->gsv.sat[4].snr);

    printf(" %7.3f  | %7.3f  |    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->height,gps->elv,gps->gsv.sat[5].prn, gps->gsv.sat[5].elevation,gps->gsv.sat[5].azimuth,gps->gsv.sat[5].snr);

    for(i = 6; i < 12;i ++){
        printf("    -     |    -     |    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->gsv.sat[i].prn, gps->gsv.sat[i].elevation,gps->gsv.sat[i].azimuth,gps->gsv.sat[i].snr);
    }

//    gps->gsv.sat[10].prn;
//    gps->gsv.num;
//    printf("\033[32m    PRN   |Elevation | azimuth  | snr(dB)\n");
//    for(i = 0; i < gps->gsv.num; i ++){
//        printf("\033[39m    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",gps->gsv.sat[i].prn, gps->gsv.sat[i].elevation,gps->gsv.sat[i].azimuth,gps->gsv.sat[i].snr);
//    }
    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%03d:  xxx\n",line_num);
    fwrite(buf_temp,1,strlen(buf_temp),fpsat);

    for(i = 0; i< gps->gsa.satnum; i++){
        for(j = 0; j<gps->gsv.num; j++){
            if(gps->gsv.sat[j].prn == gps->gsa.prn[i]) {

                memset(buf_temp,0,sizeof(buf_temp));
                sprintf(buf_temp,"%d  %f  %f  %f\n",gps->gsv.sat[j].prn,gps->gsv.sat[j].elevation,gps->gsv.sat[j].azimuth,gps->gsv.sat[j].snr);
                fwrite(buf_temp,1,strlen(buf_temp),fpsat);

                SATELLITES_INFO satstemp;//satstemp是当前帧当前卫星信息容器
                memset(&satstemp,0,sizeof(SATELLITES_INFO));
                memcpy(&satstemp,&gps->gsv.sat[j],sizeof(SATELLITES_INFO));
                sats.push_back(satstemp);
                draw.DrawSatPosition(sat_image,gps->gsv.sat[j],Point(w/2,w/2));
            }
        }
    }

    vector<SATELLITES_INFO>::iterator itr = sats.begin();
    while (itr!=sats.end())
    {
        if (itr->prn == 0)
        {
            itr=sats.erase(itr);
        }else
            itr++;//这里迭代器会更新出错
    }
//shun shi zhen N
//    printf("\033[32m    PRN   |Elevation | azimuth  | snr(dB)\n");
    float s_temp[4][4] = {0};
    static int cnt_prn = 0;

    memset(s_temp,0,sizeof(s_temp));
    for(i = 0; i < sats.size(); i ++){
        if(/*sats[i].elevation > 60 || */sats[i].elevation < 15)continue;
        float r_t = cos(sats[i].elevation/180*pi);
        float v_t = (sats[i].azimuth)/180*pi;

        int sector = ((int)sats[i].azimuth) / 90;

        s_temp[0][sector] = s_temp[0][sector] + r_t*cos(v_t)*(/*sats[i].snr>25?1:*/(sats[i].snr/50));
        s_temp[1][sector] = s_temp[1][sector] + r_t*sin(v_t)*(/*sats[i].snr>25?1:*/(sats[i].snr/50));
        s_temp[2][sector] ++;
//        s_temp[3][sector] = s_temp[3][sector] + sats[i].snr;
//        printf("\033[39m    %02d    | %07.3f  | %07.3f  | %07.3f  |\n",sats[i].prn, r_t,sats[i].azimuth,sats[i].snr);


//        if ( v_t > 0 && v_t <=90) {
//            quadrant[1].first ++;
//            snrs[1].push_back(snrtemp);
//        }
//        if ( v_t > 90 && v_t <=180) {
//            quadrant[2].first ++;
//            snrs[2].push_back(snrtemp);
//        }
//        if ( v_t > 180 && v_t <=270) {
//            quadrant[3].first ++;
//            snrs[3].push_back(snrtemp);
//        }
//        if ( v_t > 270 && v_t <=360) {
//            quadrant[4].first ++;
//            snrs[4].push_back(snrtemp);
//        }
    }


//    float center_p[2] = {0};
    for(i = 0; i < 4; i ++){
        if(s_temp[2][i] == 0){
            s_temp[0][i] = 0;
            s_temp[1][i] = 0;
//            s_temp[3][i] = 0;
        }else{
            s_temp[0][i] = s_temp[0][i]/s_temp[2][i];
            s_temp[1][i] = s_temp[1][i]/s_temp[2][i];
//            s_temp[3][i] = s_temp[3][i]/s_temp[2][i];
        }
//        center_p[0] = center_p[0] + s_temp[0][i]*s_temp[3][i];
//        center_p[1] = center_p[1] + s_temp[1][i]*s_temp[3][i];
    }
    s_temp[3][0] = s_temp[0][0]+s_temp[0][3];
    s_temp[3][1] = s_temp[0][1]+s_temp[0][2];
    s_temp[3][2] = s_temp[1][0]+s_temp[1][1];
    s_temp[3][3] = s_temp[1][3]+s_temp[1][2];
    cnt_prn ++;

    static_total[0] = (static_total[0]*(cnt_prn - 1)+ s_temp[3][0])/cnt_prn;
    static_total[1] = (static_total[1]*(cnt_prn - 1)+ s_temp[3][1])/cnt_prn;
    static_total[2] = (static_total[2]*(cnt_prn - 1)+ s_temp[3][2])/cnt_prn;
    static_total[3] = (static_total[3]*(cnt_prn - 1)+ s_temp[3][3])/cnt_prn;
    /*
    printf("%07.3f    | %07.3f  | %07.3f  | %07.3f  |\n",s_temp[0][0],s_temp[0][1],s_temp[0][2],s_temp[0][3]);
    printf("%07.3f    | %07.3f  | %07.3f  | %07.3f  |\n",s_temp[1][0],s_temp[1][1],s_temp[1][2],s_temp[1][3]);
    printf("%07.3f    | %07.3f  | %07.3f  | %07.3f  |\n",s_temp[3][0]*10,s_temp[3][1]*10,s_temp[3][2]*10,s_temp[3][3]*10);
    printf("%07.3f    | %07.3f  | %07.3f  | %07.3f  |\n",static_total[0]*10,static_total[1]*10,static_total[2]*10,static_total[3]*10);
    */
//    printf("%d\n",cnt_prn);
//    printf("%07.3f    | %07.3f  \n",center_p[0],center_p[1]);

    lattemp = (int)(gps->latitude/100)+(gps->latitude-(int)(gps->latitude/100)*100)/60.;
    lngtemp = (int)(gps->longitude/100)+(gps->longitude-(int)(gps->longitude/100)*100)/60.;

    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%03d:  ",line_num);
    fwrite(buf_temp,1,strlen(buf_temp),fppos);

    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%f  %f  %f\n",lattemp,lngtemp,gps->height);
    fwrite(buf_temp,1,strlen(buf_temp),fppos);

    draw.WritePos(sat_image,Point(w/2,w/2),lattemp,lngtemp);

    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%03d:  ",line_num);
    fwrite(buf_temp,1,strlen(buf_temp),fpdop);

    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%f  %f  %f\n",gps->gsa.pdop,gps->gsa.hdop,gps->gsa.vdop);
    fwrite(buf_temp,1,strlen(buf_temp),fpdop);

    draw.WritePdop(sat_image,Point(w/2,w/2),gps->gsa);

    /*********************************以下程序断判别接收机位置信息**********************************************/

//    if (flag == 0){
//        openfieldjudge(sats);//->satsnrs num and prn
//    }else{
//        if(flag < 5) {
//            WriteToInitsatsnrs(sats);
//            flag ++;
//        }else if(flag ==5){
//            int x = SatisInitState(initsatsnrs);
//            if(x == -1) flag == 0;
//            else flag++;
//        }else{
//            if (writeSatsIntoSatsvector(sats) == 5){ //->satsvector
//                satsnrstat(satsvector);//->satsnrs
//                int satsum = satsnrs.size();
//                satstrend.resize(satsum);
//                //get satstrend infomation
//                for( int i = 0; i<satsum; i++){
//                    LineFitLeastSquares(satsnrs[i].second,satsnrs[i].first,i);//->satstrend
//                    for (int j = 0; j<sats.size();j++){
//                        if (sats[j].prn == satstrend[i].satnum){
//                            //SATELLITES_INFO sattemp;//sattemp是当前卫星信息
//                            //memset(&sattemp,0,sizeof(SATELLITES_INFO));
//                            memcpy(&satstrend[i].sat,&sats[j],sizeof(SATELLITES_INFO));
//                        }
//                    }
//                }
//                //
//                for(int i = 0; i<satsnrs.size();i++){
//                    satsnrs[i].second.clear();
//                }
//                //judge building at which side and GPS receiver position
//                positionjudge2(satstrend);//plan 2
//            }
//        }
//    }
    /*******************************************************************************/
       positionjudge(sats);// plan 1
    sats.clear();
    satstrend.clear();
    //   sprintf(buf_temp,"\n");
    //    fwrite(buf_temp,1,strlen(buf_temp),fpsat);

    return 0;
}



int GPSS::g2ecef(GPS_G_T *g_position,double *pe)
{
    //double lat = ((int)(lati/100)+(lati -(int)(lati/100))/60.)/180*PI;
    //double lng = ((int)(longi/100)+(longi -(int)(longi/100))/60.)/180*PI;
    double E_L  = (sqrt(((L_AXIS*L_AXIS) - (S_AXIS * S_AXIS))/(L_AXIS*L_AXIS)));
    double  E_S = (sqrt(((L_AXIS*L_AXIS) - (S_AXIS * S_AXIS))/(S_AXIS*S_AXIS)));
    double N = L_AXIS/sqrt(1-E_L*E_L*sin(g_position->lat)*sin(g_position->lat));

    *pe = (N + g_position->height)*cos(g_position->lat)*cos(g_position->lng);
    *(pe + 1) = (N + g_position->height)*cos(g_position->lat)*sin(g_position->lng);
    *(pe + 2) = (N*(1-pow(E_L,2)) + g_position->height)*sin(g_position->lat);

    return 0;

}

int GPSS::gps2g(GPS_INFO *gps,GPS_G_T *g_position)
{
    g_position->lat = ((int)(gps->latitude/100)+(gps->latitude -(int)(gps->latitude/100)*100)/60.)/180*PI;
    g_position->lng = ((int)(gps->longitude/100)+(gps->longitude -(int)(gps->longitude/100)*100)/60.)/180*PI;
    g_position->height = gps->height;

    return 0;
}


int GPSS::ecef2ned(GPS_G_T *current_position,GPS_G_T *ref_position,double *ned)
{
    double r_n2e[3][3];
    int i,j;
    double pe[3];
    double pn_temp[3];
    double pn_pe_temp[3];
    double temp;

    r_n2e[0][0] = -sin(ref_position->lat)*cos(ref_position->lng);
    r_n2e[0][1] = -sin(ref_position->lat)*sin(ref_position->lng);
    r_n2e[0][2] = cos(ref_position->lat);
    r_n2e[1][0] = -sin(ref_position->lng);
    r_n2e[1][1] = cos(ref_position->lng);
    r_n2e[1][2] = 0;
    r_n2e[2][0] = -cos(ref_position->lat)*cos(ref_position->lng);
    r_n2e[2][1] = -cos(ref_position->lat)*sin(ref_position->lng);
    r_n2e[2][2] = -sin(ref_position->lat);

    g2ecef(ref_position,pe);
    g2ecef(current_position,pn_temp);

    for(i = 0; i < 3; i++){
        pn_pe_temp[i] = *(pn_temp+i) - *(pe + i);
    }

    for(i = 0; i < 3; i ++){
        temp = 0;
        for(j = 0;j < 3; j ++){
            temp += pn_pe_temp[j]*r_n2e[i][j];
        }
        *(ned + i) = temp;

    }
    return 0;
}

int GPSS::gps2ned(GPS_INFO *gps)
{
    static int flag = 0;
    static GPS_G_T ref_position;
    GPS_G_T current_position;

    //    int line_cnt = 0;

    if(flag == 0){
        if(gps->longitude != 0 || gps->latitude != 0){
            flag = 1;
            gps2g(gps,&ref_position);
        }

    }else{
        gps2g(gps,&current_position);
        ecef2ned(&current_position,&ref_position,position);
    }

//    printf("position: %f %f %f\n",position[0],position[1],position[2]);

    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%03d: ",line_num ++);
    fwrite(buf_temp,1,strlen(buf_temp),fpned);

    memset(buf_temp,0,sizeof(buf_temp));
    sprintf(buf_temp,"%f %f %f\n",position[0],position[1],position[2]);
    fwrite(buf_temp,1,strlen(buf_temp),fpned);

    return 0;
}

int GPSS::positionjudge(vector<SATELLITES_INFO> satinfo)
{
    if(satinfo.size() == 0) return 0;
    int num = satinfo.size();
    int strong = 0;
    int weak = 0;
    char posinfo[100];
    memset(posinfo,0,sizeof(posinfo));

    vector <pair<int,int> > quadrant(5);
    vector <vector<float > > snrs(5);

    for (int i = 0 ; i < num ;i++)
    {
        float snrtemp = satinfo[i].snr;
        float azitemp = satinfo[i].azimuth;
        float eletemp = satinfo[i].elevation;
        //the sat'snr vavul >25 is strong
        if (snrtemp >= 25)
        {
            strong++;
            if (eletemp >= 60){
                quadrant[0].first ++;
                snrs[0].push_back(snrtemp);
            }else{

                if ( azitemp > 0 && azitemp <=90) {
                    quadrant[1].first ++;
                    snrs[1].push_back(snrtemp);
                }
                if ( azitemp > 90 && azitemp <=180) {
                    quadrant[2].first ++;
                    snrs[2].push_back(snrtemp);
                }
                if ( azitemp > 180 && azitemp <=270) {
                    quadrant[3].first ++;
                    snrs[3].push_back(snrtemp);
                }
                if ( azitemp > 270 && azitemp <=360) {
                    quadrant[4].first ++;
                    snrs[4].push_back(snrtemp);
                }
            }
        }else
        {
            weak++;
            if (eletemp >=60){
                quadrant[0].second ++;
                snrs[0].push_back(snrtemp);
            }else {
                if ( azitemp > 0 && azitemp <=90) {
                    quadrant[1].second ++;
                    snrs[1].push_back(snrtemp);
                }
                if ( azitemp > 90 && azitemp <=180) {
                    quadrant[2].second ++;
                    snrs[2].push_back(snrtemp);
                }
                if ( azitemp > 180 && azitemp <=270) {
                    quadrant[3].second ++;
                    snrs[3].push_back(snrtemp);
                }
                if ( azitemp > 270 && azitemp <=360) {
                    quadrant[4].second ++;
                    snrs[4].push_back(snrtemp);
                }
            }
        }


    }

    vector<float> snravi(4);
    int snrtotal = 0;
    int satsnum = quadrant[1].first + quadrant[1].second +
            quadrant[2].first + quadrant[2].second +
            quadrant[3].first + quadrant[3].second +
            quadrant[4].first + quadrant[4].second;

    //ji suan mei ge xiang xian de snr ping jun zhi, he shu liang.
    for(int i = 1; i< 5; i++)
    {
        int snrsumtemp = 0;
        for(int j = 0; j<snrs[i].size(); j++)
        {
            snrsumtemp += snrs[i][j];
        }
        if(snrs[i].size() == 0) {
            snravi[i-1] = 0;
        }else{
            snravi[i-1] = (float)snrsumtemp/snrs[i].size();
            snrtotal += snrsumtemp;
        }
    }
    float avesats = (float)(snrtotal)/satsnum;

//    static int flag_status = 0;//0:open   1:one    2 two

    float std_sats = 0;
    std_sats = (snravi[0] - avesats)*(snravi[0] - avesats)+(snravi[1] - avesats)*(snravi[1] - avesats)+
            (snravi[2] - avesats)*(snravi[2] - avesats)+(snravi[3] - avesats)*(snravi[3] - avesats);
    std_sats = sqrt(std_sats);

    draw.WriteSnr(sat_image,Point(w/2,w/2),quadrant,snravi,avesats,std_sats);

    if(avesats > 27){
        if(std_sats < 12){
            strcpy(posinfo,"open field!");
        }else{
            strcpy(posinfo,"building at one side!");
        }


    }else {
        strcpy(posinfo,"between buildings!");
    }


    draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
/*
    //现规定：卫星数量大于7，信号强度弱的卫星少于2，而且四个象限均有分布卫星时，为空旷区域。
    if(num >= 10 && weak <3){
        if((quadrant[1].first+quadrant[1].second) >0 && (quadrant[4].first+quadrant[4].second)>0
                && (quadrant[2].first+quadrant[2].second)>0 && (quadrant[3].first+quadrant[3].second)>0)
        {
            printf("you are at open field! \n");
            strcpy(posinfo,"open field!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }else{
            printf("you are at unknow area!\n");
            strcpy(posinfo,"unknow area!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }
    }else if(num >=7 && weak >5){
        //现规定：卫星数量大于5，信号强度弱的卫星da于3，而且四个象限均有分布卫星时，为两楼之间区域。
        int weakdis = 0;
        if((quadrant[4].first+quadrant[4].second) >0 && (quadrant[1].first+quadrant[1].second)>0
                && (quadrant[2].first+quadrant[2].second)>0 && (quadrant[3].first+quadrant[3].second)>0){
            for ( int m =1; m<5; m++)
            {
                if(quadrant[m].second > 0) weakdis++;
            }
            if (weakdis>=3)
            {
                printf("you are between buildings! \n");
                strcpy(posinfo,"between building!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else{
                printf("you are at unknow area!\n");
                strcpy(posinfo,"unknow area!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }
        }else{
            printf("you are at unknow area!\n");
            strcpy(posinfo,"unknow area!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }

    }else if(num >=7){
        //现规定：卫星数量大于4，且相邻两象限强卫星个数少于等于1时，为单边楼区域。
        if((quadrant[1].first +quadrant[2].first)<1){
            printf("there is a building at your east side!\n");
            strcpy(posinfo,"building at east side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }
        else if((quadrant[3].first +quadrant[2].first)<1){
            printf("there is a building at your south side!\n");
            strcpy(posinfo,"building at south side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }
        else if((quadrant[4].first +quadrant[3].first)<1){
            printf("there is a building at your west side!\n");
            strcpy(posinfo,"building at west side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }
        else if((quadrant[4].first +quadrant[1].first)<1){
            printf("there is a building at your north side!\n");
            strcpy(posinfo,"building at north side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }
        else { printf("you are at unknow area!\n");
            strcpy(posinfo,"unknow area!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);}
    }else{
        printf("you are at unknow area!\n");
        strcpy(posinfo,"unknow area!");
        draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
    }



*/
    quadrant.clear();
    return 0;
}





int GPSS::positionjudge2(vector<SAT_SNR_TREND> trend)
{
    char posinfo[100];
    memset(posinfo,0,sizeof(posinfo));

    int trendnum = trend.size();
    for(int i = 0; i<trendnum; i++)
    {
        draw.WriteTrendText(sat_image, trend[i],Point(w/2,w/2));//将当前点的位置，snr变化趋势，snr强度画在图上
        //        trend[i].sat.snr = trend[i].presnr;//用预测的snr代替当前时刻该点的snr值
        //        SatPosDisStat(trend[i].sat, preframe);//将预测的下一时刻卫星分布做统计
    }

    vector<int> changesat(6);//first int is number of the area , second int is change number of the area.
    for ( int i = 0; i<trendnum; i++)
    {
        SAT_SNR_TREND Ttemp;
        memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
        memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
        for(int j = 0; j<firstframe.size(); j++)
        {
            //            changesat[j].first = firstframe[j].first.size();
            for(int m = 0; m<firstframe[j].first.size(); m++)
            {
                if (firstframe[j].first[m] == Ttemp.satnum) {
                    if(Ttemp.presnr< 28) changesat[j] ++;
                }
            }
        }
    }
    //    vector<float> changesit;
    //    for( int i = 0; i < changesat.size(); i++)
    //    {
    //        if(changesat.first == 0) changesit.push_back(2.0);
    //        else {
    //            float temp = changesat.second/changesat.first;
    //            changesit.push_back(temp);
    //        }
    //    }

    if(changesat[4]+changesat[1]+changesat[2]+changesat[3]<= 1){
        printf("you are at open field!!\n");
        strcpy(posinfo,"open field!");
        draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
    } else{
        if(changesat[1]>0 &&(changesat[2]+changesat[3]+changesat[4] == 0)){
            vector<pair<int, int> > nearnum(1);//first is 4th quadrant, second is 2nd quadrant
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[2].first.size(); j++){
                    if (firstframe[2].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.x <0.2) nearnum[0].second ++;
                    }
                }
            }
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[4].first.size(); j++){
                    if (firstframe[4].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.y <0.2) nearnum[0].first ++;
                    }
                }
            }
            if ((nearnum[0].first == firstframe[4].first.size()) && (nearnum[0].second != firstframe[2].first.size())){
                printf("there is a building at your north side!\n");
                strcpy(posinfo,"building at north side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else if((nearnum[0].first != firstframe[4].first.size()) && (nearnum[0].second == firstframe[2].first.size())){
                printf("there is a building at your east side!\n");
                strcpy(posinfo,"building at east side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else{
                printf("you are at unknow area!\n");
                strcpy(posinfo,"unknow area!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }
        }

        else if(changesat[2]>0 &&(changesat[1]+changesat[3]+changesat[4] == 0)){
            vector<pair<int, int> > nearnum(1);//first is 1th quadrant, second is 3nd quadrant
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[1].first.size(); j++){
                    if (firstframe[1].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.x <0.2) nearnum[0].first ++;
                    }
                }
            }
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[3].first.size(); j++){
                    if (firstframe[3].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.y <0.2) nearnum[0].second ++;
                    }
                }
            }
            if ((nearnum[0].first == firstframe[1].first.size()) && (nearnum[0].second != firstframe[3].first.size())){
                printf("there is a building at your east side!\n");
                strcpy(posinfo,"building at east side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else if((nearnum[0].first != firstframe[1].first.size()) && (nearnum[0].second == firstframe[3].first.size())){
                printf("there is a building at your south side!\n");
                strcpy(posinfo,"building at south side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else{
                printf("you are at unknow area!\n");
                strcpy(posinfo,"unknow area!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }
        }

        else if(changesat[3]>0 &&(changesat[2]+changesat[1]+changesat[4] == 0)){
            vector<pair<int, int> > nearnum(1);//first is 2th quadrant, second is 4nd quadrant
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[2].first.size(); j++){
                    if (firstframe[2].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.y <0.2) nearnum[0].first ++;
                    }
                }
            }
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[4].first.size(); j++){
                    if (firstframe[4].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.x <0.2) nearnum[0].second ++;
                    }
                }
            }
            if ((nearnum[0].first == firstframe[2].first.size()) && (nearnum[0].second != firstframe[4].first.size())){
                printf("there is a building at your south side!\n");
                strcpy(posinfo,"building at south side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else if((nearnum[0].first != firstframe[2].first.size()) && (nearnum[0].second == firstframe[4].first.size())){
                printf("there is a building at your west side!\n");
                strcpy(posinfo,"building at west side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else{
                printf("you are at unknow area!\n");
                strcpy(posinfo,"unknow area!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }
        }

        else if(changesat[4]>0 &&(changesat[2]+changesat[3]+changesat[1] == 0)){
            vector<pair<int, int> > nearnum(1);//first is 3th quadrant, second is 1nd quadrant
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[3].first.size(); j++){
                    if (firstframe[3].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.x <0.2) nearnum[0].first ++;
                    }
                }
            }
            for(int i = 0; i<trendnum; i++){
                SAT_SNR_TREND Ttemp;
                memset(&Ttemp,0,sizeof(SAT_SNR_TREND));
                memcpy(&Ttemp,&trend[i],sizeof(SAT_SNR_TREND));
                for(int j = 0; j<firstframe[1].first.size(); j++){
                    if (firstframe[1].first[j] == Ttemp.satnum){
                        float r = cos(Ttemp.sat.elevation/180*pi);
                        float v =(450-Ttemp.sat.azimuth)/180*pi;

                        Point2f cen;
                        cen.x = fabs(r*cos(v));
                        cen.y = fabs(-r*sin(v));

                        if(cen.y <0.2) nearnum[0].second ++;
                    }
                }
            }
            if ((nearnum[0].first == firstframe[3].first.size()) && (nearnum[0].second != firstframe[1].first.size())){
                printf("there is a building at your west side!\n");
                strcpy(posinfo,"building at west side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else if((nearnum[0].first != firstframe[3].first.size()) && (nearnum[0].second == firstframe[1].first.size())){
                printf("there is a building at your north side!\n");
                strcpy(posinfo,"building at north side!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }else{
                printf("you are at unknow area!\n");
                strcpy(posinfo,"unknow area!");
                draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
            }
        }

        else if(changesat[1]>0 && changesat[2]>0 && (changesat[4]+changesat[3] == 0)){//loudong
            printf("there is a building at your east side!\n");
            strcpy(posinfo,"building at east side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        } else if(changesat[3]>0 && changesat[2] >0 && (changesat[1]+changesat[4] == 0)){
            printf("there is a building at your south side!\n");
            strcpy(posinfo,"building at south side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        } else if(changesat[4]>0 && changesat[3] >0 && (changesat[1]+changesat[2] == 0)){
            printf("there is a building at your west side!\n");
            strcpy(posinfo,"building at west side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        } else if(changesat[1]>0 && changesat[4] >0 && (changesat[3]+changesat[2] == 0)){
            printf("there is a building at your north side!\n");
            strcpy(posinfo,"building at north side!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        } else if((changesat[4]> 0 && changesat[1]>0 && changesat[2]>0)||
                  (changesat[3]> 0 && changesat[1]>0 && changesat[2]>0) ||
                  (changesat[3]> 0 && changesat[4]>0 && changesat[2]>0)||
                  (changesat[3]> 0 && changesat[1]>0 && changesat[4]>0)){
            printf("you are between two buildings!\n");
            strcpy(posinfo,"between buildings!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }else{
            printf("you are at unknow area!\n");
            strcpy(posinfo,"unknow area!");
            draw.WriteText(sat_image,Point(w/2,w/2),posinfo);
        }

    }

    changesat.clear();
    return 0;
}


int GPSS::writeSatsIntoSatsvector(vector<SATELLITES_INFO > satinfo)
{
    int vecsize = 5;
    if (satsvector.size()<vecsize){
        satsvector.push_back(satinfo);
    }else {
        satsvector.erase(satsvector.begin());
        satsvector.push_back(satinfo);
    }

    return satsvector.size();
}

int GPSS::satsnrstat(vector<vector<SATELLITES_INFO> > satsinfo)
{
    size_t size = satsinfo.size();
    for (size_t i = 0; i<size; i++)
    {
        vector<SATELLITES_INFO> satvectemp = satsinfo[i];
        size_t size1 = satvectemp.size();
        for (size_t j =0; j<size1; j++)
        {
            SATELLITES_INFO infotemp = satvectemp[j];
            int satnum = satsnrs.size();
            for(int n = 0; n <satnum; n++){
                if(infotemp.prn == satsnrs[n].first){
                    satsnrs[n].second.push_back(infotemp.snr);//may have a bug,check after
                }
            }
        }
        for ( int m = 0; m< satsnrs.size(); m++){
            if(satsnrs[m].second.size()<(i+1)) satsnrs[m].second.push_back(0.0);
        }
    }

    return 0;
}

/*
void GPSS::LineFitLeastSquares(vector<float> snrs, int satnub, int i)
{
    float A = 0.0;
    float B = 0.0;
    float C = 0.0;
    float D = 0.0;
    //   float E = 0.0;
    //   float F = 0.0;

    int snrsize = snrs.size();
    float data_x[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    for ( int i = 0; i< snrsize; i++)
    {
        A += data_x[i]*data_x[i];
        B += data_x[i];
        C += data_x[i]*snrs[i];
        D += snrs[i];
    }

    //calculate slope a and intercept b
    float a, b, temp = 0;
    if(temp = (snrsize*A - B*B))
    {
        a = (snrsize*C-B*D)/temp;
        b = (A*D - B*C)/temp;
    }
    else
    {
        a = 1;
        b = 0;
    }

    //calculate snr at next time  of the sat
    float nextsnr = a * (snrsize +1) +b;
    satstrend[i].slope = a;
    satstrend[i].presnr = nextsnr;
    satstrend[i].satnum = satnub;

    //yan zheng zhi xian ni he jing du
    float lineacc;
    for(int i = 0;i < 5;i++){
        float acctemp = 0;
        acctemp = fabs(a *data_x[i] +b - snrs[i]);
        lineacc += acctemp;
    }

}
*/
void GPSS::BinoFitLeastSquares(vector<float> snrs, int satnub, int i)
{
    float ak[5] = {0};
    float data_x[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    int snrsize = snrs.size();


}


int GPSS::openfieldjudge(vector<SATELLITES_INFO> satinfo)
{
    if(satinfo.size() == 0) return 0;
    int num = satinfo.size();
    int strong = 0;
    int weak = 0;

    for (int i = 0 ; i < num ;i++)
    {
        float snrtemp = satinfo[i].snr;
        int prntemp = satinfo[i].prn;
        float azitemp = satinfo[i].azimuth;
        float eletemp = satinfo[i].elevation;

        //       float r = cos(eletemp/180*pi);
        //       float v =(450-azitemp)/180*pi;

        //       Point2f cen;
        //       cen.x = fabs(r*cos(v));
        //       cen.y = fabs(-r*sin(v));

        if (snrtemp >= 30)
        {
            strong++;
            if (eletemp >= 60){
                firstframe[0].first.push_back(prntemp);
            }else if(eletemp <15){
                firstframe[5].first.push_back(prntemp);
            }else{

                if ( azitemp > 0 && azitemp <=90) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[1].first.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[2].first.push_back(prntemp);
                    //                    }else
                    firstframe[1].first.push_back(prntemp);
                }
                if ( azitemp > 90 && azitemp <=180) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[4].first.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[5].first.push_back(prntemp);
                    //                    }else
                    firstframe[2].first.push_back(prntemp);
                }
                if ( azitemp > 180 && azitemp <=270) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[7].first.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[8].first.push_back(prntemp);
                    //                    }else
                    firstframe[3].first.push_back(prntemp);
                }
                if ( azitemp > 270 && azitemp <=360) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[10].first.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[11].first.push_back(prntemp);
                    //                    }else
                    firstframe[4].first.push_back(prntemp);
                }

            }
        }else
        {
            weak++;
            if (eletemp >= 60){
                firstframe[0].second.push_back(prntemp);
            }else if(eletemp <15){
                firstframe[5].second.push_back(prntemp);
            }else {
                if ( azitemp > 0 && azitemp <=90) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[1].second.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[2].second.push_back(prntemp);
                    //                    }else
                    firstframe[1].second.push_back(prntemp);
                }
                if ( azitemp > 90 && azitemp <=180) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[4].second.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[5].second.push_back(prntemp);
                    //                    }else
                    firstframe[2].second.push_back(prntemp);
                }
                if ( azitemp > 180 && azitemp <=270) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[7].second.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[8].second.push_back(prntemp);
                    //                    }else
                    firstframe[3].second.push_back(prntemp);
                }
                if ( azitemp > 270 && azitemp <=360) {
                    //                    if (cen.x <= 0.2) {
                    //                        firstframe[10].second.push_back(prntemp);
                    //                    }
                    //                    else if(cen.y <= 0.2) {
                    //                        firstframe[11].second.push_back(prntemp);
                    //                    }else
                    firstframe[4].second.push_back(prntemp);
                }
            }
        }


    }
    //    FirstFrameSatPosDisStat(satinfo,firstframe);
    //现规定：strong 卫星数量大于7，weak sat  = 0 为init空旷区域。
    if(strong >= 12 && weak <= 3 && firstframe[1].first.size()>1 && firstframe[2].first.size()>1 &&
            firstframe[3].first.size()>1 && firstframe[4].first.size()>1 ) {
        flag = 1;
        writeinitsatsnub(satinfo);
        FirstFrameSatPosDisStat(satinfo,firstframe);
    }else {
        flag = 0;
        firstframe.clear();
        firstframe.resize(6);
        //        memset(&firstframe,0,sizeof(firstframe));
    }

    return 0;
}

int GPSS::writeinitsatsnub(vector<SATELLITES_INFO> satinfo)
{
    vector<int > satidents;

    size_t size = satinfo.size();

    for (size_t i =0; i<size; i++)
    {
        SATELLITES_INFO infotemp = satinfo[i];
        int prntemp = infotemp.prn;

        if(satidents.size() == 0){
            satidents.push_back(prntemp);
        }else{
            vector<int>::iterator result = find( satidents.begin(), satidents.end(), prntemp);
            if ( result == satidents.end()){
                satidents.push_back(prntemp);
            }
        }

    }

    int satnum = satidents.size();
    satsnrs.clear();
    satsnrs.resize(satnum);
    for (int m = 0; m <satnum; m++)
    {
        satsnrs[m].first = satidents[m];
    }

    return 0;
}


int GPSS::FirstFrameSatPosDisStat(vector<SATELLITES_INFO> satinfo, vector<pair<vector<int>, vector<int> > > firstframetemp)
{
    if(satinfo.size() == 0) return 0;
    int a = 0;
    int num = satinfo.size();
    int satsum = firstframetemp[1].first.size()+firstframetemp[2].first.size()+firstframetemp[3].first.size()+firstframetemp[4].first.size();
    initsatsnrs.resize(satsum);
    for( int i = 1; i < 5; i++ )
    {
        for (int j = 0; j < firstframetemp[i].first.size(); j++)
        {
            for(int m = 0; m < num; m ++)
            {
                int satprntemp = satinfo[m].prn;
                if (satprntemp == firstframetemp[i].first[j]){
                    initsatsnrs[a].first = satprntemp;
                    initsatsnrs[a].second.push_back(satinfo[m].snr);
                    a ++;
                }
            }
        }
    }

    return 0;
}

int GPSS::WriteToInitsatsnrs(vector<SATELLITES_INFO> satinfo)
{
    if(satinfo.size()==0) return 0;
    for(int i = 0; i < satinfo.size(); i++)
    {
        for(int j = 0; j < initsatsnrs.size(); j++)
        {
            if (initsatsnrs[j].first == satinfo[i].prn) {
                initsatsnrs[j].second.push_back(satinfo[i].snr);
            }else{
                initsatsnrs[j].second.push_back(0.0);
            }
        }
    }
    return 0;
}

int GPSS::SatisInitState(vector <pair<int,vector<float> > > initsatsnrsTEMP)
{
    if (initsatsnrsTEMP.size() == 0) return 0;

    for (int i = 0; i < initsatsnrsTEMP.size(); i++)
    {
        for(int j = 0; j < initsatsnrsTEMP[i].second.size(); j++)
        {
            if(initsatsnrsTEMP[i].second[j] <30) return -1;
        }
    }

    return 0;
}


