
#include "draw.h"
#include <stdio.h>


Draw::Draw()
{

}

void Draw::MyCircle(Mat img, Point center)
{
    int  thickness = 1;
    int  lineType = 8;
    circle(img,
           center,
           w/3,
           Scalar(0,255,0),
           thickness,
           lineType,
           0);
    circle(img,
           center,
           w/6,
           Scalar(0,255,0),
           thickness,
           lineType,
           0);
    circle(img,
           center,
           w*cos(15*pi/180)/3,
           Scalar(0,255,0),
           thickness,
           lineType,
           0);

    Point a1,a2,b1,b2;
    a1.x = center.x-150;
    a1.y = center.y;
    a2.x = center.x+150;
    a2.y = center.y;
    b1.x = center.x;
    b1.y = center.y-150;
    b2.x = center.x;
    b2.y = center.y+150;

    Point c1,c2,d1,d2,e1,e2,f1,f2;
    c1.x = center.x - 150;
    c1.y = center.y - 28;
    c2.x = center.x + 150;
    c2.y = center.y -28;

    d1.x = center.x -150;
    d1.y = center.y +28;
    d2.x = center.x +150;
    d2.y = center.y +28;

    e1.y = center.y - 150;
    e1.x = center.x - 28;
    e2.y = center.y + 150;
    e2.x = center.x -28;

    f1.y = center.y - 150;
    f1.x = center.x + 28;
    f2.y = center.y + 150;
    f2.x = center.x +28;

    line(img, a1,a2,Scalar(0,255,0),1,8,0);
    line(img, b1,b2,Scalar(0,255,0),1,8,0);
//    line(img, c1,c2,Scalar(0,255,0),1,LINE_8,0);
//    line(img, d1,d2,Scalar(0,255,0),1,LINE_8,0);
//    line(img, e1,e2,Scalar(0,255,0),1,LINE_8,0);
//    line(img, f1,f2,Scalar(0,255,0),1,LINE_8,0);
    putText(img,"S",Point(center.x,center.y+170),CV_FONT_HERSHEY_COMPLEX,1,Scalar(255,0,0));
    putText(img,"N",Point(center.x,center.y-150),CV_FONT_HERSHEY_COMPLEX,1,Scalar(255,0,0));
    putText(img,"W",Point(center.x-170,center.y),CV_FONT_HERSHEY_COMPLEX,1,Scalar(255,0,0));
    putText(img,"E",Point(center.x+150,center.y),CV_FONT_HERSHEY_COMPLEX,1,Scalar(255,0,0));
}

void Draw::DrawSatPosition(Mat img, SATELLITES_INFO satinfo, Point center)
{
    float r = cos(satinfo.elevation/180*pi)*w/3;
    float v =(450-satinfo.azimuth)/180*pi;

    Point2f cen;
    cen.x = center.x +r*cos(v);
    cen.y = center.y -r*sin(v);

    circle(img,
           cen,
           2,
           Scalar(0,0,255),
           -1,
           8,
           0);
//   char temp[10];
//    sprintf(temp,"%f",satinfo.snr);
    string temp = format("%d",(int)satinfo.snr);
    putText(img,temp,Point(cen.x+3,cen.y),1,1,Scalar(0,255,0));
//    temp.clear();
//    if (satinfo.slope < 0){
//       temp = "-";
//    }else if(satinfo.slope > 0){
//        temp = "+";
//    }else{
//        temp = "=";
//    }
//    putText(img,temp,Point(cen.x+6,cen.y),1,1,Scalar(0,255,0));

}


void Draw::WritePdop(Mat img, Point center, GPS_GSA_INFO info)
{
    char temp[10];
    putText(img,"PDOP:",Point(center.x+50,center.y-160),1,1,Scalar(0,255,0));
    memset(temp,0,sizeof(temp));
    sprintf(temp,"%f",info.pdop);
    putText(img,temp,Point(center.x+100,center.y-160),1,1,Scalar(0,255,0));
}

void Draw::WritePos(Mat img, Point center, double lat, double lng)
{
    char lattemp[20],lngtemp[20];
    sprintf(lattemp,"%f",lat);
    sprintf(lngtemp,"%f",lng);
    putText(img,"LAT:",Point(center.x+50,center.y-145),1,1,Scalar(0,255,0));
    putText(img,"LNG:",Point(center.x+50,center.y-130),1,1,Scalar(0,255,0));
    putText(img,lattemp,Point(center.x+100,center.y-145),1,1,Scalar(0,255,0));
    putText(img,lngtemp,Point(center.x+100,center.y-130),1,1,Scalar(0,255,0));
}

void Draw::WriteText(Mat img, Point center, char *info)
{
    int len = strlen(info);
//    printf("len is %d \n",len);
    putText(img,info,Point(center.x-len*8/2,center.y+240),1,1,Scalar(0,255,0));
}

void Draw::WriteTrendText(Mat img, SAT_SNR_TREND satinfo, Point center)
{
    float r = cos(satinfo.sat.elevation/180*pi)*w/3;
    float v =(450-satinfo.sat.azimuth)/180*pi;

    Point2f cen;
    cen.x = center.x +r*cos(v);
    cen.y = center.y -r*sin(v);

//    circle(img,
//           cen,
//           2,
//           Scalar(0,0,255),
//           -1,
//           8,
//           0);
   char temp[10];
//    sprintf(temp,"%f",satinfo.snr);
//    string temp = format("%d",(int)satinfo.sat.snr);
//    putText(img,temp,Point(cen.x+3,cen.y),1,1,Scalar(0,255,0));
//    temp.clear();
    if (satinfo.slope < 0){
       strcpy(temp,"-");
    }else if(satinfo.slope > 0){
        strcpy(temp,"+");
    }else{
        strcpy(temp,"=");
    }
    putText(img,temp,Point(cen.x+3,cen.y+15),1,1,Scalar(0,255,0));
}

float Draw::convert_x(float x)
{
    return MAX_EDGE_X + x;
}

float Draw::convert_y(float y,float max_y)
{
    return MAX_HEIGHT - y/max_y*(MAX_HEIGHT - 4);
}

int Draw::draw_parameter(Mat &img, double data)
{
    static unsigned int cnt = 0;
    double max_data = 0;
    static double data_buf[MAX_WIDTH];
    //draw parameter
    cnt ++;
    int i,y;
    Point pos_current,pos_pre;
    float gps_data_p = random()%512;
    //float gps_data_p = data;

    if(gps_data_p > max_data)
        max_data = gps_data_p;

    //draw line
    if(cnt < MAX_WIDTH){
        data_buf[cnt - 1] = gps_data_p;

        for(i = 0; i < cnt; i ++){
            pos_current.x = (int)convert_x((float)i);
            pos_current.y = (int)convert_y(data_buf[i],max_data);
            if(i > 0){
                pos_pre.x = (int)convert_x((float)(i - 1));
                pos_pre.y = (int)convert_y(data_buf[i - 1],max_data);

                line(img, pos_pre,pos_current,Scalar(0,255,0),1,8,0);
            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
        }
    }else{
        for(i = 0; i < MAX_WIDTH - 1; i ++){
            data_buf[i] = data_buf[i + 1];
        }
        data_buf[MAX_WIDTH - 1] = gps_data_p;
        for(i = 0; i < MAX_WIDTH; i ++){
            pos_current.x = (int)convert_x(i);
            pos_current.y = (int)convert_y(data_buf[i],max_data);
            if(i > 0){
                pos_pre.x = (int)convert_x(i - 1);
                pos_pre.y = (int)convert_y(data_buf[i - 1],max_data);

                line(img, pos_pre,pos_current,Scalar(0,255,0),1,8,0);
            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
        }
    }

    return 0;
}


void Draw::WriteSnr(Mat img, Point center, vector<pair<int, int> > quadranttemp,
                    vector<float> snravitemp, float snrevi,float std_sats)
{
    int a1 = quadranttemp[1].first + quadranttemp[1].second;
    int a2 = quadranttemp[2].first + quadranttemp[2].second;
    int a3 = quadranttemp[3].first + quadranttemp[3].second;
    int a4 = quadranttemp[4].first + quadranttemp[4].second;

    char temp[30];
    sprintf(temp,"qua1:%d %f",a1,snravitemp[0]);
    putText(img,temp,Point(center.x+50,center.y+145),1,1,Scalar(0,255,0));
    memset(temp,0,sizeof(temp));
    sprintf(temp,"qua2:%d %f",a2,snravitemp[1]);
    putText(img,temp,Point(center.x+50,center.y+160),1,1,Scalar(0,255,0));
    memset(temp,0,sizeof(temp));
    sprintf(temp,"qua3:%d %f",a3,snravitemp[2]);
    putText(img,temp,Point(center.x+50,center.y+175),1,1,Scalar(0,255,0));
    memset(temp,0,sizeof(temp));
    sprintf(temp,"qua4:%d %f",a4,snravitemp[3]);
    putText(img,temp,Point(center.x+50,center.y+190),1,1,Scalar(0,255,0));
    memset(temp,0,sizeof(temp));
    sprintf(temp,"snrave: %f",snrevi);
    putText(img,temp,Point(center.x-190,center.y+170),1,1,Scalar(0,255,0));

    memset(temp,0,sizeof(temp));
    sprintf(temp,"std_sats: %f",std_sats);
    putText(img,temp,Point(center.x-190,center.y+190),1,1,Scalar(0,255,0));

}
