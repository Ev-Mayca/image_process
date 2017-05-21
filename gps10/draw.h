#ifndef DRAW_H
#define DRAW_H

#include "gps_structs.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>



#define MAX_WIDTH 512
#define MAX_HEIGHT 512
#define MAX_EDGE_X 50
#define MAX_EDGE_Y 50

#define w 400
#define pi 3.141592653

using namespace cv;
using namespace std;

class Draw
{

public:
    Draw();
    void MyCircle(Mat img, Point center);
    void DrawSatPosition(Mat img, SATELLITES_INFO satinfo,Point center);
    void WritePdop(Mat img, Point center, GPS_GSA_INFO info);
    void WritePos(Mat img, Point center, double lat, double lng);
    void WriteText(Mat img, Point center, char *info);
    void WriteTrendText(Mat img, SAT_SNR_TREND satinfo, Point center);
    float convert_x(float x);
    float convert_y(float y, float max_y);
    int draw_parameter(Mat &img, double data);
    void WriteSnr(Mat img, Point center,vector <pair<int,int> > quadranttemp,
                  vector<float> snravitemp, float snrevi,float std_sats);

};

#endif // DRAW_H
