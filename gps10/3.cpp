#include <iostream>
#include "serial.h"
//#include "gpss.h"
//#include <lcm/lcm-cpp.hpp>
//#include "gpslcm/gps_t.hpp"
#include <string.h>

#include <stdio.h>
#include <pthread.h>

using namespace std;

extern double position[3];
extern float static_total[4];

int line_num = 0;
pthread_t show_tid;
pthread_mutex_t mutex;
int quit_flag = 0;
int gps_cnt = 0;


//lcm::LCM glcm;
//gpslcm::gps_t my_data;

int main(int argc, char **argv)
{
    int fd;  //文件描述符
    int err; //返回调用函数的状态
    int len;
    int i;
    char rcv_buf[1000];
    double max_data = 0;

    int flag = 0;
    void *retval;
	int read_cnt = 100;

    static int cnt = 0;

    fd = UART0_Open(fd, "/dev/ttyUSB0"); //打开串口，返回文件描述符

//    pthread_mutex_init(&mutex, NULL);
//    pthread_create(&show_tid, NULL, show_fn, NULL);
    do
    {
        err = UART0_Init(fd, 9600, 0, 8, 1, 'N');
        printf("Set Port Exactly!\n");
    } while (FALSE == err || FALSE == fd);

    //    i = 24*30;
//    fpsat = fopen("sat.txt", "w");
//    fppos = fopen("pos.txt", "w");
//    fpdop = fopen("dop.txt", "w");
//    fpned = fopen("ned.txt", "w");
//    foriginal = fopen("gps.txt","w");

    while (1) //循环读取数据
    {
        do
        {
            memset(rcv_buf,0,sizeof(rcv_buf));
            //len = UART0_Recv(fd, rcv_buf, 512);
			len = read(fd,rcv_buf,1000);
            if (len < 1)
                printf("\n,read error\n");
            else printf("%s",rcv_buf);
//                fwrite(rcv_buf,1,strlen(rcv_buf),foriginal);

        } while (read_cnt --);
		break;

//        pthread_mutex_lock(&mutex);
//        memcpy(&gps_get, &gps_temp, sizeof(gps_temp));
//        gps_cnt++;
//
//        flag = quit_flag;
//        pthread_mutex_unlock(&mutex);
//        if (flag)
//            break;
    }
//    fclose(fpsat);
//    fclose(fppos);
//    fclose(fpdop);
//    fclose(fpned);
//      fclose(foriginal);
    UART0_Close(fd);
//    pthread_join(show_tid,&retval);
    exit(EXIT_SUCCESS);

    //将卫星的位置实时显示出来
    /*
                sat_image = Mat::zeros(w+50,w,CV_8UC3);
                draw.MyCircle(sat_image,Point(w/2,w/2));
     //           fwrite(rcv_buf,1,strlen(rcv_buf),foriginal);
                // show parameter
                parameter_image = Mat::zeros(MAX_WIDTH + MAX_EDGE_X,MAX_HEIGHT + MAX_EDGE_Y,CV_8UC3);

               // ////////////////////////////////////
                if(1)
                {
                    /*
                    //draw parameter
                    cnt ++;
 //                    int x,y;
                    Point pos_current,pos_pre;
 //                    float gps_data_p = random()%512;
                    float gps_data_p = gps.gsa.pdop;
                    float gps_data_v = gps.gsa.vdop;
                    float gps_data_h = gps.gsa.hdop;
                    if(gps_data_p > 80)gps_data_p = 4;
                    if(gps_data_v > 80)gps_data_v = 4;
                    if(gps_data_h > 80)gps_data_h = 4;

                    if(gps_data_p > max_data)
                        max_data = gps_data_p;
                    if(gps_data_v > max_data)
                        max_data = gps_data_v;
                    if(gps_data_h > max_data)
                        max_data = gps_data_h;

                    //draw line
                    line(parameter_image,Point(MAX_EDGE_X,MAX_HEIGHT),Point(MAX_WIDTH + MAX_EDGE_X,MAX_HEIGHT),Scalar(255,0,0),1,8,0);
                    line(parameter_image,Point(MAX_EDGE_X,0),Point(MAX_EDGE_X,MAX_HEIGHT),Scalar(255,0,0),1,8,0);
                    //draw axis
                    const char *temp_buf[5] = {
                        "128","256","384","512","0"
                    };
                    putText(parameter_image,temp_buf[4],Point(30,MAX_HEIGHT +20),1,0.8,Scalar(0,255,0));
                    for(i = 0; i < 4; i ++){
                        line(parameter_image,Point(MAX_EDGE_X,i*MAX_HEIGHT/4+2),Point(MAX_EDGE_X -4,i*MAX_HEIGHT/4+2),Scalar(0,0,255),1,8,0);
                        line(parameter_image,Point(MAX_EDGE_X-2 + (i+1)*MAX_WIDTH/4,MAX_HEIGHT),Point(MAX_EDGE_X -2+ (i + 1)*MAX_WIDTH/4,MAX_HEIGHT + 4),Scalar(0,0,255),1,8,0);
                        char temp[20];
                        memset(temp,0,sizeof(temp));
                        sprintf(temp,"%4.2f",(max_data/4*(4-i)));
                        putText(parameter_image,temp,Point(4,i *MAX_HEIGHT/4.+ (i?6:10)),1,0.8,Scalar(0,255,0));
                        putText(parameter_image,temp_buf[i],Point(MAX_EDGE_X + (i+1)*MAX_WIDTH/4+((i== 3)?-30:-15),MAX_HEIGHT +20),1,0.8,Scalar(0,255,0));

                    }
                    if(cnt < MAX_WIDTH){
                        data_buf_p[cnt - 1] = gps_data_p;

                        for(i = 0; i < cnt; i ++){
                            pos_current.x = (int)draw.convert_x((float)i);
                            pos_current.y = (int)draw.convert_y(data_buf_p[i],max_data);
                            if(i > 0){
                                pos_pre.x = (int)draw.convert_x((float)(i - 1));
                                pos_pre.y = (int)draw.convert_y(data_buf_p[i - 1],max_data);

                                line(parameter_image, pos_pre,pos_current,Scalar(0,0,255),1,8,0);
                            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
                        }

                        data_buf_v[cnt - 1] = gps_data_v;

                        for(i = 0; i < cnt; i ++){
                            pos_current.x = (int)draw.convert_x((float)i);
                            pos_current.y = (int)draw.convert_y(data_buf_v[i],max_data);
                            if(i > 0){
                                pos_pre.x = (int)draw.convert_x((float)(i - 1));
                                pos_pre.y = (int)draw.convert_y(data_buf_v[i - 1],max_data);

                                line(parameter_image, pos_pre,pos_current,Scalar(0,255,0),1,8,0);
                            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
                        }

                        data_buf_h[cnt - 1] = gps_data_h;

                        for(i = 0; i < cnt; i ++){
                            pos_current.x = (int)draw.convert_x((float)i);
                            pos_current.y = (int)draw.convert_y(data_buf_h[i],max_data);
                            if(i > 0){
                                pos_pre.x = (int)draw.convert_x((float)(i - 1));
                                pos_pre.y = (int)draw.convert_y(data_buf_h[i - 1],max_data);

                                line(parameter_image, pos_pre,pos_current,Scalar(255,0,0),1,8,0);
                            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
                        }
                    }else{
                        for(i = 0; i < MAX_WIDTH - 1; i ++){
                            data_buf_p[i] = data_buf_p[i + 1];
                            data_buf_v[i] = data_buf_v[i + 1];
                            data_buf_h[i] = data_buf_h[i + 1];
                        }
                        data_buf_p[MAX_WIDTH - 1] = gps_data_p;
                        data_buf_v[MAX_WIDTH - 1] = gps_data_v;
                        data_buf_h[MAX_WIDTH - 1] = gps_data_h;

                        for(i = 0; i < MAX_WIDTH; i ++){
                            pos_current.x = (int)draw.convert_x((float)i);
                            pos_current.y = (int)draw.convert_y(data_buf_p[i],max_data);
                            if(i > 0){
                                pos_pre.x = (int)draw.convert_x((float)(i - 1));
                                pos_pre.y = (int)draw.convert_y(data_buf_p[i - 1],max_data);

                                line(parameter_image, pos_pre,pos_current,Scalar(0,0,255),1,8,0);
                            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
                        }

                        for(i = 0; i < MAX_WIDTH; i ++){
                            pos_current.x = (int)draw.convert_x((float)i);
                            pos_current.y = (int)draw.convert_y(data_buf_v[i],max_data);
                            if(i > 0){
                                pos_pre.x = (int)draw.convert_x((float)(i - 1));
                                pos_pre.y = (int)draw.convert_y(data_buf_v[i - 1],max_data);

                                line(parameter_image, pos_pre,pos_current,Scalar(0,255,0),1,8,0);
                            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
                        }

                        for(i = 0; i < MAX_WIDTH; i ++){
                            pos_current.x = (int)draw.convert_x((float)i);
                            pos_current.y = (int)draw.convert_y(data_buf_h[i],max_data);
                            if(i > 0){
                                pos_pre.x = (int)draw.convert_x((float)(i - 1));
                                pos_pre.y = (int)draw.convert_y(data_buf_h[i - 1],max_data);

                                line(parameter_image, pos_pre,pos_current,Scalar(255,0,0),1,8,0);
                            }
    //                            parameter_image.at<cv::Vec3b>(pos_current.y, pos_current.x) = Vec3b(255, 0, 0);
                        }
                    }
                    //SHOW LINE
                    imshow("parameter",parameter_image);




                    gpss.show_gps_info(&gps);
                    gpss.gps2ned(&gps);
                    imshow(satwindow,sat_image);

                    char nametemp[20];
                    char key;
                    key = waitKey(300);
                    if(key == ' ')
                    {
                        memset(nametemp,0,sizeof(nametemp));
                        sprintf(nametemp,"%d:%d:%d.jpg",gps.d.hour,gps.d.minute,gps.d.second);
                        imwrite(nametemp,sat_image);
                    }
                    if(key == 'q')
                    {
                        fclose(fpsat);
                        fclose(fppos);
                        fclose(fpdop);
                        fclose(fpned);
      //                  fclose(foriginal);
                        UART0_Close(fd);
                        exit(EXIT_SUCCESS);

                    }

                    memset(&gps,0,sizeof(GPS_INFO));
                }

                memset(rcv_buf,0,sizeof(rcv_buf));

             }
*/

    return 0;
}

/*
int main(int argc, char **argv)
{
    FILE *file;
    char buff[512];
    int i = 0;
    int h = 0;
    int scal_p = 1;
    int step_p = 0;
    char key;
    Point2d ned_data[4096];
    int cnt = 0;
    //    double position[3] = {0};


    file = fopen(argv[1],"r");
    if(!file) return -1;
    printf("%s ",argv[1]);

    GPSS gpss;
    char satwindow[] = "satitudes show";

    fpsat = fopen("sat.txt","w");
    fppos = fopen("pos.txt","w");
    fpdop = fopen("dop.txt","w");
    fpned = fopen("ned.txt","w");

    while (!feof(file)) {
        int c = 0;
        i = 0;
        memset(buff, 0 ,sizeof(buff));

        while((c = fgetc(file)) != EOF){
            if(c != '\n') buff[i ++] = (char)c;
            else{
                buff[i] = (char)c;
                break;
            }
        }

        sat_image = Mat::zeros(w+50, w, CV_8UC3);
        draw.MyCircle(sat_image,Point(w/2,w/2));

        if(gpss.process_gps(buff,&gps)==6){

            if(step_p){
                key = waitKey();
                if(key == 's')step_p = 1;
                else if(key == 'n') step_p = 0;
            }
            gpss.show_gps_info(&gps);
            gpss.gps2ned(&gps);

            imshow(satwindow,sat_image);

            if(cnt < 4096){
                ned_data[cnt ++] = Point2d(position[1],position[0]);
            }else{
                for(i = 0; i < 4096 - 1; i ++){
                    ned_data[i] = ned_data[i + 1];
                }
                ned_data[4096-1]= Point2d(position[1],position[0]);
            }

            draw_ned(ned_pic,ned_data,cnt,scal_p);
            imshow("ned",ned_pic);


            char nametemp[20];

            key = waitKey(200);
            if(key == ' ')
            {
                memset(nametemp,0,sizeof(nametemp));
                sprintf(nametemp,"%d:%d:%d.jpg",gps.d.hour,gps.d.minute,gps.d.second);
                imwrite(nametemp,sat_image);
            }
            if(key == 'q')
            {
                fclose(fpsat);
                fclose(fppos);
                fclose(fpdop);
                fclose(fpned);
                //UART0_Close(fd);
                exit(EXIT_SUCCESS);
            }
            if(key == '='){
                scal_p ++;
                if(scal_p > 10)scal_p = 10;
            }else if(key == '-'){
                scal_p --;
                if(scal_p < 0)scal_p = 0;
            }
            if(key == 's')step_p = 1;
            else if(key == 'n') step_p = 0;

            memset(&gps,0,sizeof(GPS_INFO));
        }
    }
printf("%07.3f %07.3f %07.3f %07.3f\n",static_total[0]*10,static_total[1]*10,static_total[2]*10,static_total[3]*10);
    return 0;
}
*/
/*
void *saveimage(void *arg)
{
    char nametemp[20];
    char key;
    while (1)
    {
        key = waitKey(0);
        if (key == ' ')
        {
            memset(nametemp, 0, sizeof(nametemp));
            sprintf(nametemp, "%d:%d:%d.jpg", gps.d.hour, gps.d.minute, gps.d.second);
            imwrite(nametemp, sat_image);
        }
        //        if(key=='q')
        //        {
        //            exit(0);
        //        }
    }
    return ((void *)0);
}

int draw_ned(Mat &pic, Point2d *data, int data_length, int flag)
{
    int wn = 512;
    int scalar_data[10] = {
        2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
    pic = Mat::zeros(wn + 20, wn + 20, CV_8UC3);
    //    line(parameter_image, pos_pre,pos_current,Scalar(255,0,0),1,8,0);
    line(pic, Point(0, wn / 2), Point(wn, wn / 2), Scalar(0, 100, 0), 1, LINE_4, 0);
    line(pic, Point(wn / 2, 0), Point(wn / 2, wn), Scalar(0, 100, 0), 1, LINE_4, 0);
    circle(pic, Point(wn / 2, wn / 2), wn / 4, Scalar(0, 100, 0), 1, LINE_4);
    circle(pic, Point(wn / 2, wn / 2), wn / 2, Scalar(0, 100, 0), 1, LINE_4);
    char temp[20];
    memset(temp, 0, sizeof(temp));
    sprintf(temp, "%4.2f", (double)(scalar_data[flag]));
    putText(pic, temp, Point(wn / 4 + wn / 2 - 20, wn / 2 + 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    memset(temp, 0, sizeof(temp));
    sprintf(temp, "%4.2f", (double)(scalar_data[flag + 1]));
    putText(pic, temp, Point(wn - 30, wn / 2 + 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);

    putText(pic, "N", Point(wn / 2, 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    putText(pic, "S", Point(wn / 2, wn - 10), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    putText(pic, "W", Point(10, wn / 2 + 20), 1, 1, Scalar(0, 100, 0), 1, 8, 0);
    putText(pic, "E", Point(wn - 30, wn / 2 - 10), 1, 1, Scalar(0, 100, 0), 1, 8, 0);

    int i;
    Point2d posi_temp;
    for (i = 0; i < data_length; i++)
    {
        posi_temp = conv_xy(data[i], scalar_data[flag], data[data_length - 1]);
        if(posi_temp.x > wn || posi_temp.x < 0 || posi_temp.y > wn || posi_temp.y < 0)continue;
        circle(pic,
                posi_temp,
               2,
               Scalar(0, 0, 255),
               -1,
               8,
               0);
    }

    return 0;
}

Point conv_xy(Point2d p, double flag, Point2d center)
{
    Point temp;
    temp.x = (p.x - center.x) / flag * 128 + 256;
    temp.y = -(p.y - center.y) / flag * 128 + 256;
    return temp;
    //    return (x - center_x)/flag *128 + 256;
}

void *show_fn(void *arg)
{

    char satwindow[] = "satitudes show";
    GPS_INFO gps_temp;
    int flag = 0;
    int key = 0;
    int scal_p = 1;
    int step_p = 0;
    Point2d ned_data[4096];
    int cnt = 0;
    int i;

    while (1)
    {
        pthread_mutex_lock(&mutex);
        if (key == 'q')
        {
            pthread_mutex_unlock(&mutex);
            quit_flag = 1;
            break;
        }
        if (gps_cnt >= 1)
        {
            gps_cnt--;
            memcpy(&gps_temp, &gps_get, sizeof(gps_temp));
            flag = 1;
        }
        pthread_mutex_unlock(&mutex);
        if (flag)
        {
            flag = 0;
            sat_image = Mat::zeros(w + 50, w, CV_8UC3);
            draw.MyCircle(sat_image, Point(w / 2, w / 2));

            gpss.show_gps_info(&gps_temp);
            gpss.gps2ned(&gps_temp);
            imshow(satwindow, sat_image);
            key = waitKey(50);

            if(cnt < 4096){
                ned_data[cnt ++] = Point2d(position[1],position[0]);
            }else{
                for(i = 0; i < 4096 - 1; i ++){
                    ned_data[i] = ned_data[i + 1];
                }
                ned_data[4096-1]= Point2d(position[1],position[0]);
            }

            draw_ned(ned_pic,ned_data,cnt,scal_p);
            imshow("ned",ned_pic);

            if(key == '='){
                scal_p ++;
                if(scal_p > 10)scal_p = 10;
            }else if(key == '-'){
                scal_p --;
                if(scal_p < 0)scal_p = 0;
            }
        }
    }
}
*/
