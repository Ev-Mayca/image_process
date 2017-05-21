/*************************************************************************
	> File Name: ster.cpp
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2016年12月30日 星期五 16时21分51秒
 ************************************************************************/

//------------------------ Include Files -------------------------------//

#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace std;
using namespace cv;

//#define CALIB_PIC
//--------------------------- Veriable ---------------------------------//
const int imageWidth = 640;                             //摄像头的分辨率  
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;              //三维坐标

Point origin;         //鼠标按下的起始点
Rect selection;      //定义矩形选框
bool selectObject = false;    //是否选择对象

int blockSize = 0, uniquenessRatio =0, numDisparities=0;
Ptr<StereoBM> bm = StereoBM::create(16, 9);
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
int alg = 0;
/*
事先标定好的相机的参数
fx 0 cx
0 fy cy
0 0  1
*/
/*
//Mat cameraMatrixL = (Mat_<double>(3, 3) << 5.3398822221525745e+02, 0., 3.2838497016373424e+02, 0.,
//       5.2871090977394181e+02, 2.3684253625888536e+02, 0., 0., 1.);
//Mat distCoeffL = (Mat_<double>(5, 1) << -2.5899243045047843e-01, -1.2606741533569729e-01, 0., 0., 0.0);
//
//Mat cameraMatrixR = (Mat_<double>(3, 3) << 5.3398822221525745e+02, 0., 3.1377279555791461e+02, 0.,
//       5.2871090977394181e+02, 2.4187108515309174e+02, 0., 0., 1. );
//Mat distCoeffR = (Mat_<double>(5, 1) << -2.6296244652341189e-01, -1.2161092057000163e-02, 0., 0., 0.);
//
//Mat T = (Mat_<double>(3, 1) << -3.3427005392722804e+00, 4.6825258457744941e-02,
//       3.6559704904963321e-03 );//T平移向量
//Mat R = (Mat_<double>(3,3) << 9.9997154349340667e-01, 4.8207258103552001e-03,
//       5.8028274207816760e-03, -4.8862816890287925e-03,
//       9.9992380029162131e-01, 1.1336571863546583e-02,
//       -5.7477347424409247e-03, -1.1364603513685347e-02,
//       9.9991890137766037e-01);//R 旋转矩阵

//Mat cameraMatrixL = (Mat_<double>(3, 3) << 533.93007, 0, 340.91437,
//    0, 534.08496, 234.21776,
//    0, 0, 1);
//Mat distCoeffL = (Mat_<double>(5, 1) << -0.28653,  0.09998,  0.00144, -0.00047, 0.00000);

//Mat cameraMatrixR = (Mat_<double>(3, 3) << 538.74374, 0, 317.76220,
//    0, 539.61133, 249.31017,
//    0, 0, 1);
//Mat distCoeffR = (Mat_<double>(5, 1) << -0.29481,  0.15559,  -0.00068,  0.00166, 0.00000);

//Mat T = (Mat_<double>(3, 1) << 0.00701,  0.01828, -0.003720);//T平移向量
//Mat rec = (Mat_<double>(3, 1) << -33.17861,  0.38367, 0.97263);//rec旋转向量
//Mat R;//R 旋转矩阵


enum {
    STEREO_BM = 0,STEREO_SGBM = 1,STEREO_HH = 2,STEREO_VAR = 3,STEREO_3WAY = 4
};//匹配算法
*/

//--------------------- Function Prototype -----------------------------//

//------------------------- Function -----------------------------------//
static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 200/*1.0e4*/;
    FILE* fp = fopen(filename, "wt");
    cout << mat.rows << " " << mat.cols << endl;
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%d %d %f\n", y,x/*point[0], point[1]*/, point[2]);
        }
    }
    fclose(fp);
}

/*****立体匹配*****/
void stereo_match(int,void*)
{
    Mat disp, disp8;

    if(alg == 1){
        sgbm->setPreFilterCap(63);
        int sgbmWinSize = 3 + ((blockSize%2)?(blockSize - 1):blockSize);
        sgbm->setBlockSize(sgbmWinSize);

        int cn = rectifyImageL.channels();

        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numDisparities*16+16);
        sgbm->setUniquenessRatio(uniquenessRatio);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);

        sgbm->setMode(StereoSGBM::MODE_SGBM);
    //cout << "sgbmwinsize: " << sgbmWinSize << "\nn:" << cn << "\nnumdisparities" << numDisparities*16+16 << "\n" << endl;
        sgbm->compute(rectifyImageL, rectifyImageR, disp);//输入图像必须为灰度图
    }else if(alg == 0){
        bm->setBlockSize(2*blockSize+5);     //SAD窗口大小，5~21之间为宜
        bm->setROI1(validROIL);
        bm->setROI2(validROIR);
        bm->setPreFilterCap(31);
        bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
        bm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(-1);

        bm->compute(rectifyImageL, rectifyImageR, disp);//输入图像必须为灰度图
    }
    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);
}

/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        cout << origin <<"in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
        break;
    case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
        break;
    }
}

static bool readImageList( const string& filename, vector<string>& l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    fs.release();

    return true;
}

static bool readParameter( Mat &cameraMatrixL, Mat &cameraMatrixR,
             Mat &distCoeffL,Mat &distCoeffR,
                            Mat &R,Mat &T, Mat &Q)
{
    //read intrinsics data
    FileStorage fs("intrinsics.yml", FileStorage::READ);

    if( !fs.isOpened() )
        return false;
    fs["cameraMatrixL"] >> cameraMatrixL;
    fs["cameraMatrixR"] >> cameraMatrixR;
    fs["distCoeffL"] >> distCoeffL;
    fs["distCoeffR"] >> distCoeffR;
    fs.release();

    //read extrinsics data
    fs.open("extrinsics.yml",FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["Q"] >> Q;

    fs.release();
//    //read remap data
//    fs.open("remap.yml",FileStorage::READ);
//    if( !fs.isOpened() )
//        return false;
//    fs["mapLx"] >> mapLx;
//    fs["mapLy"] >> mapLy;
//    fs["mapRx"] >> mapRx;
//    fs["mapRx"] >> mapRy;
//    fs["Q"] >> Q;

//    fs.release();

    return true;
}

static void stereocalib( bool displaycorners = false, bool usecalibrated = true, bool showrectified=true)
{
    Size boardsize;
    string imagelistfn;

    boardsize.width = 9;
    boardsize.height = 6;

    imagelistfn = "./stereo_calib.xml";

    vector<string> imagelist;

    bool ok = readImageList(imagelistfn,imagelist);
    if(!ok || imagelist.empty()){
        cout << "can not open "<< imagelistfn << "or the string list is empty" << endl;
        return ;
    }

    if(imagelist.size() %2 != 0){
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }
    const int maxscale = 2;
    const float squaresize = 1.f;
    vector<vector<Point2f> > imagepoints[2];
    vector<vector<Point3f> > objectpoints;
    Size imagesize;

    int i,j,k, nimages = (int)imagelist.size()/2;

    imagepoints[0].resize(nimages);
    imagepoints[1].resize(nimages);
    vector<string> goodimagelist;

    for(i = j = 0; i < nimages; i ++){
        for(k = 0; k < 2; k ++){
            const string& filename = imagelist[i*2 + k];
            Mat img = imread(filename,0);
            if(img.empty())
                break;
            if(imagesize == Size())
                imagesize = img.size();
            else if(img.size() != imagesize){
                cout << "The image" << filename << "has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagepoints[k][j];
            for(int scale = 1; scale <= maxscale; scale++){
                Mat timg;
                if(scale == 1)
                    timg = img;
                else
                    resize(img,timg,Size(),scale,scale);

                found = findChessboardCorners(timg,boardsize,corners,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

                if(found){

                    if(scale > 1){
                        Mat cornerMat(corners);
                        cornerMat *= 1./scale;
                    }
                    break;

                }
            }

            if(displaycorners){
                cout << filename << endl;
                Mat cimg,cimg1;
                cvtColor(img,cimg,COLOR_GRAY2BGR);
                drawChessboardCorners(cimg,boardsize,corners,found);
                double sf = 640./MAX(img.rows,img.cols);
                resize(cimg,cimg1,Size(),sf,sf);
                imshow("corners",cimg1);
                char c = (char)waitKey(500);
                if(c == 27 || c == 'q' || c == 'Q')
                    exit(-1);
            }else
                putchar('.');

            cornerSubPix(img,corners,Size(11,11),Size(-1,-1),
                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,30,0.01));

        }

        if(k == 2){
            goodimagelist.push_back(imagelist[i*2]);
            goodimagelist.push_back(imagelist[i*2+1]);
            j++;

        }

    }
    cout << j << "pairs have been successfully detected.\n";
    nimages = j;
    if(nimages < 2){
        cout << "Error: too little pairs to run the calibration\n";
        return ;
    }

    imagepoints[0].resize(nimages);
    imagepoints[1].resize(nimages);
    objectpoints.resize(nimages);

    for(i = 0; i < nimages; i ++){
        for (j = 0;  j < boardsize.height; j ++)
            for(k = 0; k < boardsize.width; k ++)
                objectpoints[i].push_back(Point3f(k*squaresize,j*squaresize,0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectpoints,imagepoints[0],imagesize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectpoints,imagepoints[1],imagesize,0);
    Mat R,T,E,F;

    double rms = stereoCalibrate(objectpoints,imagepoints[0],imagepoints[1],
            cameraMatrix[0],distCoeffs[0],
            cameraMatrix[1],distCoeffs[1],
            imagesize,R,T,E,F,
            CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST + CALIB_USE_INTRINSIC_GUESS + CALIB_SAME_FOCAL_LENGTH + CALIB_RATIONAL_MODEL+CALIB_FIX_K3 +CALIB_FIX_K4+CALIB_FIX_K5,
            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,100,1e-5));
    cout << "done with RMS error=" << rms << endl;

    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for(i = 0; i < nimages; i ++){
        int npt = (int)imagepoints[0][i].size();
        Mat imgpt[2];
        for(k = 0; k < 2; k ++){
            imgpt[k] = Mat(imagepoints[k][i]);
            undistortPoints(imgpt[k],imgpt[k],cameraMatrix[k],distCoeffs[k],Mat(),cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k + 1,F,lines[k]);
        }
        for(j = 0; j < npt; j ++){
            double errij = fabs(imagepoints[0][i][j].x*lines[1][j][0] +
                    imagepoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                    fabs(imagepoints[1][i][j].x*lines[0][j][0] +
                    imagepoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " << err/npoints << endl;
    //save intrinsic parameters
    FileStorage fs("./intrinsics.yml",FileStorage::WRITE);
    if(fs.isOpened()){
        fs<< "cameraMatrixL" << cameraMatrix[0] << "distCoeffL" << distCoeffs[0] <<
            "cameraMatrixR" << cameraMatrix[1] << "distCoeffR" << distCoeffs[1];
        fs.release();
    }else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1,R2,P1,P2;
//    Rect validRoi[2];

    stereoRectify(cameraMatrix[0],distCoeffs[0],
            cameraMatrix[1],distCoeffs[1],
            imagesize,R,T,R1,R2,P1,P2,Q,
            CALIB_ZERO_DISPARITY,1,imagesize,&validROIL,&validROIR);
    fs.open("extrinsics.yml",FileStorage::WRITE);
    if(fs.isOpened()){
        fs << "R" << R <<"T"<< T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }else
        cout << "Error: can not save the extrinsic parameters\n";

//    bool isVerticalStereo = fabs(P2.at<double>(1,3)) > fabs(P2.at<double>(0,3));

    if(!showrectified)
        return ;

//    Mat rmap[2][2];
    if(usecalibrated){
        //
    }else{
        vector<Point2f> allimgpt[2];
        for(k = 0; k < 2; k ++){
            for(i = 0; i < nimages; i ++){
                std::copy(imagepoints[k][i].begin(),imagepoints[k][i].end(),back_inserter(allimgpt[k]));
            }
        }

        F = findFundamentalMat(Mat(allimgpt[0]),Mat(allimgpt[1]),FM_8POINT,0,0);
        Mat H1,H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]),Mat(allimgpt[1]),F,imagesize,H1,H2,3);

        R1 = cameraMatrix[0].inv() * H1 * cameraMatrix[0];
        R2 = cameraMatrix[1].inv() * H1 * cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
        fs.open("extrinsics2.yml",FileStorage::WRITE);
        if(fs.isOpened()){
            fs << "R" << R <<"T"<< T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
            fs.release();
        }else
            cout << "Error: can not save the extrinsic parameters\n";

    }
    //precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imagesize,CV_32FC1,mapLx,mapLy);
    initUndistortRectifyMap(cameraMatrix[1],distCoeffs[1],R2,P2,imagesize,CV_32FC1,mapRx,mapRy);
//    fs.open("remap1.yml",FileStorage::WRITE);
//    if(fs.isOpened()){
//        fs << "mapLx" << mapLx <<"mapLy"<< mapLy << "mapRx" << mapRx << "mapRy" << mapRy << "Q" << Q;
//        fs.release();
//    }else
//        cout << "Error: can not save the extrinsic parameters\n";
/*
    Mat canvas;
    double sf;
    int w,h;
    if(!isVerticalStereo){
        sf = 600./MAX(imagesize.width,imagesize.height);
        w = cvRound(imagesize.width*sf);
        h = cvRound(imagesize.height*sf);
        canvas.create(h,w*2,CV_8UC3);
    }else{
        sf = 300./MAX(imagesize.width,imagesize.height);
        w = cvRound(imagesize.width*sf);
        h = cvRound(imagesize.height*sf);
        canvas.create(h*2,w,CV_8UC3);
    }

    for(i = 0; i < nimages; i ++){
        for(k = 0; k < 2; k ++){
            Mat img = imread(goodimagelist[i*2+k],0),rimg,cimg;
            remap(img,rimg,rmap[k][0],rmap[k][1],INTER_LINEAR);
            cvtColor(rimg,cimg,COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k,0,w,h)):canvas(Rect(0,h*k,w,h));
            resize(cimg,canvasPart,canvasPart.size(),0,0,INTER_AREA);
            if(usecalibrated){
                Rect vroi(cvRound(validRoi[k].x*sf),cvRound(validRoi[k].y*sf),cvRound(validRoi[k].width*sf),cvRound(validRoi[k].height*sf));
                rectangle(canvasPart,vroi,Scalar(0,0,255),3,8);
            }
        }

        if(!isVerticalStereo)
            for(j = 0; j < canvas.rows; j += 16)
                line(canvas,Point(0,j),Point(canvas.cols,j),Scalar(0,255,0),1,8);
        else
            for(j = 0; j < canvas.cols; j += 16)
                line(canvas,Point(j,0),Point(j,canvas.rows),Scalar(0,255,0),1,8);
        imshow("rectified",canvas);
        char c = (char)waitKey();
        if(c == 27 || c == 'q' || c == 'Q')
            break;

    }

    //stereo match
    Ptr<StereoBM> bm = StereoBM::create(16,9);
    string img1_filename = "./left02.jpg";
    string img2_filename = "./right02.jpg";
    int alg = STEREO_BM;
    int SADWindowSize = 5,numberOfDisparities = 256;
    float scale = -1;
    bool no_display = false;
    string disparity_filename = "./disparity.bmp";
    string point_cloud_filename = "./data.txt";
    int color_mode = alg == STEREO_BM ? 0: -1;
    Mat img1 = imread(img1_filename,color_mode);
    Mat img2 = imread(img2_filename,color_mode);
    if(img1.empty()){
        cout << "could not load the first input image file" << endl;
        return ;
    }
    if(img2.empty()){
        cout << "could not load the second input image file" << endl;
        return ;
    }
    Size img_size = img1.size();

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities:((img_size.width/8) + 15) & -16;
    bm->setROI1(validRoi[0]);
    bm->setROI2(validRoi[1]);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    Mat disp, disp8;

    int64 t = getTickCount();
    if(alg == STEREO_BM)
        bm->compute(img1,img2,disp);
    t = getTickCount() - t;
    cout << "Time elapsed: " << t*1000/getTickFrequency() << "ms" << endl;
    if(alg != STEREO_VAR)
        disp.convertTo(disp8,CV_8U,255/(numberOfDisparities*16.));

    if(!no_display){
        namedWindow("left",1);
        imshow("left",img1);
        namedWindow("right",1);
        imshow("right",img2);
        namedWindow("disparity",1);
        imshow("disparity",disp8);
        cout << "press any key to continue..." << endl;
        waitKey();
    }

    if(!disparity_filename.empty())
        imwrite(disparity_filename,disp8);

    if(!point_cloud_filename.empty()){
        cout << "storing the point cloud..." << endl;
        Mat xyz;
        reprojectImageTo3D(disp,xyz,Q,true);
        saveXYZ(point_cloud_filename.c_str(),xyz);
    }*/

    //移植赋值
//validROIL = validRoi[0];
//validROIR = validRoi[1];
//mapLx = rmap[0][0];
//mapLy = rmap[0][1];
//mapRx = rmap[1][0];
//mapRy = rmap[1][1];

}

/*****主函数*****/
int main(int argc ,char **argv)
{

    int calib_pic = 0;
    if(argc == 3){
        alg = (argv[1][0] == '1')?1:0;
        calib_pic = (argv[2][0] == 'r')?1:0;
    }else {
        cout << argv[0] <<"  " << argv[1] << endl;
        return -1;
    }
//#ifdef CALIB_PIC
    if(calib_pic){
         //校准图像

        Mat cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR,T,R;
        bool ok = readParameter(cameraMatrixL,cameraMatrixR, distCoeffL,distCoeffR,R,T, Q);
        if(!ok || cameraMatrixL.empty()){
            cout << "can not open "<< "intrinsics.yml "<< "or the string list is empty" << endl;
            return -1;
        }
        stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
          -1, imageSize, &validROIL, &validROIR);
        initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
        initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
        FileStorage fs("./remap2.yml",FileStorage::WRITE);
        if(fs.isOpened()){
            fs << "mapLx" << mapLx <<"mapLy"<< mapLy << "mapRx" << mapRx << "mapRy" << mapRy << "Q" << Q;
            fs.release();
        }else
            cout << "Error: can not save the intrinsic parameters\n";
    }else{
    //#else

        stereocalib(false,true);
    //#endif
    }
    /*
    立体校正
    */
//    Rodrigues(rec, R); //Rodrigues变换
//    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
//      -1, imageSize, &validROIL, &validROIR);
//    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
//    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

    /*
    读取图片
    */
    int color_mode = alg == 0 ? 0: -1;
    rgbImageL = imread("./left01.jpg", color_mode);

    rgbImageR = imread("./right01.jpg", color_mode);
//    cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

//    imshow("ImageL Before Rectify", grayImageL);
//    imshow("ImageR Before Rectify", grayImageR);

    /*
    经过remap之后，左右相机的图像已经共面并且行对准了
    */
//    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
//    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    remap(rgbImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(rgbImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    int a,b,c,d;
    a = MAX(validROIL.x,validROIR.x);
    b = MAX(validROIL.y,validROIR.y);
    c = MIN(validROIL.x + validROIL.width,validROIR.width + validROIR.x) - a;
    d = MIN(validROIL.y + validROIL.height,validROIR.y + validROIR.height) - b;
    Rect roi(a,b,c,d);
    imshow("ImageL Before Rectify", rectifyImageL(roi));
    imshow("ImageR Before Rectify", rectifyImageR(roi));
    imwrite("li.jpg",rectifyImageL(roi));
    imwrite("ri.jpg",rectifyImageR(roi));
    /*
    把校正结果显示出来
    */
    Mat rgbRectifyImageL, rgbRectifyImageR;

//    if(alg == 1){
        cvtColor(rectifyImageL(roi), rgbRectifyImageL, CV_GRAY2BGR);  //伪彩色图
        cvtColor(rectifyImageR(roi), rgbRectifyImageR, CV_GRAY2BGR);

        imageSize = rectifyImageR(roi).size();
//    }else{
//        rgbRectifyImageL = rectifyImageL;
//        rgbRectifyImageR = rectifyImageR;
//    }


    //单独显示
//    rectangle(rgbRectifyImageL, validROIL, Scalar(0, 0, 255), 3, 8);
//    rectangle(rgbRectifyImageR, validROIR, Scalar(0, 0, 255), 3, 8);
//    imshow("ImageL After Rectify", rgbRectifyImageL);
//    imshow("ImageR After Rectify", rgbRectifyImageR);

    //显示在同一张图上
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);   //注意通道

    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
    resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小  
    Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域    
        cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
    rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
//    cout << "Painted ImageL" << endl;

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分  
    resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
//    cout << "Painted ImageR" << endl;

    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    imshow("rectified", canvas);

    /*
    立体匹配
    */

    namedWindow("disparity", CV_WINDOW_AUTOSIZE);
    // 创建SAD窗口 Trackbar
    createTrackbar("BlockSize:\n", "disparity",&blockSize, 8, stereo_match);

    // 创建视差唯一性百分比窗口 Trackbar
    createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
    // 创建视差窗口 Trackbar
    createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
    //鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
    setMouseCallback("disparity", onMouse, 0);
    stereo_match(0,0);
    int cnt = 1;
    char templ[20] = "./left01.jpg";
    char tempr[20] = "./right01.jpg";
    while(1){
        int key = waitKey(500);
        if(key == 'p'){
            saveXYZ("data.txt",xyz);

        }else if(key == 'q' || key == 'Q')
            return 0;
        else if(key == 'n' || key == 'N'){
            cnt ++;
            if(cnt > 9)cnt = 1;
            templ[7] = cnt + '0';
            tempr[8] = cnt + '0';

            cout << templ << "  " << tempr << endl;

            rgbImageL = imread(templ, color_mode);
            rgbImageR = imread(tempr, color_mode);

            remap(rgbImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
            remap(rgbImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

            imshow("ImageL Before Rectify", rectifyImageL);
            imshow("ImageR Before Rectify", rectifyImageR);
//            imwrite("ri.jpg",rectifyImageL);
//            imwrite("li.jpg",rectifyImageR);
            stereo_match(0,0);
        }
    }
}
