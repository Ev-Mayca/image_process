/*************************************************************************
	> File Name: stereocalib.cpp
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2016年12月27日 星期二 15时44分47秒
 ************************************************************************/

//------------------------ Include Files -------------------------------//

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

//--------------------------- Veriable ---------------------------------//

using namespace cv;
using namespace std;

enum {
	STEREO_BM = 0,STEREO_SGBM = 1,STEREO_HH = 2,STEREO_VAR = 3,STEREO_3WAY = 4
};

//--------------------- Function Prototype -----------------------------//

//------------------------- Function -----------------------------------//

static void saveXYZ(const char* filename,const Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename,"wt");
	for(int y = 0; y < mat.rows; y++){
		for(int x = 0; x < mat.cols; x ++){
			Vec3f point = mat.at<Vec3f>(y,x);
			if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2])  >max_z) continue;
			fprintf(fp,"%f %f %f\n",point[0],point[1],point[2]);
		}
	}
	fclose(fp);
}

static void stereocalib(const vector<string> &imagelist, Size boardsize, bool displaycorners = false, bool usecalibrated = true, bool showrectified=true)
{
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
		fs<< "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1,R2,P1,P2,Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0],distCoeffs[0],
			cameraMatrix[1],distCoeffs[1],
			imagesize,R,T,R1,R2,P1,P2,Q,
			CALIB_ZERO_DISPARITY,1,imagesize,&validRoi[0],&validRoi[1]);
	fs.open("extrinsics.yml",FileStorage::WRITE);
	if(fs.isOpened()){
		fs << "R" << R <<"T"<< T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}else
		cout << "Error: can not save the extrinsic parameters\n";

	bool isVerticalStereo = fabs(P2.at<double>(1,3)) > fabs(P2.at<double>(0,3));

	if(!showrectified)
		return ;

	Mat rmap[2][2];
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
	}
	//precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imagesize,CV_16SC2,rmap[0][0],rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1],distCoeffs[1],R2,P2,imagesize,CV_16SC2,rmap[1][0],rmap[1][1]);
	
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
	}

	
}

static bool readStringList( const string& filename, vector<string>& l )
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
    return true;
}

int main(int argc, char** argv)
{
	Size boardSize;
	string imagelistfn;
	bool showRectified;

	boardSize.width = 9;
	boardSize.height = 6;
	showRectified = true;
	imagelistfn = "./stereo_calib.xml";

	vector<string> imagelist;
	bool ok = readStringList(imagelistfn,imagelist);
	if(!ok || imagelist.empty()){
		cout << "can not open "<< imagelistfn << "or the string list is empty" << endl;
		return -1;
	}

	stereocalib(imagelist,boardSize,false,false,showRectified);






	return 0;

}
