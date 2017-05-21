#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static bool readStringList(const string& filename,vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename,FileStorage::READ);
	if(!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if(n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(),it_end = n.end();
	for(;it != it_end; ++ it)
		l.push_back((string) *it);
	return true;
}

static bool run_save(const string& outputFilename,
		const vector<vector<Point2f> > & imagePoints,
		Size imageSize,Size boardSize,Pattern patternType,
		float squareSize,float aspectRatio,int flags,Mat & cameraMatrix,
		Mat & distCoeffs)
{
	vector<Mat> rvecs,tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;
	cameraMatrix = Mat::eye(3,3,CV_64F);
	if(flags & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0,0) = aspectRatio;

	distCoeffs = Mat::zeros(8,1,CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	objectPoints[0].resize(0);
	for(int i = 0; i < boardSize.height; i ++)
		for(int j = 0;j < boardSize.width; j ++)
			objectPoints[0].push_back(Point3f((j*squareSize),(i*squareSize),0.));
	objectPoints.resize(imagePoints.size(),objectPoints[0]);

	double rms = calibrateCamera(objectPoints,imagePoints,imageSize,
			cameraMatrix,distCoeffs,rvecs,tvecs,flags|CALIB_FIX_K4|CALIB_FIX_K5);
	
	printf("RMS error reported by calibrateCamera: %g\n",rms);
	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	vector<Point2f> imagePoints2;
	int i,totalPoints = 0;
	double totalErr = 0,err;
	vector<float> perViewErrors;
	perViewErrors.resize(objectPoints.size());
	for(i = 0; i < (int)objectPoints.size(); i ++){
		projectPoints(Mat(objectPoints[i]),rvecs[i],tvecs[i],cameraMatrix,distCoeffs,imagePoints2);
		err = norm(Mat(imagePoints[i]),Mat(imagePoints2),NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err/n);
		totalErr += err*err;
		totalPoints += n;

	}
	totalAvgErr = std::sqrt(totalErr/totalPoints);
	printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);
	return ok;
}

int main()
{
	Size boardSize,imageSize;
	float squareSize;
	Mat cameraMatrix, distCoeffs;
	string outputFilename;

	int i,flags,nframes;
	//int delay;
	float aspectRatio = 1.;
	int mode = DETECTION;
	vector<vector<Point2f> > imagePoints;
	vector<string> imageList;
	Pattern pattern = CHESSBOARD;

	nframes = 0;
	flags = 0;
	flags |= CALIB_FIX_ASPECT_RATIO;
	boardSize.width = 9;
	boardSize.height = 6;
	squareSize = 1.0;
	//delay = 1000;
	string inputFilename = "./cali.xml";

	readStringList(inputFilename,imageList);
	mode = CAPTURING;
	if(!imageList.empty())
		nframes = (int)imageList.size();
	namedWindow("Image View",1);

	for(i = 0; ; i ++){
		Mat view,viewgray;
		//bool blink = false;

		if(i < (int)imageList.size()){
			view = imread(imageList[i],1);
		}
		
		if(view.empty()){
			if(imagePoints.size() > 0)
				run_save(outputFilename,imagePoints,imageSize,
						boardSize,pattern,squareSize,aspectRatio,
						flags,cameraMatrix,distCoeffs
						);
			cout << "empty"<<endl;
			break;
		}

		imageSize = view.size();
		cout << i << "," << imageSize << endl;

		vector<Point2f> pointbuf;
		cvtColor(view,viewgray,COLOR_BGR2GRAY);

		bool found;
		found = findChessboardCorners(view,boardSize,pointbuf,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		//improve the found corners' coordinate accuracy
		if(found){
			
			cornerSubPix(viewgray,pointbuf,Size(11,11),
				Size(-1,-1),TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,30,0.1));
			if(mode == CAPTURING){
				imagePoints.push_back(pointbuf);
				
			}
			drawChessboardCorners(view,boardSize,Mat(pointbuf),found);

			string msg = mode == CAPTURING ? "100 / 100":"Calibrated ";
			int baseLine = 0;
			Size textSize = getTextSize(msg,1,1,1,&baseLine);
			Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2* baseLine - 10);
			if(mode == CAPTURING){
				msg = format("%d / %d",(int)imagePoints.size(),nframes);
			}
			putText(view,msg,textOrigin,1,1,Scalar(0,0,255));
			imshow("Image View",view);
			int key = 0xff & waitKey(500);
			if((key & 0xff) == 27)
				break;

		}
		if(mode == CAPTURING && imagePoints.size() >= (unsigned)nframes){
			if(run_save(outputFilename,imagePoints,imageSize,
						boardSize,pattern,squareSize,aspectRatio,
						flags,cameraMatrix,distCoeffs))
				mode = CALIBRATED;
			else
				mode = DETECTION;
			cout << "run_save"<<endl;
			break;

		}
		
	}
	
	return 0;

}
