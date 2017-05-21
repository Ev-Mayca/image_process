/*************************************************************************
	> File Name: merge.cpp
	> Author: Ev
	> Mail: wang2011yiwei@sina.com 
	> Created Time: 2017年05月11日 星期四 16时35分58秒
 ************************************************************************/

//------------------------ Include Files -------------------------------//
#include <iostream>
#include "opencv2/highgui/highgui.hpp"      
#include "opencv2/nonfree/nonfree.hpp"      
#include "opencv2/legacy/legacy.hpp"     

//--------------------------- Veriable ---------------------------------//
using namespace cv;  

//--------------------- Function Prototype -----------------------------//
void findroi(Mat &img,Mat &roi);

//------------------------- Function -----------------------------------//
  
  
//计算原始图像点位在经过矩阵变换后在目标图像上对应位置    
Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri);  
  
int main(int argc, char *argv[])  
{  
    Mat image01,image02;  
    if (argc < 2)  
    {  
        image01 = imread("left.jpg");  
        image02 = imread("right.jpg");  
    }  
    else  
    {  
        image01 = imread(argv[1]);  
        image02 = imread(argv[2]);  
    }  
    if (image01.empty() || image02.empty())  
    {  
        return 0;//图像没有全部读取成功  
    }  
    imshow("拼接图像1", image01);  
    imshow("拼接图像2", image02);  
    double time = getTickCount();  
    //灰度图转换    
    Mat image1, image2;  
    cvtColor(image01, image1, CV_RGB2GRAY);  
    cvtColor(image02, image2, CV_RGB2GRAY);  
  
    //提取特征点      
    SiftFeatureDetector siftDetector(800);  // 海塞矩阵阈值    
    vector<KeyPoint> keyPoint1, keyPoint2;  
    siftDetector.detect(image1, keyPoint1);  
    siftDetector.detect(image2, keyPoint2);  
  
    //特征点描述，为下边的特征点匹配做准备      
    SiftDescriptorExtractor siftDescriptor;  
    Mat imageDesc1, imageDesc2;  
    siftDescriptor.compute(image1, keyPoint1, imageDesc1);  
    siftDescriptor.compute(image2, keyPoint2, imageDesc2);  
  
    //获得匹配特征点，并提取最优配对       
    FlannBasedMatcher matcher;  
    vector<DMatch> matchePoints;  
    matcher.match(imageDesc1, imageDesc2, matchePoints, Mat());  
    if (matchePoints.size() < 10)  
    {  
        return 0;  
    }  
    sort(matchePoints.begin(), matchePoints.end()); //特征点排序，opencv按照匹配点准确度排序      
    //获取排在前N个的最优匹配特征点    
    vector<Point2f> imagePoints1, imagePoints2;  
    for (int i = 0; i<10; i++)  
    {  
        imagePoints1.push_back(keyPoint1[matchePoints[i].queryIdx].pt);  
        imagePoints2.push_back(keyPoint2[matchePoints[i].trainIdx].pt);  
    }  
  
    //获取图像1到图像2的投影映射矩阵，尺寸为3*3    
    Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);  
    Mat adjustMat = (Mat_<double>(3, 3) << 1.0, 0, image01.cols, 0, 1.0, 0, 0, 0, 1.0);//向后偏移image01.cols矩阵  
    Mat adjustHomo = adjustMat*homo;//矩阵相乘，先偏移  
  
    //获取最强配对点（就是第一个配对点）在原始图像和矩阵变换后图像上的对应位置，用于图像拼接点的定位    
    Point2f originalLinkPoint, targetLinkPoint, basedImagePoint;  
    originalLinkPoint = keyPoint1[matchePoints[0].queryIdx].pt;  
    targetLinkPoint = getTransformPoint(originalLinkPoint, adjustHomo);  
    basedImagePoint = keyPoint2[matchePoints[0].trainIdx].pt;  
  
    //图像配准    
    Mat imageTransform;  
    //将图片1进行映射到图像2，本来映射后x值为负值，但是把映射矩阵向后偏移image01.cols矩阵  
    //我们很难判断出拼接后图像的大小尺寸，为了尽可能保留原来的像素，我们尽可能的大一些，对于拼接后的图片可以进一步剪切无效或者不规则的边缘  
    warpPerspective(image01, imageTransform, adjustMat*homo, Size(image02.cols + image01.cols+10, image02.rows));  
  
    //在最强匹配点的位置处衔接，最强匹配点左侧是图1，右侧是图2，这样直接替换图像衔接不好，光线有突变    
    //Mat ROIMat = image02(Rect(Point(basedImagePoint.x, 0), Point(image02.cols, image02.rows)));  
    //ROIMat.copyTo(Mat(imageTransform1, Rect(targetLinkPoint.x, 0, image02.cols - basedImagePoint.x + 1, image02.rows)));  
  
    //在最强匹配点左侧的重叠区域进行累加，是衔接稳定过渡，消除突变    
    Mat image1Overlap, image2Overlap; //图1和图2的重叠部分       
    image1Overlap = imageTransform(Rect(Point(targetLinkPoint.x - basedImagePoint.x, 0), Point(targetLinkPoint.x, image02.rows)));  
    image2Overlap = image02(Rect(0, 0, image1Overlap.cols, image1Overlap.rows));  
    Mat image1ROICopy = image1Overlap.clone();  //复制一份图1的重叠部分    
    for (int i = 0; i<image1Overlap.rows; i++)  
    {  
        for (int j = 0; j<image1Overlap.cols; j++)  
        {  
            double weight;  
            weight = (double)j / image1Overlap.cols;  //随距离改变而改变的叠加系数    
            image1Overlap.at<Vec3b>(i, j)[0] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[0] + weight*image2Overlap.at<Vec3b>(i, j)[0];  
            image1Overlap.at<Vec3b>(i, j)[1] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[1] + weight*image2Overlap.at<Vec3b>(i, j)[1];  
            image1Overlap.at<Vec3b>(i, j)[2] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[2] + weight*image2Overlap.at<Vec3b>(i, j)[2];  
        }  
    }  
    Mat ROIMat = image02(Rect(Point(image1Overlap.cols, 0), Point(image02.cols, image02.rows)));  //图2中不重合的部分    
    ROIMat.copyTo(Mat(imageTransform, Rect(targetLinkPoint.x, 0, ROIMat.cols, image02.rows))); //不重合的部分直接衔接上去    
  
    time = getTickCount() - time;  
    time /= getTickFrequency();  
    printf("match time=%f\n",time);  
	Mat result_image;
	findroi(imageTransform,result_image);
    namedWindow("拼接结果", 0);  
    imshow("拼接结果", result_image);  
    imwrite("matchResult.jpg",result_image);  
    waitKey();  
    return 0;  
}  
  
//计算原始图像点位在经过矩阵变换后在目标图像上对应位置    
Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri)  
{  
    Mat originelP, targetP;  
    originelP = (Mat_<double>(3, 1) << originalPoint.x, originalPoint.y, 1.0);  
    targetP = transformMaxtri*originelP;  
    float x = targetP.at<double>(0, 0) / targetP.at<double>(2, 0);  
    float y = targetP.at<double>(1, 0) / targetP.at<double>(2, 0);  
    return Point2f(x, y);  
}  

void findroi(Mat &img,Mat &roi)
{
	Mat gray,binary;
	cvtColor(img,gray,CV_BGR2GRAY);
	threshold(gray,binary,145,255,THRESH_BINARY);
	int left = img.cols-1,right = 0,top = 0,down = img.rows - 1;
	for(int i = 0; i < img.rows; i ++){
		uchar *ptr_mask = binary.ptr<uchar>(i);
		for(int j = 0; j < img.cols; j ++){
			if(ptr_mask[j] == 255){
				if(i > top) top = i;
				if(i < down) down = i;
				if(j < left) left = j;
				if(j > right) right = j;
			}
		}
	}
	//std::cout << "iamge :" << left <<"down " << down << "right:"<<  right  << "top:"<< top << std::endl;

	int roi_height = top-down;    //ROI圖像的尺寸
    int roi_widgh = right-left;
//    cout<<img.rows<<","<<img.cols<<","<<roi_height<<","<<roi_widgh<<endl;
    roi = Mat::zeros(roi_height,roi_widgh,CV_8UC3);
    for(int i=0;i<roi.rows;i++)
    {
        Vec3b *ptr_img = img.ptr<Vec3b>(i+down);
        Vec3b *ptr_roi = roi.ptr<Vec3b>(i);
        for(int j=0;j<roi.cols;j++)
        {
            ptr_roi[j] = ptr_img[j+left];    //ROI.x = 原圖.x+left
        }                                    //ROI.y = 原圖.y+down
    }
	//Mat result_image = img(Rect(left,down,right,top));
	imshow("jieguo",roi);
	return ;


}
