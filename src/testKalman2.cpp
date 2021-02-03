#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include "positionKalmanFilter2.h"
using namespace cv;
using namespace std;
 
const int winHeight=600;
const int winWidth=800;
 
 
Point mousePosition= Point(winWidth>>1,winHeight>>1);
 
//mouse event callback
void mouseEvent(int event, int x, int y, int flags, void *param )
{
	if (event==CV_EVENT_MOUSEMOVE)
	{
		mousePosition = Point(x,y);
	}
}

const char* usage = 
"\n"
"./testKalman2\n "
"\n";

static void help()
{
    std::cout << usage;
}
 
int main (int argc, char** argv)
{
	help();
	const int stateNum=4;                                      //状态值4×1向量(x,y,vx,vy) (x,y,△x,△y)
	const int measureNum=2;                                    //测量值2×1向量(x,y)
	const int controlNum=4;				          //控制量4x1向量ax,ay,ax,ay
	const double initDeltaTime = 0.2;
	//1.初始化kalman对象
	POSITIONKALMANFILTER::positionKalmanFilter kF(stateNum, measureNum, controlNum, initDeltaTime);
    
	
	namedWindow("kalman");
	setMouseCallback("kalman",mouseEvent);
	Mat image(winHeight,winWidth,CV_8UC3,Scalar(0));
 
	while (1)
	{
		//2.重新时间间隔，更新转换矩阵和控制矩阵
		float deltaTime = 0.2;//每帧间隔的时间
		kF.reSetupTransitionMatrixAndControlMatrix(deltaTime);//更新转换矩阵和控制矩阵
		
		//3.kalman prediction
		float ax = 0.0;
		float ay = 0.0;
		cv::Mat control = (cv::Mat_<float>(4, 1)<<ax,ay,ax,ay);
		cv::Mat prediction = kF.predictStateAndCovariance(control);
		Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1));   //预测值(x',y')
		
		//4.update measurement
		cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);
		measurement.at<float>(0) = (float)mousePosition.x;
		measurement.at<float>(1) = (float)mousePosition.y;
 
		//5.update
		Mat fusion = kF.correctStateCovarianceAndGain(measurement);
		Point fusion_pt = Point(fusion.at<float>(0),fusion.at<float>(1) );   //预测值(x',y')
 
		//draw 
		image.setTo(Scalar(255,255,255,0));
		circle(image,predict_pt,5,Scalar(255,0,0),3);    //predicted point with blue
		circle(image,fusion_pt,5,Scalar(0,255,0),3);    //fusion point with green
		circle(image,mousePosition,5,Scalar(0,0,255),3); //current position with red		
		
		char buf[256];
		snprintf(buf,256,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
		putText(image,buf,Point(10,30),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
		snprintf(buf,256,"current position :(%3d,%3d)",mousePosition.x,mousePosition.y);
		putText(image,buf,cvPoint(10,60),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
		
		imshow("kalman", image);
		int key=waitKey(3);
		if (key==27){//esc   
			break;   
		}		
	}
}
