#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include "positionKalmanFilter.h"
using namespace cv;
using namespace std;
 
const int winHeight=600;
const int winWidth=800;
 
 
Point mousePosition= Point(winWidth>>1,winHeight>>1);
 
//mouse event callback
void mouseEvent(int event, int x, int y, int flags, void *param )
{
	if (event==CV_EVENT_MOUSEMOVE) {
		mousePosition = Point(x,y);
	}
}

const char* usage = 
"\n"
"./testKalman\n "
"\n";

static void help()
{
    std::cout << usage;
}
 
int main (int argc, char** argv)
{
	help();

//	POSITIONKALMANFILTER::positionKalmanFilter kf(4,2,1,0.2);
	RNG rng;
	//1.kalman filter setup
	const int stateNum=4;                                      //状态值4×1向量(x,y,△x,△y)
	const int measureNum=2;                                    //测量值2×1向量(x,y)	
	KalmanFilter KF(stateNum, measureNum, 0);	
 
//	KF.transitionMatrix = (Mat_<float>(4, 4)<<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
	KF.transitionMatrix = (Mat_<float>(4, 4)<<1,0,0.2,0,0,1,0,0.2,0,0,1,0,0,0,0,1);  //转移矩阵A，状态转移，从前一个状态到下一个状态的转换矩阵
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
	rng.fill(KF.statePost, RNG::UNIFORM, 0, winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
	
	namedWindow("kalman");
	setMouseCallback("kalman",mouseEvent);
		
	Mat image(winHeight,winWidth,CV_8UC3,Scalar(0));
 
	while (1)
	{
		//2.kalman prediction
		Mat prediction = KF.predict();
		Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1) );   //预测值(x',y')
 
		//3.update measurement
		measurement.at<float>(0) = (float)mousePosition.x;
		measurement.at<float>(1) = (float)mousePosition.y;		
 
		//4.update
		Mat fusion = KF.correct(measurement);
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
