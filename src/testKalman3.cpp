#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include "positionKalmanFilter2.h"
//using namespace cv;
//using namespace std;
 
const int winHeight=600;
const int winWidth=800;
 
 
cv::Point mousePosition= cv::Point(winWidth>>1,winHeight>>1);
 
//mouse event callback
void mouseEvent(int event, int x, int y, int flags, void *param )
{
	if (event==CV_EVENT_MOUSEMOVE)
	{
		mousePosition = cv::Point(x,y);
	}
}

const char* usage = 
"\n"
"./testKalman3\n "
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
    
	
	cv::namedWindow("kalman");
	cv::setMouseCallback("kalman",mouseEvent);
	cv::Mat image(winHeight,winWidth,CV_8UC3,cv::Scalar(0));
 
	while (1)
	{
		//2.重置时间间隔，更新转换矩阵和控制矩阵，外部影响(加速度ax,ay),传感器测量值(位置，鼠标位置)
		float deltaTime = 0.2;//每帧间隔的时间
		float ax = 0.0;
		float ay = 0.0;
		cv::Point2f fusion_pt = kF.getFilterData(mousePosition, deltaTime, ax, ay);
 
		//draw 
		image.setTo(cv::Scalar(255,255,255,0));
//		circle(image,predict_pt,5,Scalar(255,0,0),3);    //predicted point with blue
		cv::circle(image,fusion_pt,5,cv::Scalar(0,255,0),3);    //fusion point with green
		cv::circle(image,mousePosition,5,cv::Scalar(0,0,255),3); //current position with red		
		
		char buf[256];
//		snprintf(buf,256,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
		cv::putText(image,buf,cv::Point(10,30),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,cv::Scalar(0,0,0),1,8);
		snprintf(buf,256,"current position :(%3d,%3d)",mousePosition.x,mousePosition.y);
		cv::putText(image,buf,cv::Point(10,60),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,cv::Scalar(0,0,0),1,8);
		
		cv::imshow("kalman", image);
		int key=cv::waitKey(3);
		if (key==27){//esc   
			break;   
		}		
	}
}
