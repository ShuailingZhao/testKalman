#include "positionKalmanFilter.h"

namespace POSITIONKALMANFILTER
{

	positionKalmanFilter::positionKalmanFilter(int dynamParams, int measureParams, int controlParams, double deltaTime)
	{
		kF.init(dynamParams, measureParams, controlParams);
		deltaT=deltaTime;
		kF.transitionMatrix = (cv::Mat_<float>(4, 4) <<1,0,deltaT,0,0,1,0,deltaT,0,0,1,0,0,0,0,1);  	//转移矩阵A		
		kF.controlMatrix = (cv::Mat_<float>(4, 1) <<deltaT*deltaT/2.0,deltaT*deltaT/2.0,deltaT,deltaT);            	//控制矩阵B
		setIdentity(kF.measurementMatrix);                                             //测量矩阵H
		setIdentity(kF.processNoiseCov, cv::Scalar::all(9e-2));                            //系统噪声方差矩阵Q
		setIdentity(kF.measurementNoiseCov, cv::Scalar::all(9e-2));                        //测量噪声方差矩阵R
		setIdentity(kF.errorCovPost, cv::Scalar::all(1));                               	//后验错误估计协方差矩阵P
		randn(kF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));						//初始状态值x(0)
		measurement = cv::Mat::zeros(measureParams, 1, CV_32F);
	}
	
	cv::Mat positionKalmanFilter::predictStateAndCovariance()
	{
		return kF.predict();
	}
	
	cv::Mat positionKalmanFilter::correctStateCovarianceAndGain(cv::Mat measurement)
	{
		return kF.correct(measurement);
	}


}


