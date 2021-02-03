#include "positionKalmanFilter2.h"

namespace POSITIONKALMANFILTER
{

	positionKalmanFilter::positionKalmanFilter(int dynamParams, int measureParams, int controlParams, float deltaTime)
	{
		kF.init(dynamParams, measureParams, controlParams);
		deltaT = deltaTime;
		kF.transitionMatrix = (cv::Mat_<float>(4, 4) <<1,0,deltaT,0,0,1,0,deltaT,0,0,1,0,0,0,0,1);  	//转移矩阵A		
		kF.controlMatrix = (cv::Mat_<float>(4, 4) <<deltaT*deltaT/2.0,0.0,0.0,0.0,0.0,deltaT*deltaT/2.0,0.0,0.0,0.0,0.0,deltaT,0.0,0.0,0.0,0.0,deltaT);           //控制矩阵B
		setIdentity(kF.measurementMatrix);                                             //测量矩阵H
		setIdentity(kF.processNoiseCov, cv::Scalar::all(9e-2));                        //系统噪声方差矩阵Q
		setIdentity(kF.measurementNoiseCov, cv::Scalar::all(9e-2));                    //测量噪声方差矩阵R
		setIdentity(kF.errorCovPost, cv::Scalar::all(1));                              //后验错误估计协方差矩阵P
		randn(kF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));		       //初始状态值x(0)
		measureM = cv::Mat::zeros(measureParams, 1, CV_32F);
	}
	
	bool positionKalmanFilter::reSetupTransitionMatrixAndControlMatrix(const float deltaTime)
	{
		deltaT = deltaTime;
		kF.transitionMatrix = (cv::Mat_<float>(4, 4) <<1,0,deltaT,0,0,1,0,deltaT,0,0,1,0,0,0,0,1);  	//转移矩阵A
		kF.controlMatrix = (cv::Mat_<float>(4, 4) <<deltaT*deltaT/2.0,0.0,0.0,0.0,0.0,deltaT*deltaT/2.0,0.0,0.0,0.0,0.0,deltaT,0.0,0.0,0.0,0.0,deltaT);          //控制矩阵B
		return true;
	}
	
	cv::Mat positionKalmanFilter::predictStateAndCovariance(const cv::Mat control)
	{
		return kF.predict(control);
	}
	
	cv::Mat positionKalmanFilter::correctStateCovarianceAndGain(const cv::Mat measurement)
	{
		measureM = measurement;
		return kF.correct(measureM);
	}
	
	cv::Mat positionKalmanFilter::getFilterData(const cv::Point2f mousePosition, const float deltaTime, const float ax, const float ay)
	{
		//2.重新时间间隔，更新转换矩阵和控制矩阵
		reSetupTransitionMatrixAndControlMatrix(deltaTime);//更新转换矩阵和控制矩阵
		
		//3.kalman prediction
		cv::Mat control = (cv::Mat_<float>(4, 1)<<ax,ay,ax,ay);
		cv::Mat prediction = predictStateAndCovariance(control);
		cv::Point2f predict_pt = cv::Point2f(prediction.at<float>(0),prediction.at<float>(1));   //预测值(x',y')
		
		//4.update measurement
		cv::Mat measurement = cv::Mat::zeros(measureM.rows, 1, CV_32F);
		measurement.at<float>(0) = (float)mousePosition.x;
		measurement.at<float>(1) = (float)mousePosition.y;
 
		//5.update
		cv::Mat fusion = correctStateCovarianceAndGain(measurement);		
		
		return fusion;
	}
	
	


}


