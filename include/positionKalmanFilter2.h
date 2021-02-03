#ifndef POSITIONKALMANFILTER2_H
#define POSITIONKALMANFILTER2_H
#include "opencv2/video/tracking.hpp"

namespace POSITIONKALMANFILTER
{
	class positionKalmanFilter
	{
		public:
			positionKalmanFilter(int dynamParams, int measureParams, int controlParams, float deltaTime);
			bool reSetupTransitionMatrixAndControlMatrix(const float deltaTime);
			cv::Mat predictStateAndCovariance(const cv::Mat control);
			cv::Mat correctStateCovarianceAndGain(const cv::Mat measurement);
			cv::Point2f getFilterData(const cv::Point2f mousePosition, const float deltaTime = 0.2, const float ax = 0.0, const float ay = 0.0);
		
		private:
			cv::KalmanFilter kF;
			float deltaT;
			cv::Mat measureM;
		
	};
}
#endif
