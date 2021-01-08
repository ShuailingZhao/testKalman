#ifndef POSITIONKALMANFILTER_H
#define POSITIONKALMANFILTER_H
#include "opencv2/video/tracking.hpp"

namespace POSITIONKALMANFILTER
{
	class positionKalmanFilter
	{
		public:
			positionKalmanFilter(int dynamParams, int measureParams, int controlParams, double deltaTime);
			cv::Mat predictStateAndCovariance();
			cv::Mat correctStateCovarianceAndGain(cv::Mat measurement);
		
		private:
			cv::KalmanFilter kF;
			double deltaT;
			cv::Mat measurement;
		
	};
}
#endif
