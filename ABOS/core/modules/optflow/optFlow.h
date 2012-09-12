#ifndef LKOPTFLOW_H
#define LKOPTFLOW_H

#include <cmath>
#include <cv.h>

/**
  This class is written to show an example how to
  use this architecture with OpenCV code.

  It provides the way to use the Lucas-Kanade's optical flow
  algorithm. 
  */

class OptFlow 
{
public:
    cv::Mat lucas_kanade( const cv::Mat& currentImg, int winSize);
    cv::Mat farneback( const cv::Mat& currentImg, double pryScale, int iterations, int winSize, int levels );
    void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, const cv::Scalar& color);

private:
	//cv::Mat currentImg;

	cv::Mat convert2flow(cv::Mat& velx, const cv::Mat& vely);
	void calcOpticalFlowLK( const cv::Mat& prev, const cv::Mat& curr, cv::Size winSize, cv::Mat& flow );
    void calcOpticalFlowFarneback( const cv::Mat& prev, const cv::Mat& curr, double pryScale, int iterations, int winSize, int levels, cv::Mat& flow );
};


#endif // LKOPTFLOW_H
