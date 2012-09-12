#include "optFlow.h"
#include "cvaux.h"
#include <iostream>

#define ELEM(type,start,step,size,xpos,ypos,ichannel) *((type*)(start+step*(ypos)+(xpos)*size+ichannel))

/**
 * process Lucas & Kanade's optical flow algorithm
 */
cv::Mat OptFlow::lucas_kanade( const cv::Mat& currentImg, int winSize){
	static cv::Mat prev;
	static bool isInitPrev = false;

	// convert image to greyscale
	cv::Mat curr(cv::Size(currentImg.cols, currentImg.rows), CV_8UC1);
	cv::cvtColor(currentImg, curr, CV_BGR2GRAY);

    // initialize prev
	if( !isInitPrev ) {
		prev = curr;
		isInitPrev = true;
	}

	cv::Mat flow;	// this would be the flow matrix returned from calcOpticalFlowLK()
	calcOpticalFlowLK(prev, curr, cv::Size(winSize,winSize), flow);
	
	// save for next step
	prev = curr;

    return flow;
}


void OptFlow::drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, const cv::Scalar& color) 
{
   
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            double vel = sqrt(pow(fxy.x, 2) + pow(fxy.y, 2));
            //if (vel > 1)
               cv::line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);

               //cv::circle(cflowmap, cv::Point(x, y), 2, color,-1);
        }
}

/** 
 * process Farneback's optical flow algorithm
 *      G.Farneback, "Two-Frame Motion Estimation Based on Polynomial Expansion",
 *      Proceeding SCIA, 2003
 */
cv::Mat OptFlow::farneback( const cv::Mat& currentImg, double pryScale, int iterations, int winSize, int levels ){

	static cv::Mat prev;
	static bool isInitPrev = false;

	// convert image to greyscale
	cv::Mat curr(cv::Size(currentImg.cols, currentImg.rows), CV_8UC1);
	cv::cvtColor(currentImg, curr, CV_BGR2GRAY);

    // initialize prev
	if( !isInitPrev ) {
		prev = curr;
		isInitPrev = true;
	}

	cv::Mat flow;	// this would be the flow matrix returned from calcOpticalFlowLK()
	//calcOpticalFlowFarneback(prev, curr, pryScale, iterations, winSize, levels, flow);
   cv::calcOpticalFlowFarneback(prev, curr, flow, pryScale, levels, winSize, iterations, 5, 1.1, 0);
	
	// save for next step
	prev = curr;

   return flow;
}



cv::Mat OptFlow::convert2flow(cv::Mat& velx, const cv::Mat& vely) {
	//cv::Mat flow(velx.size(), CV_32FC2);
	cv::Mat flowColorMap(velx.size(), CV_32FC3);
    for(int y = 0 ; y < flowColorMap.rows; ++y){
        for(int x = 0 ; x < flowColorMap.cols; ++x){
            //flow.at<cv::Point2f>(y, x) = cv::Point2f(velx.at<float>(y, x), vely.at<float>(y, x) ); 
			float angle = atan2( vely.at<float>(y,x), velx.at<float>(y,x) );
			float v = sqrt( pow(velx.at<float>(y,x),2) + pow(vely.at<float>(y,x),2) );
			if( (angle < -2.09 && angle > -3.14) || (angle > 2.09 && angle < 3.14) ) angle = 0.467;
			else if( (angle > -1.05 && angle < 0.0) || (angle < 1.05 && angle > 0.0) ) angle = 0.0;
			else if( (angle > 1.05 && angle < 2.09) ) angle = 0.233;
            else angle = 0.1167;
            flowColorMap.at<cv::Point3f>(y, x) = cv::Point3f(angle, v/5.0, v/5.0); 
		}
	}
	cv::Mat tmp, tmp2;
	// The convention is, that for the type CV_8UC3 the pixels values range from 0 to 255, 
	// and for type CV_32FC3 from 0.0 to 1.0.
	flowColorMap.convertTo(tmp, CV_8UC3, 255.0);	
	cv::cvtColor(tmp, tmp2, CV_HSV2BGR); 
	//imwrite("flow.png", tmp);
    //return flow;
    return tmp2;
}

void OptFlow::calcOpticalFlowLK( const cv::Mat& prev, const cv::Mat& curr, cv::Size winSize, cv::Mat& flow ) {
	cv::Mat velx(prev.size(), CV_32F), vely(prev.size(), CV_32F); 
	// wrap cv::Mat to CvMat
    CvMat cvvelx = velx;    CvMat cvvely = vely;
    CvMat cvprev = prev;    CvMat cvcurr = curr;
	// Lucas-Kanade Optical Flow (OpenCV's implementation)
    cvCalcOpticalFlowLK( &cvprev, &cvcurr, winSize, &cvvelx, &cvvely );
	// convert the result into a matrix form
    flow = convert2flow(velx, vely);
}

void OptFlow::calcOpticalFlowFarneback( const cv::Mat& prev, const cv::Mat& curr, double pryScale, int iterations, int winSize, int levels, cv::Mat& flow ) {
	int polyN = 5;
	double polySigma = 1.1;

	static cv::Mat tmp(curr.size(), CV_32FC2);
    static bool isInit = false;
    if(!isInit){
        for(int y = 0 ; y < tmp.rows; ++y){
            for(int x = 0 ; x < tmp.cols; ++x){
                tmp.at<cv::Point2f>(y,x) = cv::Point2f(0.0,0.0);
            }
        }
        isInit = true;
    }

	cv::calcOpticalFlowFarneback(prev, curr, tmp, pryScale, levels, winSize, iterations, polyN, polySigma, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
	//cv::calcOpticalFlowFarneback(prev, curr, tmp, pryScale, levels, winSize, iterations, polyN, polySigma, cv::OPTFLOW_USE_INITIAL_FLOW);

	cv::Mat flowColorMap(tmp.size(), CV_32FC3);
    for(int y = 0 ; y < tmp.rows; ++y){
        for(int x = 0 ; x < tmp.cols; ++x){
            //flow.at<cv::Point2f>(y, x) = cv::Point2f(velx.at<float>(y, x), vely.at<float>(y, x) ); 
			double velx = tmp.at<cv::Point2f>(y,x).x;
			double vely = tmp.at<cv::Point2f>(y,x).y;
            //printf("(%f,%f)\t", velx, vely);
			float angle = (float)atan2( vely, velx );
			float v = (float)sqrt( pow(velx,2) + pow(vely,2) );
			if( (angle < -2.09 && angle > -3.14) || (angle > 2.09 && angle < 3.14) ) angle = 0.467;
			else if( (angle > -1.05 && angle < 0.0) || (angle < 1.05 && angle > 0.0) ) angle = 0.0;
			else if( (angle > 1.05 && angle < 2.09) ) angle = 0.233;
            else angle = 0.1167;
            flowColorMap.at<cv::Point3f>(y, x) = cv::Point3f(angle, v/5.0, v/5.0 ); 
            //flowColorMap.at<cv::Point3f>(y, x) = cv::Point3f(angle, 0, v/5.0 ); 
      }
	}

	cv::Mat tmp2;
	flowColorMap.convertTo(tmp2, CV_8UC3, 255.0);	
	cv::cvtColor(tmp2, flow, CV_HSV2BGR); 
}
