#ifndef OFTHREAD_H
#define OFTHREAD_H

#include <cmath>
#include <cstdlib>

#include "AbosThread.h"
#include "optFlow.h"


/**
  This class performs optic flow on the input image and produces the
  flow image as output.
  */
using namespace std;
class OFThread : public AbosThread
{
public:
        OFThread(AbosPool *input, AbosPool *output_flow, AbosPool *output_mask, int fps);

    void run();

private:
	AbosPool *readpool, *writepool_flow, *writepool_mask;
	cv::Mat currentImg;

   
   // params for optic flow algorithm
   int winSize;
   int levels;
   double pryScale;
   int iterations;

   // params to find blobs after optic flow
   int areaThresh;
   double velThresh;

   OptFlow optFlow;

   void init();
   cv::Mat getBinaryMask(const cv::Mat&);
   void velThreshold(const cv::Mat&, cv::Mat&);
   void areaThreshold(cv::Mat&);
};


#endif // OFTHREAD_H
