#ifndef OFTHREAD_H
#define OFTHREAD_H

#include <cmath>

#include "AbosThread.h"
#include "optFlow.h"


/**
  This class is written to show an example how to
  use this architecture with OpenCV code.

  It reads from captured image sequence from ImageDataPool,
  and returns blurred image sequence to another ImageDataPool
  so that it can be used later(other post processing, etc).
  */

class OFThread : public AbosThread
{
public:
	OFThread(AbosPool *input, AbosPool *output, int fps);

    void run();

private:
	AbosPool *readpool, *writepool;
	cv::Mat currentImg;

    int winSize;
    int levels;
    double pryScale;
    int iterations;

    OptFlow optFlow;

    void init();
};


#endif // OFTHREAD_H
