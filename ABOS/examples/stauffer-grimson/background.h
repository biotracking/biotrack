#ifndef BACKGROUND_H
#define BACKGROUND_H

#include <cmath>

#include "AbosThread.h"
#include "MoG.h"


/**
  This class is written to show an example how to
  use this architecture with OpenCV code.

  It reads from captured image sequence from ImageDataPool,
  and returns blurred image sequence to another ImageDataPool
  so that it can be used later(other post processing, etc).
  */

class Background : public AbosThread
{
public:
	Background(AbosPool *input, AbosPool *output, int fps);

    void run();

private:
	AbosPool *readpool, *writepool;
	cv::Mat currentImg;
	MoG *mog;
    int K_VALUE;
    double alpha;
    double T_PORTION;

	//void stauffer_grimson(int k, int nChannels);
    void init();
};


#endif // BACKGROUND_H
