#ifndef BACKGROUND_H
#define BACKGROUND_H

#include <cmath>

#include "AbosThread.h"

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
	Background(AbosPool *input, AbosPool *output, double alpha, int fps);

    void run();

private:
	AbosPool *readpool, *writepool;
	cv::Mat prevBG;
	double alpha;
    bool bgSet;
};

#endif // BACKGROUND_H
