#ifndef BGSUBTRACT_H
#define BGSUBTRACT_H

#include <cmath>
#include "AbosThread.h"

/**
  This class is written to show an example how to
  use this architecture with OpenCV code.

  It reads from captured image sequence from ImageDataPool,
  and returns blurred image sequence to another ImageDataPool
  so that it can be used later(other post processing, etc).
  */

class BGSubtract : public AbosThread
{
public:
    BGSubtract();
	BGSubtract(	AbosPool *input, 
				AbosPool *backgorund, 
				AbosPool *output, 
				int thresh, int fps);

    void run();


private:

	int fps,thresh;
	AbosPool *origpool, *readpool, *writepool;
	cv::Mat ret;

};

#endif // BGSUBTRACT_H
