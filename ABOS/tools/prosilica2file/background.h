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
	Background(AbosPool *input, AbosPool *output, char *outFolder, double alpha, int fps);
        void loadTemplateImage();
        //void on_mouse( int event, int x, int y, int flags, void* param );
        CvScalar hsv2rgb( float hue );
    void run();

private:
	AbosPool *readpool, *writepool;

        //CvVideoWriter *writer;
char *outFolder;
	cv::Mat prevBG;
	double alpha;
    bool bgSet;

	IplImage *image, *hsv, *hue, *mask, *backproject, *histimg;
	CvHistogram *hist;
	int backproject_mode;
	int select_object;
	int track_object;
	int show_hist;
	CvPoint origin;
	CvRect selection;
	CvRect track_window;
	CvBox2D track_box;
	CvConnectedComp track_comp;
	int hdims;
	float hranges_arr[2];
	float* hranges;
	int vmin, vmax, smin;
};

#endif // BACKGROUND_H
