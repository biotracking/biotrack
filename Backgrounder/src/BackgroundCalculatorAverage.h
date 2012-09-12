#pragma once

#include <cv.h> //for linux
#include <highgui.h> //for linux
using namespace cv;

#include "BackgroundCalculator.h"

class BackgroundCalculatorAverage : public BackgroundCalculator
{
public:
    BackgroundCalculatorAverage(VideoCapture* video, float in, float out);
	~BackgroundCalculatorAverage();

	//void addFrame(ofPixelsRef pixels);
	void step();
	int getProgress();
	bool isFinished();
	void reset();

	//ofImage * getCurrentBackground();

	int getFrameCount();

private:
	int frameCount;
};

