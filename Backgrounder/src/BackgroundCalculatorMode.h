#pragma once


#include "BackgroundCalculator.h"

#include "ModeAccumulator.h"
#include <cv.h> //for linux
#include <highgui.h> //for linux
#include <iostream>

using namespace cv;
using namespace std;

// TODO abstract this to templates, max_val and stuff
#define NUMBER_OF_VALID_PIXEL_VALUES 256 

#define WORK_UNIT_PIXEL_COUNT (1024*1024)



enum Method { Mode, Median }; // TODO consolidate with #defines

// TODO rename to BackgroundCalculatorModeOrMedian
class BackgroundCalculatorMode : public BackgroundCalculator
{
public:
    BackgroundCalculatorMode(VideoCapture* video, float in, float out, Method method);
	~BackgroundCalculatorMode();

	void step();
	int getProgress();
	bool isFinished();
	void reset();

	//ofImage * getCurrentBackground();

	int getFrameCount();

private:
	void finishUnit();

	int frameCount;
	unsigned int channelPixels;
	int passes;

	Method method;

	//vector<ModeAccumulator> pixelModeAccumulators;
	//vector< vector< unsigned short > > pixelsValueCounts;
	unsigned short* pixelsValueCounts;
	int currentPass;

	unsigned short modePixelInit[NUMBER_OF_VALID_PIXEL_VALUES];
};

