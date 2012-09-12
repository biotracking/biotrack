
#ifndef BACKGROUNDCALCULATOR_H
#define BACKGROUNDCALCULATOR_H



#include <cv.h> //for linux
#include <highgui.h> //for linux
using namespace cv;
class BackgroundCalculator
{
public:

    BackgroundCalculator(cv::VideoCapture *video, int in, int out);
     ~BackgroundCalculator();

    // void addFrame(ofPixelsRef pixels) = 0;
     virtual void step() = 0;
     virtual int getProgress() = 0;
    virtual bool isFinished() = 0;
    // void finish() = 0;
    virtual void reset() = 0;

    // ofImage * getCurrentBackground() = 0;
   virtual  int getFrameCount() = 0;

     Mat currentBackground;
     unsigned int inFrame;
     unsigned int outFrame;

     int lastFrame;

protected:
    Mat getNextFrame();
	bool gotOutFrame();
	void gotoInFrame();

    VideoCapture* video;
    int in;
    int out;

private:
	
	//ofPixelsRef frame;



};

#endif // BACKGROUNDCALCULATOR_H
