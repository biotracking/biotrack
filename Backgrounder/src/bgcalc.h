#ifndef BGCALC_H
#define BGCALC_H




#include <cv.h> //for linux
#include <highgui.h> //for linux
class BGCalc
{
public:
    BGCalc(VideoCapture* video, int in, int out);
};

#endif // BGCALC_H
