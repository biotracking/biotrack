#pragma once

#include "BackgroundCalculator.h"
#include <iostream>

#define BACKGROUND_METHOD_AVERAGE 0
#define BACKGROUND_METHOD_MODE 1
#define BACKGROUND_METHOD_MEDIAN 2

BackgroundCalculator::BackgroundCalculator(VideoCapture * video, int in, int out)
{
    this->video = video;
    this->in = in;
    this->out = out;

    outFrame= out;
    inFrame=in;

    gotoInFrame();
}

BackgroundCalculator::~BackgroundCalculator()
{

}

Mat BackgroundCalculator::getNextFrame()
{
    Mat nextFrame;
    int frame = lastFrame + 1;
    video->set(CV_CAP_PROP_POS_FRAMES,frame);

    lastFrame = frame;

    video->retrieve(nextFrame);
    video->read(nextFrame);
    return nextFrame;
}

bool BackgroundCalculator::gotOutFrame()
{
    return lastFrame >= outFrame;
}

void BackgroundCalculator::gotoInFrame()
{
    lastFrame = inFrame - 1;
}
