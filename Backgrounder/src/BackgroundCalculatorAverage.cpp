#include "BackgroundCalculatorAverage.h"


BackgroundCalculatorAverage::BackgroundCalculatorAverage(VideoCapture* video, float in, float out)
	: BackgroundCalculator(video, in, out)
{
	frameCount = 0;

}


BackgroundCalculatorAverage::~BackgroundCalculatorAverage(void)
{
}

void BackgroundCalculatorAverage::step()
{


    Mat newpixels = getNextFrame();

	if(frameCount == 0)
       newpixels.copyTo(currentBackground);
	else
	{
		// do bg accumulation
                float currentWeight = float(frameCount) / float(frameCount + 1);
                float newWeight = 1.0 - currentWeight;
        //Loop through pixels
       int irows=newpixels.rows; // number of lines
        int icols = newpixels.cols; // number of columns
        int istep = newpixels.step;
        int ielemsize= newpixels.elemSize();


        uchar pixval = 0;

#define aPixel(type, dataStart,step,size,x,y,channel)*((type*)(dataStart+step*(y)+(x)*size+channel)) // This says it is fast (maybe a lie)

    for (int y=0; y < irows; y++)
    {
        for (int x=0; x < icols; x++)
        {

    aPixel(uchar,currentBackground.data,istep,ielemsize,x,y,3)=        aPixel(uchar,currentBackground.data,istep,ielemsize,x,y,3) *currentWeight + newWeight*  aPixel(uchar,newpixels.data,istep,ielemsize,x,y,3);
    aPixel(uchar,currentBackground.data,istep,ielemsize,x,y,2)=        aPixel(uchar,currentBackground.data,istep,ielemsize,x,y,2) *currentWeight + newWeight*  aPixel(uchar,newpixels.data,istep,ielemsize,x,y,2);
    aPixel(uchar,currentBackground.data,istep,ielemsize,x,y,1)=        aPixel(uchar,currentBackground.data,istep,ielemsize,x,y,1) *currentWeight + newWeight* ( aPixel(uchar,newpixels.data,istep,ielemsize,x,y,1));

        }
    }


	}

	frameCount++;

    printf("background ave uses %i frames\n", frameCount);
}

int BackgroundCalculatorAverage::getProgress()
{
    return int(100.0 * float(video->get(CV_CAP_PROP_POS_FRAMES) - in) / (out - in));

}

bool BackgroundCalculatorAverage::isFinished()
{
	return gotOutFrame();
}


int BackgroundCalculatorAverage::getFrameCount()
{
	return frameCount;
}

void BackgroundCalculatorAverage::reset()
{
	frameCount = 0;
}
