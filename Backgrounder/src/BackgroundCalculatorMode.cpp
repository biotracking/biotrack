#include "BackgroundCalculatorMode.h"



BackgroundCalculatorMode::BackgroundCalculatorMode(VideoCapture* video, float in, float out, Method method)
    : BackgroundCalculator(video, in, out)
{
    frameCount = 0;
    pixelsValueCounts = NULL;
    this->method = method;

    for(int v = 0; v < NUMBER_OF_VALID_PIXEL_VALUES; v++)
        modePixelInit[v] = 0;

    Mat pixels;

    video->retrieve(pixels);
    video->read(pixels);

    channelPixels = pixels.channels()*pixels.cols*pixels.rows;
    pixels.copyTo(	currentBackground);
    passes = ceilf( float(channelPixels) / WORK_UNIT_PIXEL_COUNT );
    cout << "will do " << passes << " passes" << endl;

    currentPass = 0;
}


BackgroundCalculatorMode::~BackgroundCalculatorMode(void)
{
    if (pixelsValueCounts != NULL)
        delete[](pixelsValueCounts);
}

void BackgroundCalculatorMode::step()
{
    Mat newpixels = getNextFrame();




    if(frameCount == 0)
    {



        if(pixelsValueCounts != NULL)
            delete[](pixelsValueCounts);
        pixelsValueCounts = new unsigned short[WORK_UNIT_PIXEL_COUNT * NUMBER_OF_VALID_PIXEL_VALUES];

        for(unsigned int p = 0; p < WORK_UNIT_PIXEL_COUNT * NUMBER_OF_VALID_PIXEL_VALUES; p++)
            pixelsValueCounts[p] = 0;

        cout << "before setFromPixels() " << endl;
        cout << "did allocation" << endl;



    }
    else
    {
        float currentWeight = float(frameCount) / float(frameCount + 1);
        float newWeight = 1.0 - currentWeight;
        //Loop through pixels
        int irows=newpixels.rows; // number of lines
        int icols = newpixels.cols; // number of columns
        int istep = newpixels.step;
        int ielemsize= newpixels.elemSize();



#define aPixel(type, dataStart,step,size,x,y,channel)*((type*)(dataStart+step*(y)+(x)*size+channel)) // This says it is fast (maybe a lie)

        uchar* pixelPtr = newpixels.data;

        for (int unitP = 0; unitP < WORK_UNIT_PIXEL_COUNT   //Iterate through each pass
             && currentPass * WORK_UNIT_PIXEL_COUNT + unitP < channelPixels; unitP++) //Stop when we get to the end of data
        {

            pixelsValueCounts[unitP * NUMBER_OF_VALID_PIXEL_VALUES +   pixelPtr[currentPass * WORK_UNIT_PIXEL_COUNT + unitP]  ]++;

            int currentLoc= currentPass * WORK_UNIT_PIXEL_COUNT + unitP;
            int x=currentLoc - icols;
            int  y= currentLoc%icols;
                 }

    }

    frameCount++;

	printf("BG uses %i frames\n", frameCount);

    if(gotOutFrame())
    {
        finishUnit();

        currentPass++;

        if(currentPass < passes)
        {
            frameCount = 0;
            gotoInFrame();
        }
    }
}

int BackgroundCalculatorMode::getProgress()
{
    float segmentProgress = (video->get(CV_CAP_PROP_POS_FRAMES) - in) / (out - in);
    float stepLength =                  (( out - in ) / passes); // Proportion (not really how it works) theamount of frames covered in a pass
    // fractioncoveredinpass = (TotalFrames/NumPasses)/TotalFrames
    float progress = 100 * (( float(currentPass) / passes ) + 1/passes * segmentProgress);

    //cout << "segmentProgress: " << segmentProgress << " stepLength: " << stepLength << "progress: " << progress << endl;

    return progress;
}

bool BackgroundCalculatorMode::isFinished()
{
    return currentPass == passes;
}

void BackgroundCalculatorMode::finishUnit()
{

    Mat bgPixels= currentBackground;


    for (int unitP = 0; unitP < WORK_UNIT_PIXEL_COUNT && currentPass * WORK_UNIT_PIXEL_COUNT + unitP < channelPixels; unitP++)
    {
        unsigned char filteredPixelValue;
        unsigned short maxCount = 0;

        switch(method)
        {
        case Mode:
            unsigned char mostCommonValue;
            for(int v = 0; v < NUMBER_OF_VALID_PIXEL_VALUES; v++)
            {
                if(pixelsValueCounts[unitP * NUMBER_OF_VALID_PIXEL_VALUES + v] > maxCount)
                {
                    maxCount = pixelsValueCounts[unitP * NUMBER_OF_VALID_PIXEL_VALUES + v];
                    mostCommonValue = v;
                }
            }
            filteredPixelValue = mostCommonValue;
            break;

        case Median:
            int samplesSeen = 0;
            int indexOfMedianValue = frameCount / 2;
            bool done = false;
            for(int v = 0; !done && v < NUMBER_OF_VALID_PIXEL_VALUES; v++)
            {
                samplesSeen += pixelsValueCounts[unitP * NUMBER_OF_VALID_PIXEL_VALUES + v];
                if(samplesSeen > indexOfMedianValue)
                {
                    filteredPixelValue = v;
                    done = true;
                }
            }
            break;
        }


        uchar* pixelPtr = bgPixels.data;

        pixelPtr[currentPass * WORK_UNIT_PIXEL_COUNT + unitP] = filteredPixelValue;
    }


}

int BackgroundCalculatorMode::getFrameCount()
{
    return frameCount;
}

void BackgroundCalculatorMode::reset()
{
    frameCount = 0;
}
