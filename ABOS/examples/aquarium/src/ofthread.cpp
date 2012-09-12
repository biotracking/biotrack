#include "ofthread.h"
#include <opencv/cvaux.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <QApplication>
#include <QVariant>

/**
 Initialize the input, output and all parameters
 */
OFThread::OFThread(AbosPool *input, AbosPool *output_flow, AbosPool *output_mask, int fps)
{
    stopped = false;
    readpool = input;
    writepool_flow = output_flow;
    writepool_mask = output_mask;
    this->fps = fps;
    
    // default values
    winSize = 3;
    levels = 1;
    pryScale = 0.5;
    iterations = 2;

    areaThresh = 750;
    velThresh = 0.75;
    init();
}

/**
  Read the parameters from file
  */
void OFThread::init(){

    char tmp[80]; 

    std::ifstream fileReadStream;
    fileReadStream.open("conf/winSize.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        winSize = atoi(tmp);  // assign the value
    }
    fileReadStream.close();
    fileReadStream.open("conf/levels.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        levels = atoi(tmp);  // assign the value
    }
    fileReadStream.close();
    fileReadStream.open("conf/pryScale.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        pryScale = atof(tmp);  // assign the value
    }
    fileReadStream.close();
    fileReadStream.open("conf/iterations.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        iterations = atof(tmp);  // assign the value
    }
    fileReadStream.close();

    fileReadStream.open("conf/areaThresh.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        areaThresh = atoi(tmp);  // assign the value
    }
    fileReadStream.close();
    fileReadStream.open("conf/velThresh.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        velThresh = atof(tmp);  // assign the value
    }
    fileReadStream.close();
    /*
    std::cerr << "Reading size of windows " << winSize << std::endl;
    std::cerr << "Reading number of prymid levels: " << levels << std::endl;
    std::cerr << "Reading prymid scale: " << pryScale << std::endl;
    std::cerr << "Reading number of iterations: " << iterations << std::endl;
*/
}


/**
  main loop of the thread
  take an image from the readpool, perform optic flow, store the flow image
  and a binary mask of the blobs to the two writepools.
  */
void OFThread::run(){

    stopped = false;
    static unsigned int prev_frameNumber = 0;

    unsigned long long currentFrameNumber = -1;

    while(!stopped ){
        PoolFrame *srcFrame;
        const cv::Mat* srcMat;

        // load an image from readpool
        if(readpool->getSize() == 0) {
            srcFrame = NULL;
        }
        else {

            if(qApp->property("processingMode") == "Realtime")
            {
                srcFrame = readpool->getRecentFrame();
            }
            else
            {
                //srcFrame = readpool->getOldestFrame();
                if(currentFrameNumber == -1)
                {
                    srcFrame = readpool->getOldestFrame();
                    if(srcFrame != NULL)
                        currentFrameNumber = srcFrame->getFrameNumber() + 1;
                }

                srcFrame = readpool->getFrameByNumber(currentFrameNumber);
                if(srcFrame != NULL)
                    currentFrameNumber++;
            }

            //cout << "srcFrame: " << srcFrame << endl;


            if(srcFrame != NULL)
            {
                if( srcFrame->getFrameNumber() == prev_frameNumber ){
                    srcFrame->release();
                    srcFrame = NULL;
                }
                else{
                    prev_frameNumber = srcFrame->getFrameNumber();
                    srcMat = srcFrame->getImage();
                }
            }
        }

        cv::Mat flow, binaryMask;
        if(srcFrame != NULL){
            currentImg = srcMat->clone();
            // call optflow in ABOS/core/modules
            flow = optFlow.farneback( currentImg, pryScale, iterations, winSize, levels);

            binaryMask = getBinaryMask(flow);
        }

        // store the flow image and the binary mask
        if(srcFrame != NULL){
            writepool_flow->storeImage(flow, srcFrame->getFrameNumber());
            writepool_mask->storeImage(binaryMask, srcFrame->getFrameNumber());

            // release reference counter after using frames
            srcFrame->release();
        }

        QThread::usleep(1000000 / fps);
    }
}

/**
  Generate  a binary mask of blobs from the flow image
  */
cv::Mat OFThread::getBinaryMask(const cv::Mat& flow) {

    cv::Mat binary(cv::Size(flow.cols, flow.rows), CV_8UC1, cv::Scalar(0));
    velThreshold(flow, binary);
    cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 3);
    areaThreshold(binary);

    return binary;
}

/**
  Threshold the flow based on velocity
  */
void OFThread::velThreshold(const cv::Mat& flow, cv::Mat& binary) {

    for(int y = 0; y < binary.rows; y += 1) {
        for(int x = 0; x < binary.cols; x += 1) {

            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            double vel = sqrt(pow(fxy.x, 2) + pow(fxy.y, 2));
            if (vel > velThresh)
                cv::circle(binary, cv::Point(x, y), 1, cv::Scalar(255),-1);
        }
    }
}

/**
  Threshold the binary mask based on blob size
  */
void OFThread::areaThreshold(cv::Mat& binary) {

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    cv::findContours (binary, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    cv::Mat binary_temp(cv::Size(binary.cols, binary.rows), CV_8UC1, cv::Scalar(0));
    if (contours.size() > 0) {

        // remove small contours
        vector<vector<cv::Point> >::iterator iter;
        for (iter = contours.begin(); iter!= contours.end();) {

            double area = cv::contourArea(cv::Mat(*iter));
            if (area <= areaThresh)
                iter = contours.erase(iter);
            else
                ++iter;
        }

        // draw the contours for display
        for (unsigned int id = 0; id < contours.size(); id++)
        {
            cv::Scalar color(255);
            cv::drawContours( binary_temp, contours, id, color, CV_FILLED, 8 );
        }
    }
    binary = binary_temp;
    //cerr<<contours.size()<<"\n";
}
