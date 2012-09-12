#include "sliders_thread.h"
#include <iostream>
#include <fstream>

#include "BlobPairing.h"

#include <QApplication>
#include <QVariant>

#define MAX_BLOB_FRAMES_IN_MEMORY 10

/**
  Initialize the pools and other parameters
  */
SlidersThread::SlidersThread(AbosPool* input_flow, AbosPool* input_mask, AbosPool* output, AbosPool* orig, list<Blobs> *sliders, int fps) {

    stopped = false;
    readpool_flow = input_flow;
    readpool_mask = input_mask;
    writepool = output;
    origpool = orig;
    blobs_history = sliders;
    this->fps = fps;

    init();

    cout << "SlidersThread: " << this << endl;
}

void SlidersThread::init() {
}

/**
  Compute the slider values for all the frames
  */
void SlidersThread::run() {

    stopped = false;
    static unsigned int prev_frameNumber = 0;

    unsigned long long currentFrameNumber = 1;

    while(!stopped ){
        PoolFrame *flowFrame, *maskFrame, *origFrame;
        const cv::Mat *flowMat, *maskMat, *origMat;

        // load an image from readpool
        if(readpool_flow->getSize() == 0) {
            flowFrame = NULL;
            maskFrame = NULL;
            origFrame = NULL;
        }
        else {
            if(qApp->property("processingMode") == "Realtime")
            {
                flowFrame = readpool_flow->getRecentFrame();
            }
            else
            {
                //flowFrame = readpool_flow->getOldestFrame();
                flowFrame = readpool_flow->getFrameByNumber(currentFrameNumber);
                if(flowFrame != NULL)
                    currentFrameNumber++;
            }

            if(flowFrame != NULL)
            {
                if( flowFrame->getFrameNumber() == prev_frameNumber ){
                    flowFrame->release();
                    flowFrame = NULL;
                    maskFrame = NULL;
                    origFrame = NULL;
                }
                else{
                    prev_frameNumber = flowFrame->getFrameNumber();
                }
            }
        }

        cv::Mat blobs;

        if(flowFrame != NULL){
            flowMat = flowFrame->getImage();
            flowImg = flowMat->clone();

            maskFrame = readpool_mask->getFrameByNumber(flowFrame->getFrameNumber());
            origFrame = origpool->getFrameByNumber(flowFrame->getFrameNumber());
            if (maskFrame != NULL && origFrame != NULL) {
                maskMat = maskFrame->getImage();
                maskImg = maskMat->clone();

                origMat = origFrame->getImage();
                origImg = origMat->clone();

                // compute and store slider values
                publishSliders(flowImg, maskImg, origImg, flowFrame->getFrameNumber());
            }
        }

        // generate output and release the source frame
        if(origFrame!= NULL){
            writepool->storeImage(origImg, origFrame->getFrameNumber());
            origFrame->release();
        }

        // release reference counter after using frames
        if (flowFrame != NULL)
            flowFrame->release();
        if (maskFrame != NULL)
            maskFrame->release();

        QThread::usleep(1000000 / fps);
    }
}

/**
  * Compute and store the blobs inside the shared vector "blobs_history"
  */
void SlidersThread::publishSliders(cv::Mat& flow, cv::Mat& mask, cv::Mat& orig, unsigned long long int frameNo) {

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours (mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    Blobs blobs (flow, contours, frameNo);

    blobs_history->push_back(blobs);
    if(blobs_history->size() > MAX_BLOB_FRAMES_IN_MEMORY)
        blobs_history->pop_front();

    matchNewBlobsWithLastFrameBlobs();
    //cerr << "Number of matched new blobs: " << blobs_history->back().numberOfMatchedBlobs() << endl;

    blobs_history->back().drawBlobsInto(orig);
}




/**
  * Match newest blobs with closest blobs in previous frame, aka track blobs, aka associate blobs.
  */
void SlidersThread::matchNewBlobsWithLastFrameBlobs()
{

    list<Blobs>::iterator blobs_iter = blobs_history->end();

    blobs_iter--;
    Blobs* currentBlobs = &(*blobs_iter);

    Blobs* previousBlobs;


    set<Blob*> unpairedNewBlobs = currentBlobs->toSet();
    set<Blob*> unpairedPreviousBlobs;


    if(blobs_iter != blobs_history->begin())
    {
        blobs_iter--;
        previousBlobs = &(*blobs_iter);
        unpairedPreviousBlobs = previousBlobs->toSet();
    }


    /**************** slightly less naive ************/

    // enumerate all possible pairings
    set<BlobPairing> possiblePairings;
    for(set<Blob*>::iterator previousBlob = unpairedPreviousBlobs.begin(); previousBlob != unpairedPreviousBlobs.end(); previousBlob++)
    {
        for(set<Blob*>::iterator newBlob = unpairedNewBlobs.begin(); newBlob != unpairedNewBlobs.end(); newBlob++)
        {
            BlobPairing pairing(*previousBlob, *newBlob);
            possiblePairings.insert(pairing);
        }
    }

    set<Blob*> taken;
    set<BlobPairing> chosenPairings;

    // accept closest pairs until empty
    for(set<BlobPairing>::iterator candidatePairing = possiblePairings.begin(); candidatePairing != possiblePairings.end(); candidatePairing++)
    {
        Blob* prevBlob = (*candidatePairing).blob1;
        Blob* nextBlob = (*candidatePairing).blob2;

        if(taken.find(prevBlob) == taken.end() && taken.find(nextBlob) == taken.end()) // both not in taken. damn, Python >> C++ sometimez.
        {
            chosenPairings.insert(*candidatePairing);
            taken.insert(prevBlob);
            taken.insert(nextBlob);

            nextBlob->color = prevBlob->color;
            nextBlob->label = prevBlob->label;
            nextBlob->matched = true;
        }
    }

}
