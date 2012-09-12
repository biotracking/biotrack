#include "ofthread.h"
#include "cvaux.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

OFThread::OFThread(AbosPool *input, AbosPool *output, int fps)
{
    stopped = false;
	readpool = input;
	writepool = output;
    
    // default values
    winSize = 3;
    levels = 1;
    pryScale = 0.5;
    iterations = 2;

	this->fps = fps;
    init();

}

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

    std::cout << "Reading size of windows " << winSize << std::endl;
    std::cout << "Reading number of prymid levels: " << levels << std::endl;
    std::cout << "Reading prymid scale: " << pryScale << std::endl;
    std::cout << "Reading number of iterations: " << iterations << std::endl;
}


/**
  main loop of the thread
  take an image from the readpool,
  blur it and store to writepool.
  Other threads can be read the result of the blurred image
  from the writepool.
  */
void OFThread::run(){
	static unsigned int prev_frameNumber = 0;
    while(!stopped ){
		PoolFrame *srcFrame;
		const cv::Mat* srcMat;

        // load an image from readpool
        if(readpool->getSize() == 0){
            srcFrame = NULL;
        }
        else{
            srcFrame = readpool->getRecentFrame();
			srcMat = srcFrame->getImage();

			if( srcFrame->getFrameNumber() == prev_frameNumber ){
				srcFrame->release();
				srcFrame = NULL;
			}else{
				prev_frameNumber = srcFrame->getFrameNumber();
			}
		}

        // processes background
        if(srcFrame != NULL){
			currentImg = srcMat->clone();
            // call Stauffer-Grimson to get background masked images
			currentImg = optFlow.farneback( currentImg, pryScale, iterations, winSize, levels);
        }

		// generate output and release the source frame
        if(srcFrame != NULL){
            //writepool->storeFrame(srcFrame);
            writepool->storeImage(currentImg, srcFrame->getFrameNumber());
			// release reference counter after using frames
			srcFrame->release();
        }

		QThread::usleep(1000000 / fps);
    }
}

