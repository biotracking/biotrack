#include "background.h"
#include "cvaux.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

Background::Background(AbosPool *input, AbosPool *output, int fps)
{
    stopped = false;
	readpool = input;
	writepool = output;
    
    // default values
    K_VALUE = 3;   
    T_PORTION = 0.0;
    alpha = 0.0;

	this->fps = fps;
    init();

}

void Background::init(){

    char tmp[80]; 

    std::ifstream fileReadStream;
    fileReadStream.open("conf/K.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        K_VALUE = atoi(tmp);  // assign the value
    }
    fileReadStream.close();
    fileReadStream.open("conf/learningrate.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        alpha = atof(tmp);  // assign the value
    }
    fileReadStream.close();
    fileReadStream.open("conf/T.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;  /// learning rate is on the first line
        T_PORTION = atof(tmp);  // assign the value
    }
    fileReadStream.close();

    std::cout << "Reading K value (number of Gaussians): " << K_VALUE << std::endl;
    std::cout << "Reading Learning rate: " << alpha << std::endl;
    std::cout << "Reading T portion rate: " << T_PORTION << std::endl;
}


/**
  main loop of the thread
  take an image from the readpool,
  blur it and store to writepool.
  Other threads can be read the result of the blurred image
  from the writepool.
  */
void Background::run(){
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
			currentImg = MoG::stauffer_grimson(currentImg, K_VALUE, currentImg.channels(), alpha, T_PORTION);
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

