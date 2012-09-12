#include "bgsubtract.h"
#include <iostream>

BGSubtract::BGSubtract(	AbosPool *input, 
						AbosPool *background, 
						AbosPool *output, 
						int thresh, int fps)
{
    stopped = false;
	readpool = background;
	origpool = input;
	writepool = output;

	this->fps = fps;
	this->thresh = thresh;

}

/**
  main loop of the thread
  take an image from the readpool,
  blur it and store to writepool.
  Other threads can be read the result of the blurred image
  from the writepool.
  */
void BGSubtract::run(){
    while(!stopped){
		PoolFrame *srcFrame;
		const cv::Mat* src_ID;
		PoolFrame *origFrame;
		const cv::Mat* orig_ID;
		if(readpool->getSize() != 0 || origpool->getSize() != 0){
			srcFrame = readpool->getRecentFrame();
			if(srcFrame != NULL){ 
				origFrame = origpool->getFrameByNumber( srcFrame->getFrameNumber() );
				if(origFrame != NULL){
					src_ID = srcFrame->getImage();
					orig_ID = origFrame->getImage();
					cv::Mat tmp;
					cv::resize(*orig_ID,tmp,cv::Size(src_ID->size().width,src_ID->size().height));
					tmp = tmp - *src_ID;
					cv::Mat grayscale(src_ID->size(),CV_8UC1);
					cv::cvtColor(tmp,grayscale,CV_BGR2GRAY);
					cv::threshold(grayscale,grayscale,thresh,255,cv::THRESH_BINARY);
					cv::cvtColor(grayscale,ret,CV_GRAY2BGR);
    	        	//store the result
					writepool->storeImage(ret, srcFrame->getFrameNumber());
					origFrame->release();
				}
				srcFrame->release();
			}
		}
		QThread::usleep(1000000 / fps);
    }
}

