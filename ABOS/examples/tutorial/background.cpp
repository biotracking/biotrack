#include "background.h"

Background::Background(AbosPool *input, AbosPool *output, double alpha, int fps)
{
    stopped = false;
	readpool = input;
	writepool = output;

	this->fps = fps;
	this->alpha = alpha;
	bgSet = false;

}


/**
  main loop of the thread
  take an image from the readpool,
  blur it and store to writepool.
  Other threads can be read the result of the blurred image
  from the writepool.
  */
void Background::run(){
    while(!stopped ){
		PoolFrame *srcFrame;
		const cv::Mat* src_ID;
        // load an image from readpool
        if(readpool->getSize() == 0){
            srcFrame = NULL;
        }
        else{
            srcFrame = readpool->getRecentFrame();
			src_ID = srcFrame->getImage();
		}
        // processes background
        if(srcFrame != NULL){
			if(!bgSet){
				prevBG = src_ID->clone();
				bgSet = true;
			}
			addWeighted(*src_ID,alpha,prevBG,1-alpha,0.0,prevBG);
			//store the result
            writepool->storeImage(prevBG, srcFrame->getFrameNumber());
			// release reference counter after using frames
			srcFrame->release();
        }
		QThread::usleep(1000000 / fps);
    }
}
