#include "imagecapture.h"

#include <iostream>
#include <cstdio>
#include <unistd.h>

using namespace cv;
// Constructor
ImageCapture::ImageCapture(int framerate) {
   this->stopped = false;
   paused = false;
	fps = framerate;
   input_set = true;
	nextFrameNo = 0;
}

void ImageCapture::stop(){
    stopped = true;
    goToFrame(0);
}

void ImageCapture::run() {
    // main function of the thread
    // read a fram from the capture configured and
    // store it to the global buffer

    // make sure all capture related settings are
    // already set before this method
   stopped = false;
   paused = false;
   
   video_capture.set(CV_CAP_PROP_POS_FRAMES, nextFrameNo);
   
    do {
		Mat frame,scaled_frame;

		video_capture >> frame;
      if (frame.rows > 0 && frame.cols > 0) {

         scaled_frame = frame.clone();
         double currFrameNo = video_capture.get(CV_CAP_PROP_POS_FRAMES);
         if (unscaledpool != NULL) {
            unscaledpool->storeImage(scaled_frame, (currFrameNo < 0 ? 0 : currFrameNo));
       }
         static cv::Size scaled_size = getScaledSize(frame.cols, frame.rows);
         cv::resize(frame,scaled_frame,scaled_size,0,0, INTER_AREA);

		   readpool->storeImage(scaled_frame, (currFrameNo < 0 ? 0 : currFrameNo));
   
		   QThread::usleep(1000000 / fps);
         }

      } while (!stopped && !paused);// && img != NULL);
}

void ImageCapture::pause() {
   if (file_name.length() > 0) {
      paused = true;
      nextFrameNo = video_capture.get(CV_CAP_PROP_POS_FRAMES);
   }
   else {
       // cannot pause when reading from camera
      stop();
   }
}

bool ImageCapture::goToFrame(long long int frameNo) {
   
	if(file_name.length() > 0){

      if (frameNo > (signed long long)getFrameCount())
         frameNo = getFrameCount() - 1;
      if (frameNo < 0)
         frameNo = 0;
      
      //cout<<frameNo<<" ";
      video_capture.set(CV_CAP_PROP_POS_FRAMES, frameNo);
      //cout<<video_capture.get(CV_CAP_PROP_POS_FRAMES)<<"\n";
	} 
   else {
      // cannot go to frame when reading from camera
		return false;
	}

	Mat frame,scaled_frame;

   //if (video_capture.get(CV_CAP_PROP_POS_FRAMES) == frameNo) {
   //   video_capture >> frame;
   //}
   //while (video_capture.get(CV_CAP_PROP_POS_FRAMES) < frameNo) {
	   video_capture >> frame;
   //}
   
   if (frame.rows > 0 && frame.cols > 0) {

		scaled_frame = frame.clone();
      double currFrameNo = video_capture.get(CV_CAP_PROP_POS_FRAMES);
       if (unscaledpool != NULL) {
          unscaledpool->storeImage(scaled_frame, (currFrameNo < 0 ? 0 : currFrameNo));
      }
      static cv::Size scaled_size = getScaledSize(frame.cols, frame.rows);
      cv::resize(frame,scaled_frame,scaled_size,0,0, INTER_AREA);

		readpool->storeImage(scaled_frame, (currFrameNo < 0 ? 0 : currFrameNo));
      mutex_nextFameNo.lock();
      nextFrameNo = video_capture.get(CV_CAP_PROP_POS_FRAMES);
      mutex_nextFameNo.unlock();
   }
   return true;
}

void ImageCapture::setImagePool(AbosPool *readpool, AbosPool *unscaledpool){
    this->readpool = readpool;
    this->unscaledpool = unscaledpool;
}

void ImageCapture::setDeviceFile(string file_name){
    this->file_name = file_name;
    //this->capture = cvCreateFileCapture(file_name.c_str());
    this->capture = cvCaptureFromAVI(file_name.c_str());

    // open the video capture
    if(file_name.length() > 0){
		std::cout<<"Ready to capture from ["<<file_name<<"]"<<std::endl;
		video_capture.open(file_name);
	 } 
    else {
		std::cout<<"Ready to capture from first available camera"<<std::endl;
		video_capture.open(0);
	}
}

CvCapture* ImageCapture::getCvCapture(){
    return capture;
}

void ImageCapture::readFromCamera(bool b_read_camera){
    if(b_read_camera)
        capture = cvCreateCameraCapture(0);
}

/**
   Returns the new scaled down size while preserving the aspect ratio.
*/
cv::Size ImageCapture::getScaledSize(int cols, int rows) {

   int multiplier;

   int gcd = this->gcd(cols, rows);
   int aspect_cols = cols / gcd;
   int aspect_rows = rows / gcd;

   if (aspect_cols == 3 && aspect_rows == 2)
      multiplier = 160;
   else if (aspect_cols == 4 && aspect_rows == 3)
      multiplier = 120;
   else if (aspect_cols == 16 && aspect_rows == 9)
      multiplier = 30;
   else
      multiplier = 320 / aspect_cols;

   return cv::Size(aspect_cols * multiplier, aspect_rows * multiplier);
}

int ImageCapture::gcd(int a, int b)
{
   if(b == 0) {
      return a;
   }
   else {
      return gcd(b, a % b);
   }
}


