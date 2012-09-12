#ifndef IMAGECAPTURE_H
#define IMAGECAPTURE_H

/**
This need to be extend to support various input method
such as from video files or many kinds of camera models

Currently, this simply reads from a video format file and
generate CvCapture file to use.
*/

#include <string>
#include <qmutex.h>
/*
#include <cv.h>
#include <highgui.h>
#include <qthread.h>
#include <qwaitcondition.h>
*/

//#include "imagedatapool.h"
#include "abospool.h"
#include "AbosThread.h"

using namespace std;

class ImageCapture : public AbosThread
{
public:
   ImageCapture(int framerate);

   void setDeviceFile(string file_name);
   CvCapture* getCvCapture();
   void readFromCamera(bool b_read_camera);
   void setImagePool(AbosPool *readpool, AbosPool *unscaledpool=NULL);

   // functions for thread configuration
   virtual void run();
   virtual void stop();

   virtual bool isSet() { return input_set; }

   // get or set properties of CvCapture instance
	void setFPS(int fps) { this->fps = fps; }	
	int getFPS() { return fps; }
   int getWidth(){
      return (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH );
   }
   int getHeight(){
      return (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT );
   }
   unsigned long long int getFrameCount() {
      return (unsigned long long int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_COUNT );
   }
   unsigned long long int getNextFrameNo() {
      return nextFrameNo;
   }

   void pause();
   bool goToFrame(long long int);


private:

   cv::VideoCapture video_capture;
   string file_name;
   CvCapture* capture;
   bool paused;

	int fps;
   unsigned long long int nextFrameNo;

   QMutex mutex_nextFameNo;
    
   cv::Size getScaledSize(int cols, int rows);
   int gcd(int a, int b);
 
protected:
    AbosPool *readpool, *unscaledpool;

    // variables for thread configuration
    bool stopped;

    bool input_set;



};

#endif // IMAGECAPTURE_H
