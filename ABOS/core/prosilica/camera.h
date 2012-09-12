#ifndef CAMERA_H_
#define CAMERA_H_

#include "ImageSource.h"

// this is only for linux machines
#define PVDECL
#define _LINUX
#define _x86

#include <PvApi.h>

#include <qmutex.h>
#include <qwaitcondition.h>

class Camera : public ImageSource
{
	public:
		/** create a new camera using the info provided */
		Camera(tPvCameraInfo info);
		Camera();
		~Camera();
		
		virtual void open();
		virtual void close();
		virtual const Image* grabFrame();

                bool isSet(){ return camera_set; }
		
		tPvHandle handle() const { return _handle; }
		
		//QWidget* configWidget() const { return _configWindow; }
		
		static void PVDECL newFrame(tPvFrame *frame);
		
	public:
		/** number of frames to buffer */
		static const unsigned int NumFrames = 4;
		
	private:
		/** info on the camera to open */
		tPvCameraInfo _info;
		
		/** handle to open camera */
		tPvHandle _handle;
		
		//CameraWindow* _configWindow;
		
		/** captured frames */
		tPvFrame _frames[NumFrames];
		
		/** image data for captured frames */
		Image* _image[NumFrames];
		
		Image* _lastImage;
		
		Image* _lastFrame;
		
		/** locks new frame access */
		QMutex _newFrameMutex;
		QWaitCondition _newFrameWait;

                bool camera_set;
};

#endif /*CAMERA_H_*/
