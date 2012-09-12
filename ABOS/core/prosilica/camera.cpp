#include "camera.h"

#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <QMutexLocker>

const unsigned int NumErrors = 22;
const char* PvErrors[NumErrors] =
{
	"Success",
    "CameraFault",
	"InternalFault",
    "BadHandle",
    "BadParameter",
    "BadSequence",
    "NotFound",
    "AccessDenied",
    "Unplugged",
    "InvalidSetup",
    "Resources",
    "Bandwidth",
    "QueueFull",
    "Cancelled",
    "DataLost",
    "DataMissing",
    "Timeout",
    "OutOfRange",
    "WrongType",
    "Forbidden",
    "Unavailable",
    "Firewall"
};

Camera::Camera(tPvCameraInfo info) : ImageSource(),
	_info(info), _handle(0)//, _configWindow(0)
{	
	//clear the frames
	for (unsigned int i = 0; i < NumFrames; i++)
	{
		memset(&_frames[i], 0, sizeof(tPvFrame));
		_image[i] = new Image(0,0);
		//TODO maybe start with _image[i] = 0? 
	}
}

Camera::Camera() : ImageSource(),
	_handle(0)
{
    unsigned long myCameraUID;
    int n = 1;
    tPvCameraInfo cameraInfo;
    camera_set = true;

    if( !PvInitialize() ) {
        sleep(250000);  // initialize and wait fot the response from a camera

        int n = PvCameraList(&cameraInfo, 1, NULL);

        if( n > 0 ) {
            printf("There are %d camera(s) installed.\n", n);
            // get camera ID
            myCameraUID = cameraInfo.UniqueId;
            printf("Camera_1's ID = %ld\n", myCameraUID);
        }
    } else {
        printf("Can't initialize\n");
        camera_set = false;
        return;
    }

	if (n == 0){
		printf("Can't open camera: it might be disconnected?\n");
                camera_set = false;

                return;

            }

    _info = cameraInfo;
    
	//clear the frames
	for (unsigned int i = 0; i < NumFrames; i++)
	{
		memset(&_frames[i], 0, sizeof(tPvFrame));
		_image[i] = new Image(0,0);
		//TODO maybe start with _image[i] = 0? 
	}
}


Camera::~Camera()
{
	close();
}

void Camera::open()
{
	tPvErr err = PvCameraOpen(_info.UniqueId, ePvAccessMaster, &_handle);
	if (err && err < NumErrors)
	{
		printf("Can't open %ld\n", _info.UniqueId);
		printf("Prosilica Error: %s\n", PvErrors[err]);
		
		//TODO throw exception with error code
                camera_set = false;
                return;
            }
	else if (err >= NumErrors)
	{
		//TODO internal failure...don't know what this error is
                camera_set = false;
		return;
	}
	
	//make new config widget
	//_configWindow = new CameraWindow(_handle);
	
	/// setup and start capture...
	err = PvAttrStringSet(_handle, "PixelFormat", "Rgb24");
	if (err != ePvErrSuccess)
	{
	   printf("Error: %s\n", PvErrors[err]);
	}
	    
	unsigned long int frameSize = 0;
	PvAttrUint32Get(_handle, "TotalBytesPerFrame", &frameSize);
	
	printf("Framesize: %lu\n", frameSize);
	
	PvAttrUint32Get(_handle, "Width", (tPvUint32*)&_width);
	PvAttrUint32Get(_handle, "Height", (tPvUint32*)&_height);
	
	for (unsigned int i = 0; i < NumFrames; i++)
	{
		//setup images to use image buffers as frame buffers
		_image[i] = new Image(_width, _height, 3);
		
		if (frameSize != _image[i]->bytes())
		{
			//TODO exception
			printf("INTERNAL ERROR: created image size != expected size\n");
		}
		
		//using the image data as the buffer prevents unnecessary copies of the image
		_frames[i].ImageBuffer = _image[i]->data();
		_frames[i].ImageBufferSize = _image[i]->bytes();
		
		_frames[i].Context[0] = this;
		_frames[i].Context[1] = _image[i];
	}
	
	//start capture
	PvCaptureStart(_handle);
	PvCommandRun(_handle, "AcquisitionStart");
	
	for (unsigned int i = 0; i < NumFrames; i++)
        {
		PvCaptureQueueFrame(_handle, &_frames[i], newFrame);
	}
	
	_lastFrame = 0;
	_lastFrame = new Image(_width, _height);
}

void Camera::newFrame(tPvFrame *frame)
{ 
    Camera* camera = (Camera*)frame->Context[0];
    
    if (frame->Status == ePvErrSuccess)
	{
		camera->_lastImage = (Image*)frame->Context[1];
		
    	//wakeup anyone waiting for the new frame
    	camera->_newFrameWait.wakeOne();
	}
	
	//release the frame back into the queue
	if (frame->Status != ePvErrUnplugged && frame->Status != ePvErrCancelled)
	{
	   PvCaptureQueueFrame(camera->_handle, frame, Camera::newFrame);
	}
}

void Camera::close()
{
	PvCommandRun(_handle, "AcquisitionStop");
	PvCaptureEnd(_handle);
	PvCaptureQueueClear(_handle);
	
	if (_handle)
	{
		PvCameraClose(_handle);
		_handle = 0;
	}
	
//	if (_configWindow)
//	{
//		delete _configWindow;
//		_configWindow = 0;
//	}
	
	//cleanup image data
	for (unsigned int i = 0; i < NumFrames; i++)
	{
		if (_image[i])
		{
			delete _image[i];
			_image[i] = 0;
		}
	}
}

const Image* Camera::grabFrame()
{
	QMutexLocker ml(&_newFrameMutex);
	_newFrameWait.wait(&_newFrameMutex);
	
	uint8_t* img = _lastImage->data();
	uint8_t* out = _lastFrame->data();
	
	const unsigned int end = _width * _height;
	for (unsigned int i=0; i<end ; ++i)
	{
		out[3]  = 0xff;
		out[2] = *img++;
		out[1] = *img++;
		out[0] = *img++;
		out += 4;
	}
	
	return _lastFrame;
}
