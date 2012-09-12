#ifndef PROSILICACAPTURE_H
#define PROSILICACAPTURE_H

/**
This class extends ImageCapture class to support
Prosilica GigE camera model.

Grab frames from prosilica, and convert to IplImage to 
put to the image pool.
*/

#include <string>
#include <cv.h>
#include <highgui.h>
#include <qthread.h>
#include <qwaitcondition.h>

#include "imagecapture.h"
#include "prosilica/camera.h"

using namespace std;

class ProsilicaCapture : public ImageCapture
{
public:
    ProsilicaCapture(int framerate);
    ~ProsilicaCapture();

    void setImagePool(AbosPool *readpool);


    // functions for thread configuration
    void run();
    void stop();

private:
    Camera* camera;
    Image* image;

	int fps;

    // variables for thread configuration
    //bool stopped;




};

#endif // PROSILICACAPTURE_H
