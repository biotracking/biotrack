/**
This class extends ImageCapture class to support
Prosilica GigE camera model.

Grab frames from prosilica, and convert to IplImage to 
put to the image pool.
*/


#include "prosilicacapture.h"
#include <iostream>

ProsilicaCapture::ProsilicaCapture(int framerate) : ImageCapture(framerate)
{
    this->stopped = false;
    image = new Image();
    camera = new Camera();

    camera->open();

    if( camera->isSet() == false ){
        this->input_set = false;
        delete camera;
        camera = NULL;
    }
    else{
        input_set = true;
    }

	fps = framerate;
}

ProsilicaCapture::~ProsilicaCapture() {
	if(camera != NULL ){
		camera->close();
		delete camera;
	}
	delete image;
}

void ProsilicaCapture::stop() {
    this->stopped = true;
}

void ProsilicaCapture::setImagePool(AbosPool *readpool){
    this->readpool = readpool;
}

void ProsilicaCapture::run() {
    // main function of the thread
    // read a frame from the Prosilica camera,
    // convert to IplImage and store it to the global pool.

    if( camera != NULL ) {


		//declerations for old code
        //IplImage *cvimg;
        //IplImage *scaled_cvimg, *chn_cvimg;

        do{
            image = (Image*) camera->grabFrame();
            if(image == NULL) {
                readpool->clear();
                continue;
            }
			//this is our original image
			cv::Mat tmp1(cv::Size(image->width(), image->height()),CV_8UC4);
			//copy the data from the frame we just pulled
			tmp1.data = image->data();
			//reshape it to give it 3 channels
			//tmp1.reshape(3);
			cv::Mat tmp2;
			cv::cvtColor(tmp1, tmp2, CV_BGRA2BGR);
			//make a scaled copy of the image
			cv::Mat tmp3(cv::Size(480,360),CV_8UC3);
			cv::resize(tmp2,tmp3,cv::Size(480,360));
			//begin old code
			/*
            cvimg = cvCreateImage( cvSize(image->width(), image->height()), IPL_DEPTH_8U, 4 );
            cvimg->imageData = (char*)image->data();    // share buffers btw/ image and cvimg

            chn_cvimg = cvCreateImage( cvSize(image->width(), image->height()), IPL_DEPTH_8U, 3);
            cvConvertImage(cvimg, chn_cvimg);

            scaled_cvimg = cvCreateImage( cvSize(480, 360), IPL_DEPTH_8U, 3);
            cvResize(chn_cvimg, scaled_cvimg);
			*/
			//end old code
            // put the image to the pool

			/*
                    ID_image *id_image = new ID_image;
                    id_image->image = tmp2;
                    id_image->orig_image = tmp1;
            readpool->storeIDImage(id_image);
			*/
			readpool->storeImage(tmp2);

            //cvReleaseImage(&cvimg);
            //cvReleaseImage(&chn_cvimg);

                    QThread::usleep(1000000 / fps);
        } while (image != NULL && !stopped);

    }
}
        

        

